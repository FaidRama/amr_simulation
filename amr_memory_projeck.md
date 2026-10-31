# 🧠 AMR Memory Project: Dokumentasi, Arsitektur & Aturan Permanen

Dokumen ini adalah **catatan ingatan resmi proyek Delivery AMR** (ROS 2 Foxy pada Nvidia Jetson Nano/Xavier/Orin & Laptop Ubuntu). 
Tujuannya adalah mencatat seluruh solusi, arsitektur yang sudah dikunci, akar masalah yang pernah terjadi, dan **aturan mutlak agar tidak mengulang kesalahan yang sama**.

---

## 📌 1. Arsitektur TF Tree (DIKUNCI PERMANEN - JANGAN DIUBAH LAGI)

Pohon koordinat TF robot telah **dikunci mati** mengikuti standar ROS REP 105:

```
[map]
  └── (AMCL / SLAM Toolbox)
       └── [odom]
             └── (arduino_bridge.py)
                  └── [base_link]  <-- Pusat fisik robot
                        ├── (static_transform_publisher) ──> [laser]          (yaw = 180° / 3.14159 rad)
                        └── (static_transform_publisher) ──> [base_footprint] (identitas z = 0 lantai)
```

### Aturan Baku TF:
1. **`arduino_bridge.py`**:
   - Mem-publish: `odom -> base_link` (frame id `odom`, child frame id `base_link`).
2. **`static_transform_publisher`**:
   - `base_link -> laser` (argumen: `0 0 0 3.14159 0 0 base_link laser`).
   - `base_link -> base_footprint` (argumen: `0 0 0 0 0 0 base_link base_footprint`).
3. **`nav2_params.yaml`**:
   - Parameter `robot_base_frame` dan `base_frame_id` **SEMUANYA MENGGUNAKAN `base_footprint`** (pada AMCL, BT Navigator, Local Costmap, Global Costmap, dan Recoveries Server).
   - **DILARANG MENGGANTI-GANTI LAGI** ke `base_link`! Mengubahnya ke `base_link` akan merusak action server Nav2 (`send_goal failed` dan `bt_navigator exit code -11`).
4. **`adaptive_velocity_node.py`**:
   - Hanya mem-bootstrap `map -> odom` sebelum AMCL aktif. Begitu AMCL aktif (`/amcl_pose` diterima), kendali `map -> odom` diserahkan 100% ke AMCL.
   - **TIDAK BOLEH** mem-broadcast duplikat transform `laser` atau `base_footprint` di `/tf`.

---

## 📌 2. Solusi Nav2 Lifecycle & Biang Kerok Local Costmap Crash (`exit code -11`)

### Masalah:
- `planner_server` sering mati mendadak dengan `exit code -11` (Segmentation Fault) saat start.
- `local_costmap` di RViz statusnya error / bertanda seru kuning, atau tidak muncul.
- `lifecycle_manager_navigation` gagal mengaktifkan node karena menunggu `planner_server`.

### Penyebab:
- Di ROS 2 Foxy, pemisahan `localization_launch.py` dan `navigation_launch.py` membuat `map_server` aktif lebih dulu dan mem-publish `/map`.
- Topik `/map` diterima oleh `StaticLayer` di `planner_server` (`StaticLayer: Resizing costmap to 1074 X 494`) **tepat ketika thread utama sedang menginisialisasi plugin `inflation_layer`**.
- Terjadi tabrakan memori C++ (*race condition*) di `nav2_costmap_2d`.

### Solusi Permanen:
- **SATU Lifecycle Manager Tunggal**: Menggabungkan seluruh 7 node Nav2 ke dalam satu `lifecycle_manager_navigation` di [`real_navigation.launch.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/launch/real_navigation.launch.py):
  `['map_server', 'amcl', 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']`.
- **Urutan Aman**:
  1. Semua node di-`Configure` terlebih dahulu (semua layer costmap selesai di-load).
  2. Baru semua node di-`Activate` (peta baru disebarkan setelah layer aman).
- **Hasil**: Tidak ada lagi tabrakan resize costmap, `planner_server` tidak crash, dan `local_costmap` aktif 100% setiap kali diluncurkan.

---

## 📌 3. Driver RPLiDAR A1 & Anti-Macet USB/Serial

### Masalah:
- LiDAR sering macet di awal, me-restart setiap 6 detik, atau harus cabut-colok kabel USB fisik agar mau membaca port `/dev/rplidar` lagi (`OPERATION_TIMEOUT`).
- Muncul proses "hantu" (*ghost respawn*) yang membunuh proses sensor (`exit code -9`).

### Solusi Permanen di [`rplidar_bridge.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/rplidar_bridge.py):
1. **Direct Binary Execution**: Menjalankan langsung binary `/opt/ros/foxy/lib/rplidar_ros/rplidar_composition` (menghindari lambatnya overhead `ros2 run`).
2. **Startup Timeout 15 Detik**: Memberi waktu motor berputar penuh (Jetson butuh ~6.5 detik saat load Nav2 tinggi).
3. **Reset Serial & MCU**: Mengirim kode perintah resmi Slamtec `STOP (\xA5\x25)` dan `REBOOT MCU (\xA5\x40)` via serial sebelum menyalakan driver.
4. **Reset Hardware USB via Kernel (`USBDEVFS_RESET`)**:
   - Jika terjadi restart berulang, script memanggil ioctl `USBDEVFS_RESET` pada port USB Linux. Efeknya **sama persis seperti mencabut dan mencolok kabel USB secara fisik** tanpa menyentuh kabel.
5. **Anti-Ghost Respawn**: Menghapus `respawn=True` tingkat luar di launch file dan menambahkan fungsi sinkron `_cleanup_stale_processes()` di awal launch file agar tidak ada sisa proses lama yang saling membunuh.

---

## 📌 4. Integrasi Human Awareness, Safe Stop & Waypoint Manager

### Masalah:
- Saat ada manusia berdiri di depan robot dalam waktu lama (misal $> 60$ detik), Nav2 `SimpleProgressChecker` menganggap robot gagal bergerak (`Failed to make progress`) lalu membatalkan misi (*Abort*) atau melakukan *recovery spin* di dekat manusia.

### Solusi Permanen:
1. **`adaptive_velocity_node.py`**:
   - Menerima status dari topik `/human_state` (`NORMAL`, `REDUCED_SPEED`, `SAFE_STOP`).
   - Mengerem kecepatan (`cmd_vel_safe`) menjadi `0.0` seketika saat `SAFE_STOP`.
2. **`waypoint_manager.py` (Metode `navigate_to_point`)**:
   - Jika status `SAFE_STOP` berlangsung $\ge 10.0$ detik (atau tombol Pause ditekan), node **otomatis men-suspend (cancel) Goal Nav2 sementara** dan menahan rem (`emergency_stop()`).
   - Mencegah Nav2 memicu timeout 60 detik atau melakukan *recovery spin* berbahaya di depan orang.
   - **Indefinite Wait**: Menunggu manusia lewat tanpa batasan waktu (tidak akan pernah abort).
   - **Auto-Resume**: Begitu status kembali ke `NORMAL`/`REDUCED_SPEED`, node otomatis mengirim ulang (*re-send*) goal ke titik tujuan yang sama.

---

## 📌 5. Arduino Bridge & Single PID Dashboard

### Masalah:
- Di Dashboard tombol Start Arduino memunculkan **2 PID** di `pgrep -f arduino_bridge.py`.
- Saat dimatikan (`Ctrl+C` atau Stop), proses menggantung 5 detik dan memunculkan error `InvalidHandle`.

### Solusi Permanen:
1. **Single PID di [`app.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/app.py)**:
   - Mengganti `ros2 run amr_simulation arduino_bridge.py` menjadi pemanggilan langsung `python3 arduino_bridge.py`. Hanya 1 PID yang tercipta.
2. **Clean Exit di [`arduino_bridge.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/arduino_bridge.py)**:
   - Handler `SIGINT`/`SIGTERM` langsung mengirim `V,0.000,0.000` sebanyak 3 kali ke serial untuk mengunci motor, menutup port serial, dan memanggil `os._exit(0)`.
   - Proses mati seketika tanpa error handle atau jeda 5 detik.

---

## 📌 6. Standar Jaringan Multi-Machine (Laptop + Jetson)

Seluruh environment ROS 2 dikunci pada:
- **`ROS_DOMAIN_ID=0`** (wajib sama di `.bashrc` laptop `faid` dan Jetson `jetson`).
- **`ROS_LOCALHOST_ONLY=0`** (wajib 0 agar topik bisa berkomunikasi lintas Wi-Fi/Ethernet).
- DDS Middleware: FastRTPS default ROS 2 Foxy.

---

## 📌 7. Notifikasi Kesiapan Navigasi (Dashboard & Terminal)

Sistem hanya menyatakan siap navigasi setelah kelima syarat terpenuhi:
1. `/map` sudah diterima.
2. `/scan_filtered` mengalir lancar.
3. `/amcl_pose` terbit (lokalisasi AMCL terinisialisasi).
4. `/local_costmap/published_footprint` dan `/global_costmap/published_footprint` aktif.
5. Node `bt_navigator`, `planner_server`, dan `controller_server` aktif.

Output yang muncul:
- Terminal: `🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR!`
- Dashboard: Badge berubah dari `⏳ MENYIAPKAN...` menjadi `🟢 AMR SIAP NAVIGASI & MENGANTAR!`.

---

## 📌 8. Optimasi RViz2 (Anti-Crash di Laptop Non-NVIDIA / RAM Terbatas)

### Masalah:
- Peta `peta_l1_ddc` sangat besar ($1074 \times 494$ sel).
- RViz2 sering crash mendadak (`exit code -11` / Segfault) di laptop tanpa GPU NVIDIA (Intel HD / AMD Vega) karena driver OpenGL/OGRE kehabisan buffer saat merender 20 FPS + panah TF 3D.

### Solusi Permanen di [`slam_config.rviz`](file:///home/faid/Documents/amr_ws/src/amr_simulation/rviz/slam_config.rviz):
1. **Frame Rate diturunkan ke `8` FPS** (memotong 60% konsumsi resource CPU/iGPU tanpa mengurangi responsivitas navigasi 2D).
2. **Display `TF` dinonaktifkan secara default** (bisa dicentang jika perlu debug koordinat).
3. **Perintah Menjalankan RViz Super Ringan (Anti Glitch Driver Intel):**
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 ros2 run rviz2 rviz2 -d ~/Documents/amr_ws/src/amr_simulation/rviz/slam_config.rviz
   ```
   *(Atau jalankan normal tanpa `LIBGL_ALWAYS_SOFTWARE=1`, sekarang sudah jauh lebih ringan karena 8 FPS).*

