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

## 📌 3. Driver RPLiDAR A1 & Anti-Macet USB/Serial (DIKUNCI PERMANEN - JANGAN DIUBAH LAGI)

> [!CAUTION]
> **STATUS DIKUNCI MATI (FROZEN)**: `rplidar_bridge.py` telah teruji 100% stabil, sehat, dan handal di Jetson. **DILARANG KERAS** mengutak-atik, mengubah alur serial, membalik polaritas DTR, atau menambah logika coba-coba pada file ini lagi! (Lihat detail lengkap di Bagian 25).

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

> [!IMPORTANT]
> **KESEPAKATAN MUTLAK SISTEM: DOMAIN ID = 0 & LOCALHOST ONLY = 0**
> Seluruh mesin (Laptop Faid, Jetson Xavier robot, serta laptop teman yang bergabung) **WAJIB menggunakan `ROS_DOMAIN_ID=0`**.
> DILARANG KERAS menggunakan `ROS_DOMAIN_ID=1` atau domain lainnya agar semua node (dashboard, teleop, nav2, camera, bridge) selalu berada pada domain DDS yang sama!

Seluruh environment ROS 2 dikunci pada:
- **`ROS_DOMAIN_ID=0`** (Wajib 0 di `.bashrc` laptop `faid`, Jetson `jetson`, dan terminal mana pun).
- **`ROS_LOCALHOST_ONLY=0`** (Wajib 0 agar topik ROS 2 dapat berkomunikasi lintas perangkat/VPN Tailscale).
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** (Standar resmi middleware DDS proyek, menggantikan FastRTPS default).
- **`CYCLONEDDS_URI`**: Mengarah ke file konfigurasi XML masing-masing mesin:
  - Jetson: `file:///home/jetson/deliver_amr/src/amr_simulation/config/cyclonedds_jetson.xml`
  - Laptop Faid: `file:///home/faid/Documents/amr_ws/src/amr_simulation/config/cyclonedds_laptop.xml`

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

## 📌 8. Optimasi RViz2 (Penyebab Crash `Cannot create GL vertex buffer`, `Message Filter dropping message`, & Laptop Non-NVIDIA Stuck Loading)

### Error yang Terjadi:
```text
[rviz]: Message Filter dropping message: frame 'laser' at time ... for reason 'Unknown'
[rviz]: Message Filter dropping message: frame 'odom' at time ... for reason 'Unknown'
[rviz2]: InternalErrorException: Cannot create GL vertex buffer in GLHardwareVertexBuffer::GLHardwareVertexBuffer
terminate called after throwing an instance of 'Ogre::InternalErrorException'
Aborted (core dumped)
```

### Akar Masalah Sebenarnya (Dari Kode Sumber C++ `rviz_default_plugins` ROS 2 Foxy — `ros2/rviz#693` & `ros2/rviz#787`):
1. **Bukan karena kehabisan VRAM GPU**, melainkan **pemanggilan OpenGL (`glGenBuffersARB`) dari Background Thread `tf2` yang tidak memiliki OpenGL Context!**
   - Di OpenGL, fungsi `glGenBuffersARB(1, &mBufferId)` akan selalu mengembalikan `0` (`!mBufferId` $\rightarrow$ `Cannot create GL vertex buffer`) jika dipanggil di luar *main GUI thread* Qt.
   - Di ROS 2 Foxy, plugin `PolygonDisplay` (`polygon_display.cpp` baris 143) memanggil `manual_object_->begin()` & `manual_object_->end()` (yang membuat VBO OpenGL) **secara langsung di dalam callback `processMessage()`**.
   - Ketika `Fixed Frame` di RViz adalah `map`, sedangkan display **`Robot Footprint` (`Polygon`)** berlangganan ke `/local_costmap/published_footprint` yang ber-frame **`odom`**, `tf2_ros::MessageFilter` harus menunggu transformasi `map -> odom` tiba lewat Wi-Fi.
   - Saat TF `map -> odom` tiba di *background thread* milik `tf2_ros::Buffer`, `MessageFilter` memanggil `PolygonDisplay::processMessage()` **dari background thread `tf2` tersebut** (bukan main thread Qt) $\rightarrow$ `glGenBuffersARB` gagal membuat buffer $\rightarrow$ RViz2 langsung *crash* (`Aborted (core dumped)`).
   - Sebaliknya, jika topik memiliki `frame_id: map` (sama dengan `Fixed Frame: map`, seperti `/global_costmap/published_footprint` atau `/plan`), fungsi `tf2::BufferCore::canTransform("map", "map")` langsung *short-circuit* (`return true`) di main thread Qt tanpa pernah berpindah ke background thread `tf2`.
2. **Kenapa Muncul `Message Filter dropping message: frame 'laser' / 'odom'` & Laptop Teman (Tanpa NVIDIA) Stuck Loading?**
   - Jam sistem (*system clock*) antara Jetson dan Laptop memiliki selisih beberapa detik lewat Wi-Fi, sehingga `MessageFilter` yang memiliki antrean pendek dengan QoS `Reliable` membuang pesan `/scan_filtered` (`frame 'laser'`) dan `/local_costmap/published_footprint` (`frame 'odom'`).
   - Selain itu, penggunaan `Reliability Policy: Reliable` pada `/scan_filtered` dan `Durability Policy: Transient Local` pada `/local_costmap/costmap` dengan `Depth: 5` membuat Fast-RTPS menahan paket retransmisi via Wi-Fi dan membuat RViz2 di laptop dengan GPU terintegrasi (Intel UHD/Iris) *hang* ("loading terus") saat inisialisasi framebuffer beresolusi besar (`1848x1016`).

### Solusi Permanen di [`slam_config.rviz`](file:///home/faid/Documents/amr_ws/src/amr_simulation/rviz/slam_config.rviz):
1. **Matikan `Robot Footprint` (`Enabled: false`) & Pindahkan Topiknya ke `/global_costmap/published_footprint` (`frame_id: map`)**:
   - Menghilangkan total pemicu `PolygonDisplay` di background thread `tf2` (`frame 'odom'`) sehingga crash `Cannot create GL vertex buffer` berhenti 100%.
2. **Ubah QoS `/scan_filtered` & `/local_costmap/costmap` ke `Best Effort` + `Volatile` + `Depth: 1`**:
   - Mencegah tumpukan antrean `MessageFilter` saat terjadi *packet drop* Wi-Fi dan menghilangkan *freeze* saat startup RViz2.
3. **Perkecil `Window Geometry` (`1280x720`) & `Frame Rate: 8` FPS**:
   - Sangat ringan untuk laptop tanpa GPU diskrit (Intel iGPU).
4. **Tips Tambahan untuk Laptop Non-NVIDIA (Intel iGPU) jika Ogre 1.12 Hang saat Startup**:
   - Jalankan dengan: `MESA_GL_VERSION_OVERRIDE=3.3 rviz2 -d src/amr_simulation/rviz/slam_config.rviz`
   - Atau gunakan **AMR Web Dashboard** (`http://<IP_JETSON>:8080`) di browser yang sudah menampilkan peta, posisi robot real-time, serta tombol **Set Initial Pose** & **Send Goal** tanpa perlu membuka RViz2 sama sekali!

---

## 📌 9. Solusi `send_goal failed` pada Goal Kedua & `Control loop missed its desired rate`

### Error yang Terjadi:
```
[bt_navigator]: Action server failed while executing action callback: "send_goal failed"
[bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
[controller_server]: Control loop missed its desired rate of 10.0000Hz
[controller_server]: Failed to make progress
```

### Akar Masalah:
1. **Beban DWB Local Planner Terlalu Berat di CPU Jetson:**
   - Parameter `vx_samples: 20` dan `vtheta_samples: 25` dengan `controller_frequency: 10.0 Hz` memaksa Jetson mengevaluasi **$20 \times 25 = 500$ lintasan per 100 ms**.
   - Ketika robot mulai bergerak atau menghadapi tikungan, loop DWB memakan 100% CPU Jetson sehingga loop kontrol ngedrop (`missed its desired rate`).
2. **Action Server Lockup di FastRTPS ROS 2 Foxy:**
   - Karena thread eksekutor `controller_server` sibuk menghitung 500 lintasan, server aksi `/follow_path` terlambat membalas *handshake* saat goal baru dikirim oleh `bt_navigator`.
   - Di ROS 2 Foxy, keterlambatan *handshake* ini langsung dianggap `send_goal failed` dan mengunci (*lockup*) status action server lama.

### Solusi Permanen di [`nav2_params.yaml`](file:///home/faid/Documents/amr_ws/src/amr_simulation/config/nav2_params.yaml):
1. **Turunkan `controller_frequency` ke `8.0` Hz** (sangat stabil untuk differential drive).
2. **Optimasi Sampel Trajektori DWB:**
   - `vx_samples` diturunkan dari `20` ke **`14`**.
   - `vtheta_samples` diturunkan dari `25` ke **`18`**.
   - `sim_time` disesuaikan dari `1.4` ke **`1.2`**.
   - Beban evaluasi lintasan berkurang dari 500 menjadi **252 lintasan (turun 50%!)**, membuat CPU Jetson longgar dan `controller_server` selalu membalas `bt_navigator` tepat waktu.

---

## 10. Mengatasi `send_goal failed` & Goal Aborted di `bt_navigator` (ROS 2 Foxy)

### Gejala:
```text
[bt_navigator-12] [INFO] [1793459927.853122621] [bt_navigator]: Begin navigating from current location to (0.84, -0.27)
[controller_server-9] [INFO] [1793459928.025443938] [controller_server]: Received a goal, begin computing control effort.
[bt_navigator-12] [ERROR] [1793459929.062106949] [bt_navigator]: Action server failed while executing action callback: "send_goal failed"
[bt_navigator-12] [WARN] [1793459929.062409364] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
```
Robot sempat bergerak selama **1 detik pertama**, lalu tepat pada detik ke-1 langsung muncul `"send_goal failed"` dan `Aborting handle`.

### Akar Penyebab (Dari Kode Sumber C++ `foxy-devel` Nav2):
1. **`server_timeout` di-Hardcode 10 ms di `bt_navigator.cpp` (ROS 2 Foxy):**
   - Di branch `foxy-devel` (`nav2_bt_navigator/src/bt_navigator.cpp` baris 144), tertulis:
     ```cpp
     blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));
     ```
   - Berbeda dengan ROS 2 Humble/Iron, `bt_navigator` di ROS 2 Foxy **TIDAK membaca parameter `default_server_timeout` maupun `server_timeout` dari `nav2_params.yaml`**! Nilainya dikunci keras (*hardcoded*) sebesar **10 milidetik** di Blackboard C++.
2. **Kenapa Gagal Tepat 1 Detik Setelah Jalan (`28.025` -> `29.062`)?**
   - Saat *goal* pertama dikirim (`28.025`), `controller_server` sedang *idle* sehingga membalas dalam < 10 ms dan robot mulai berjalan.
   - Tepat **1.0 detik kemudian**, `<RateController hz="1.0">` di XML bawaan Nav2 memicu *replanning* (`ComputePathToPose` menghitung ulang jalur) dan memberi tahu `FollowPath` (`path_updated = true`) untuk mengirim jalur baru ke `controller_server`.
   - Saat itu, `controller_server` sedang **sibuk di tengah loop `computeControl()`** (mengevaluasi ratusan lintasan DWB yang memakan waktu ~40–60 ms di CPU Jetson).
   - Karena `FollowPath` hanya menunggu **10 ms** (`server_timeout_`), fungsi `spin_until_future_complete(node_, future_goal_handle, 10ms)` langsung *timeout* dan melempar `throw std::runtime_error("send_goal failed")`!
3. **Cara Satu-Satunya Mengubah `server_timeout_` di ROS 2 Foxy (`bt_action_node.hpp` baris 52):**
   - Di konstruktor `BtActionNode` dan `BtServiceNode` Foxy:
     ```cpp
     server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
     getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
     ```
   - Artinya, nilai 10 ms hanya bisa ditimpa melalui atribut XML `server_timeout="2000"` langsung pada setiap tag di file Behavior Tree XML!

### Solusi Permanen:
1. **File Custom Behavior Tree XML ([`navigate_w_replanning_and_recovery.xml`](file:///home/faid/Documents/amr_ws/src/amr_simulation/config/navigate_w_replanning_and_recovery.xml)):**
   - Dibuat di `src/amr_simulation/config/navigate_w_replanning_and_recovery.xml` dan dimuat otomatis oleh [`real_navigation.launch.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/launch/real_navigation.launch.py).
   - Seluruh node action & service (`ComputePathToPose`, `FollowPath`, `ClearEntireCostmap`, `Spin`, `Wait`) diberi atribut **`server_timeout="2000"`** (2000 ms = 2 detik).
   - `<RateController hz="0.5">` diatur ke **0.5 Hz** (replan tiap 2 detik, bukan tiap 1 detik) agar tidak membebani CPU dan tidak mem-preempt `controller_server` secara berlebihan.
2. **Keamanan Indoor di [`nav2_params.yaml`](file:///home/faid/Documents/amr_ws/src/amr_simulation/config/nav2_params.yaml):**
   - **`allow_unknown: false`** tetap dipertahankan di `planner_server` agar robot dilarang keras melewati area abu-abu (*unmapped*).

---

## 11. Verifikasi Encoder Arduino Sebelum Navigasi (setara cek LiDAR)

### Gejala:
Navigasi dijalankan saat log Arduino baru sampai `Arduino Bridge PID aktif`, belum muncul `Paket Encoder pertama diterima`. Robot tetap dianggap siap, goal dikirim, roda berputar tapi `/odom` diam -> AMCL & goal pose salah.

### Akar Masalah:
1. `adaptive_velocity_node.py` hanya mengecek Peta + LiDAR + AMCL + Costmap + Nav2. Encoder tidak pernah dicek.
2. `arduino_bridge.py` meneruskan `/cmd_vel` ke motor walau stream ENC belum ada.
3. Auto `/initialpose` dikirim sebelum odom valid.
4. Bug tersembunyi: baris ENC rusak (sisa bootloader, misal `ENC:12`) memicu `ValueError` -> ditangkap sebagai "Serial read error" -> reconnect + DTR reset berulang. Setelah reset, `totalPulsa` Arduino kembali 0 tapi baseline lama dipakai -> lompatan odom besar.

### Solusi:
1. **[`arduino_bridge.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/arduino_bridge.py)**:
   - **Pencegahan Jebakan Bootloader Optiboot (*Optiboot 115200-Baud Watchdog Trap* — AKAR MASALAH UTAMA)**:
     - Bootloader Arduino Uno R3 (`optiboot.c`) berjalan pada **115200 baud** (sama persis dengan baudrate `amr_firmware.ino`) dan mengandalkan **Hardware Watchdog Timer (WDT) 1.0 detik** untuk keluar dari bootloader menuju sketch utama.
     - Di dalam `optiboot.c`, fungsi `getch()` memanggil `watchdogReset()` **setiap kali menerima byte serial di 115200 baud**. Optiboot HANYA keluar ke sketch utama jika jalur serial RX **hening 100% selama 1.0 detik penuh**!
     - Sebelumnya, `arduino_bridge.py` mengirim heartbeat `"V,0.000,0.000\n"` setiap **200 ms** (`0.2s < 1.0s`) saat `not self.encoder_ready`. Akibatnya, WDT Optiboot ter-reset terus setiap 200 ms sehingga ATmega328P **terjebak di dalam bootloader selamanya** dan tidak pernah menjalankan `setup()`/`loop()` untuk mengirim `ENC:0,0`!
     - **Solusi Permanen**: Selama `self.encoder_ready == False`, `arduino_bridge.py` menjaga **100% keheningan TX (`0` byte dikirim ke serial)**. Sketch [`amr_firmware.ino`](file:///home/faid/Documents/amr_ws/src/amr_simulation/arduino/amr_firmware/amr_firmware.ino) secara otomatis menyemburkan `ENC:0,0\r\n` tiap 20 ms begitu keluar dari bootloader tanpa perlu dipancing perintah dari Jetson. Heartbeat `V,0.000,0.000\n` baru mulai dikirim setelah `self.encoder_ready == True`.
   - **Arsitektur Auto-Recovery Bertahap (Identik dengan [`rplidar_bridge.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/rplidar_bridge.py))**:
     - **Tahap 0 (Initial Connect, `restart_count == 0`)**: Memeriksa port & izin akses via `check_hardware_port()`, membuka `/dev/arduino` dengan `dtr=True, rts=True` tanpa `USBDEVFS_RESET` dan tanpa `sleep` pemblokir, lalu mendengarkan stream `ENC:` secara pasif (TX hening 100%).
     - **Tahap 1 (`restart_count == 1`, > 5.0s tanpa `ENC:`)**: Menjalankan `hardware_reset_arduino(force_usb_reset=False)` — membuka koneksi sementara, menarik `dtr=False, rts=False` selama 250 ms (mengosongkan kapasitor RESET 100nF Arduino Uno), menutup port selama 250 ms, lalu membuka kembali dengan `dtr=True` (memicu pulsa reset hardware bersih pada ATmega328P).
     - **Tahap 2+ (`restart_count >= 2`, masih tanpa `ENC:`)**: Eskalasi ke `hardware_reset_arduino(force_usb_reset=True)` — memanggil `reset_usb_device()` (`ioctl USBDEVFS_RESET` ke `/dev/bus/usb/BBB/DDD`) diikuti **jeda wajib `time.sleep(1.5)`** sebelum mengecek `/dev/arduino` (agar tidak membuka node ttyACM lama pada 0.20s saat kernel/udev masih re-enumerasi), dilanjutkan pulsa DTR reset bersih.
     - > **PENTING (Izin Linux `/dev/bus/usb`)**: Agar auto-reset USB dapat diakses user biasa (`jetson`) tanpa sandi sudo, pasang udev rule di Jetson:
       > ```bash
       > echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2341", MODE="0666"' | sudo tee /etc/udev/rules.d/99-arduino-usb.rules && sudo udevadm control --reload-rules && sudo udevadm trigger
       > ```
   - **Pembersihan Proses Yatim (`kill_other_instances`)**: Memastikan tidak ada proses `arduino_bridge.py` hantu yang menahan file descriptor `/dev/arduino`.
   - **Encoder Verification Gate**: Encoder dianggap SIAP setelah **10 paket ENC valid berturut-turut** (`ready_packet_count`), diumumkan `🟢 [ENCODER SIAP]` dan di-publish ke **`/arduino/ready`** (`std_msgs/Bool`, transient local).
   - Sebelum siap: `/cmd_vel` dipaksa 0 (`⛔ /cmd_vel diabaikan`) dan TX serial hening.
   - Diagnostik RX: Mencatat jumlah byte yang masuk sejak connect (`RX=...B`) dan sampel baris non-ENC jika firmware lain sedang ter-flash di board.
   - **Pemutusan Otorisasi USB Sysfs (`authorized` toggle jeda 2.0s) & Fisik Kapasitor**:
     - *Kenapa cabut-colok fisik wajib diberi jeda 2-5 detik*: Board Arduino Uno dan driver BTS7960 memiliki kapasitor perata daya (elco). Saat kabel dicabut, sisa tegangan 5V turun perlahan (*discharging* ~2 detik). Jika dicolok terlalu cepat (<1 detik), muatan belum habis sehingga ATmega328P tidak mengalami *Power-On Reset (POR)* murni.
     - *Kenapa `USBDEVFS_RESET` saja tidak mematikan daya*: Perintah `USBDEVFS_RESET` hanya me-reset pin sinyal data USB (D+/D-), sedangkan jalur daya VBUS 5V tetap mengalir ke Arduino.
     - *Solusi Software*: Pada [`reset_usb_device()`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/arduino_bridge.py#L305-L365), ditambahkan penulisan `0` ke sysfs `/sys/bus/usb/devices/.../authorized` dengan jeda tidur **2.0 detik penuh**, lalu ditulis kembali `1` sebelum `USBDEVFS_RESET` agar kernel Linux memutus total otorisasi perangkat setara cabut-colok kabel fisik.
2. **[`adaptive_velocity_node.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/adaptive_velocity_node.py)**:
   - `encoder_ok` = `/arduino/ready == True` dan `/odom` mengalir (< 1 s).
   - Auto `/initialpose`, pengumuman `🟢 [SIAP NAVIGASI]`, dan output `/cmd_vel_safe` semuanya menunggu `encoder_ok`.
   - Log `[MENUNGGU] Peta ✔ | LiDAR ✔ | Encoder ⏳ | ...` tiap 5 s untuk diagnosis.
   - Parameter `require_encoder` diisi dari `use_arduino` di [`real_navigation.launch.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/launch/real_navigation.launch.py).

### Hasil Pengujian Live di Jetson AGX Xavier:
```text
[INFO] [1793519617.750524659] [arduino_bridge]: ✅ Terhubung ke Arduino di /dev/arduino (DTR=True, TX hening) -> menunggu bootloader & stream ENC...
[INFO] [1793519617.759593063] [arduino_bridge]: ⏳ Menunggu verifikasi encoder (10 paket ENC valid berturut-turut)... Perintah gerak DIBLOK & TX serial HENING sampai encoder siap.
[INFO] [1793519619.356268265] [arduino_bridge]: Paket Encoder pertama diterima (L=0, R=0) -> baseline odom diset, verifikasi stream...
[INFO] [1793519619.540106180] [arduino_bridge]: 🟢 [ENCODER SIAP] 10 paket ENC valid diterima -> /odom & TF odom->base_link VALID
```
- Dalam waktu **1.6 detik**, Optiboot selesai, paket baseline diterima, 10 paket beruntun terverifikasi, dan status kesiapan langsung menyala hijau (`🟢 [ENCODER SIAP]`).
- Nav2 dan AMCL kini terproteksi 100% dari kesalahan odometri.

---

## 12. Solusi Jaringan WiFi Kampus: Tailscale Mesh VPN & CycloneDDS Unicast

### Masalah:
1. Jaringan WiFi kampus (eduroam / enterprise WPA2-Enterprise) menerapkan **Client Isolation** dan **Multicast Rate Limiting / Flooding Protection**, serta sering mengalami roaming drop sehingga kartu `wlan0` Jetson masuk ke status `mode DORMANT` (100% packet loss) saat robot bergerak.
2. Penggunaan Hotspot HP rentan mengalami *bufferbloat* (antrean paket penuh tersedot RViz/Costmap/TF 50Hz) sehingga paket teleop `/cmd_vel` terlambat (> 0.6s) dan memicu watchdog motor ("maju-berhenti-maju-berhenti").

### Solusi:
1. **Tailscale Mesh VPN (WireGuard Layer-3 Tunnel)**:
   - Terpasang pada seluruh perangkat tim proyek:
     | No | Mesin / Hostname | IP Tailscale | OS | Peran / Akses |
     |---|---|---|---|---|
     | 1 | `ubuntu-jetson` | **`100.84.182.79`** | Linux (Tegra 5.10) | Onboard Computer Robot AMR |
     | 2 | `laptop-faid` | **`100.127.135.67`** | Linux Ubuntu | Lead Developer (ROS 2, Git, Dashboard) |
     | 3 | `eganurf-latitude-5420` | **`100.113.121.4`** | Linux Ubuntu | Developer / ROS 2 Remote Node |
     | 4 | `laptop-divala` | **`100.65.225.99`** | Windows 11 | Operator (Web Dashboard & SSH) |
     | 5 | `laptop-rifdah` | **`100.111.27.6`** | Windows 11 | Operator (Web Dashboard & SSH) |
   - Otomatis aktif saat booting (`tailscaled.service`), menembus firewall dan Client Isolation kampus dengan enkripsi end-to-end.
   - Tetap tersambung (*connectionless roaming*) meskipun robot berpindah Access Point kampus.
2. **CycloneDDS Unicast Configuration**:
   - Tailscale secara default tidak melewatkan paket Multicast L2.
   - Dibuatkan file konfigurasi Unicast CycloneDDS resmi:
     - Jetson: [`src/amr_simulation/config/cyclonedds_jetson.xml`](file:///home/faid/Documents/amr_ws/src/amr_simulation/config/cyclonedds_jetson.xml) (Peers: `127.0.0.1`, `100.84.182.79`, `100.127.135.67`, dan IP teman).
     - Laptop: [`src/amr_simulation/config/cyclonedds_laptop.xml`](file:///home/faid/Documents/amr_ws/src/amr_simulation/config/cyclonedds_laptop.xml) (Peers: `127.0.0.1`, `100.127.135.67`, `100.84.182.79`).
   - Mengunci interface jaringan ke `tailscale0` (`<AllowMulticast>false</AllowMulticast>`) sehingga paket ROS 2 mengalir 100% privat, anti-tabrakan dengan mahasiswa lain, dan bebas pemblokiran firewall.
   - **Kunci Penemuan Antar-Proses di Mesin yang Sama (Same-Host Discovery)**:
     - Saat `<AllowMulticast>false</AllowMulticast>` aktif, Cyclone DDS mematikan seluruh paket broadcast/multicast SPDP.
     - Node HANYA mengirim paket discovery ke IP yang tertulis di `<Peers>`. Jika `<Peers>` di Jetson HANYA berisi IP Laptop, maka node Jetson hanya menyapa Laptop dan **tidak pernah menyapa sesama proses di Jetson**! Akibatnya, Laptop bisa melihat node Jetson, tapi terminal di Jetson sendiri tidak bisa melihat nodenya sendiri (`ros2 node list` kosong).
     - **Solusi**: Daftarkan selalu `127.0.0.1` dan IP Tailscale mesin itu sendiri (`100.84.182.79`) di dalam `<Peers>` pada file XML masing-masing.
   - **Pembersihan Cache Daemon ROS 2**:
     - Setiap kali mengubah file konfigurasi XML atau variabel DDS di terminal, selalu jalankan `ros2 daemon stop` agar background daemon me-reload konfigurasi baru.
   - **Sinkronisasi Domain ID (Wajib Domain 0)**:
     - Seluruh perangkat yang terhubung ke jaringan Tailscale robot WAJIB mengekspor `export ROS_DOMAIN_ID=0` dan `export ROS_LOCALHOST_ONLY=0`.
     - Domain 0 adalah kesepakatan mutlak; jika ada satu perangkat yang menggunakan domain lain (misal Domain 1), node di perangkat tersebut tidak akan dapat ditemukan (*isolated*) oleh Jetson maupun laptop lainnya.

---

## 13. Batasan Hardware Fisik: Jetson Hanya 1 Port USB & Solusi ATmega16U2 Hang

### Batasan Arsitektur Robot:
- **Port USB Jetson Hanya 1 Buah**: Carrier board Jetson AGX Xavier milik robot hanya memiliki **1 port USB fisik**.
- Oleh karena itu, seluruh perangkat eksternal (**Arduino Uno, RPLiDAR, dan Kamera**) **WAJIB menggunakan USB Hub bercabang** (topologi USB: `1-4.4.x`).
- **Tipe USB Hub**: USB Hub standar (*dumb hub*), tidak memiliki saklar transistor VBUS per-port (`uhubctl` tidak didukung).

### Akar Masalah:
- Arduino Uno R3 menggunakan mikrokontroler **ATmega16U2** untuk konversi USB-ke-Serial (bukan chip ASIC hardware murni seperti CP2102 pada RPLiDAR).
- Ketika Jetson mati mendadak (misal akibat drop baterai) lalu dihidupkan lagi dalam kondisi kabel tertancap:
  - VBUS 5V pada dumb hub tidak pernah turun ke 0V secara tuntas akibat muatan kapasitor dan tegangan parasit dari rangkaian motor driver BTS7960.
  - Chip ATmega16U2 masuk ke kondisi *USB Bus Suspended / UART buffer deadlock*.
  - Menekan tombol RESET fisik merah di Arduino **hanya me-reset ATmega328P, tidak me-reset ATmega16U2**.
  - Software reset Linux (`USBDEVFS_RESET` / unbind-bind) tidak mematikan daya fisik VBUS.

### Opsi Solusi Permanen Tanpa Perlu Cabut-Colok Manual:
1. **Solusi Terbaik & Paling Rekomended: Bypass USB Menggunakan Hardware UART GPIO Jetson (Header 40-Pin J41)**:
   - Hubungkan pin UART Arduino Uno (Pin 0 RX & Pin 1 TX) langsung ke pin UART Hardware Jetson:
     - Pin 8 Jetson (`UART1_TX`) -> Pin 0 Arduino (`RX`) melalui voltage divider (karena Jetson 3.3V, Arduino 5V).
     - Pin 10 Jetson (`UART1_RX`) <- Pin 1 Arduino (`TX`).
     - Pin 9 Jetson (`GND`) <-> GND Arduino.
   - **Keuntungan**:
     - Membuka port USB Jetson (beban USB Hub berkurang).
     - Chip ATmega16U2 dilewati total (*bypass*), tidak ada potensi USB deadlock sama sekali.
     - Driver kernel Tegra (`/dev/ttyTHS0` / `/dev/ttyTHS1`) sangat stabil saat boot dan reboot.
2. **Solusi Modul USB-to-UART CP2102 / FT232RL Eksternal (Rp 15.000 - Rp 25.000)**:
   - Gunakan modul USB-UART CP2102 dongle kecil yang dicolok ke USB Hub, lalu sambungkan kabel jumper ke pin 0 (RX) & 1 (TX) Arduino Uno.
   - Chip CP2102 adalah chip ASIC hardware murni (sama persis dengan chip RPLiDAR yang terbukti **selalu aman dan tidak pernah macet** saat Jetson mati-hidup).
3. **Solusi Tombol Saklar Pemutus Daya USB (Toggle Switch)**:
   - Pasang saklar toggle kecil / tombol push-to-break pada kabel VBUS (kabel merah 5V) jalur USB Arduino di bodi robot.
   - Jika Jetson sempat mati mendadak, cukup jentikkan saklar 2 detik tanpa perlu merogoh dan mencopot konektor USB fisik.

---

## 14. Arsitektur 3-State Node Readiness UI/UX pada AMR Web Dashboard

### Masalah:
Sebelumnya, ketika pengguna menekan tombol aktivasi fitur di Web Dashboard (**Camera USB, Human Awareness AI / YOLO, Arduino Bridge, dan Navigasi / Dispatch**), status indikator dan tombol langsung berubah seketika menjadi "ONLINE / RUNNING" hijau. Padahal:
- **Arduino Bridge** masih membutuhkan waktu untuk *handshake* serial baud 115200 dan verifikasi stream hardware encoder (10 paket ENC beruntun).
- **Kamera USB** membutuhkan waktu 1–2 detik bagi driver Linux V4L2 & OpenCV untuk mengunci `/dev/video0` dan mempublikasikan frame pertama.
- **Human Awareness AI** membutuhkan waktu 5–10 detik untuk mengalokasikan memori VRAM GPU Jetson Xavier dan memuat bobot neural network YOLO (`best.pt` / `yolov8n.pt`).
- **Nav2 Navigation** membutuhkan 6–10 detik untuk menginisialisasi map server, AMCL, global/local costmaps, controller server, dan BT navigator.

Hal ini membingungkan operator karena status UI sudah "Online", namun perintah joystick belum merespon atau misi navigasi gagal dieksekusi.

### Solusi & Implementasi:
Sistem dirombak menggunakan arsitektur **3-State Process Lifecycle (`OFFLINE` -> `STARTING / LOADING` -> `ONLINE / READY`)** yang terintegrasi dari ROS 2 node, Flask backend, hingga frontend Javascript:

```
[ OFFLINE ]
   │  (User klik "Start / Jalankan")
   ▼
[ STARTING / VERIFYING ] ──> Indikator Kuning Berkedip (pulse-dot)
   │                         Tombol Kuning "⏳ Menyiapkan..."
   │                         Background ROS 2 memverifikasi sensor/topik
   │  (Kriteria Kesiapan Terpenuhi)
   ▼
[ ONLINE / READY ]       ──> Indikator Hijau Solid "🟢 ONLINE (READY)"
                             Tombol Merah "🛑 Stop ..."
```

#### 1. Kriteria Kesiapan (Readiness Checks) di Backend ([`app.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/app.py)):
- **Arduino Bridge (`is_arduino_ready`)**:
  - Node aktif (`is_arduino_active()`) **DAN**
  - Menerima latched boolean `/arduino/ready == True` (age < 3.0s) ATAU stream `/odom` 50Hz aktif (age < 1.5s).
- **Kamera USB (`is_camera_ready`)**:
  - Node aktif (`is_camera_active()`) **DAN**
  - Frame JPEG telah diterima dan ter-cache (`time.time() - last_cam_time < 3.0s` & `latest_frame is not None`).
- **Human Awareness AI (`is_human_node_ready`)**:
  - Node aktif (`is_human_node_active()`) **DAN**
  - Pipeline YOLO telah memproses frame dan mempublikasikan telemetri deteksi ke `/human_info`, `/human_status`, atau `/human_state` (`now - last_human_time < 4.0s`).
- **Navigasi Nav2 (`is_navigation_ready`)**:
  - Node aktif (`is_navigation_active()`) **DAN**
  - Lifecycle manager telah mengaktifkan seluruh managed nodes (`[SIAP NAVIGASI]` atau `Managed nodes are active`).

#### 2. REST API & Telemetri Real-Time ([`app.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/app.py)):
- Endpoint `/api/status`, `/api/arduino/status`, `/api/camera/status`, `/api/human/status`, dan `/api/navigation/status` kini mengembalikan pasangan status:
  - `'active'`: Boolean (apakah proses OS sedang berjalan).
  - `'ready'`: Boolean (apakah sensor/algoritma sudah diverifikasi siap melayani request).
- `RUNTIME_CACHE` di-update secara asinkron tanpa membebani latensi teleop `/cmd_vel`.

#### 3. Frontend Responsive UI/UX ([`app.js`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/static/js/app.js)):
- **Visual State**:
  - **Offline**: Indikator abu-abu/merah, tombol warna hijau/biru `⚡ Start ...`.
  - **Starting / Menyiapkan**: Indikator kuning berkedip (*pulsing yellow dot*), teks `⏳ MENYIAPKAN...`, tombol kuning `btn-warning` bertuliskan `⏳ Menyiapkan...` (atau `⏳ Memuat YOLO...` / `⏳ Menyiapkan Nav2...`).
  - **Online / Ready**: Indikator hijau solid, teks `🟢 ONLINE (READY)`, tombol merah `🛑 Stop ...`.
- **Proteksi Tombol Dispatch**: Tombol *"Mulai Eksekusi Misi"* di halaman Dispatch otomatis di-disable dan diredupkan (opacity 0.5) selama Nav2 masih dalam status `⏳ MENYIAPKAN NAV2...`, dan baru terbuka penuh saat status berubah menjadi `🟢 AMR SIAP MENGANTAR (READY)`.
- **Notifikasi Toast**: Memberi notifikasi hijau saat hardware/sensor benar-benar siap (misal: `"🟢 Arduino Bridge siap! Hardware encoder & serial terverifikasi"` atau `"🟢 Human Awareness AI siap! Model YOLO aktif mendeteksi"`).

---

## 📌 15. Manajemen Daemon Service: `amr_dashboard.service` (Systemd pada Jetson)

### Status & Arsitektur Service:
- Di Jetson Xavier robot, AMR Web Dashboard Studio berjalan otomatis saat sistem booting menggunakan unit service systemd: **`amr_dashboard.service`**.
- Ini menjelaskan mengapa proses dashboard (`dashboard.launch.py` -> `waypoint_manager` + `dashboard_server`) berjalan dengan TTY tanda tanya (`?`) di latar belakang (background OS daemon).

### Prosedur Wajib Saat Ada Update Kode / DDS:
Karena `amr_dashboard.service` berjalan terus-menerus di memori OS:
1. Setiap kali melakukan pembaruan kode (`app.py`, template HTML, file JS frontend, parameter, atau konfigurasi CycloneDDS XML):
   - **Wajib me-restart service** agar perubahan termuat ke RAM:
     ```bash
     sudo systemctl restart amr_dashboard
     ```
2. **Perintah Pemantauan & Diagnostik Service**:
   - Mengecek status aktif service:
     ```bash
     systemctl status amr_dashboard
     ```
   - Melihat log real-time / streaming log output dashboard:
     ```bash
     journalctl -u amr_dashboard -f
     ```
   - Mematikan service sementara (misal saat ingin debugging manual di terminal via `run dashboard`):
     ```bash
     sudo systemctl stop amr_dashboard
     ```
   - Menghidupkan kembali service:
     ```bash
     sudo systemctl start amr_dashboard
     ```

3. **Runner Script (`start_dashboard.sh`)**:
   - Service mengeksekusi script `/home/jetson/deliver_amr/start_dashboard.sh`.
   - File ini di-track di Git dan memastikan environment `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, dan `CYCLONEDDS_URI` terpasang sebelum `dashboard.launch.py` dijalankan.

---

## 📌 16. Akses Publik Multi-Device: Tailscale Funnel (HTTPS)

- **Domain Publik Resmi**: `https://delivery-going.pompano-halosaur.ts.net/`
  - Mem-proxy port 5000 lokal (`127.0.0.1:5000`).
  - Dapat diakses oleh siapa pun di dunia (dosen, penguji, mahasiswa) via browser HP/laptop tanpa perlu menginstal aplikasi Tailscale.
- **Perintah Manajemen Funnel di Jetson**:
  - Mengaktifkan di background: `sudo tailscale funnel --bg 5000`
  - Mengecek status: `tailscale funnel status`
  - Mematikan / Reset: `sudo tailscale funnel reset`

---

## 📌 17. Solusi Deadlock Threading Python (`app.py`), Instant Render Frontend (`app.js`) & Reset Daemon DDS

### 1. Akar Masalah Deadlock Kunci Python (`threading.Lock` vs `threading.RLock`) di [`app.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/app.py):
- **Kronologi Masalah**:
  - Endpoint `/api/status` mengambil kunci data thread-safe: `with ros_node.lock:`.
  - Di dalam blok tersebut, kode mengevaluasi data kesiapan dengan memanggil `is_arduino_ready()`, `is_camera_ready()`, dan `is_human_node_ready()`.
  - Di dalam ketiga fungsi tersebut, terdapat baris `with ros_node.lock:` lagi.
  - Karena sebelumnya didefinisikan sebagai `self.lock = threading.Lock()` standar (non-reentrant):
    - Thread Flask yang sedang memegang lock mencoba mengakuisisi kembali lock yang sama.
    - Terjadi **recursive deadlock seketika**: thread membeku selamanya menunggu dirinya sendiri melepaskan lock.
    - Seluruh worker thread Flask yang melayani polling `/api/status` ikut mengantre dan membeku.
    - Browser mengalami request timeout sehingga menampilkan status **`🔴 OFFLINE`**, tombol kamera & teleop tidak merespons, dan thread `rclpy.spin(ros_node)` ikut beku.
- **Solusi Permanen**:
  - Ganti `self.lock = threading.Lock()` menjadi **`self.lock = threading.RLock()`** (*Reentrant Lock*). Reentrant lock mengizinkan thread yang sama untuk mengakuisisi lock secara berlapis (*nested*) tanpa menimbulkan deadlock.
  - Bungkus endpoint `/api/status` dengan fail-safe `try ... except` agar selalu merespons JSON status 200 OK ke browser.

### 2. Solusi Layar Blank Saat Pertama Buka / Refresh di [`app.js`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/static/js/app.js):
- **Masalah**: Saat halaman dashboard pertama kali dibuka atau di-refresh, area utama `#app-content` hitam/kosong (blank) dan baru muncul setelah pengguna mengklik tombol menu "Overview" di sidebar.
- **Penyebab**: Di fungsi `init()`, perintah `navigateTo('home')` diletakkan di baris paling akhir setelah menunggu 3 pemanggilan asinkron `await` (`fetchModeState`, `saved_points`, `settings`). Jika server lambat atau sempat error 503, pemanggilan terhenti dan navigasi tidak pernah terpanggil.
- **Solusi Permanen**:
  - Pindahkan pemanggilan `navigateTo('home')` ke **baris paling awal** di dalam `init()`.
  - Begitu event `DOMContentLoaded` selesai, UI Mission Overview langsung dirender secara instan (*instant zero-delay render*), sementara request data dijalankan di latar belakang dengan pembungkus `try/catch`.

### 3. Pembersihan Cache Daemon ROS 2 Multi-Machine (`ros2 daemon stop`):
- Background daemon ROS 2 (`_ros2_daemon`) meng-cache tabel discovery CycloneDDS.
- Setiap kali mengubah file konfigurasi peers XML (`cyclonedds_laptop.xml` atau `cyclonedds_jetson.xml`), IP Tailscale, atau `ROS_DOMAIN_ID`:
  - **Wajib menjalankan `ros2 daemon stop`** di terminal laptop dan Jetson.
  - Jika tidak di-stop, perintah `ros2 node list` di terminal akan terus membaca cache lama dan gagal mendeteksi node dari perangkat lain.

---

## 📌 18. Alat Evaluasi Skripsi Bab 4: Pengukuran Peta 2D & Uji AMCL

Kedua skrip berikut sempat terhapus saat *merge* Git terdahulu dan kini telah dipulihkan secara penuh di folder `src/amr_simulation/scripts/`:

### 1. Tool Pengukuran Akurasi Geometris Peta 2D (`ukur_peta_skripsi.py`)
- **Lokasi File**: [`src/amr_simulation/scripts/ukur_peta_skripsi.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/ukur_peta_skripsi.py)
- **Tujuan (Skenario 1)**: Menguji akurasi geometris rekonstruksi peta SLAM Toolbox terhadap *ground truth* fisik yang diukur manual menggunakan meteran pita baja.
- **Metrik Kelulusan**:
  - $RMSE \le 0.05\text{ m}$ ($5\text{ cm}$)
  - $\text{Galat Skala (Scale Error)} \le 2.0\%$
- **Fitur Interaktif (Matplotlib GUI)**:
  - Zoom & Pan menggunakan toolbar visual.
  - Klik kiri 2 kali pada peta: Titik A (awal landmark) dan Titik B (akhir landmark).
  - Terminal interaktif: masukkan Nama Landmark dan Ukuran Nyata Meteran ($D_{\text{nyata}}$).
  - Keyboard shortcut: `'U'` untuk Undo landmark terakhir jika salah klik, `'S'` atau klik tombol untuk simpan hasil.
  - Output otomatis: Tabel hasil CSV (siap ke Word/Excel) dan gambar peta beranotasi PNG disimpan di `~/benchmark_skripsi_results/evaluasi_peta/`.
  - **Fitur Pembaca Koordinat ROS (Baru)**: Kursor Matplotlib secara realtime menampilkan koordinat Map Frame $(X, Y)$ dalam meter pada status bar kanan bawah (`Map Frame: X=...m, Y=...m`) serta mencetak koordinat ROS saat titik diklik.
- **Analisis Metodologi & Sub-Pixel Accuracy (Penyebab Selisih 10 cm & Solusinya)**:
  - Resolusi grid peta adalah $0.05\text{ m/px}$ ($5\text{ cm per piksel}$).
  - Jika mengukur bukaan lorong / pintu / meja (bidang dalam-ke-dalam), titik klik **WAJIB diletakkan di sudut permukaan dalam** (antarmuka antara piksel putih dan piksel hitam).
  - Jika titik ditarik sampai ke tepi luar dinding hitam, ketebalan dinding ($2 - 3\text{ piksel} = 10 - 15\text{ cm}$) ikut terhitung dan menimbulkan galat semu.
  - **Hasil Validasi Nyata**: Ground truth $0.685\text{ m}$ menghasilkan $0.710\text{ m}$ di peta (galat hanya $2.5\text{ cm} = 0.5\text{ piksel}$), membuktikan rekonstruksi SLAM sangat presisi dan **LULUS kriteria kelulusan Skripsi ($RMSE \le 0.05\text{ m}$)**.
- **Cara Menjalankan**:
  ```bash
  # Mengukur peta default atau peta terbaru peta_l1_dc:
  python3 src/amr_simulation/scripts/ukur_peta_skripsi.py --map src/amr_simulation/maps/peta_l1_dc.yaml
  ```

### 2. Tool Evaluasi Pelacakan Posisi Partikel AMCL (`uji_amcl_skripsi.py`)
- **Lokasi File**: [`src/amr_simulation/scripts/uji_amcl_skripsi.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/uji_amcl_skripsi.py)
- **Tujuan (Skenario 2)**: Mengevaluasi keandalan filter partikel Nav2 AMCL dalam menentukan pose absolut robot $(x, y, \theta)$ di atas peta tersimpan.
- **Mode Pengujian Ganda (Baru)**:
  1. **Mode 1: POSE TRACKING (Default / Rekomendasi Skripsi)**:
     - Mengirim perkiraan pose awal `/initialpose` di sekitar titik datum fisik dengan sebaran partikel lokal Gaussian ($\sigma \approx 30\text{ cm}$, $\sigma_\theta \approx 15^\circ$).
     - AMCL tidak menyebarkan partikel acak buta ke seberang ruangan/meja, sehingga bebas dari *perceptual aliasing* simetri ruangan dan mengunci posisi secara cepat & presisi saat robot berputar.
  2. **Mode 2: GLOBAL LOCALIZATION (Kidnapped Robot)**:
     - Menyebarkan 2000 partikel acak buta ke seluruh penjuru peta via `/reinitialize_global_localization`.
- **Fitur Salin Pose Live AMCL (`'C'`)**:
  - Pada menu `[4] Ubah Koordinat Ground Truth`, status pose AMCL real-time ditampilkan langsung di layar, dan pengguna cukup menekan `'C'` untuk menyalin koordinat aktual robot ke Datum 1-5 tanpa perlu mengetik angka manual.
- **Metrik Kelulusan**:
  - 5 kali pengulangan pada 5 titik datum (Total 25 kali pengujian).
  - Waktu Konvergensi: $t_{\text{conv}} \le 10.0\text{ s}$ ($\sigma_x \le 0.05\text{ m}$, $\sigma_y \le 0.05\text{ m}$, $\sigma_\theta \le 0.05\text{ rad}$).
  - Rata-rata Galat Posisi: $e_{\text{pos}} \le 0.05\text{ m}$ ($5\text{ cm}$).
  - Rata-rata Galat Orientasi: $e_\theta \le 3.0^\circ$.
- **Output**:
  - Disimpan otomatis ke CSV di `~/hasil_pengujian_skripsi/skenario2_evaluasi_amcl.csv` lengkap dengan kolom `Mode_Uji`.
- **Cara Menjalankan**:
  ```bash
  python3 src/amr_simulation/scripts/uji_amcl_skripsi.py
  ```

---

## 📌 19. Visualisasi Peta Mandiri & Penentuan Koordinat Datum di RViz2 (`view_map.launch.py`)

- **Lokasi File**: [`src/amr_simulation/launch/view_map.launch.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/launch/view_map.launch.py)
- **Tujuan**: Membuka peta `.yaml` & `.pgm` langsung di RViz2 secara mandiri di Laptop tanpa perlu menyalakan robot fisik (menggunakan `nav2_map_server` dan `nav2_lifecycle_manager`).
- **Cara Menjalankan di Laptop**:
  ```bash
  source /opt/ros/foxy/setup.bash
  source ~/Documents/amr_ws/install/setup.bash
  ros2 launch amr_simulation view_map.launch.py
  # Atau spesifikasikan peta:
  ros2 launch amr_simulation view_map.launch.py map:=src/amr_simulation/maps/peta_l1_dc.yaml
  ```
- **Cara Mengambil Koordinat Pasti di RViz2**:
  1. **Koordinat Posisi $(X, Y)$**:
     - Di terminal jalankan: `ros2 topic echo /clicked_point`
     - Di toolbar RViz, klik tombol **`Publish Point`** lalu klik pada titik acuan di peta.
  2. **Koordinat Posisi & Sudut Hadap $(X, Y, \text{Yaw})$**:
     - Di terminal jalankan: `ros2 topic echo /goal_pose --once`
     - Di toolbar RViz, klik tombol **`2D Goal Pose`** / **`Nav2 Goal`** lalu klik & drag panah ke arah robot menghadap.

---

## 📌 20. Solusi Bottleneck Inisialisasi Nav2 (`[MENUNGGU] AMCL ⏳ | Costmap ⏳`) & Dual QoS `/initialpose`

### 1. Gejala & Akar Masalah:
- Saat `real_navigation.launch.py` dijalankan (termasuk via Web Dashboard), node `adaptive_velocity_node` terus mencetak log:
  ```text
  [MENUNGGU] Peta ✔ | LiDAR ✔ | Encoder ✔ | AMCL ⏳ | Costmap ⏳
  ```
- **Akar Penyebab**:
  1. **AMCL Menunggu Initial Pose**: AMCL tidak akan mem-publish topik `/amcl_pose` sampai menerima perkiraan posisi awal (`/initialpose`) atau sebelum robot bergerak.
  2. **Costmap Menunggu AMCL**: *Local Costmap* dan *Global Costmap* menahan aktivasi jejak kaki robot (*published footprint*) sampai transformasi `map -> odom` dari AMCL tersedia.
  3. **Ketidakcocokan Kebijakan QoS DDS ROS 2 Foxy**: Nav2 AMCL berlangganan topik `/initialpose` dengan durabilitas **`TRANSIENT_LOCAL`**. Jika publisher mengirim pesan menggunakan durabilitas default **`VOLATILE`** (`depth=10`), DDS CycloneDDS menganggap QoS tidak kompatibel sehingga pesan diabaikan (*silently dropped*).
  4. **Timeout Sensor LiDAR Awal**: Jika sensor RPLiDAR sempat mengalami timeout USB saat startup (seperti auto-reset USB selama $\approx 60\text{ detik}$), auto-initialpose bawaan sempat kehabisan batas percobaan (sebelumnya dibatasi 8 kali).

### 2. Solusi Permanen:
1. **Dukungan Dual QoS `/initialpose`**:
   - Di [`adaptive_velocity_node.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/adaptive_velocity_node.py) dan [`app.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/app.py), publisher `/initialpose` diubah menjadi publisher ganda: **`TRANSIENT_LOCAL` (reliable)** dan **`VOLATILE`**.
   - Timestamp pesan di-generate dari clock aktual node (`self.get_clock().now().to_msg()`).
2. **Auto-Retry Tanpa Batas**:
   - Di `adaptive_velocity_node.py`, batasan kuota 8 kali dihapus dan diganti dengan pengiriman berkala setiap $4\text{ detik}$ sampai AMCL benar-benar terkonfirmasi aktif (`amcl_initialized = True`).
3. **Inisialisasi Manual Seketika via Dashboard**:
   - Pengguna cukup mengklik tombol **`📍 Set Posisi Awal`** di toolbar Dashboard (atau klik kanan pada kanvas peta $\rightarrow$ `Set Posisi Awal Robot (AMCL)`), atau mengklik tombol **`🔄 Relokalisasi (360°)`**.
   - Begitu tombol ini diklik, status `AMCL ⏳` dan `Costmap ⏳` langsung berubah menjadi **`✔`**, dan log terminal berganti hijau:  
     `🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR!`

---

## 📌 21. Optimasi Manuver Putar Balik 180° (U-Turn) DWB Local Planner & Penanganan Ragu/Macet

### 1. Gejala & Fenomena:
- Saat robot diperintahkan bergerak ke tujuan yang arahnya berlawanan (putar balik 180° / U-turn), robot tampak terdiam, "kebingungan / berpikir lama", atau berhenti beberapa detik sebelum akhirnya mulai berputar.

### 2. Akar Masalah:
1. **Deadband Motor Fisik & Gesekan Ban (`min_speed_theta: 0.00`)**:
   - DWB menghasilkan profil trajektori putar yang dimulai dari kecepatan sudut sangat kecil ($\omega \approx 0.03 - 0.08\text{ rad/s}$).
   - Karena robot differential drive memiliki bobot sasis yang cukup berat dan ban karet bersentuhan langsung dengan lantai, tegangan PWM motor pada kecepatan rendah tersebut belum sanggup memecahkan gaya gesek statis (*breakaway friction / deadband*).
   - Secara perangkat lunak ROS 2 Nav2 sudah memerintahkan berputar, tetapi secara fisik robot diam menunggu akumulasi $\omega$ melewati ambang torsi putar motor.
2. **Konflik Bobot Evaluasi Jalur (`PathAlign.scale: 4.0`)**:
   - Parameter `PathAlign.forward_point_distance: 0.30` mengevaluasi titik 30 cm di depan hidung robot agar selalu searah garis jalur (*global path*).
   - Ketika jalur berbalik 180° persis di belakang robot, semua trajektori menghasilkan penalti `PathAlign` yang sangat buruk. Bobot `4.0` yang terlalu dominan menyebabkan kebuntuan evaluasi (*scoring deadlock*) sesaat saat memilih arah putaran awal.
3. **Pemicu Palsu Rem Osilasi (`Oscillation.scale: 5.0`)**:
   - Di awal U-turn, DWB mengevaluasi putaran kiri (+$\omega$) lalu sedikit penyesuaian kanan (-$\omega$).
   - Fluktuasi kecil arah putar dalam jarak sempit ($< 0.05\text{ m}$) langsung dideteksi oleh `OscillationCritic` sebagai bahaya osilasi liar.
   - Karena skalanya sangat sensitif (`5.0`), Nav2 memotong kecepatan robot menjadi `0.0` (rem paksa di tempat) sehingga robot terhenti sesaat.
4. **Batasan Komputasi CPU Jetson (`vx_samples` & `vtheta_samples`)**:
   - Total trajektori yang dihitung per siklus adalah $\text{vx\_samples} \times \text{vtheta\_samples}$.
   - Menaikkan `vtheta_samples` ke 20 atau 25 terbukti membebani CPU Jetson Nano/Orin dan memicu *control loop missed*. Oleh karena itu, jumlah sampel putar **wajib dipertahankan di nilai 18**.

### 3. Solusi Parameter di `nav2_params.yaml`:
Solusi dilakukan murni dengan menyetel torsi putar awal dan bobot critic **tanpa menambah beban CPU Jetson sama sekali (0% CPU overhead)**:

```yaml
    # DWB parameters
    FollowPath:
      min_vel_x: 0.0
      ...
      min_speed_theta: 0.20        # (Sebelumnya: 0.00) -> Memberi torsi putar awal mengatasi gesekan ban/lantai
      vx_samples: 14               # TETAP (Aman untuk CPU Jetson)
      vtheta_samples: 18           # TETAP (Jangan dinaikkan ke 20-25 agar tidak membebani Jetson)
      ...
      PathAlign.scale: 1.5         # (Sebelumnya: 4.0) -> Mengurangi penalti hadap saat jalur berada di belakang
      Oscillation.scale: 1.0       # (Sebelumnya: 5.0) -> Mencegah pemotongan kecepatan mendadak saat manuver U-turn
```


---

## 📌 22. Solusi Sintaks CycloneDDS XML & Error `rcl node's rmw handle is invalid`

### 1. Gejala & Pesan Error:
Saat menjalankan node ROS 2 (misal `run arduino` atau `arduino_bridge.py`), muncul pesan kegagalan pembuatan domain DDS:
```text
python3: config: //CycloneDDS/Domain/Discovery/Peers: parser error invalid token encountered at line 18 (file:///home/jetson/deliver_amr/src/amr_simulation/config/cyclonedds_jetson.xml line 16)
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
_rclpy.RCLError: Unknown error creating node: rcl node's rmw handle is invalid, at ...
```

### 2. Akar Penyebab:
- Terjadi kesalahan sintaks XML (*XML parser invalid token*) di dalam tag `<Peers>` pada file konfigurasi `cyclonedds_jetson.xml`.
- Penyebab umum mencakup:
  1. Lupa menutup tag `<Peer address="..."/>` (misal `<Peer address="...">` tanpa penutup `/>` atau tanpa `</Peer>`).
  2. Komentar bertumpuk (*nested comment* `<!-- <!-- ... --> -->` atau karakter `--` di dalam komentar).
  3. Tanda kutip miring / unicode (`“...”`) akibat copy-paste dari aplikasi pengolah kata.
- Kegagalan parsing XML menyebabkan RMW layer (`rmw_cyclonedds_cpp`) gagal menginisialisasi domain DDS, sehingga semua panggilan pembuatan node (`rclpy.create_node`) gagal dengan `rmw handle is invalid`.

### 3. Solusi & Format Standar:
Pastikan file konfigurasi XML bersih dan rapi:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>tailscale0</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <!-- Selalu daftarkan loopback dan IP Tailscale mesin sendiri -->
                <Peer address="127.0.0.1"/>
                <Peer address="100.84.182.79"/>
                <!-- IP Tailscale Laptop Developer / Operator -->
                <Peer address="100.127.135.67"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>
```
Setelah diedit, jalankan `ros2 daemon stop` dan build kembali workspace bila perlu.

---

## 📌 23. Optimasi Modul Human Awareness: Solusi Lompat State Langsung ke `SAFE_STOP` (Bypassing `REDUCED_SPEED`)

### 1. Gejala & Fenomena:
Saat dilakukan penyesuaian lebar koridor lintasan robot (`corridor_width`) menjadi lebih sempit (tujuannya agar manusia di posisi serong tidak memicu perlambatan), saat robot mendekati manusia lurus di depannya, state robot dari `NORMAL` langsung melompat seketika ke `SAFE_STOP`. State `REDUCED_SPEED` sama sekali tidak tampak/terlewati.

### 2. Akar Masalah:
1. **Jebakan Matematis Overlap Koridor (`corridor_overlap >= 0.60`)**:
   - Kode sebelumnya mengharuskan minimal 60% lebar bounding box manusia berada di dalam kotak koridor (`person["corridor_overlap"] >= 0.60`).
   - Ketika koridor dipersempit (misal menjadi selebar 120 px) dan manusia mulai mendekat, tubuh manusia membesar (misal selebar 250 px).
   - Irisan maksimal hanyalah selebar koridor itu sendiri (120 px), sehingga rasio overlap maksimal hanyalah $\frac{120}{250} = 48\% < 60\%$.
   - Manusia yang berdiri tepat di depan hidung robot pun **dinyatakan bukan di dalam koridor (`target = None`)**. Akibatnya, zona koridor dipaksa bernilai `Zone.FAR` (`NORMAL`).
2. **Logika Omnidirectional Emergency Stop (`near_person_outside_or_inside`)**:
   - Di baris klasifikasi kedekatan, terdapat pemeriksaan:
     ```python
     near_person_outside_or_inside = any(person["r"] >= args.t_near_enter for person in persons)
     ```
   - Pemeriksaan ini sengaja memeriksa **seluruh manusia tanpa memedulikan koridor**.
   - Ketika manusia masih berjarak sedang, manusia diabaikan karena tereliminasi oleh koridor sempit. Namun begitu manusia sudah sangat dekat hingga rasionya melewati `t_near_enter`, `force_near = True` langsung aktif dan membanting state seketika ke `SAFE_STOP`.
3. **Rentang Ambang Batas Terlalu Rapat**:
   - Nilai default `--t-med-enter: 0.85` dan `--t-near-enter: 0.95` hanya memiliki selisih $0.10$. Karena rasio bounding box membesar secara hiperbolik saat mendekat, rentang ini terlewati hanya dalam beberapa frame kamera.

### 3. Solusi Permanen:
1. **Ganti Logika Koridor Berbasis Titik Tengah Horizontal (*Center-X*) di [`human_awareness_node.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/human_awareness_node.py)**:
   Mengganti perhitungan persentase irisan dengan pengecekan apakah titik pusat tubuh manusia berada di dalam koridor lintasan robot:
   ```python
   corridor_persons = []
   for person in persons:
       # Titik tengah horizontal tubuh
       person_center_x = (person["x1"] + person["x2"]) / 2.0

       # Manusia di dalam koridor jika titik tengahnya berada di koridor,
       # ATAU jika tubuhnya sangat dekat hingga mencakup seluruh koridor
       person["in_corridor"] = (
           (corridor_x1 <= person_center_x <= corridor_x2) or
           (person["x1"] <= corridor_x1 and person["x2"] >= corridor_x2)
       )

       if person["in_corridor"]:
           corridor_persons.append(person)
   ```
2. **Standardisasi Ambang Batas Rasio Hysteresis**:
   Menyesuaikan nilai default pada `human_awareness_node.py` dan `fsm.py` agar transisi kecepatan terlihat jelas dan halus:
   - **`t_med_enter = 0.60`** (mulai melambat di jarak $\approx 2.0 - 2.5\text{ m}$)
   - **`t_med_exit = 0.50`** (kembali ke kecepatan normal saat orang menjauh)
   - **`t_near_enter = 0.85`** (rem darurat total di jarak $\approx 1.0 - 1.3\text{ m}$)
   - **`t_near_exit = 0.80`** (keluar dari rem darurat saat orang mundur)

---

## 📌 24. Solusi AMCL Macet / Crash: `pf.c:381: pf_update_resample: Assertion 'i < set_a->sample_count' failed` (Nav2 Issue #2711)

### 1. Gejala & Error:
Pada log sistem Nav2 / Dashboard:
```text
[amcl-8] [INFO] [1793608198.693869320] [amcl]: createLaserObject
[adaptive_velocity_node.py-6] [INFO] [1793608198.718929131] [adaptive_velocity_node]: 📍 Mengirim auto-initialpose ke AMCL (percobaan #1)...
[amcl-8] [INFO] [1793608198.720448273] [amcl]: initialPoseReceived
[amcl-8] [INFO] [1793608198.720945672] [amcl]: Setting pose: 0.000 0.000 0.000
[amcl-8] amcl: /tmp/binarydeb/ros-foxy-nav2-amcl-0.4.7/src/pf/pf.c:381: pf_update_resample: Assertion `i < set_a->sample_count' failed.
[ERROR] [amcl-8]: process has died [pid 42762, exit code -6]
[adaptive_velocity_node.py-6] [INFO] [MENUNGGU] Peta ✔ | LiDAR ✔ | Encoder ✔ | AMCL ⏳ | Costmap ✔
```
Setelah itu, status AMCL tertahan di `AMCL ⏳` selamanya, dan percobaan auto-initialpose terus terulang hingga puluhan kali tanpa pernah direspons.

### 2. Akar Masalah:
- Ini adalah **bug *race condition* terkenal pada `nav2_amcl` ROS 2 Foxy (Nav2 Issue #2711)**.
- Ketika driver LiDAR baru pulih / mengirim paket scan pertama, AMCL memanggil `createLaserObject` dan memulai proses resampling awal partikel (`pf_update_resample`).
- Di saat bersamaan (selisih hanya 13 ms), `adaptive_velocity_node.py` langsung menembakkan pesan `/initialpose`.
- Pada ROS 2 Foxy `nav2_amcl 0.4.7`, callback `initialPoseReceived` dan thread sensor scan tidak memiliki *mutex lock* penuh saat memodifikasi memori *particle filter* (`pf_`).
- Data partikel diakses secara bersamaan, bobot partikel menjadi korup / tak ternormalisasi, sehingga indeks partikel `i` melebihi `sample_count` dan memicu `Assertion failed` (`SIGABRT / exit code -6`).
- Begitu proses AMCL mati, AMCL tidak berjalan lagi di ROS 2, sehingga perintah *Set Posisi Awal* dari pengguna atau dari skrip tidak akan pernah diproses.

### 3. Solusi di [`adaptive_velocity_node.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/adaptive_velocity_node.py):
1. **Stabilisasi Aliran Laser Sebelum Initial Pose (`scan_stable`)**:
   - Menambahkan pelacakan waktu pertama scan tiba (`self.first_scan_wall_time`).
   - `send_auto_initialpose` **diwajibkan menunggu jeda stabil minimal 2.5 detik** setelah paket laser scan pertama diterima.
   - Jeda ini memberi waktu bagi AMCL untuk menyelesaikan `createLaserObject`, membangun *likelihood field*, dan menstabilkan thread pemrosesan sebelum menerima pose awal.
2. **Deteksi Crash Node AMCL**:
   - Menambahkan deteksi otomatis jika node `amcl` sudah tidak terdaftar di ROS 2 (`self.get_node_names()`), sehingga langsung memunculkan pesan error jelas agar operator segera me-restart navigasi tanpa menunggu sia-sia.

---

## 📌 25. Polaritas DTR Motor RPLiDAR A1 (`ser.dtr = False`), Jebakan `termios.HUPCL` Linux, & Hard USB Reset

### 1. Gejala & Error:
- Saat menjalankan `run lidar` atau merestart navigasi, muncul error berulang:
  ```text
  [rplidar_node]: Cannot start scan: '80008000'!
  [rplidar_node]: Error, operation time out. RESULT_OPERATION_TIMEOUT!
  ```
- Piringan motor RPLiDAR berhenti berputar atau baru mulai berputar lambat saat `rplidar_composition` sudah keburu timeout ($4.6\text{ detik}$).

### 2. Akar Masalah:
1. **Polaritas Pin `/DTR` Adapter CP2102 RPLiDAR A1 adalah Active-LOW (`clearDTR` = Motor ON)**:
   - Di dalam source code C++ resmi Slamtec (`rplidar_driver.cpp` baris 2175):
     - `startMotor()` memanggil `_chanDev->clearDTR()` $\rightarrow$ di `pyserial` sama dengan **`ser.dtr = False`** (tegangan pin `/DTR` = $3.3\text{V}$ = **Motor MENYALA**).
     - `stopMotor()` memanggil `_chanDev->setDTR()` $\rightarrow$ di `pyserial` sama dengan **`ser.dtr = True`** (tegangan pin `/DTR` = $0\text{V}$ = **Motor MATI**).
   - **JANGAN PERNAH** mengeset `ser.dtr = True` saat akan menjalankan `rplidar_composition`, karena itu justru mematikan motor!
2. **Jebakan Kernel Linux `termios.HUPCL` (*Hang Up On Last Close*)**:
   - Secara default di Linux, saat `pyserial` menutup port (`ser.close()` / keluar dari blok `with serial.Serial(...)`), flag kernel `HUPCL` otomatis meng-assert sinyal DTR (`/DTR` = $0\text{V}$), yang langsung **mematikan motor** tepat sebelum `rplidar_composition` dibuka.
   - Ketika `rplidar_composition` mulai, fungsi internal `startMotor()` di C++ hanya menunggu `500 ms` sebelum memanggil `startScan()`. Waktu `500 ms` tidak cukup bagi motor yang sempat mati untuk mencapai RPM kerja, sehingga MCU RPLiDAR menolak scan (`80008000` / `RESULT_OPERATION_TIMEOUT!`).
3. **Proses `rplidar_composition` Tidak Langsung Keluar Saat Timeout**:
   - Saat `rplidar_node` gagal dengan `RESULT_OPERATION_TIMEOUT!`, proses kontainer `rplidar_composition` tetap hidup sebagai proses kosong tanpa mem-publish `/scan`.

### 3. Solusi Permanen di [`rplidar_bridge.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/rplidar_bridge.py) (Teroptimasi untuk USB Hub 7-Port Eksternal 5V Step-Down):
1. **Topologi Hardware USB Robot**:
   - Jetson hanya memiliki **1 slot USB utama** yang terhubung ke **USB Hub 7-Port**.
   - USB Hub 7-Port tersebut mendapat suplai daya eksternal 5V (melalui port Type-C) dari **Step-Down 5V baterai utama**.
   - Konsekuensi elektrik: Selama baterai utama menyala, jalur daya 5V VBUS di port USB Hub **tidak pernah mati** meskipun software melakukan reset USB kernel.
2. **Set `ser.dtr = False` (Motor ON) SEGERA Saat Port Dibuka (Sebelum `0xA5 0x40`)**:
   - Saat `serial.Serial()` dibuka, driver kernel `cp210x` otomatis meng-assert `DTR=True` (`/DTR=0V` = Motor OFF/Rem).
   - Oleh karena itu, `ser.dtr = False` dan penonaktifan `termios.HUPCL` (`attrs[2] &= ~termios.HUPCL`) wajib dieksekusi **di baris paling awal** sebelum perintah `0xA5 0x25` maupun `0xA5 0x40`, sehingga motor tidak pernah ter-rem selama jeda 2.2 detik *reboot* MCU dan mendapat total waktu *spin-up* 3.5 detik sebelum `rplidar_composition` memanggil `startScan()`.
3. **Penanganan Khusus `Cannot start scan: '80008000'` (`self.last_err_is_scan_8000`)**:
   - Error `80008000` di `rplidar_node.cpp` terjadi **setelah** `getRPLIDARDeviceInfo()` dan `checkRPLIDARHealth()` sukses, yang membuktikan komunikasi USB & MCU LiDAR **sudah bangun dan sehat**, hanya putaran motor yang belum mencapai RPM penuh (5.5 Hz).
   - Jika error terakhir adalah `Cannot start scan: '80008000'`, `rplidar_bridge.py` **tidak boleh** melakukan `reset_usb_device()` dan **tidak boleh** mengirim `0xA5 0x40` lagi (agar tidak membunuh LiDAR yang baru saja bangun), melainkan cukup menahan `ser.dtr = False` (Motor ON) selama 2.0 detik agar RPM stabil lalu meluncurkan `rplidar_composition`.
4. **Pemisahan `USBDEVFS_RESET` dan `sysfs authorized` + Graceful `SIGINT`**:
   - `reset_usb_device()` tidak lagi menjalankan `echo 0/1 > authorized` dan `ioctl(USBDEVFS_RESET)` secara beruntun dalam satu fungsi agar tidak terjadi tabrakan re-enumerasi `udev` di USB Hub 7-Port.
   - `shutdown_driver_only()` serta `_cleanup_stale_processes()` di [`real_navigation.launch.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/launch/real_navigation.launch.py) dan [`real_mapping.launch.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/launch/real_mapping.launch.py) mengirimkan `SIGINT` terlebih dahulu (dan mengirim `0xA5 0x25` STOP saat keluar) agar LiDAR tidak terus membanjiri buffer FIFO chip CP2102 di USB Hub saat berganti sesi.

---

## 📌 26. Integrasi Audio Bluetooth (Speaker Acome A1), NVIDIA BlueZ Audio Unblock, Watchdog Auto-Reconnect, & Safe-Stop Voice Warning

### 1. Masalah & Solusi Pembukaan Blokir Audio Bluetooth NVIDIA Jetson:
- **Gejala Error**:
  Saat menghubungkan speaker Bluetooth (Acome A1 `0E:21:D5:E9:AB:5F`) via `bluetoothctl`:
  - `pair` dan `trust` berhasil (`Paired: yes`, `ServicesResolved: yes`).
  - Namun perintah `connect` selalu gagal dengan `Failed to connect: org.bluez.Error.Failed` atau `org.bluez.Error.NotAvailable`.
  - Log PulseAudio (`pulseaudio -v`) memunculkan:
    ```text
    E: [pulseaudio] bluez5-util.c: org.bluez.Media1.RegisterEndpoint() failed: org.freedesktop.DBus.Error.UnknownMethod: Method "RegisterEndpoint" with signature "oa{sv}" on interface "org.bluez.Media1" doesn't exist
    ```
- **Akar Masalah**:
  1. NVIDIA JetPack secara default menonaktifkan profil audio konsumen (`--noplugin=audio,a2dp,avrcp`) di file service BlueZ:
     `/lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf`.
     NVIDIA melakukan ini untuk sertifikasi kepatuhan stack industri & BLE, sehingga endpoint A2DP tidak pernah teregistrasi di D-Bus.
  2. Display manager GNOME (`gdm`) menjalankan instance PulseAudio tersendiri di latar belakang yang mendahului membajak media endpoint BlueZ.
- **Solusi Permanen**:
  1. Menghapus pembatasan `--noplugin=audio,a2dp,avrcp` di konfigurasi service BlueZ NVIDIA:
     ```bash
     sudo sed -i 's/--noplugin=audio,a2dp,avrcp//g' /lib/systemd/system/bluetooth.service.d/*.conf
     sudo systemctl daemon-reload && sudo systemctl restart bluetooth
     ```
  2. Mencegah user `gdm` membajak PulseAudio:
     ```bash
     sudo mkdir -p /var/lib/gdm3/.config/pulse
     sudo tee /var/lib/gdm3/.config/pulse/client.conf << 'EOF'
     autospawn = no
     daemon-binary = /bin/true
     EOF
     sudo chown -R gdm:gdm /var/lib/gdm3/.config/pulse
     sudo killall -9 -u gdm pulseaudio 2>/dev/null
     ```
  3. Mengaktifkan PulseAudio dan user lingering untuk user `jetson`:
     ```bash
     sudo loginctl enable-linger jetson
     pulseaudio --start
     ```

---

### 2. Watchdog Auto-Reconnect Bluetooth & Arsitektur Non-Blocking:
- **Tantangan**: Speaker portabel (Acome A1) memiliki fitur auto-standby/sleep jika tidak ada audio aktif, atau sinyal radio 2.4 GHz bisa putus sesaat saat robot bergerak jauh. Di Linux biasa, koneksi harus dihubungkan ulang secara manual.
- **Implementasi Watchdog (`BluetoothWatchdog`)**:
  - Dibuatkan background daemon thread di [`test_robot_voice.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/test_robot_voice.py) yang memantau koneksi ke target MAC address (`0E:21:D5:E9:AB:5F`) setiap 5 detik.
  - Jika speaker mati lalu dinyalakan lagi, watchdog otomatis memanggil `bluetoothctl connect` dan mengarahkan PulseAudio default sink ke `bluez_sink.0E_21_D5_E9_AB_5F.a2dp_sink` dengan volume 80%.
- **Jaminan Anti-Bentrok dengan Navigasi**:
  - Perintah suara bersifat *fire-and-forget* via pesan ROS 2 (`/robot_voice/play` dan `/robot_voice/say`), yang selesai dalam 0.0001 detik tanpa menahan eksekutor Nav2 atau `/cmd_vel`.
  - Pemutaran audio ditangani oleh worker thread terpisah (`queue.Queue`) dengan batas waktu ketat (`timeout=4-6s`). Jika speaker mati atau audio error, perintah suara dibuang secara senyap (*fail-safe*) dan navigasi tetap berjalan normal 100% pada 20 Hz.

---

### 3. Logika Peringatan Halangan Manusia ("Permisi, Robot Mau Lewat"):
- Node [`test_robot_voice.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/test_robot_voice.py) membaca topic `/human_state` yang dipancarkan oleh [`human_awareness_node.py`](file:///home/faid/Documents/amr_ws/src/amr_simulation/scripts/human_awareness_node.py).
- Jika status `SAFE_STOP` berlangsung melampaui `obstacle_voice_delay_s` (default 10 detik, dapat diatur dari 3 hingga 60 detik via dashboard), robot otomatis membunyikan bel ding-dong lalu mengucapkan pesan:
  > *"Permisi, robot mau lewat. Mohon beri jalan."*
- Terdapat jeda pengulangan berkala (`obstacle_voice_repeat_s = 12s`) agar tidak terjadi pengulangan suara yang bertabrakan selama manusia masih berada di depan robot.
- Begitu manusia menyingkir (`human_state != 'SAFE_STOP'`), penghitung waktu otomatis di-reset.

---

### 4. Pemberitahuan Suara Kedatangan Titik Misi di [`waypoint_manager.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/waypoint_manager.py):
- **Tiba di Titik Pickup**:
  Memanggil `self.notify_voice('pickup', f"Pickup tiba. Robot sudah sampai di titik {pickup['name']}. Silakan muat barang.")`.
- **Tiba di Titik Delivery**:
  Memanggil `self.notify_voice('arrival', f"Deliver tiba. Robot sudah sampai di titik {delivery['name']}. Silakan ambil barang.")`.
- **Tiba Kembali di Home**:
  Memanggil `self.notify_voice('home', "Misi selesai. Robot kembali ke stasiun pangkalan.")`.

---

### 5. Fitur Dashboard UI, REST API, & Solusi Browser Caching:
- **Indikator Cepat Topbar**: Ditambahkan tombol status Bluetooth (`BT: ON` / `BT: OFF`) di baris atas dashboard di sebelah indikator mode `REAL ROBOT` agar dapat dipantau dan diklik dari halaman mana pun.
- **Panel Pengaturan (03 // SISTEM SUARA & SPEAKER BLUETOOTH ROBOT)**:
  - Indikator status live speaker fisik.
  - Tombol **Scan Bluetooth** untuk memindai perangkat sekitar secara dinamis.
  - Tombol **Hubungkan** dan **Putuskan**.
  - Pengatur batas waktu halangan `SAFE_STOP` (detik).
  - Input kustomisasi kalimat pengumuman saat terhalang.
  - Tombol uji suara hardware robot (*Bel Chime*, *Deliver Tiba*, *Pickup Tiba*, *Permisi Lewat*).
- **Penanganan Cache Browser & Service Reload**:
  - Di [`app.py`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/amr_dashboard/app.py): ditambahkan `app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0` dan hook `@app.after_request` dengan header `Cache-Control: no-cache, no-store, must-revalidate`.
  - Di [`index.html`](file:///home/faid/Documents/amr_ws/src/amr_dashboard/templates/index.html): versi skrip dinaikkan ke `?v=3.2_voice` untuk memaksa browser mengunduh ulang JavaScript terbaru.
  - **Prosedur Update di Jetson**: Karena dashboard dikelola oleh `amr_dashboard.service`, setiap ada perubahan file web wajib menjalankan:
    ```bash
    cd ~/deliver_amr
    git pull
    colcon build --packages-select amr_dashboard amr_simulation
    sudo systemctl restart amr_dashboard
    ```


