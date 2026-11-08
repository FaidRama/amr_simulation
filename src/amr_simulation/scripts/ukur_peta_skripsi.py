#!/usr/bin/env python3
"""
=============================================================================
TOOL EVALUASI AKURASI GEOMETRIS PEMETAAN 2D (SKRIPSI BAB 4 - SKENARIO 1)
Validasi Presisi Geometris Rekonstruksi SLAM Toolbox vs Ground Truth Meteran Baja
=============================================================================
Fitur Unggulan:
  1. Highlight Grid/Piksel Terlewati (Bresenham 2D Ray-Tracing) sehingga pengguna
     dapat melihat visual kotak-kotak piksel yang dihitung oleh resolusi peta.
  2. Scroll-to-Zoom interaktif untuk ketelitian seleksi titik batas dinding/pilar.
  3. Dukungan multi-run (Run 1 s.d. 5) dengan auto-caching Ground Truth (tidak perlu
     mengetik ulang nama dan dimensi nyata 10 landmark di setiap run).
  4. Perhitungan metrik standar skripsi: RMSE, Galat Skala (Scale Error), dan Galat Absolut.
  5. Pengujian Kriteria Kelulusan: RMSE <= 0.05 m (5 cm) dan Galat Skala <= 2.0%.
  6. Ekspor otomatis ke CSV (Word/Excel ready) dan citra beranotasi PNG (300 DPI).
  7. Modul Rekapitulasi Gabungan 5 Run untuk tabel akhir Bab 4 Skripsi.
=============================================================================
"""

import os
import sys
import json
import math
import time
import glob
import argparse
from datetime import datetime
import yaml
import numpy as np
from PIL import Image

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection


def bresenham_cells(x0, y0, x1, y1):
    """
    Algoritma Bresenham 2D (Ray-Tracing Grid Occupancy)
    Mengembalikan seluruh indeks sel piksel diskrit (cx, cy) yang dilintasi garis.
    Identik dengan algoritma raytracing standar pada ROS 2 nav2_costmap_2d.
    """
    x0, y0 = int(round(x0)), int(round(y0))
    x1, y1 = int(round(x1)), int(round(y1))

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    cells = []
    cx, cy = x0, y0

    while True:
        cells.append((cx, cy))
        if cx == x1 and cy == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy

    return cells


class MapGeometricEvaluator:
    def __init__(self, yaml_path: str, run_number: int = 1):
        self.yaml_path = os.path.abspath(yaml_path)
        self.map_dir = os.path.dirname(self.yaml_path)
        self.run_number = run_number
        self.load_map_metadata()

        # Data pengujian landmark
        self.landmarks = []
        self.current_points = []
        self.temp_artists = []
        self.target_landmarks_count = 10

        # Direktori Output & Cache
        self.output_dir = os.path.join(os.path.expanduser('~'), 'benchmark_skripsi_results', 'evaluasi_peta')
        os.makedirs(self.output_dir, exist_ok=True)
        self.cache_file = os.path.join(self.output_dir, 'ground_truth_landmarks_template.json')
        self.gt_template = self.load_ground_truth_template()

        print("\n" + "=" * 80)
        print(f"  📏 SISTEM PENGUKURAN AKURASI GEOMETRIS PETA SLAM 2D (SKENARIO 1)")
        print(f"     Sesi Pengujian: RUN KE-{self.run_number} DARI 5")
        print("=" * 80)
        print(f"  • File Metadata  : {os.path.basename(self.yaml_path)}")
        print(f"  • File Citra Peta: {os.path.basename(self.image_path)}")
        print(f"  • Resolusi Grid  : {self.resolution:.4f} meter / piksel (1 px = {self.resolution*100:.1f} cm)")
        print(f"  • Dimensi Matriks: {self.img_w} x {self.img_h} piksel ({self.img_w*self.resolution:.2f}m x {self.img_h*self.resolution:.2f}m)")
        print(f"  • Ambang Batas   : RMSE <= 0.05 meter (5 cm) | Galat Skala <= 2.0%")
        print(f"  • Folder Hasil   : {self.output_dir}")
        print("-" * 80)
        print("  PANDUAN INTERAKSI GRAFIS:")
        print("  • SCROLL RODA MOUSE : Zoom In / Zoom Out langsung di posisi kursor.")
        print("  • KLIK KIRI 2 KALI  : Klik Titik Awal (Titik A) lalu Titik Akhir (Titik B).")
        print("  • HIGHLIGHT KOTAK   : Kotak-kotak piksel cyan akan otomatis menyala di peta")
        print("                        menandakan sel-sel grid yang dilintasi (Bresenham).")
        print("  • TEKAN 'U'         : Undo (Membatalkan pengukuran terakhir jika salah klik).")
        print("  • TEKAN 'S'         : Selesai & Simpan laporan pengujian (CSV & PNG).")
        print("  • TEKAN 'R'         : Reset Zoom ke tampilan peta penuh.")
        print("=" * 80 + "\n")

    def load_map_metadata(self):
        """Membaca parameter resolusi dan path file gambar dari .yaml."""
        if not os.path.exists(self.yaml_path):
            raise FileNotFoundError(f"File yaml peta tidak ditemukan: {self.yaml_path}")

        with open(self.yaml_path, 'r') as f:
            self.metadata = yaml.safe_load(f) or {}

        self.resolution = float(self.metadata.get('resolution', 0.05))
        self.origin = self.metadata.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = float(self.origin[0])
        self.origin_y = float(self.origin[1])

        img_filename = self.metadata.get('image', '')
        self.image_path = os.path.join(self.map_dir, img_filename)
        if not os.path.exists(self.image_path):
            self.image_path = self.yaml_path.replace('.yaml', '.pgm')

        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Gambar peta .pgm tidak ditemukan: {self.image_path}")

        # Load citra peta
        pil_img = Image.open(self.image_path)
        self.map_img = np.array(pil_img)
        self.img_h, self.img_w = self.map_img.shape[:2]

    def load_ground_truth_template(self):
        """Memuat preset daftar nama & ukuran nyata landmark jika sudah pernah diinput."""
        if os.path.exists(self.cache_file):
            try:
                with open(self.cache_file, 'r') as f:
                    data = json.load(f)
                    if isinstance(data, list) and len(data) > 0:
                        print(f"[💡 PRESET TERDETEKSI] Ditemukan template {len(data)} landmark acuan dari sesi sebelumnya.")
                        pilih = input(">> Gunakan template nama & dimensi nyata tersebut sebagai default? [Y/n]: ").strip().lower()
                        if pilih in ['', 'y', 'ya', 'yes']:
                            print("   ✅ Template dimuat! Anda cukup tekan [ENTER] saat konfirmasi nama/dimensi nyata.\n")
                            return data
            except Exception as e:
                print(f"[!] Gagal membaca cache template landmark: {e}")
        return []

    def save_ground_truth_template(self):
        """Menyimpan template daftar 10 landmark acuan untuk mempercepat run berikutnya."""
        template_data = []
        for lm in self.landmarks:
            template_data.append({
                'id': lm['id'],
                'name': lm['name'],
                'dist_nyata_m': lm['dist_nyata_m']
            })
        try:
            with open(self.cache_file, 'w') as f:
                json.dump(template_data, f, indent=2)
        except Exception as e:
            print(f"[!] Gagal menyimpan cache template: {e}")

    def pixel_to_map_coords(self, px, py):
        """Konversi koordinat piksel citra ke koordinat ROS 2 map frame (meter)."""
        x_m = self.origin_x + px * self.resolution
        y_m = self.origin_y + (self.img_h - 1 - py) * self.resolution
        return x_m, y_m

    def run(self):
        """Menjalankan interface interaktif Matplotlib."""
        self.fig, self.ax = plt.subplots(figsize=(12, 8.5))
        self.fig.canvas.manager.set_window_title(
            f"Evaluasi Geometris SLAM 2D - Run #{self.run_number} | Skripsi Teknik Komputer"
        )

        # Tampilkan citra peta
        self.ax.imshow(self.map_img, cmap='gray', origin='upper')
        self.ax.set_title(
            f"EVALUASI AKURASI GEOMETRIS PEMETAAN 2D (RUN #{self.run_number}/5)\n"
            f"Peta: {os.path.basename(self.image_path)} | Resolusi: {self.resolution:.4f} m/px (1 px = {self.resolution*100:.1f} cm)\n"
            "KLIK KIRI: Titik A lalu Titik B | Scroll: Zoom | Tekan 'u': Undo | Tekan 's': Simpan | Tekan 'r': Reset Zoom",
            fontsize=10.5, fontweight='bold', pad=12
        )
        self.ax.set_xlabel("Piksel X (Grid Index)", fontsize=9.5)
        self.ax.set_ylabel("Piksel Y (Grid Index)", fontsize=9.5)

        # Format status bar Matplotlib
        self.ax.format_coord = lambda px, py: (
            f"Piksel: ({px:.1f}, {py:.1f}) | "
            f"ROS Frame: X={self.origin_x + px * self.resolution:+.3f}m, "
            f"Y={self.origin_y + (self.img_h - 1 - py) * self.resolution:+.3f}m"
        )

        # Simpan batas awal untuk reset zoom ('r')
        self.default_xlim = self.ax.get_xlim()
        self.default_ylim = self.ax.get_ylim()

        # Connect events
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_scroll = self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        plt.tight_layout()
        plt.show()

    def on_scroll(self, event):
        """Fitur interaktif Zoom-In / Zoom-Out mengikuti posisi kursor mouse."""
        if event.inaxes != self.ax:
            return

        scale_factor = 0.75 if event.button == 'up' else 1.33
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        xdata = event.xdata
        ydata = event.ydata

        if xdata is None or ydata is None:
            return

        cur_width = cur_xlim[1] - cur_xlim[0]
        cur_height = cur_ylim[1] - cur_ylim[0]
        new_width = cur_width * scale_factor
        new_height = cur_height * scale_factor

        relx = (cur_xlim[1] - xdata) / cur_width
        rely = (cur_ylim[1] - ydata) / cur_height

        self.ax.set_xlim([xdata - new_width * (1.0 - relx), xdata + new_width * relx])
        self.ax.set_ylim([ydata - new_height * (1.0 - rely), ydata + new_height * rely])
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        """Event klik mouse pada canvas gambar peta."""
        if event.inaxes != self.ax:
            return
        if self.fig.canvas.manager.toolbar.mode != "":
            return

        x, y = event.xdata, event.ydata
        self.current_points.append((x, y))

        # Titik 1 terpilih
        if len(self.current_points) == 1:
            ix, iy = int(round(x)), int(round(y))
            # Highlight sel piksel titik awal (kotak oranye)
            pt_box = Rectangle((ix - 0.5, iy - 0.5), 1.0, 1.0,
                               facecolor='#ff9800', edgecolor='#e65100', alpha=0.7, linewidth=1.2, zorder=5)
            pt_dot = self.ax.plot(x, y, 'o', color='#ff3d00', markersize=5, zorder=6)[0]
            self.ax.add_patch(pt_box)
            self.temp_artists.extend([pt_box, pt_dot])
            self.fig.canvas.draw_idle()

            x_m, y_m = self.pixel_to_map_coords(x, y)
            print(f"\n[📍 TITIK 1 DIPILIH]: Piksel ({x:.1f}, {y:.1f}) | ROS Frame: (X={x_m:+.3f}m, Y={y_m:+.3f}m)")
            print("👉 Sekarang silakan KLIK TITIK 2 (ujung segmen landmark)...")

        # Titik 2 terpilih -> Hitung Jarak, Bresenham & Highlight Grid
        elif len(self.current_points) == 2:
            pt1 = self.current_points[0]
            pt2 = self.current_points[1]

            # Jarak Euclidean piksel & meter
            dx = pt2[0] - pt1[0]
            dy = pt2[1] - pt1[1]
            dist_px = math.hypot(dx, dy)
            dist_m = dist_px * self.resolution

            # Algoritma Bresenham 2D Ray-Tracing untuk mendapatkan seluruh kotak piksel yang dilintasi
            cells = bresenham_cells(pt1[0], pt1[1], pt2[0], pt2[1])

            # Buat patch kotak-kotak piksel cyan transparan
            patches = [
                Rectangle((cx - 0.5, cy - 0.5), 1.0, 1.0)
                for cx, cy in cells
            ]
            pc = PatchCollection(
                patches,
                facecolor='#00e5ff',
                edgecolor='#0288d1',
                alpha=0.45,
                linewidth=0.75,
                zorder=3
            )
            self.ax.add_collection(pc)

            # Highlight sel titik akhir (kotak hijau terang)
            ix2, iy2 = int(round(pt2[0])), int(round(pt2[1]))
            end_box = Rectangle((ix2 - 0.5, iy2 - 0.5), 1.0, 1.0,
                                facecolor='#76ff03', edgecolor='#2e7d32', alpha=0.7, linewidth=1.2, zorder=5)
            end_dot = self.ax.plot(pt2[0], pt2[1], 'o', color='#00e676', markersize=5, zorder=6)[0]
            self.ax.add_patch(end_box)

            # Garis ukur euklidean
            line = self.ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], color='#ffff00', linewidth=2.0, zorder=7)[0]

            idx = len(self.landmarks) + 1
            mid_x = (pt1[0] + pt2[0]) / 2.0
            mid_y = (pt1[1] + pt2[1]) / 2.0
            txt = self.ax.text(
                mid_x, mid_y, f"#{idx} ({len(cells)} sel | {dist_m:.3f}m)",
                color='yellow', fontsize=8.5, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.25", fc="black", ec="#00e5ff", alpha=0.85),
                zorder=8
            )

            self.temp_artists.extend([pc, end_box, end_dot, line, txt])
            self.fig.canvas.draw_idle()

            x1_m, y1_m = self.pixel_to_map_coords(pt1[0], pt1[1])
            x2_m, y2_m = self.pixel_to_map_coords(pt2[0], pt2[1])

            print("-" * 80)
            print(f"[🎯 LANDMARK #{idx} TERPILIH]:")
            print(f"  • Koordinat Piksel    : ({pt1[0]:.1f}, {pt1[1]:.1f}) -> ({pt2[0]:.1f}, {pt2[1]:.1f})")
            print(f"  • Koordinat ROS Frame : (X={x1_m:+.3f}m, Y={y1_m:+.3f}m) -> (X={x2_m:+.3f}m, Y={y2_m:+.3f}m)")
            print(f"  • Kotak Piksel Terlewati : {len(cells)} sel grid (Algoritma Bresenham 2D)")
            print(f"  • Jarak Euclidean Piksel : {dist_px:.2f} piksel")
            print(f"  • Dimensi Terukur Peta   : {dist_px:.2f} px  x  {self.resolution:.4f} m/px = {dist_m:.4f} meter ({dist_m*100:.2f} cm)")
            print("-" * 80)

            # Auto-suggest nama dan nilai nyata dari template jika ada
            default_name = f"Landmark_{idx}"
            default_nyata = None
            if self.gt_template and idx <= len(self.gt_template):
                template_item = self.gt_template[idx - 1]
                default_name = template_item.get('name', default_name)
                default_nyata = template_item.get('dist_nyata_m', None)
            else:
                standard_names = [
                    "Panjang Dinding Utama Barat",
                    "Lebar Koridor Utama Depan",
                    "Jarak Antar-Pilar 1 - 2",
                    "Jarak Antar-Pilar 2 - 3",
                    "Lebar Koridor Arah Lift",
                    "Panjang Partisi Meja Boks",
                    "Lebar Partisi Meja Boks",
                    "Lebar Pintu Akses Ruangan",
                    "Panjang Dinding Lorong Timur",
                    "Dimensi Diagonal Ruang Terbuka"
                ]
                if idx <= len(standard_names):
                    default_name = standard_names[idx - 1]

            nama_input = input(f">> Nama Landmark #{idx} [Default: '{default_name}']: ").strip()
            landmark_name = nama_input if nama_input else default_name

            d_nyata = None
            prompt_nyata = f">> Masukkan Dimensi Fisik Nyata / Meteran Baja [meter]"
            if default_nyata is not None:
                prompt_nyata += f" [Default: {default_nyata:.3f}m]: "
            else:
                prompt_nyata += f" (Peta: {dist_m:.3f}m): "

            while d_nyata is None:
                nyata_str = input(prompt_nyata).strip()
                if not nyata_str and default_nyata is not None:
                    d_nyata = float(default_nyata)
                    break
                try:
                    d_nyata = float(nyata_str)
                    if d_nyata <= 0:
                        print("[!] Nilai dimensi fisik harus lebih besar dari 0.")
                        d_nyata = None
                except ValueError:
                    print("[!] Input angka desimal tidak valid. Silakan coba lagi.")

            # Kalkulasi Galat Absolut & Galat Skala (%)
            error_abs = abs(dist_m - d_nyata)
            scale_error_pct = (error_abs / d_nyata) * 100.0

            # Simpan data landmark
            data_entry = {
                'id': idx,
                'name': landmark_name,
                'pt1': pt1,
                'pt2': pt2,
                'cell_count': len(cells),
                'dist_px': dist_px,
                'dist_peta_m': dist_m,
                'dist_nyata_m': d_nyata,
                'error_abs_m': error_abs,
                'scale_error_pct': scale_error_pct,
                'artists': list(self.temp_artists)
            }
            self.landmarks.append(data_entry)
            self.current_points = []
            self.temp_artists = []

            print(f"[✅ DISIMPAN]: {landmark_name} | {len(cells)} sel grid")
            print(f"   Peta = {dist_m:.3f}m | Ground Truth = {d_nyata:.3f}m | Galat = {error_abs*100:.2f} cm ({scale_error_pct:.2f}%)")
            print(f"[*] Progres Landmark: {len(self.landmarks)}/{self.target_landmarks_count}")
            if len(self.landmarks) >= self.target_landmarks_count:
                print("\n[🎉 TARGET 10 LANDMARK TERCAPAI!] Tekan tombol 'S' pada keyboard untuk menyelesaikan & menyimpan laporan.\n")

    def on_key(self, event):
        """Shortcut keyboard untuk navigasi cepat."""
        if event.key in ['u', 'U']:
            self.undo_last_landmark()
        elif event.key in ['s', 'S']:
            self.finish_and_export()
        elif event.key in ['r', 'R']:
            self.reset_zoom()

    def reset_zoom(self):
        """Mengembalikan level zoom ke ukuran peta awal."""
        self.ax.set_xlim(self.default_xlim)
        self.ax.set_ylim(self.default_ylim)
        self.fig.canvas.draw_idle()
        print("[🔍 ZOOM RESET] Tampilan peta dikembalikan ke ukuran penuh.")

    def undo_last_landmark(self):
        """Membatalkan titik klik pertama atau menghapus landmark terakhir."""
        if self.current_points:
            self.current_points = []
            for a in self.temp_artists:
                try:
                    a.remove()
                except Exception:
                    pass
            self.temp_artists = []
            self.fig.canvas.draw_idle()
            print("[↩ UNDO] Titik pertama dibatalkan. Silakan klik ulang.")
            return

        if self.landmarks:
            removed = self.landmarks.pop()
            for a in removed['artists']:
                try:
                    a.remove()
                except Exception:
                    pass
            self.fig.canvas.draw_idle()
            print(f"[↩ UNDO] Landmark #{removed['id']} ({removed['name']}) berhasil dihapus.")

    def finish_and_export(self):
        """Menghitung metrik RMSE, Galat Skala, validasi kelulusan skripsi, dan ekspor CSV/PNG."""
        if not self.landmarks:
            print("[!] Belum ada landmark yang diukur.")
            return

        n = len(self.landmarks)
        # Kalkulasi Root Mean Square Error (RMSE)
        sum_sq_err = sum(item['error_abs_m'] ** 2 for item in self.landmarks)
        rmse = math.sqrt(sum_sq_err / n)

        # Rata-rata Galat Skala (%) & Galat Absolut (cm)
        mean_scale_err = sum(item['scale_error_pct'] for item in self.landmarks) / n
        mean_abs_err_cm = (sum(item['error_abs_m'] for item in self.landmarks) / n) * 100.0

        # Kriteria Kelulusan Skenario 1 Proposal Skripsi:
        # RMSE <= 0.05 meter (5 cm) dan Galat Skala <= 2.0%
        status_rmse = "MEMENUHI (<= 0.05 m)" if rmse <= 0.05 else "TIDAK MEMENUHI (> 0.05 m)"
        status_scale = "MEMENUHI (<= 2.0%)" if mean_scale_err <= 2.0 else "TIDAK MEMENUHI (> 2.0%)"
        status_final = "LULUS / AKURAT & LAYAK" if (rmse <= 0.05 and mean_scale_err <= 2.0) else "EVALUASI ULANG"

        # Simpan Ground Truth Template untuk run berikutnya
        self.save_ground_truth_template()

        # Rekap Terminal
        print("\n" + "=" * 100)
        print(f"  📊 REKAPITULASI EVALUASI AKURASI GEOMETRIS PETA 2D | RUN #{self.run_number}")
        print("=" * 100)
        print(f"{'No':<3} | {'Landmark Acuan':<26} | {'Sel Grid':<8} | {'Nyata (m)':<10} | {'Peta (m)':<10} | {'Galat (cm)':<11} | {'Galat Skala':<11}")
        print("-" * 100)
        for row in self.landmarks:
            print(
                f"#{row['id']:<2} | {row['name']:<26} | {row['cell_count']:>4} sel | "
                f"{row['dist_nyata_m']:>8.3f} m | {row['dist_peta_m']:>8.3f} m | "
                f"{row['error_abs_m']*100:>9.2f} cm | {row['scale_error_pct']:>9.2f} %"
            )
        print("-" * 100)
        print(f"• Rata-rata Galat Absolut       : {mean_abs_err_cm:.2f} cm ({mean_abs_err_cm/100:.4f} m)")
        print(f"• Root Mean Square Error (RMSE) : {rmse:.4f} meter ({rmse*100:.2f} cm) -> {status_rmse}")
        print(f"• Rata-rata Galat Skala (%)     : {mean_scale_err:.2f}% -> {status_scale}")
        print(f"• KESIMPULAN KELULUSAN SKRIPSI  : [ {status_final} ]")
        print("=" * 100)

        # File Export
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_map_name = os.path.basename(self.yaml_path).replace('.yaml', '')
        csv_filename = os.path.join(
            self.output_dir, f"eval_peta_run_{self.run_number}_{base_map_name}_{timestamp_str}.csv"
        )
        img_export_name = os.path.join(
            self.output_dir, f"anotasi_peta_run_{self.run_number}_{base_map_name}_{timestamp_str}.png"
        )

        with open(csv_filename, 'w') as f:
            f.write("# =========================================================================\n")
            f.write(f"# EVALUASI AKURASI GEOMETRIS PEMETAAN 2D SLAM - RUN #{self.run_number}\n")
            f.write(f"# File Peta        : {self.image_path}\n")
            f.write(f"# Resolusi Grid    : {self.resolution:.4f} m/pixel\n")
            f.write(f"# Waktu Pengujian  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# RMSE             : {rmse:.4f} m ({status_rmse})\n")
            f.write(f"# Mean Scale Error : {mean_scale_err:.2f}% ({status_scale})\n")
            f.write(f"# Status Kelulusan : {status_final}\n")
            f.write("# =========================================================================\n")
            f.write("No,Nama_Landmark,Kotak_Piksel_Bresenham,Dimensi_Nyata_m,Dimensi_Peta_m,Selisih_Absolut_cm,Galat_Skala_Persen\n")
            for r in self.landmarks:
                f.write(
                    f"{r['id']},{r['name']},{r['cell_count']},{r['dist_nyata_m']:.3f},"
                    f"{r['dist_peta_m']:.3f},{r['error_abs_m']*100:.2f},{r['scale_error_pct']:.2f}\n"
                )
            f.write(f"RATA_RATA,-,-,-,-,{mean_abs_err_cm:.2f},{mean_scale_err:.2f}\n")
            f.write(f"RMSE_METER,-,-,-,{rmse:.4f},-,-\n")

        # Simpan gambar peta beranotasi resolusi tinggi
        self.fig.savefig(img_export_name, dpi=300, bbox_inches='tight')

        print(f"\n[💾 FILE BERHASIL DISIMPAN]:")
        print(f"  1. Tabel CSV (Word/Excel Ready) : {csv_filename}")
        print(f"  2. Gambar Peta Beranotasi (PNG) : {img_export_name}")
        print(f"\n[🎉 RUN #{self.run_number} SELESAI] Data tersimpan rapi untuk Bab 4 Skripsi!")
        plt.close(self.fig)


def generate_5_runs_recap(output_dir: str):
    """
    Membaca seluruh file CSV eval_peta_run_*.csv dan membuat tabel rekapitulasi gabungan 5 run.
    """
    csv_pattern = os.path.join(output_dir, "eval_peta_run_*.csv")
    csv_files = sorted(glob.glob(csv_pattern))

    if not csv_files:
        print(f"[!] Belum ada file hasil pengujian CSV ditemukan di {output_dir}")
        return

    print("\n" + "=" * 90)
    print("  📑 REKAPITULASI STATISTIK 5 RUN PEMETAAN (SKENARIO 1 - BAB 4 SKRIPSI)")
    print("=" * 90)

    run_summaries = []
    landmark_dict = {}

    for fpath in csv_files:
        fname = os.path.basename(fpath)
        try:
            with open(fpath, 'r') as f:
                lines = [line.strip() for line in f if line.strip()]

            # Ambil metadata header
            run_idx = "?"
            rmse_val = 0.0
            scale_val = 0.0
            for l in lines:
                if "RUN #" in l:
                    run_idx = l.split("RUN #")[1].split()[0]
                if "# RMSE" in l:
                    rmse_val = float(l.split(":")[1].split("m")[0].strip())
                if "# Mean Scale Error" in l:
                    scale_val = float(l.split(":")[1].split("%")[0].strip())

            # Ambil baris data
            data_started = False
            for l in lines:
                if l.startswith("No,Nama_Landmark"):
                    data_started = True
                    continue
                if data_started:
                    if l.startswith("RATA_RATA") or l.startswith("RMSE_METER"):
                        continue
                    parts = l.split(",")
                    if len(parts) >= 7:
                        no = int(parts[0])
                        name = parts[1]
                        cells = int(parts[2])
                        d_nyata = float(parts[3])
                        d_peta = float(parts[4])
                        err_cm = float(parts[5])
                        scale_pct = float(parts[6])

                        if no not in landmark_dict:
                            landmark_dict[no] = {
                                'name': name,
                                'd_nyata': d_nyata,
                                'runs_peta': {},
                                'runs_err_cm': {},
                                'runs_scale_pct': {}
                            }
                        landmark_dict[no]['runs_peta'][run_idx] = d_peta
                        landmark_dict[no]['runs_err_cm'][run_idx] = err_cm
                        landmark_dict[no]['runs_scale_pct'][run_idx] = scale_pct

            run_summaries.append({
                'file': fname,
                'run_idx': run_idx,
                'rmse': rmse_val,
                'scale_err': scale_val,
                'status': "LULUS" if (rmse_val <= 0.05 and scale_val <= 2.0) else "EVALUASI"
            })
        except Exception as e:
            print(f"[!] Gagal memproses file {fname}: {e}")

    # Cetak Tabel Performa Run
    print(f"{'Run Ke':<8} | {'File Sumber':<42} | {'RMSE (m)':<10} | {'Galat Skala':<11} | {'Status'}")
    print("-" * 90)
    total_rmse = 0.0
    total_scale = 0.0
    for r in run_summaries:
        print(f"Run #{r['run_idx']:<4} | {r['file'][:40]:<42} | {r['rmse']:>8.4f} m | {r['scale_err']:>9.2f} % | {r['status']}")
        total_rmse += r['rmse']
        total_scale += r['scale_err']
    print("-" * 90)

    n_runs = len(run_summaries)
    grand_mean_rmse = total_rmse / n_runs if n_runs > 0 else 0
    grand_mean_scale = total_scale / n_runs if n_runs > 0 else 0
    final_lulus = "MEMENUHI SYARAT KELULUSAN SKRIPSI (LULUS)" if (grand_mean_rmse <= 0.05 and grand_mean_scale <= 2.0) else "EVALUASI ULANG"

    print(f"• GRAND MEAN RMSE (5 Run)        : {grand_mean_rmse:.4f} meter ({grand_mean_rmse*100:.2f} cm)")
    print(f"• GRAND MEAN GALAT SKALA (5 Run) : {grand_mean_scale:.2f} %")
    print(f"• STATUS AKHIR KELULUSAN         : [ {final_lulus} ]")
    print("=" * 90)

    # Cetak Tabel Per Landmark (Rata-rata 5 Run)
    print("\n📊 TABEL EVALUASI 10 LANDMARK ACUAN (RATA-RATA 5 RUN):")
    print(f"{'No':<3} | {'Nama Landmark':<26} | {'Nyata (m)':<10} | {'Peta Mean (m)':<13} | {'Galat Mean (cm)':<15} | {'Scale Err Mean'}")
    print("-" * 90)
    for no in sorted(landmark_dict.keys()):
        lm = landmark_dict[no]
        peta_vals = list(lm['runs_peta'].values())
        err_vals = list(lm['runs_err_cm'].values())
        scale_vals = list(lm['runs_scale_pct'].values())

        mean_peta = sum(peta_vals) / len(peta_vals) if peta_vals else 0
        mean_err = sum(err_vals) / len(err_vals) if err_vals else 0
        mean_scale = sum(scale_vals) / len(scale_vals) if scale_vals else 0

        print(
            f"#{no:<2} | {lm['name']:<26} | {lm['d_nyata']:>8.3f} m | "
            f"{mean_peta:>11.3f} m | {mean_err:>13.2f} cm | {mean_scale:>12.2f} %"
        )
    print("=" * 90)

    # Ekspor CSV Rekapitulasi Gabungan
    rekap_csv = os.path.join(output_dir, "rekapitulasi_gabungan_5_run_skenario1.csv")
    with open(rekap_csv, 'w') as f:
        f.write("# =========================================================================\n")
        f.write("# REKAPITULASI EVALUASI AKURASI GEOMETRIS PEMETAAN 2D 5 RUN (BAB 4 SKRIPSI)\n")
        f.write(f"# Grand Mean RMSE         : {grand_mean_rmse:.4f} meter\n")
        f.write(f"# Grand Mean Scale Error  : {grand_mean_scale:.2f} %\n")
        f.write(f"# Kesimpulan Kelulusan    : {final_lulus}\n")
        f.write("# =========================================================================\n")
        f.write("No,Nama_Landmark,Dimensi_Nyata_m,Peta_Mean_m,Galat_Absolut_Mean_cm,Galat_Skala_Mean_Persen\n")
        for no in sorted(landmark_dict.keys()):
            lm = landmark_dict[no]
            peta_vals = list(lm['runs_peta'].values())
            err_vals = list(lm['runs_err_cm'].values())
            scale_vals = list(lm['runs_scale_pct'].values())
            mean_peta = sum(peta_vals) / len(peta_vals) if peta_vals else 0
            mean_err = sum(err_vals) / len(err_vals) if err_vals else 0
            mean_scale = sum(scale_vals) / len(scale_vals) if scale_vals else 0
            f.write(f"{no},{lm['name']},{lm['d_nyata']:.3f},{mean_peta:.3f},{mean_err:.2f},{mean_scale:.2f}\n")
        f.write(f"GRAND_MEAN,-,-,-,{sum(sum(lm['runs_err_cm'].values())/len(lm['runs_err_cm'].values()) for lm in landmark_dict.values())/len(landmark_dict):.2f},{grand_mean_scale:.2f}\n")
        f.write(f"RMSE_GRAND_MEAN,-,-,-,{grand_mean_rmse:.4f},-\n")

    print(f"\n[💾 FILE REKAP TERSIMPAN]: {rekap_csv}")
    print("[💡 INFO] File ini siap diimpor ke Microsoft Word atau Google Docs untuk Bab 4 Skripsi!\n")


def main():
    parser = argparse.ArgumentParser(description="Tool Evaluasi Akurasi Geometris Peta 2D SLAM Skripsi")
    parser.add_argument('--map', type=str, default='', help='Path file peta .yaml')
    parser.add_argument('--run', type=int, default=1, help='Nomor Run Pengujian (1 s.d. 5)')
    parser.add_argument('--rekap', action='store_true', help='Tampilkan rekapitulasi 5 run pemetaan')
    args = parser.parse_args()

    output_dir = os.path.join(os.path.expanduser('~'), 'benchmark_skripsi_results', 'evaluasi_peta')

    if args.rekap:
        generate_5_runs_recap(output_dir)
        return

    # Jika argument --map tidak diberikan, tampilkan menu interaktif
    yaml_target = args.map
    run_target = args.run

    if not yaml_target:
        print("\n" + "=" * 80)
        print("  📏 TOOL EVALUASI AKURASI GEOMETRIS PEMETAAN 2D (SKENARIO 1 SKRIPSI)")
        print("=" * 80)
        print("  Pilih Menu:")
        print("  [1] Mulai Pengukuran Peta Landmark (Interaktif GUI Matplotlib)")
        print("  [2] Rekapitulasi Statistik Gabungan 5 Run Pemetaan (Tabel Bab 4)")
        print("  [3] Keluar")
        print("=" * 80)
        pilihan = input(">> Masukkan Pilihan [1-3] (Default: 1): ").strip()

        if pilihan == '2':
            generate_5_runs_recap(output_dir)
            return
        elif pilihan == '3':
            print("[*] Selesai.")
            return

        # Menu 1: Cari peta otomatis
        default_candidates = [
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_l1_dc.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_l1_ddc.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_1b.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_dc.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_kampus.yaml'),
        ]
        available_maps = [c for c in default_candidates if os.path.exists(c)]

        print("\nPilih File Peta yang akan Dievaluasi:")
        for idx, m in enumerate(available_maps, 1):
            print(f"  [{idx}] {os.path.basename(m)} ({m})")
        print(f"  [{len(available_maps)+1}] Masukkan Path File Lain Secara Manual")

        pilih_peta = input(f">> Pilih Peta [1-{len(available_maps)+1}] (Default: 1): ").strip()
        try:
            p_idx = int(pilih_peta) if pilih_peta else 1
            if 1 <= p_idx <= len(available_maps):
                yaml_target = available_maps[p_idx - 1]
            else:
                yaml_target = input(">> Masukkan path lengkap file peta .yaml: ").strip()
        except ValueError:
            yaml_target = available_maps[0] if available_maps else ""

        # Tanya nomor run
        run_input = input(f">> Masukkan Nomor Run Pengujian (1 - 5) [Default: {run_target}]: ").strip()
        if run_input:
            try:
                run_target = int(run_input)
            except ValueError:
                run_target = 1

    if not yaml_target or not os.path.exists(yaml_target):
        print(f"[❌ ERROR] File peta .yaml tidak ditemukan: {yaml_target}")
        sys.exit(1)

    try:
        app = MapGeometricEvaluator(yaml_target, run_number=run_target)
        app.run()
    except KeyboardInterrupt:
        print("\n[*] Pengukuran dibatalkan oleh pengguna.")
    except Exception as e:
        print(f"\n[❌ ERROR]: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
