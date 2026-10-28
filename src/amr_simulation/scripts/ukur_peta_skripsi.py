#!/usr/bin/env python3
"""
=============================================================================
TOOL PENGUKURAN AKURASI GEOMETRIS PETA 2D (SKRIPSI BAB 4 - SKENARIO 1)
Integrasi Evaluasi Rekonstruksi SLAM Toolbox vs Ground Truth Meteran Pita Baja
=============================================================================
Fungsi:
  1. Membuka file peta SLAM (.yaml & .pgm) secara visual interaktif.
  2. Memungkinkan pengguna zoom/pan dan mengklik 2 titik landmark di peta.
  3. Menghitung jarak peta (D_peta) otomatis berdasarkan perkalian piksel x resolusi (0.05 m/px).
  4. Menerima input dimensi fisik nyata pita baja (D_nyata).
  5. Menghitung Root Mean Square Error (RMSE) dan Persentase Galat Skala (Scale Error).
  6. Menguji Kriteria Kelulusan: RMSE <= 0.05 m (5 cm) dan Galat Skala <= 2.0%.
  7. Menyimpan tabel hasil ke CSV (siap ke Word/Excel) dan gambar peta beranotasi (.png).
=============================================================================
"""

import os
import sys
import math
import time
import argparse
from datetime import datetime
import yaml
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.widgets import Button


class MapGeometricEvaluator:
    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self.map_dir = os.path.dirname(os.path.abspath(yaml_path))
        self.load_map_metadata()

        # Data pengujian landmark
        self.landmarks = []
        self.current_points = []
        self.temp_artist = []
        self.target_landmarks_count = 10

        # Direktori Output
        self.output_dir = os.path.join(os.path.expanduser('~'), 'benchmark_skripsi_results', 'evaluasi_peta')
        os.makedirs(self.output_dir, exist_ok=True)

        print("=" * 75)
        print("  📏 SISTEM PENGUKURAN AKURASI GEOMETRIS PETA SLAM 2D (BAB 4)")
        print("=" * 75)
        print(f"  • File Peta      : {self.image_path}")
        print(f"  • Resolusi Grid  : {self.resolution:.4f} meter / piksel")
        print(f"  • Dimensi Gambar : {self.img_w} x {self.img_h} piksel")
        print(f"  • Folder Hasil   : {self.output_dir}")
        print("-" * 75)
        print("  PANDUAN INTERAKSI GRAFIS:")
        print("  1. Gunakan icon Kaca Pembesar / Pan di toolbar Matplotlib untuk zoom dinding/pilar.")
        print("  2. KLIK KIRI 2 KALI pada peta: Titik A (awal landmark) lalu Titik B (akhir landmark).")
        print("  3. Masukkan Nama Landmark dan Ukuran Nyata (Meteran) di terminal.")
        print("  4. Tekan 'U' pada keyboard untuk Undo pengukuran terakhir jika salah klik.")
        print("  5. Tekan 'S' atau tombol 'SELESAI & SIMPAN' jika seluruh landmark sudah diukur.")
        print("=" * 75)

    def load_map_metadata(self):
        """Membaca parameter resolusi dan path file gambar dari .yaml."""
        if not os.path.exists(self.yaml_path):
            raise FileNotFoundError(f"File yaml peta tidak ditemukan: {self.yaml_path}")

        with open(self.yaml_path, 'r') as f:
            self.metadata = yaml.safe_load(f) or {}

        self.resolution = float(self.metadata.get('resolution', 0.05))
        img_filename = self.metadata.get('image', '')
        self.image_path = os.path.join(self.map_dir, img_filename)
        if not os.path.exists(self.image_path):
            # Fallback jika nama file sama dengan yaml beda ekstensi .pgm
            self.image_path = self.yaml_path.replace('.yaml', '.pgm')

        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Gambar peta .pgm tidak ditemukan: {self.image_path}")

        # Load gambar peta
        pil_img = Image.open(self.image_path)
        self.map_img = np.array(pil_img)
        self.img_h, self.img_w = self.map_img.shape[:2]

    def run(self):
        """Menjalankan interface interaktif Matplotlib."""
        self.fig, self.ax = plt.subplots(figsize=(11, 8))
        self.fig.canvas.manager.set_window_title("Pengukuran Geometris Peta SLAM 2D - Skripsi Bab 4")

        # Tampilkan peta (origin='upper' sesuai orientasi citra matriks)
        self.ax.imshow(self.map_img, cmap='gray', origin='upper')
        self.ax.set_title(
            f"Peta: {os.path.basename(self.image_path)} | Resolusi: {self.resolution} m/px\n"
            "KLIK KIRI: Titik A lalu Titik B | Tekan 'u': Undo | Tekan 's': Selesai & Simpan",
            fontsize=11, fontweight='bold', pad=12
        )
        self.ax.set_xlabel("Piksel X")
        self.ax.set_ylabel("Piksel Y")

        # Hubungkan event mouse & keyboard
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        plt.tight_layout()
        plt.show()

    def on_click(self, event):
        """Event klik mouse pada canvas gambar peta."""
        # Abaikan klik di luar area peta atau saat tool pan/zoom sedang aktif
        if event.inaxes != self.ax:
            return
        if self.fig.canvas.manager.toolbar.mode != "":
            return

        x, y = event.xdata, event.ydata
        self.current_points.append((x, y))

        # Titik pertama
        if len(self.current_points) == 1:
            pt = self.ax.plot(x, y, 'ro', markersize=6)[0]
            self.temp_artist.append(pt)
            self.fig.canvas.draw_idle()
            print(f"\n[*] Titik 1 terpilih pada piksel ({x:.1f}, {y:.1f}). Sekarang KLIK TITIK 2...")

        # Titik kedua -> Hitung jarak & konfirmasi terminal
        elif len(self.current_points) == 2:
            pt1 = self.current_points[0]
            pt2 = self.current_points[1]

            # Jarak piksel dan meter
            dx = pt2[0] - pt1[0]
            dy = pt2[1] - pt1[1]
            dist_px = math.hypot(dx, dy)
            dist_m = dist_px * self.resolution

            idx = len(self.landmarks) + 1
            # Gambar garis dan nomor landmark
            line = self.ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], color='lime', linewidth=2)[0]
            pt = self.ax.plot(pt2[0], pt2[1], 'go', markersize=6)[0]
            mid_x = (pt1[0] + pt2[0]) / 2.0
            mid_y = (pt1[1] + pt2[1]) / 2.0
            txt = self.ax.text(
                mid_x, mid_y, f"#{idx} ({dist_m:.2f}m)",
                color='yellow', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.2", fc="black", ec="lime", alpha=0.7)
            )

            self.temp_artist.extend([line, pt, txt])
            self.fig.canvas.draw_idle()

            print("-" * 75)
            print(f"[🎯 LANDMARK #{idx} TERPILIH]:")
            print(f"  • Koordinat Piksel : ({pt1[0]:.1f}, {pt1[1]:.1f}) -> ({pt2[0]:.1f}, {pt2[1]:.1f})")
            print(f"  • Panjang di Peta  : {dist_px:.2f} px  x  {self.resolution:.4f} m/px = {dist_m:.4f} meter ({dist_m*100:.2f} cm)")
            print("-" * 75)

            # Input interaktif di terminal
            default_name = f"Landmark_{idx}"
            if idx == 1:
                default_name = "Panjang Dinding Utama"
            elif idx == 2:
                default_name = "Lebar Koridor Depan"
            elif idx == 3:
                default_name = "Jarak Antar-Pilar 1-2"
            elif idx == 4:
                default_name = "Lebar Pintu Ruangan"
            elif idx == 5:
                default_name = "Dimensi Partisi Boks"

            nama_input = input(f">> Nama Landmark #{idx} [Default: '{default_name}']: ").strip()
            landmark_name = nama_input if nama_input else default_name

            d_nyata = None
            while d_nyata is None:
                nyata_str = input(f">> Masukkan Dimensi Fisik Nyata / Meteran Baja [meter] (contoh: {dist_m:.2f}): ").strip()
                try:
                    d_nyata = float(nyata_str)
                    if d_nyata <= 0:
                        print("[!] Nilai harus lebih besar dari 0.")
                        d_nyata = None
                except ValueError:
                    print("[!] Input angka desimal tidak valid. Coba lagi.")

            # Kalkulasi Galat
            error_abs = abs(dist_m - d_nyata)
            scale_error_pct = (error_abs / d_nyata) * 100.0

            data_entry = {
                'id': idx,
                'name': landmark_name,
                'pt1': pt1,
                'pt2': pt2,
                'dist_px': dist_px,
                'dist_peta_m': dist_m,
                'dist_nyata_m': d_nyata,
                'error_abs_m': error_abs,
                'scale_error_pct': scale_error_pct,
                'artists': list(self.temp_artist)
            }
            self.landmarks.append(data_entry)
            self.current_points = []
            self.temp_artist = []

            print(f"[✅ DISIMPAN]: {landmark_name} | Peta={dist_m:.3f}m | Nyata={d_nyata:.3f}m | Galat={error_abs*100:.2f} cm ({scale_error_pct:.2f}%)")
            print(f"[*] Jumlah landmark terkumpul: {len(self.landmarks)}/{self.target_landmarks_count}")
            if len(self.landmarks) >= self.target_landmarks_count:
                print("[💡 INFO] Sudah mencapai 10 landmark! Anda bisa tekan 'S' untuk selesai & rekap laporan.")

    def on_key(self, event):
        """Shortcut keyboard: 'u' untuk undo, 's' untuk simpan."""
        if event.key in ['u', 'U']:
            self.undo_last_landmark()
        elif event.key in ['s', 'S']:
            self.finish_and_export()

    def undo_last_landmark(self):
        """Membatalkan pengukuran titik/landmark terakhir."""
        if self.current_points:
            self.current_points = []
            for a in self.temp_artist:
                a.remove()
            self.temp_artist = []
            self.fig.canvas.draw_idle()
            print("[↩ UNDO] Titik pertama dibatalkan. Silakan klik ulang.")
            return

        if self.landmarks:
            removed = self.landmarks.pop()
            for a in removed['artists']:
                a.remove()
            self.fig.canvas.draw_idle()
            print(f"[↩ UNDO] Landmark #{removed['id']} ({removed['name']}) berhasil dihapus.")

    def finish_and_export(self):
        """Menghitung metrik RMSE, Scale Error, validasi kelulusan dan ekspor CSV/PNG."""
        if not self.landmarks:
            print("[!] Belum ada landmark yang diukur.")
            return

        n = len(self.landmarks)
        # Kalkulasi Root Mean Square Error (RMSE)
        sum_sq_err = sum(item['error_abs_m'] ** 2 for item in self.landmarks)
        rmse = math.sqrt(sum_sq_err / n)

        # Rata-rata Galat Skala (%)
        mean_scale_err = sum(item['scale_error_pct'] for item in self.landmarks) / n
        mean_abs_err_cm = (sum(item['error_abs_m'] for item in self.landmarks) / n) * 100.0

        # Kriteria Kelulusan Skenario 1 Proposal Skripsi:
        # RMSE <= 0.05 meter (5 cm) dan Galat Skala <= 2.0%
        status_rmse = "MEMENUHI (<= 0.05 m)" if rmse <= 0.05 else "TIDAK MEMENUHI (> 0.05 m)"
        status_scale = "MEMENUHI (<= 2.0%)" if mean_scale_err <= 2.0 else "TIDAK MEMENUHI (> 2.0%)"
        status_final = "LULUS / AKURAT & LAYAK" if (rmse <= 0.05 and mean_scale_err <= 2.0) else "EVALUASI ULANG"

        # Tampilkan Rekap Tabel Terminal
        print("\n" + "=" * 95)
        print("  📊 HASIL EVALUASI AKURASI GEOMETRIS PEMETAAN 2D (SKRIPSI BAB 4)")
        print("=" * 95)
        print(f"{'No':<4} | {'Landmark Acuan':<26} | {'Nyata (m)':<10} | {'Peta (m)':<10} | {'Selisih (cm)':<13} | {'Galat Skala':<11}")
        print("-" * 95)
        for row in self.landmarks:
            print(
                f"#{row['id']:<3} | {row['name']:<26} | {row['dist_nyata_m']:>8.3f} m | {row['dist_peta_m']:>8.3f} m | "
                f"{row['error_abs_m']*100:>10.2f} cm | {row['scale_error_pct']:>9.2f} %"
            )
        print("-" * 95)
        print(f"• Rata-rata Selisih Absolut     : {mean_abs_err_cm:.2f} cm ({mean_abs_err_cm/100:.4f} m)")
        print(f"• Root Mean Square Error (RMSE) : {rmse:.4f} meter ({rmse*100:.2f} cm) -> {status_rmse}")
        print(f"• Rata-rata Galat Skala (%)     : {mean_scale_err:.2f}% -> {status_scale}")
        print(f"• KESIMPULAN KELULUSAN          : [ {status_final} ]")
        print("=" * 95)

        # Simpan CSV
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_map_name = os.path.basename(self.yaml_path).replace('.yaml', '')
        csv_filename = os.path.join(self.output_dir, f"hasil_evaluasi_peta_{base_map_name}_{timestamp_str}.csv")
        img_export_name = os.path.join(self.output_dir, f"anotasi_peta_{base_map_name}_{timestamp_str}.png")

        with open(csv_filename, 'w') as f:
            f.write("# =========================================================================\n")
            f.write("# EVALUASI AKURASI GEOMETRIS PEMETAAN 2D SLAM (SKRIPSI TEKNIK KOMPUTER)\n")
            f.write(f"# File Peta      : {self.image_path}\n")
            f.write(f"# Resolusi Grid  : {self.resolution} m/pixel\n")
            f.write(f"# Waktu Pengujian: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# RMSE           : {rmse:.4f} m ({status_rmse})\n")
            f.write(f"# Mean Scale Err : {mean_scale_err:.2f}% ({status_scale})\n")
            f.write(f"# Status Akhir   : {status_final}\n")
            f.write("# =========================================================================\n")
            f.write("No,Nama_Landmark,Dimensi_Nyata_m,Dimensi_Peta_m,Selisih_Absolut_cm,Galat_Skala_Persen\n")
            for r in self.landmarks:
                f.write(f"{r['id']},{r['name']},{r['dist_nyata_m']:.3f},{r['dist_peta_m']:.3f},{r['error_abs_m']*100:.2f},{r['scale_error_pct']:.2f}\n")
            f.write(f"RATA_RATA,-,-,-,{mean_abs_err_cm:.2f},{mean_scale_err:.2f}\n")
            f.write(f"RMSE_METER,-,-,-,{rmse:.4f},-\n")

        # Simpan gambar peta beranotasi
        self.fig.savefig(img_export_name, dpi=300, bbox_inches='tight')

        print(f"\n[💾 BERHASIL DISIMPAN]:")
        print(f"  1. Tabel Data CSV (Bisa dibuka di Excel / Word) : {csv_filename}")
        print(f"  2. Gambar Peta Beranotasi Garis Landmark (PNG)   : {img_export_name}")
        print("\n[🎉 SELESAI] Data pengujian Skenario 1 sudah siap dicantumkan ke Bab 4 Skripsi!")
        plt.close(self.fig)


def main():
    parser = argparse.ArgumentParser(description="Tool Pengukur Akurasi Geometris Peta 2D SLAM Skripsi")
    parser.add_argument(
        '--map',
        type=str,
        default='',
        help='Path lengkap ke file peta .yaml (Default: peta_l1_ddc.yaml)'
    )
    args = parser.parse_args()

    yaml_target = args.map
    if not yaml_target:
        # Cari file peta di direktori workspace
        default_candidates = [
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_l1_ddc.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_1b.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_dc.yaml'),
            os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_kampus.yaml'),
        ]
        for c in default_candidates:
            if os.path.exists(c):
                yaml_target = c
                break

    if not yaml_target or not os.path.exists(yaml_target):
        print("[!] File peta .yaml tidak ditemukan secara otomatis.")
        input_path = input("Masukkan path lengkap ke file peta .yaml: ").strip()
        yaml_target = input_path

    try:
        app = MapGeometricEvaluator(yaml_target)
        app.run()
    except KeyboardInterrupt:
        print("\n[*] Pengukuran dibatalkan.")
    except Exception as e:
        print(f"\n[❌ ERROR]: {e}")


if __name__ == '__main__':
    main()
