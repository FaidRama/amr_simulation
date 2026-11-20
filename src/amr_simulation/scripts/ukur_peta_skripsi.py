#!/usr/bin/env python3
"""
=============================================================================
TOOL EVALUASI AKURASI GEOMETRIS PEMETAAN 2D (SKRIPSI BAB 4 - SKENARIO 1)
Validasi Presisi Geometris Rekonstruksi SLAM Toolbox vs Ground Truth Meteran Baja
=============================================================================
Fitur Unggulan (Full On-Screen Matplotlib GUI - Bebas Error Terminal Readline):
  1. Snap-to-Pixel Diskrit Otomatis: Klik di bagian mana saja dalam kotak piksel
     (tengah, pinggir, pojok) langsung dikunci ke indeks piksel bulat [ix, iy].
  2. Panel Hasil & Kontrol Langsung di Matplotlib:
      Menampilkan pembacaan 3 Mode Batas Piksel sekaligus di layar Matplotlib:
     - [1] Tepi Dalam (Inner Edge  : Center - 1 px)
     - [2] Pusat Piksel (Center-to-Center Index)
     - [3] Tepi Luar  (Outer Edge  : Center + 1 px)
  3. Input Nama Landmark & Ukuran Meteran Nyata langsung di jendela Matplotlib
     (tanpa memanggil input() di dalam event klik yang menyebabkan error readline).
  4. Highlight Grid/Piksel Terlewati (Bresenham 2D Ray-Tracing) + Garis Kisi
     Piksel otomatis saat Scroll-to-Zoom.
  5. Tabel Rekapitulasi Live (RMSE & Galat Skala) langsung di layar Matplotlib,
     serta ekspor otomatis ke CSV dan citra peta beranotasi PNG (300 DPI).
=============================================================================
"""

import os
import sys
import json
import math
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
from matplotlib.widgets import Button, TextBox


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


def _resolve_workspace_root() -> str:
    """Temukan root workspace ROS 2 secara dinamis (aman dijalankan dari direktori mana saja)."""
    for base in (os.path.dirname(os.path.abspath(__file__)), os.getcwd()):
        cur = os.path.abspath(base)
        while True:
            if os.path.isdir(os.path.join(cur, '.git')) or (
                os.path.basename(cur) != 'src'
                and os.path.isdir(os.path.join(cur, 'src', 'amr_simulation'))
                and os.path.isdir(os.path.join(cur, 'src', 'amr_dashboard'))
            ):
                return cur
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    for ws in (os.path.expanduser('~/deliver_amr'), os.path.expanduser('~/Documents/amr_ws')):
        if os.path.isdir(ws):
            return ws
    return os.getcwd()


def resolve_thesis_output_dir(subfolder: str) -> str:
    """Simpan hasil ke <workspace_root>/hasil_data_pengujian_skripsi/<subfolder>."""
    return os.path.join(_resolve_workspace_root(), 'hasil_data_pengujian_skripsi', subfolder)


def discover_available_maps() -> list:
    """Scan semua file peta .yaml valid di folder maps workspace, CWD, dan folder pengujian_1_pemetaan."""
    ws_root = _resolve_workspace_root()
    search_dirs = [
        os.path.join(ws_root, 'src', 'amr_simulation', 'maps'),
        os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps'),
        os.path.join(ws_root, 'hasil_data_pengujian_skripsi', 'pengujian_1_pemetaan'),
        os.getcwd(),
    ]
    seen_real = set()
    found = []

    for d in search_dirs:
        if not os.path.isdir(d):
            continue
        for ypath in glob.glob(os.path.join(d, '*.yaml')):
            rpath = os.path.realpath(ypath)
            if rpath in seen_real:
                continue
            try:
                with open(ypath, 'r', encoding='utf-8') as f:
                    meta = yaml.safe_load(f) or {}
                if not (isinstance(meta, dict) and 'image' in meta and 'resolution' in meta):
                    continue
                img_path = meta['image']
                if not os.path.isabs(img_path):
                    img_path = os.path.join(os.path.dirname(ypath), img_path)
                if not os.path.exists(img_path):
                    continue
                seen_real.add(rpath)
                found.append(ypath)
            except Exception:
                continue

    found.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return found


STANDARD_LANDMARK_NAMES = [
    "Panjang Dinding Utama Barat",
    "Lebar Koridor Utama Depan",
    "Jarak Antar-Pilar 1 - 2",
    "Jarak Antar-Pilar 2 - 3",
    "Lebar Koridor Arah Lift",
    "Panjang Partisi Meja Boks",
    "Lebar Partisi Meja Boks",
    "Lebar Pintu Akses Ruangan",
    "Panjang Dinding Lorong Timur",
    "Dimensi Diagonal Ruang Terbuka",
]


class MapGeometricEvaluator:
    def __init__(self, yaml_path: str, run_number: int = 1):
        self.yaml_path = os.path.abspath(yaml_path)
        self.map_dir = os.path.dirname(self.yaml_path)
        self.run_number = run_number
        self.load_map_metadata()

        # Data pengujian landmark yang sudah disimpan
        self.landmarks = []
        # Titik klik aktif untuk pengukuran saat ini: [(ix1, iy1), (ix2, iy2)]
        self.current_points = []
        self.temp_artists = []
        self.pending_measurement = None
        self.selected_mode = 'center'  # 'inner', 'center', 'outer'
        self.target_landmarks_count = 10

        # Direktori Output & Cache Template Ground Truth
        self.output_dir = resolve_thesis_output_dir('pengujian_1_pemetaan')
        os.makedirs(self.output_dir, exist_ok=True)
        self.cache_file = os.path.join(self.output_dir, 'ground_truth_landmarks_template.json')
        self.gt_template = self.load_ground_truth_template()

        print("\n" + "=" * 80)
        print(f"  SISTEM PENGUKURAN AKURASI GEOMETRIS PETA SLAM 2D (RUN KE-{self.run_number})")
        print("=" * 80)
        print(f"  • File Metadata  : {os.path.basename(self.yaml_path)}")
        print(f"  • File Citra Peta: {os.path.basename(self.image_path)}")
        print(f"  • Resolusi Grid  : {self.resolution:.4f} meter / piksel (1 px = {self.resolution*100:.1f} cm)")
        print(f"  • Dimensi Matriks: {self.img_w} x {self.img_h} piksel ({self.img_w*self.resolution:.2f}m x {self.img_h*self.resolution:.2f}m)")
        print("  • Seluruh pembacaan piksel, pilihan mode batas (Tepi Dalam/Pusat/Tepi Luar),")
        print("    dan input ukuran meteran kini tampil langsung di jendela Matplotlib!")
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

        pil_img = Image.open(self.image_path)
        self.map_img = np.array(pil_img)
        self.img_h, self.img_w = self.map_img.shape[:2]

    def load_ground_truth_template(self):
        """Memuat preset daftar nama & ukuran nyata landmark secara otomatis jika ada."""
        if os.path.exists(self.cache_file):
            try:
                with open(self.cache_file, 'r') as f:
                    data = json.load(f)
                    if isinstance(data, list) and len(data) > 0:
                        return data
            except Exception:
                pass
        return []

    def save_ground_truth_template(self):
        """Menyimpan template daftar landmark acuan untuk mempercepat run berikutnya."""
        template_data = []
        for lm in self.landmarks:
            template_data.append({
                'id': lm['id'],
                'name': lm['name'],
                'dist_nyata_m': lm['dist_nyata_m'],
                'boundary_mode': lm.get('boundary_mode', 'center')
            })
        try:
            with open(self.cache_file, 'w') as f:
                json.dump(template_data, f, indent=2)
        except Exception as e:
            print(f"[!] Gagal menyimpan cache template: {e}")

    def pixel_to_map_coords(self, px, py):
        """Konversi koordinat indeks piksel citra ke koordinat ROS 2 map frame (meter)."""
        x_m = self.origin_x + px * self.resolution
        y_m = self.origin_y + (self.img_h - 1 - py) * self.resolution
        return x_m, y_m

    def get_pixel_occupancy_label(self, ix: int, iy: int) -> str:
        """Membaca nilai intensitas piksel .pgm dan mengembalikan status okupansi sel."""
        if 0 <= iy < self.img_h and 0 <= ix < self.img_w:
            val = int(np.asarray(self.map_img[iy, ix]).flat[0])
            if val < 80:
                return f"Dinding ({val})"
            if val > 230:
                return f"Lantai Bebas ({val})"
            return f"Unknown ({val})"
        return "Luar Batas"

    def _get_default_for_index(self, idx: int):
        """Ambil default nama, dimensi nyata, dan mode batas untuk landmark ke-idx."""
        default_name = (
            STANDARD_LANDMARK_NAMES[idx - 1]
            if idx <= len(STANDARD_LANDMARK_NAMES)
            else f"Landmark_{idx}"
        )
        default_gt = ""
        default_mode = "center"
        if self.gt_template and idx <= len(self.gt_template):
            item = self.gt_template[idx - 1]
            default_name = item.get('name', default_name)
            if item.get('dist_nyata_m') is not None:
                default_gt = f"{float(item['dist_nyata_m']):.3f}"
            default_mode = item.get('boundary_mode', 'center')
        return default_name, default_gt, default_mode

    def run(self):
        """Menjalankan interface interaktif Matplotlib dengan Panel Kontrol di Sisi Kanan."""
        self.fig = plt.figure(figsize=(15.5, 9.0), facecolor='#f8fafc')
        self.fig.canvas.manager.set_window_title(
            f"Evaluasi Geometris SLAM 2D (Snap-to-Pixel) - Run #{self.run_number} | Skripsi Teknik Komputer"
        )

        # Panel Kiri: Kanvas Peta Interaktif
        self.ax = self.fig.add_axes([0.04, 0.07, 0.61, 0.84])
        self.ax.imshow(self.map_img, cmap='gray', origin='upper', interpolation='nearest')
        self.ax.set_title(
            f"EVALUASI AKURASI GEOMETRIS PEMETAAN 2D - SNAP-TO-PIXEL (RUN #{self.run_number}/5)\n"
            f"Peta: {os.path.basename(self.image_path)} | Resolusi: {self.resolution:.4f} m/px (1 px = {self.resolution*100:.1f} cm)\n"
            "KLIK KIRI: Piksel 1 lalu Piksel 2 | Scroll: Zoom | Pilih Mode [1/2/3] & Simpan di Panel Kanan",
            fontsize=9.8, fontweight='bold', pad=10
        )
        self.ax.set_xlabel("Indeks Piksel X (Discrete Grid Index)", fontsize=9.0)
        self.ax.set_ylabel("Indeks Piksel Y (Discrete Grid Index)", fontsize=9.0)

        def _fmt_coord(px, py):
            ix, iy = int(round(px)), int(round(py))
            xm, ym = self.pixel_to_map_coords(ix, iy)
            occ = self.get_pixel_occupancy_label(ix, iy)
            return f"Piksel: [{ix}, {iy}] ({occ}) | ROS: X={xm:+.3f}m, Y={ym:+.3f}m"

        self.ax.format_coord = _fmt_coord
        self.default_xlim = self.ax.get_xlim()
        self.default_ylim = self.ax.get_ylim()

        # Panel Kanan Atas: Kotak Informasi Pembacaan 3 Acuan Piksel
        self.ax_info = self.fig.add_axes([0.67, 0.56, 0.31, 0.38])
        self.ax_info.set_facecolor('#0f172a')
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])
        for sp in self.ax_info.spines.values():
            sp.set_edgecolor('#334155')
            sp.set_linewidth(1.2)

        self.info_text_artist = self.ax_info.text(
            0.035, 0.96, "",
            transform=self.ax_info.transAxes,
            va='top', ha='left',
            color='#f8fafc',
            family='monospace',
            fontsize=8.3,
            linespacing=1.35
        )

        # 3 Tombol Pilihan Mode Batas Piksel langsung di Matplotlib
        ax_m1 = self.fig.add_axes([0.67, 0.505, 0.10, 0.042])
        ax_m2 = self.fig.add_axes([0.775, 0.505, 0.10, 0.042])
        ax_m3 = self.fig.add_axes([0.88, 0.505, 0.10, 0.042])
        self.btn_mode_inner = Button(ax_m1, "[1] Tepi Dalam", color='#e2e8f0', hovercolor='#cbd5e1')
        self.btn_mode_center = Button(ax_m2, "[2] Pusat Piksel", color='#38bdf8', hovercolor='#0284c7')
        self.btn_mode_outer = Button(ax_m3, "[3] Tepi Luar", color='#e2e8f0', hovercolor='#cbd5e1')
        for b in (self.btn_mode_inner, self.btn_mode_center, self.btn_mode_outer):
            b.label.set_fontsize(8.0)
            b.label.set_fontweight('bold')

        self.btn_mode_inner.on_clicked(lambda _e: self.set_boundary_mode('inner'))
        self.btn_mode_center.on_clicked(lambda _e: self.set_boundary_mode('center'))
        self.btn_mode_outer.on_clicked(lambda _e: self.set_boundary_mode('outer'))

        # Input TextBox Nama Landmark & Ukuran Meteran Fisik (m) langsung di Matplotlib
        def_name, def_gt, def_mode = self._get_default_for_index(1)
        self.selected_mode = def_mode

        self.fig.text(0.67, 0.478, "Nama Landmark:", fontsize=8.5, fontweight='bold', color='#0f172a')
        ax_tb_name = self.fig.add_axes([0.67, 0.435, 0.31, 0.038])
        self.tb_name = TextBox(ax_tb_name, "", initial=def_name)
        self.tb_name.text_disp.set_fontsize(8.8)

        self.fig.text(
            0.67, 0.405,
            "Ukuran Meteran Fisik / Ground Truth (meter):",
            fontsize=8.5, fontweight='bold', color='#0f172a'
        )
        ax_tb_gt = self.fig.add_axes([0.67, 0.362, 0.31, 0.038])
        self.tb_gt = TextBox(ax_tb_gt, "", initial=def_gt)
        self.tb_gt.text_disp.set_fontsize(9.2)
        self.tb_gt.on_text_change(self._on_gt_text_changed)

        # Tombol Aksi Utama di Matplotlib
        ax_save_lm = self.fig.add_axes([0.67, 0.305, 0.195, 0.045])
        ax_undo = self.fig.add_axes([0.875, 0.305, 0.105, 0.045])
        self.btn_save_lm = Button(ax_save_lm, "SIMPAN LANDMARK", color='#10b981', hovercolor='#059669')
        self.btn_save_lm.label.set_fontsize(8.6)
        self.btn_save_lm.label.set_fontweight('bold')
        self.btn_save_lm.label.set_color('white')

        self.btn_undo = Button(ax_undo, "UNDO (U)", color='#f59e0b', hovercolor='#d97706')
        self.btn_undo.label.set_fontsize(8.3)
        self.btn_undo.label.set_fontweight('bold')
        self.btn_undo.label.set_color('white')

        self.btn_save_lm.on_clicked(lambda _e: self.confirm_current_landmark())
        self.btn_undo.on_clicked(lambda _e: self.undo_last_landmark())

        # Panel Kanan Bawah: Tabel Rekapitulasi Landmark Tersimpan & RMSE Live
        self.ax_table = self.fig.add_axes([0.67, 0.095, 0.31, 0.198])
        self.ax_table.set_facecolor('#ffffff')
        self.ax_table.set_xticks([])
        self.ax_table.set_yticks([])
        for sp in self.ax_table.spines.values():
            sp.set_edgecolor('#cbd5e1')

        self.table_text_artist = self.ax_table.text(
            0.03, 0.95, "",
            transform=self.ax_table.transAxes,
            va='top', ha='left',
            color='#0f172a',
            family='monospace',
            fontsize=7.6,
            linespacing=1.28
        )

        # Tombol Reset Zoom & Ekspor Akhir CSV/PNG
        ax_reset = self.fig.add_axes([0.67, 0.035, 0.11, 0.045])
        ax_export = self.fig.add_axes([0.79, 0.035, 0.19, 0.045])
        self.btn_reset = Button(ax_reset, "RESET ZOOM (R)", color='#e2e8f0', hovercolor='#cbd5e1')
        self.btn_reset.label.set_fontsize(8.0)
        self.btn_reset.label.set_fontweight('bold')

        self.btn_export = Button(ax_export, "SELESAI & SIMPAN CSV/PNG", color='#0284c7', hovercolor='#0369a1')
        self.btn_export.label.set_fontsize(8.2)
        self.btn_export.label.set_fontweight('bold')
        self.btn_export.label.set_color('white')

        self.btn_reset.on_clicked(lambda _e: self.reset_zoom())
        self.btn_export.on_clicked(lambda _e: self.finish_and_export())

        # Connect events
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_scroll = self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self._refresh_mode_button_colors()
        self._refresh_info_panel()
        self._refresh_summary_panel()
        plt.show()

    def _parse_gt_input(self):
        """Parse angka meteran dari TextBox Ground Truth (mendukung titik maupun koma)."""
        raw = (self.tb_gt.text or "").strip().replace(',', '.')
        if not raw:
            return None
        try:
            val = float(raw)
            return val if val > 0 else None
        except ValueError:
            return None

    def _on_gt_text_changed(self, _text):
        """Ketika angka meteran di TextBox berubah, perbarui perhitungan galat & rekomendasi di layar."""
        if self.pending_measurement is not None:
            self._refresh_info_panel()
            self.fig.canvas.draw_idle()

    def _refresh_mode_button_colors(self):
        """Sorot warna tombol mode batas piksel yang sedang aktif."""
        mapping = [
            ('inner', self.btn_mode_inner),
            ('center', self.btn_mode_center),
            ('outer', self.btn_mode_outer),
        ]
        for code, btn in mapping:
            if self.selected_mode == code:
                btn.color = '#0284c7'
                btn.hovercolor = '#0369a1'
                btn.ax.set_facecolor('#0284c7')
                btn.label.set_color('white')
            else:
                btn.color = '#e2e8f0'
                btn.hovercolor = '#cbd5e1'
                btn.ax.set_facecolor('#e2e8f0')
                btn.label.set_color('#0f172a')

    def set_boundary_mode(self, mode_code: str):
        """Mengubah mode batas piksel ('inner', 'center', 'outer') secara live di Matplotlib."""
        if mode_code not in ('inner', 'center', 'outer'):
            return
        self.selected_mode = mode_code
        self._refresh_mode_button_colors()
        if self.pending_measurement is not None:
            self._update_pending_measurement_line()
        self._refresh_info_panel()
        self.fig.canvas.draw_idle()

    def _update_pending_measurement_line(self):
        """Perbarui posisi ujung garis ukur kuning dan label di peta sesuai mode batas piksel."""
        pm = self.pending_measurement
        if pm is None:
            return

        ix1, iy1 = pm['pt1']
        ix2, iy2 = pm['pt2']
        dx_px = ix2 - ix1
        dy_px = iy2 - iy1
        dist_center_px = pm['dist_center_px']

        ux, uy = (dx_px / dist_center_px, dy_px / dist_center_px) if dist_center_px > 0 else (0.0, 0.0)
        if self.selected_mode == 'inner' and dist_center_px >= 1.0:
            draw_x1, draw_y1 = ix1 + 0.5 * ux, iy1 + 0.5 * uy
            draw_x2, draw_y2 = ix2 - 0.5 * ux, iy2 - 0.5 * uy
            dist_px = pm['dist_inner_px']
            dist_m = pm['dist_inner_m']
            mode_lbl = "Tepi_Dalam"
        elif self.selected_mode == 'outer' and dist_center_px > 0:
            draw_x1, draw_y1 = ix1 - 0.5 * ux, iy1 - 0.5 * uy
            draw_x2, draw_y2 = ix2 + 0.5 * ux, iy2 + 0.5 * uy
            dist_px = pm['dist_outer_px']
            dist_m = pm['dist_outer_m']
            mode_lbl = "Tepi_Luar"
        else:
            draw_x1, draw_y1 = float(ix1), float(iy1)
            draw_x2, draw_y2 = float(ix2), float(iy2)
            dist_px = pm['dist_center_px']
            dist_m = pm['dist_center_m']
            mode_lbl = "Pusat_Piksel"

        if pm.get('line_artist') is not None:
            pm['line_artist'].set_data([draw_x1, draw_x2], [draw_y1, draw_y2])
        if pm.get('text_artist') is not None:
            idx = len(self.landmarks) + 1
            pm['text_artist'].set_text(
                f"#{idx} [{mode_lbl}]\n"
                f"{dist_px:.2f} px = {dist_m:.3f} m ({dist_m*100:.1f} cm)"
            )

    def _refresh_info_panel(self):
        """Tampilkan seluruh informasi pembacaan piksel langsung di panel kanan Matplotlib."""
        idx = len(self.landmarks) + 1
        if self.pending_measurement is None:
            if len(self.current_points) == 0:
                msg = (
                    f"STATUS: SIAP MENGUKUR LANDMARK #{idx}\n"
                    f"{'-'*38}\n"
                    "Langkah Pengukuran di Jendela Ini:\n"
                    "1. Scroll roda mouse pada peta untuk\n"
                    "   Zoom-In ke ujung dinding/koridor.\n"
                    "2. Klik Kiri pada PIKSEL 1 (Kotak Oranye).\n"
                    "3. Klik Kiri pada PIKSEL 2 (Kotak Hijau).\n"
                    "4. Hasil 3 mode piksel (Tepi Dalam,\n"
                    "   Pusat Piksel, Tepi Luar) akan langsung\n"
                    "   tampil di kotak ini!\n"
                    "5. Isi ukuran meteran di bawah lalu klik\n"
                    "   tombol [SIMPAN LANDMARK]."
                )
            else:
                ix1, iy1 = self.current_points[0]
                x1_m, y1_m = self.pixel_to_map_coords(ix1, iy1)
                occ1 = self.get_pixel_occupancy_label(ix1, iy1)
                msg = (
                    f"LANDMARK #{idx} - PIKSEL 1 TERKUNCI\n"
                    f"{'-'*38}\n"
                    f"• Piksel 1 : [{ix1}, {iy1}] ({occ1})\n"
                    f"• Posisi   : X={x1_m:+.3f}m, Y={y1_m:+.3f}m\n\n"
                    "👉 Silakan KLIK KIRI pada PIKSEL 2\n"
                    "   (ujung segmen landmark) di peta...\n\n"
                    "   (Klik tombol UNDO jika salah klik)"
                )
            self.info_text_artist.set_text(msg)
            return

        pm = self.pending_measurement
        ix1, iy1 = pm['pt1']
        ix2, iy2 = pm['pt2']
        dx_px = abs(ix2 - ix1)
        dy_px = abs(iy2 - iy1)
        gt_val = self._parse_gt_input()

        modes = [
            ('1', 'inner', 'Tepi Dalam (-1px)', pm['dist_inner_px'], pm['dist_inner_m']),
            ('2', 'center', 'Pusat Piksel (0px)', pm['dist_center_px'], pm['dist_center_m']),
            ('3', 'outer', 'Tepi Luar  (+1px)', pm['dist_outer_px'], pm['dist_outer_m']),
        ]

        best_code = None
        if gt_val is not None:
            best_code = min(modes, key=lambda item: abs(item[4] - gt_val))[1]

        lines = [
            f"HASIL UKUR PIKSEL LANDMARK #{idx}",
            f"{'-'*38}",
            f"Piksel A : [{ix1}, {iy1}] ({pm['occ1']})",
            f"Piksel B : [{ix2}, {iy2}] ({pm['occ2']})",
            f"Grid dX,dY: {dx_px} px, {dy_px} px ({pm['cell_count']} sel)",
            f"{'-'*38}",
            "PERBANDINGAN 3 ACUAN BATAS PIKSEL:",
        ]

        for num, code, label, px_val, m_val in modes:
            active_mark = ">>" if self.selected_mode == code else "  "
            rec_mark = " ★" if best_code == code else ""
            if gt_val is not None:
                err_cm = abs(m_val - gt_val) * 100.0
                lines.append(
                    f"{active_mark}[{num}] {label}\n"
                    f"     {px_val:6.2f}px = {m_val:.3f}m (Galat: {err_cm:.1f}cm){rec_mark}"
                )
            else:
                lines.append(
                    f"{active_mark}[{num}] {label}\n"
                    f"     {px_val:6.2f}px = {m_val:.3f}m ({m_val*100:.1f}cm)"
                )

        if gt_val is None:
            lines.append(f"{'-'*38}")
            lines.append("Ketik angka meteran di kotak bawah")
            lines.append("untuk melihat selisih galat (cm) otomatis.")
        else:
            lines.append(f"{'-'*38}")
            lines.append("★ = Mode terdekat dengan meteran fisik.")

        self.info_text_artist.set_text("\n".join(lines))

    def _refresh_summary_panel(self):
        """Tampilkan daftar landmark yang sudah disimpan beserta RMSE live di panel kanan bawah."""
        if not self.landmarks:
            self.table_text_artist.set_text(
                f"DAFTAR LANDMARK TERSIMPAN (0/{self.target_landmarks_count})\n"
                f"{'-'*40}\n"
                "Belum ada landmark yang disimpan.\n"
                "Klik 2 piksel di peta lalu tekan tombol\n"
                "[SIMPAN LANDMARK]."
            )
            return

        n = len(self.landmarks)
        rmse = math.sqrt(sum(item['error_abs_m'] ** 2 for item in self.landmarks) / n)
        mean_scale = sum(item['scale_error_pct'] for item in self.landmarks) / n
        status_ok = "LULUS" if (rmse <= 0.05 and mean_scale <= 2.0) else "EVALUASI"

        header = (
            f"TERSIMPAN: {n}/{self.target_landmarks_count} | "
            f"RMSE: {rmse:.4f}m ({rmse*100:.1f}cm) | "
            f"Skala: {mean_scale:.2f}% [{status_ok}]\n"
            f"{'-'*42}"
        )
        rows = [header]
        # Tampilkan hingga 7 landmark terakhir agar muat di kotak
        visible_lms = self.landmarks[-7:]
        for r in visible_lms:
            short_name = r['name'][:15].ljust(15)
            m_tag = "In" if r['boundary_mode'] == 'inner' else ("Out" if r['boundary_mode'] == 'outer' else "Ctr")
            rows.append(
                f"#{r['id']:<2} {short_name} [{m_tag:<3}] "
                f"P:{r['dist_peta_m']:.2f}m N:{r['dist_nyata_m']:.2f}m "
                f"E:{r['error_abs_m']*100:.1f}cm"
            )
        self.table_text_artist.set_text("\n".join(rows))

    def _update_pixel_grid_overlay(self):
        """Tampilkan garis kisi batas piksel (x - 0.5, y - 0.5) saat pengguna melakukan Zoom-In dekat."""
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        span_x = abs(cur_xlim[1] - cur_xlim[0])
        span_y = abs(cur_ylim[1] - cur_ylim[0])

        if span_x <= 110 and span_y <= 110:
            x_min = max(0, int(math.floor(min(cur_xlim))))
            x_max = min(self.img_w, int(math.ceil(max(cur_xlim))))
            y_min = max(0, int(math.floor(min(cur_ylim))))
            y_max = min(self.img_h, int(math.ceil(max(cur_ylim))))
            self.ax.set_xticks(np.arange(x_min - 0.5, x_max + 0.5, 1.0), minor=True)
            self.ax.set_yticks(np.arange(y_min - 0.5, y_max + 0.5, 1.0), minor=True)
            self.ax.grid(which='minor', color='#00e5ff', linestyle='-', linewidth=0.35, alpha=0.38)
        else:
            self.ax.grid(which='minor', visible=False)

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
        self._update_pixel_grid_overlay()
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        """Event klik mouse pada kanvas gambar peta: otomatis snap ke indeks piksel bulat (ix, iy)."""
        if event.inaxes != self.ax:
            return
        if self.fig.canvas.manager.toolbar.mode != "":
            return
        if event.xdata is None or event.ydata is None:
            return

        # Jika pengukuran sebelumnya belum disimpan tetapi user sudah klik lagi,
        # bersihkan titik sementara supaya user bisa ganti pasangan titik dengan bebas
        if len(self.current_points) >= 2:
            self._clear_pending_selection()

        # SNAP KE INDEKS PIKSEL DISKRIT BULAT
        ix = int(round(event.xdata))
        iy = int(round(event.ydata))
        ix = max(0, min(self.img_w - 1, ix))
        iy = max(0, min(self.img_h - 1, iy))

        self.current_points.append((ix, iy))

        # Titik 1 terpilih
        if len(self.current_points) == 1:
            pt_box = Rectangle(
                (ix - 0.5, iy - 0.5), 1.0, 1.0,
                facecolor='#ff9800', edgecolor='#e65100', alpha=0.82, linewidth=1.4, zorder=5
            )
            pt_dot = self.ax.plot(ix, iy, 's', color='#ff3d00', markersize=4.5, zorder=6)[0]
            self.ax.add_patch(pt_box)
            self.temp_artists.extend([pt_box, pt_dot])
            self._refresh_info_panel()
            self.fig.canvas.draw_idle()

        # Titik 2 terpilih -> Tampilkan hasil langsung di Matplotlib tanpa memblokir terminal!
        elif len(self.current_points) == 2:
            pt1 = self.current_points[0]
            pt2 = self.current_points[1]
            ix1, iy1 = pt1[0], pt1[1]
            ix2, iy2 = pt2[0], pt2[1]

            dx_px = ix2 - ix1
            dy_px = iy2 - iy1
            dist_center_px = math.hypot(dx_px, dy_px)
            dist_inner_px = max(0.0, dist_center_px - 1.0)
            dist_outer_px = dist_center_px + 1.0

            dist_inner_m = dist_inner_px * self.resolution
            dist_center_m = dist_center_px * self.resolution
            dist_outer_m = dist_outer_px * self.resolution

            cells = bresenham_cells(ix1, iy1, ix2, iy2)

            patches = [Rectangle((cx - 0.5, cy - 0.5), 1.0, 1.0) for cx, cy in cells]
            pc = PatchCollection(
                patches,
                facecolor='#00e5ff',
                edgecolor='#0288d1',
                alpha=0.45,
                linewidth=0.75,
                zorder=3
            )
            self.ax.add_collection(pc)

            end_box = Rectangle(
                (ix2 - 0.5, iy2 - 0.5), 1.0, 1.0,
                facecolor='#76ff03', edgecolor='#2e7d32', alpha=0.82, linewidth=1.4, zorder=5
            )
            end_dot = self.ax.plot(ix2, iy2, 's', color='#00e676', markersize=4.5, zorder=6)[0]
            self.ax.add_patch(end_box)

            line = self.ax.plot([ix1, ix2], [iy1, iy2], color='#ffff00', linewidth=2.0, zorder=7)[0]

            mid_x = (ix1 + ix2) / 2.0
            mid_y = (iy1 + iy2) / 2.0
            txt = self.ax.text(
                mid_x, mid_y, "",
                color='yellow', fontsize=8.2, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.28", fc="black", ec="#00e5ff", alpha=0.88),
                zorder=8
            )

            self.temp_artists.extend([pc, end_box, end_dot, line, txt])

            self.pending_measurement = {
                'pt1': pt1,
                'pt2': pt2,
                'occ1': self.get_pixel_occupancy_label(ix1, iy1),
                'occ2': self.get_pixel_occupancy_label(ix2, iy2),
                'cell_count': len(cells),
                'dist_inner_px': dist_inner_px,
                'dist_center_px': dist_center_px,
                'dist_outer_px': dist_outer_px,
                'dist_inner_m': dist_inner_m,
                'dist_center_m': dist_center_m,
                'dist_outer_m': dist_outer_m,
                'line_artist': line,
                'text_artist': txt,
            }

            # Jika kotak ukuran meteran masih kosong, isi otomatis dengan nilai pusat piksel
            # supaya pengguna bisa langsung menyimpan atau tinggal mengedit angkanya
            if not (self.tb_gt.text or "").strip():
                self.tb_gt.set_val(f"{dist_center_m:.3f}")
            else:
                # Jika sudah ada angka meteran (mis. dari template), otomatis pilih mode terdekat
                gt_val = self._parse_gt_input()
                if gt_val is not None:
                    candidates = [
                        ('inner', abs(dist_inner_m - gt_val)),
                        ('center', abs(dist_center_m - gt_val)),
                        ('outer', abs(dist_outer_m - gt_val)),
                    ]
                    self.selected_mode = min(candidates, key=lambda c: c[1])[0]
                    self._refresh_mode_button_colors()

            self._update_pending_measurement_line()
            self._refresh_info_panel()
            self.fig.canvas.draw_idle()

    def confirm_current_landmark(self):
        """Simpan pengukuran piksel yang sedang aktif ke dalam daftar landmark."""
        if self.pending_measurement is None:
            print("[!] Silakan klik 2 piksel pada peta terlebih dahulu sebelum menyimpan landmark.")
            return

        pm = self.pending_measurement
        idx = len(self.landmarks) + 1
        def_name, _, _ = self._get_default_for_index(idx)
        landmark_name = (self.tb_name.text or "").strip() or def_name

        if self.selected_mode == 'inner':
            dist_px = pm['dist_inner_px']
            dist_m = pm['dist_inner_m']
            boundary_label = 'Tepi_Dalam'
        elif self.selected_mode == 'outer':
            dist_px = pm['dist_outer_px']
            dist_m = pm['dist_outer_m']
            boundary_label = 'Tepi_Luar'
        else:
            dist_px = pm['dist_center_px']
            dist_m = pm['dist_center_m']
            boundary_label = 'Pusat_Piksel'

        d_nyata = self._parse_gt_input()
        if d_nyata is None:
            d_nyata = dist_m if dist_m > 0 else 0.05

        error_abs = abs(dist_m - d_nyata)
        scale_error_pct = (error_abs / d_nyata) * 100.0

        # Perbarui label permanen di atas peta
        if pm.get('text_artist') is not None:
            pm['text_artist'].set_text(
                f"#{idx} ({dist_px:.1f}px [{boundary_label}] | {dist_m:.3f}m)"
            )

        data_entry = {
            'id': idx,
            'name': landmark_name,
            'pt1': pm['pt1'],
            'pt2': pm['pt2'],
            'cell_count': pm['cell_count'],
            'dist_px': dist_px,
            'boundary_mode': self.selected_mode,
            'boundary_label': boundary_label,
            'dist_peta_m': dist_m,
            'dist_nyata_m': d_nyata,
            'error_abs_m': error_abs,
            'scale_error_pct': scale_error_pct,
            'artists': list(self.temp_artists),
        }
        self.landmarks.append(data_entry)
        self.current_points = []
        self.temp_artists = []
        self.pending_measurement = None

        print(
            f"[DISIMPAN #{idx}] {landmark_name} | Mode: {boundary_label} ({dist_px:.2f} px) | "
            f"Peta={dist_m:.3f}m vs Meteran={d_nyata:.3f}m | Galat={error_abs*100:.2f}cm ({scale_error_pct:.2f}%)"
        )

        # Siapkan default untuk landmark berikutnya
        next_idx = len(self.landmarks) + 1
        next_name, next_gt, next_mode = self._get_default_for_index(next_idx)
        self.selected_mode = next_mode
        self._refresh_mode_button_colors()
        self.tb_name.set_val(next_name)
        self.tb_gt.set_val(next_gt)

        self._refresh_info_panel()
        self._refresh_summary_panel()
        self.fig.canvas.draw_idle()

    def _clear_pending_selection(self):
        """Hapus garis/kotak sementara dari pengukuran yang belum disimpan."""
        self.current_points = []
        self.pending_measurement = None
        for a in self.temp_artists:
            try:
                a.remove()
            except Exception:
                pass
        self.temp_artists = []

    def on_key(self, event):
        """Shortcut keyboard untuk memilih mode (1/2/3), Simpan (Enter), Undo (U), Ekspor (S), Reset (R)."""
        # Jangan jalankan shortcut jika pengguna sedang mengetik di dalam TextBox
        if getattr(self.tb_name, 'capturekeystrokes', False) or getattr(self.tb_gt, 'capturekeystrokes', False):
            return

        if event.key == '1':
            self.set_boundary_mode('inner')
        elif event.key == '2':
            self.set_boundary_mode('center')
        elif event.key == '3':
            self.set_boundary_mode('outer')
        elif event.key == 'enter':
            if self.pending_measurement is not None:
                self.confirm_current_landmark()
        elif event.key in ['u', 'U']:
            self.undo_last_landmark()
        elif event.key in ['s', 'S']:
            self.finish_and_export()
        elif event.key in ['r', 'R']:
            self.reset_zoom()

    def reset_zoom(self):
        """Mengembalikan level zoom ke ukuran peta awal."""
        self.ax.set_xlim(self.default_xlim)
        self.ax.set_ylim(self.default_ylim)
        self._update_pixel_grid_overlay()
        self.fig.canvas.draw_idle()

    def undo_last_landmark(self):
        """Membatalkan titik klik aktif atau menghapus landmark terakhir yang sudah disimpan."""
        if self.current_points or self.pending_measurement is not None:
            self._clear_pending_selection()
            self._refresh_info_panel()
            self.fig.canvas.draw_idle()
            return

        if self.landmarks:
            removed = self.landmarks.pop()
            for a in removed['artists']:
                try:
                    a.remove()
                except Exception:
                    pass
            # Kembalikan nama & ukuran meteran landmark yang baru dihapus ke TextBox
            self.tb_name.set_val(removed['name'])
            self.tb_gt.set_val(f"{removed['dist_nyata_m']:.3f}")
            self.selected_mode = removed.get('boundary_mode', 'center')
            self._refresh_mode_button_colors()
            self._refresh_info_panel()
            self._refresh_summary_panel()
            self.fig.canvas.draw_idle()

    def finish_and_export(self):
        """Menghitung metrik RMSE, Galat Skala, validasi kelulusan skripsi, dan ekspor CSV/PNG."""
        # Jika masih ada pengukuran aktif yang belum ditekan Simpan, simpan otomatis dulu
        if self.pending_measurement is not None:
            self.confirm_current_landmark()

        if not self.landmarks:
            print("[!] Belum ada landmark yang diukur.")
            return

        n = len(self.landmarks)
        sum_sq_err = sum(item['error_abs_m'] ** 2 for item in self.landmarks)
        rmse = math.sqrt(sum_sq_err / n)

        mean_scale_err = sum(item['scale_error_pct'] for item in self.landmarks) / n
        mean_abs_err_cm = (sum(item['error_abs_m'] for item in self.landmarks) / n) * 100.0

        status_rmse = "MEMENUHI (<= 0.05 m)" if rmse <= 0.05 else "TIDAK MEMENUHI (> 0.05 m)"
        status_scale = "MEMENUHI (<= 2.0%)" if mean_scale_err <= 2.0 else "TIDAK MEMENUHI (> 2.0%)"
        status_final = "LULUS / AKURAT & LAYAK" if (rmse <= 0.05 and mean_scale_err <= 2.0) else "EVALUASI ULANG"

        self.save_ground_truth_template()

        print("\n" + "=" * 114)
        print(f"  REKAPITULASI EVALUASI AKURASI GEOMETRIS PETA 2D (SNAP-TO-PIXEL) | RUN #{self.run_number}")
        print("=" * 114)
        print(
            f"{'No':<3} | {'Landmark Acuan':<26} | {'Mode Batas':<13} | {'Piksel':<9} | "
            f"{'Nyata (m)':<10} | {'Peta (m)':<10} | {'Galat (cm)':<11} | {'Galat Skala':<11}"
        )
        print("-" * 114)
        for row in self.landmarks:
            b_lbl = row.get('boundary_label', 'Pusat_Piksel')
            print(
                f"#{row['id']:<2} | {row['name']:<26} | {b_lbl:<13} | {row['dist_px']:>6.2f} px | "
                f"{row['dist_nyata_m']:>8.3f} m | {row['dist_peta_m']:>8.3f} m | "
                f"{row['error_abs_m']*100:>9.2f} cm | {row['scale_error_pct']:>9.2f} %"
            )
        print("-" * 114)
        print(f"• Rata-rata Galat Absolut       : {mean_abs_err_cm:.2f} cm ({mean_abs_err_cm/100:.4f} m)")
        print(f"• Root Mean Square Error (RMSE) : {rmse:.4f} meter ({rmse*100:.2f} cm) -> {status_rmse}")
        print(f"• Rata-rata Galat Skala (%)     : {mean_scale_err:.2f}% -> {status_scale}")
        print(f"• KESIMPULAN KELULUSAN SKRIPSI  : [ {status_final} ]")
        print("=" * 114)

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
            f.write(
                "No,Nama_Landmark,Kotak_Piksel_Bresenham,Dimensi_Nyata_m,Dimensi_Peta_m,"
                "Selisih_Absolut_cm,Galat_Skala_Persen,Piksel_Efektif,Mode_Batas_Piksel\n"
            )
            for r in self.landmarks:
                b_lbl = r.get('boundary_label', 'Pusat_Piksel')
                f.write(
                    f"{r['id']},{r['name']},{r['cell_count']},{r['dist_nyata_m']:.3f},"
                    f"{r['dist_peta_m']:.3f},{r['error_abs_m']*100:.2f},{r['scale_error_pct']:.2f},"
                    f"{r['dist_px']:.2f},{b_lbl}\n"
                )
            f.write(f"RATA_RATA,-,-,-,-,{mean_abs_err_cm:.2f},{mean_scale_err:.2f},-,-\n")
            f.write(f"RMSE_METER,-,-,-,{rmse:.4f},-,-,-,-\n")

        # Ekspor PNG hanya pada area kanvas peta (self.ax) agar bersih untuk dokumen laporan Bab 4
        try:
            extent = self.ax.get_tightbbox(self.fig.canvas.get_renderer()).transformed(
                self.fig.dpi_scale_trans.inverted()
            )
            self.fig.savefig(img_export_name, dpi=300, bbox_inches=extent.expanded(1.03, 1.05))
        except Exception:
            self.fig.savefig(img_export_name, dpi=300, bbox_inches='tight')

        print(f"\n[FILE BERHASIL DISIMPAN]:")
        print(f"  1. Tabel CSV (Word/Excel Ready) : {csv_filename}")
        print(f"  2. Gambar Peta Beranotasi (PNG) : {img_export_name}")
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
    print("  REKAPITULASI STATISTIK 5 RUN PEMETAAN (SKENARIO 1 - BAB 4 SKRIPSI)")
    print("=" * 90)

    run_summaries = []
    landmark_dict = {}

    for fpath in csv_files:
        fname = os.path.basename(fpath)
        try:
            with open(fpath, 'r') as f:
                lines = [line.strip() for line in f if line.strip()]

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

    print("\nTABEL EVALUASI 10 LANDMARK ACUAN (RATA-RATA 5 RUN):")
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
        f.write(
            f"GRAND_MEAN,-,-,-,"
            f"{sum(sum(lm['runs_err_cm'].values())/len(lm['runs_err_cm'].values()) for lm in landmark_dict.values())/len(landmark_dict):.2f},"
            f"{grand_mean_scale:.2f}\n"
        )
        f.write(f"RMSE_GRAND_MEAN,-,-,-,{grand_mean_rmse:.4f},-\n")

    print(f"\n[FILE REKAP TERSIMPAN]: {rekap_csv}\n")


def main():
    parser = argparse.ArgumentParser(description="Tool Evaluasi Akurasi Geometris Peta 2D SLAM Skripsi")
    parser.add_argument('--map', type=str, default='', help='Path file peta .yaml')
    parser.add_argument('--run', type=int, default=1, help='Nomor Run Pengujian (1 s.d. 5)')
    parser.add_argument('--rekap', action='store_true', help='Tampilkan rekapitulasi 5 run pemetaan')
    args = parser.parse_args()

    output_dir = resolve_thesis_output_dir('pengujian_1_pemetaan')

    if args.rekap:
        generate_5_runs_recap(output_dir)
        return

    yaml_target = args.map
    run_target = args.run

    if not yaml_target:
        print("\n" + "=" * 80)
        print("  TOOL EVALUASI AKURASI GEOMETRIS PEMETAAN 2D (SKENARIO 1 SKRIPSI)")
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

        available_maps = discover_available_maps()

        print("\nPilih File Peta yang akan Dievaluasi (Diurutkan dari Terbaru):")
        for idx, m in enumerate(available_maps, 1):
            mtime_str = datetime.fromtimestamp(os.path.getmtime(m)).strftime('%Y-%m-%d %H:%M')
            print(f"  [{idx}] {os.path.basename(m):<24} ({mtime_str}) -> {m}")
        print(f"  [{len(available_maps)+1}] Masukkan Path File Lain Secara Manual")

        pilih_peta = input(f">> Pilih Peta [1-{len(available_maps)+1}] (Default: 1): ").strip()
        try:
            p_idx = int(pilih_peta) if pilih_peta else 1
            if 1 <= p_idx <= len(available_maps):
                yaml_target = available_maps[p_idx - 1]
            else:
                yaml_target = os.path.expanduser(input(">> Masukkan path lengkap file peta .yaml: ").strip())
        except ValueError:
            yaml_target = available_maps[0] if available_maps else ""

        run_input = input(f">> Masukkan Nomor Run Pengujian (1 - 5) [Default: {run_target}]: ").strip()
        if run_input:
            try:
                run_target = int(run_input)
            except ValueError:
                run_target = 1

    if not yaml_target or not os.path.exists(yaml_target):
        print(f"[ERROR] File peta .yaml tidak ditemukan: {yaml_target}")
        sys.exit(1)

    try:
        app = MapGeometricEvaluator(yaml_target, run_number=run_target)
        app.run()
    except KeyboardInterrupt:
        print("\n[*] Pengukuran dibatalkan oleh pengguna.")
    except Exception as e:
        print(f"\n[ERROR]: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
