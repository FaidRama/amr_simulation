#!/usr/bin/env python3
"""
=============================================================================
GENERATOR GRAFIK BAB IV SKRIPSI (300 DPI - FORMAT A4 PORTRAIT, DIPISAH)
Subsistem Navigasi Global Delivery AMR-01

Dirancang khusus untuk dokumen A4 portrait (margin 4-3 cm -> area teks ~14 cm
/ 5,5 inci). Ukuran kanvas diset 5,7 inci agar saat ditempel selebar teks
Word/OnlyOffice, ukuran font tercetak tetap proporsional (8-10 pt) dan tajam.

Menghasilkan 8 file terpisah (1 grafik = 1 PNG, tanpa judul ganda di atas):
  1. skenario1_a_rmse_pemetaan.png       (Skenario 1: RMSE Pemetaan per Run)
  2. skenario1_b_galat_skala.png         (Skenario 1: Galat Skala per Run)
  3. skenario2_a_amcl_galat_posisi.png   (Skenario 2: Galat Posisi AMCL)
  4. skenario2_b_amcl_galat_sudut.png    (Skenario 2: Galat Sudut Hadap AMCL)
  5. skenario3_a_waktu_komputasi.png     (Skenario 3: Waktu A* vs Dijkstra)
  6. skenario3_b_efisiensi_waktu.png     (Skenario 3: Efisiensi Waktu A*)
  7. skenario4_a_waktu_replanning.png    (Skenario 4: Waktu Replanning A*)
  8. skenario4_b_perbandingan_detour.png (Skenario 4: Rute Awal vs Detour)
=============================================================================
"""

import os
import math
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Mode headless agar aman dijalankan di terminal / SSH
import matplotlib.pyplot as plt


# =============================================================================
# KONFIGURASI GAYA PUBLIKASI A4 PORTRAIT (SKALA CETAK 1:1)
# =============================================================================
FIG_W = 5.7   # inci (~14,5 cm, pas dengan lebar area teks A4 margin 4-3 cm)
FIG_H = 3.5   # inci (~8,9 cm, proporsi lanskap ideal di halaman portrait)

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans', 'Arial', 'Helvetica'],
    'mathtext.fontset': 'dejavusans',
    'font.size': 9.0,
    'axes.labelsize': 9.5,
    'axes.labelweight': 'bold',
    'axes.edgecolor': '#334155',
    'axes.linewidth': 0.85,
    'xtick.labelsize': 8.5,
    'ytick.labelsize': 8.5,
    'legend.fontsize': 8.2,
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.08,
})

# Palet warna bersih & kontras (tetap terbaca saat dicetak grayscale)
COL_BLUE   = '#1D4ED8'   # Biru utama (A* / Data utama)
COL_TEAL   = '#0F766E'   # Hijau toska (Terpilih / Lulus / Rata-rata)
COL_ORANGE = '#D97706'   # Oranye (Pembanding / Dijkstra / Semua segmen)
COL_SLATE  = '#64748B'   # Abu-abu netral (Sisa rute awal)
COL_RED    = '#DC2626'   # Merah (Garis batas kelayakan / Anomali)

LABEL_HALO = dict(facecolor='white', edgecolor='none', alpha=0.88, pad=1.0)


def resolve_output_dir() -> str:
    base = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    out_dir = os.path.join(base, 'hasil_data_pengujian_skripsi', 'grafik_bab4')
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def style_axes(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(axis='y', linestyle='--', linewidth=0.55, alpha=0.45, color='#94A3B8')
    ax.set_axisbelow(True)


def save_fig(fig, out_dir: str, filename: str):
    out_path = os.path.join(out_dir, filename)
    fig.savefig(out_path)
    plt.close(fig)
    print(f"[OK] Tersimpan: {out_path}")


# =============================================================================
# 1. SKENARIO 1 - PEMETAAN 2D SLAM (DIPISAH: RMSE & GALAT SKALA)
# =============================================================================
def buat_grafik_skenario_1(out_dir: str):
    # Data aktual (cm) dari 10 landmark x 5 run
    data_cm = [
        (61,  [70, 60, 50, 60, 70]),
        (537, [536, 536, 501, 532, 535]),
        (68,  [65, 65, 80, 70, 80]),
        (305, [308, 307, 306, 301, 306]),
        (540, [537, 541, 536, 536, 540]),
        (219, [220, 215, 216, 215, 220]),
        (133, [130, 130, 130, 130, 125]),
        (185, [185, 185, 185, 180, 180]),
        (75,  [75, 75, 70, 75, 75]),
        (534, [536, 536, 531, 531, 535]),
    ]

    labels = ['Run #1', 'Run #2\n(Terpilih)', 'Run #3', 'Run #4', 'Run #5', 'Agregat\n(Rata-rata)']
    rmses_m, scale_all, scale_macro = [], [], []

    for r in range(5):
        errs_m = [(runs[r] - df) / 100.0 for df, runs in data_cm]
        rmses_m.append(math.sqrt(sum(e * e for e in errs_m) / len(errs_m)))
        scale_all.append(statistics.mean(abs(runs[r] - df) / df * 100.0 for df, runs in data_cm))
        scale_macro.append(statistics.mean(abs(runs[r] - df) / df * 100.0 for df, runs in data_cm if df >= 200))

    avg_errs_m = [(statistics.mean(runs) - df) / 100.0 for df, runs in data_cm]
    rmses_m.append(math.sqrt(sum(e * e for e in avg_errs_m) / len(avg_errs_m)))
    scale_all.append(statistics.mean(abs(statistics.mean(runs) - df) / df * 100.0 for df, runs in data_cm))
    scale_macro.append(statistics.mean(abs(statistics.mean(runs) - df) / df * 100.0 for df, runs in data_cm if df >= 200))

    x = np.arange(len(labels))

    # --- 1a: RMSE Pemetaan (meter) ---
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    bar_colors = [COL_BLUE, COL_TEAL, COL_RED, COL_BLUE, COL_ORANGE, COL_TEAL]
    bars = ax.bar(x, rmses_m, width=0.54, color=bar_colors, edgecolor='#1E293B', linewidth=0.7)
    ax.axhline(0.05, color=COL_RED, linestyle='--', linewidth=1.4, label='Batas Kelayakan (0,05 m)')
    style_axes(ax)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('RMSE (meter)')
    ax.set_ylim(0, 0.16)
    ax.legend(loc='upper right', frameon=True, framealpha=0.95)

    for bar, val in zip(bars, rmses_m):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 0.0035,
            f"{val:.4f}".replace('.', ','),
            ha='center', va='bottom', fontsize=8.2, fontweight='bold', color='#0F172A'
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario1_a_rmse_pemetaan.png')

    # --- 1b: Galat Skala (%) ---
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    w = 0.36
    b_all = ax.bar(x - w / 2, scale_all, width=w, color=COL_ORANGE, edgecolor='#1E293B', linewidth=0.7, label='Semua 10 Segmen')
    b_mac = ax.bar(x + w / 2, scale_macro, width=w, color=COL_TEAL, edgecolor='#1E293B', linewidth=0.7, label='Segmen Panjang ($\\geq$ 2 m)')
    ax.axhline(2.0, color=COL_RED, linestyle='--', linewidth=1.4, label='Batas Kelayakan (2,0%)')
    style_axes(ax)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Galat Skala (%)')
    ax.set_ylim(0, 6.6)
    ax.legend(loc='upper right', frameon=True, framealpha=0.95)

    for bar, val in zip(b_all, scale_all):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 0.12,
            f"{val:.2f}%".replace('.', ','),
            ha='center', va='bottom', fontsize=7.6, fontweight='bold', color='#92400E',
            bbox=LABEL_HALO, zorder=5
        )
    for bar, val in zip(b_mac, scale_macro):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 0.12,
            f"{val:.2f}%".replace('.', ','),
            ha='center', va='bottom', fontsize=7.6, fontweight='bold', color='#0F766E',
            bbox=LABEL_HALO, zorder=5
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario1_b_galat_skala.png')


# =============================================================================
# 2. SKENARIO 2 - LOKALISASI DINAMIS AMCL (DIPISAH: POSISI & SUDUT HADAP)
# =============================================================================
def buat_grafik_skenario_2(out_dir: str):
    targets = ['(-5, -1)', '(4, -3)', '(-11, 0)', '(7, 2)', '(5, -5)', 'Rata-rata\n(25 Uji)']

    # Data mentah per target (5 run)
    epos_raw = [
        [12.8, 10.9, 12.4, 13.9, 13.2],
        [11.6, 12.2, 11.5, 13.9, 13.2],
        [16.2, 11.4, 8.5, 9.2, 12.0],
        [10.5, 9.7, 9.0, 9.6, 11.5],
        [11.5, 11.6, 11.8, 10.9, 8.6],
    ]
    eyaw_raw = [
        [34.4, 29.4, 31.3, 30.6, 33.2],
        [31.3, 27.0, 31.1, 26.6, 31.0],
        [31.6, 27.3, 32.2, 33.3, 30.1],
        [31.0, 29.5, 29.3, 31.4, 30.5],
        [31.0, 31.3, 29.2, 28.1, 32.6],
    ]

    all_epos = [v for grp in epos_raw for v in grp]
    all_eyaw = [v for grp in eyaw_raw for v in grp]

    epos_mean = [statistics.mean(g) for g in epos_raw] + [statistics.mean(all_epos)]
    epos_sd   = [statistics.stdev(g) for g in epos_raw] + [statistics.stdev(all_epos)]
    eyaw_mean = [statistics.mean(g) for g in eyaw_raw] + [statistics.mean(all_eyaw)]
    eyaw_sd   = [statistics.stdev(g) for g in eyaw_raw] + [statistics.stdev(all_eyaw)]

    x = np.arange(len(targets))
    colors = [COL_BLUE] * 5 + [COL_TEAL]

    # --- 2a: Galat Posisi Fisik (cm) ---
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    bars = ax.bar(
        x, epos_mean, yerr=epos_sd, capsize=4.5, width=0.54,
        color=colors, edgecolor='#1E293B', linewidth=0.7,
        error_kw=dict(ecolor='#0F172A', elinewidth=1.1)
    )
    ax.axhline(15.0, color=COL_RED, linestyle='--', linewidth=1.4, label='Toleransi Posisi (15,0 cm)', zorder=3)
    style_axes(ax)
    ax.set_xticks(x)
    ax.set_xticklabels(targets)
    ax.set_xlabel('Koordinat Titik Tujuan (m)')
    ax.set_ylabel(r'Galat Posisi $e_{\mathrm{pos}}$ (cm)')
    ax.set_ylim(0, 20.8)
    ax.legend(loc='upper right', frameon=True, framealpha=0.95)

    for bar, m, s in zip(bars, epos_mean, epos_sd):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            m + s + 0.5,
            f"{m:.2f}\n±{s:.2f}".replace('.', ','),
            ha='center', va='bottom', fontsize=8.0, fontweight='bold', color='#0F172A',
            bbox=LABEL_HALO, zorder=5
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario2_a_amcl_galat_posisi.png')

    # --- 2b: Galat Sudut Hadap Fisik (derajat) ---
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    bars = ax.bar(
        x, eyaw_mean, yerr=eyaw_sd, capsize=4.5, width=0.54,
        color=colors, edgecolor='#1E293B', linewidth=0.7,
        error_kw=dict(ecolor='#0F172A', elinewidth=1.1)
    )
    ax.axhline(34.4, color=COL_RED, linestyle='--', linewidth=1.4, label='Toleransi Sudut (34,4° / 0,60 rad)', zorder=3)
    style_axes(ax)
    ax.set_xticks(x)
    ax.set_xticklabels(targets)
    ax.set_xlabel('Koordinat Titik Tujuan (m)')
    ax.set_ylabel(r'Galat Sudut Hadap $e_{\theta}$ (derajat)')
    ax.set_ylim(0, 46.5)
    ax.legend(loc='upper right', frameon=True, framealpha=0.95)

    for bar, m, s in zip(bars, eyaw_mean, eyaw_sd):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            m + s + 1.0,
            f"{m:.2f}°\n±{s:.2f}°".replace('.', ','),
            ha='center', va='bottom', fontsize=7.9, fontweight='bold', color='#0F172A',
            bbox=LABEL_HALO, zorder=5
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario2_b_amcl_galat_sudut.png')


# =============================================================================
# 3. SKENARIO 3 - KOMPARASI A* VS DIJKSTRA (DIPISAH: WAKTU & EFISIENSI)
# =============================================================================
def buat_grafik_skenario_3(out_dir: str):
    rows = [
        ('SG-01', 0.097,  3.175, 96.9),
        ('SG-02', 0.132,  6.068, 97.8),
        ('SG-03', 0.277,  3.893, 92.9),
        ('SG-04', 0.280,  3.822, 92.7),
        ('SG-05', 0.181, 17.670, 99.0),
        ('SG-06', 1.131,  2.863, 60.5),
        ('SG-07', 0.798,  3.111, 74.4),
        ('SG-08', 0.310,  4.324, 92.8),
        ('SG-09', 1.417,  7.701, 81.6),
        ('SG-10', 0.192,  9.892, 98.1),
        ('SG-11', 0.525, 25.671, 98.0),
        ('SG-12', 0.882, 17.388, 94.9),
        ('SG-13', 0.311, 13.574, 97.7),
        ('SG-14', 7.036, 30.488, 76.9),
        ('SG-15', 10.335, 28.628, 63.9),
    ]

    ids = [r[0] for r in rows]
    t_astar = [r[1] for r in rows]
    t_dijk  = [r[2] for r in rows]
    eff_pct = [r[3] for r in rows]

    x = np.arange(len(ids))
    grp_box = dict(boxstyle='round,pad=0.22', facecolor='#F1F5F9', edgecolor='#CBD5E1', linewidth=0.75)

    # --- 3a: Waktu Komputasi A* vs Dijkstra (ms) ---
    fig, ax = plt.subplots(figsize=(FIG_W, 3.85))
    w = 0.39
    b_a = ax.bar(x - w / 2, t_astar, width=w, color=COL_BLUE, edgecolor='#1E293B', linewidth=0.6, label='A* (Rata-rata: 1,594 ms)')
    b_d = ax.bar(x + w / 2, t_dijk,  width=w, color=COL_ORANGE, edgecolor='#1E293B', linewidth=0.6, label='Dijkstra (Rata-rata: 11,885 ms)')
    style_axes(ax)
    ax.set_ylabel('Waktu Komputasi (ms)')
    ax.set_xlabel('Pasangan Titik Uji Start-Goal')
    ax.set_xticks(x)
    ax.set_xticklabels(ids, rotation=45, ha='right', fontsize=8.0)
    ax.set_ylim(0, 41.5)
    ax.legend(loc='upper left', bbox_to_anchor=(0.01, 0.86), frameon=True, framealpha=0.95)

    for sep in (4.5, 9.5):
        ax.axvline(sep, color='#94A3B8', linestyle=':', linewidth=1.1)

    ax.text(2.0, 38.2, 'Rendah', ha='center', fontsize=8.2, fontweight='bold', color='#334155', bbox=grp_box)
    ax.text(7.0, 38.2, 'Sedang', ha='center', fontsize=8.2, fontweight='bold', color='#334155', bbox=grp_box)
    ax.text(12.0, 38.2, 'Tinggi', ha='center', fontsize=8.2, fontweight='bold', color='#334155', bbox=grp_box)

    # Label vertikal (90 derajat) agar tidak pernah bertabrakan di lebar A4
    for bar, val in zip(b_a, t_astar):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 0.6,
            f"{val:.2f}".replace('.', ','),
            ha='center', va='bottom', rotation=90, fontsize=6.9, color=COL_BLUE, fontweight='bold'
        )
    for bar, val in zip(b_d, t_dijk):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 0.6,
            f"{val:.1f}".replace('.', ','),
            ha='center', va='bottom', rotation=90, fontsize=6.9, color='#92400E', fontweight='bold'
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario3_a_waktu_komputasi.png')

    # --- 3b: Persentase Pemangkasan Waktu Komputasi (%) ---
    fig, ax = plt.subplots(figsize=(FIG_W, 3.7))
    b_e = ax.bar(x, eff_pct, width=0.58, color=COL_TEAL, edgecolor='#1E293B', linewidth=0.6, label='Efisiensi A* terhadap Dijkstra', zorder=2)
    ax.axhline(40.0, color=COL_RED, linestyle='--', linewidth=1.4, label='Target Hipotesis ($\\geq$ 40%)', zorder=3)
    ax.axhline(statistics.mean(eff_pct), color=COL_BLUE, linestyle='-.', linewidth=1.3, label=f"Rata-rata ({statistics.mean(eff_pct):.2f}%)".replace('.', ','), zorder=3)
    style_axes(ax)
    ax.set_ylabel('Pemangkasan Waktu (%)')
    ax.set_xlabel('Pasangan Titik Uji Start-Goal')
    ax.set_xticks(x)
    ax.set_xticklabels(ids, rotation=45, ha='right', fontsize=8.0)
    ax.set_ylim(0, 140)
    ax.legend(loc='upper center', ncol=2, frameon=True, framealpha=0.95, fontsize=7.8)

    for sep in (4.5, 9.5):
        ax.axvline(sep, color='#94A3B8', linestyle=':', linewidth=1.1)

    for bar, val in zip(b_e, eff_pct):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 2.2,
            f"{val:.1f}".replace('.', ','),
            ha='center', va='bottom', rotation=90, fontsize=7.5, fontweight='bold', color='#0F172A',
            bbox=LABEL_HALO, zorder=5
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario3_b_efisiensi_waktu.png')


# =============================================================================
# 4. SKENARIO 4 - REPLANNING FISIK A* (DIPISAH: WAKTU HITUNG & DETOUR)
# =============================================================================
def buat_grafik_skenario_4(out_dir: str):
    # Data 9 percobaan fisik sementara (tambah 1 baris lagi di sini saat Uji #10 selesai)
    # Format: (no, T_astar_ms, L_rem_m, L_new_m, dev_lat_m)
    replan_rows = [
        (1,  22.84, 2.27, 2.49, 0.44),
        (2,  22.23, 3.27, 3.77, 0.79),
        (3,  10.35, 2.32, 3.81, 1.26),
        (4,  10.15, 2.07, 2.27, 0.46),
        (5,  13.58, 4.05, 4.59, 0.72),
        (6,   9.62, 4.91, 4.67, 0.36),
        (7, 108.45, 0.97, 1.24, 0.39),
        (8,   9.10, 3.47, 3.84, 0.68),
        (9,   9.00, 3.97, 4.22, 0.68),
    ]

    labels = [f"Uji {r[0]}" for r in replan_rows]
    t_ast  = [r[1] for r in replan_rows]
    l_rem  = [r[2] for r in replan_rows]
    l_new  = [r[3] for r in replan_rows]
    d_lat  = [r[4] for r in replan_rows]

    mean_t = statistics.mean(t_ast)
    med_t  = statistics.median(t_ast)
    x = np.arange(len(labels))

    # --- 4a: Waktu Komputasi Ulang A* (ms) ---
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    bars = ax.bar(x, t_ast, width=0.56, color=COL_BLUE, edgecolor='#1E293B', linewidth=0.7, label=r'Waktu Hitung Ulang $T_{\mathrm{A^*}}$', zorder=2)
    ax.axhline(200.0, color=COL_RED, linestyle='--', linewidth=1.5, label='Batas Kelayakan (200 ms)', zorder=3)
    ax.axhline(mean_t, color=COL_ORANGE, linestyle='-.', linewidth=1.3, label=f"Rata-rata ({mean_t:.2f} ms)".replace('.', ','), zorder=3)
    ax.axhline(med_t, color=COL_TEAL, linestyle=':', linewidth=1.5, label=f"Median ({med_t:.2f} ms)".replace('.', ','), zorder=3)
    style_axes(ax)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel('Percobaan Pengujian Fisik')
    ax.set_ylabel(r'Waktu Komputasi $T_{\mathrm{A^*}}$ (ms)')
    ax.set_ylim(0, 224.0)
    ax.legend(loc='upper left', bbox_to_anchor=(0.01, 0.85), ncol=2, frameon=True, framealpha=0.95, fontsize=7.8)

    for bar, val in zip(bars, t_ast):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            val + 4.5,
            f"{val:.1f}".replace('.', ','),
            ha='center', va='bottom', fontsize=8.0, fontweight='bold', color='#0F172A',
            bbox=LABEL_HALO, zorder=5
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario4_a_waktu_replanning.png')

    # --- 4b: Sisa Rute Awal vs Rute Replan + Deviasi Lateral ---
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    w = 0.37
    ax.bar(x - w / 2, l_rem, width=w, color=COL_SLATE, edgecolor='#1E293B', linewidth=0.7, label='Sisa Rute Awal (m)')
    ax.bar(x + w / 2, l_new, width=w, color=COL_TEAL,  edgecolor='#1E293B', linewidth=0.7, label='Panjang Rute Replan (m)')
    style_axes(ax)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel('Percobaan Pengujian Fisik')
    ax.set_ylabel('Panjang Lintasan (meter)')
    ax.set_ylim(0, max(max(l_rem), max(l_new)) * 1.34)
    ax.legend(loc='upper right', frameon=True, framealpha=0.95)

    for i, (r_val, n_val, lat) in enumerate(zip(l_rem, l_new, d_lat)):
        top_y = max(r_val, n_val)
        ax.text(
            x[i], top_y + 0.14,
            f"dev\n{lat:.2f} m".replace('.', ','),
            ha='center', va='bottom', fontsize=7.5, color='#334155', fontweight='bold'
        )

    fig.tight_layout()
    save_fig(fig, out_dir, 'skenario4_b_perbandingan_detour.png')


def main():
    out_dir = resolve_output_dir()
    print("=" * 72)
    print("  MEMBANGUN 8 GRAFIK BAB IV SKRIPSI (300 DPI - A4 PORTRAIT TERPISAH)")
    print(f"  Folder Output: {out_dir}")
    print("=" * 72)
    buat_grafik_skenario_1(out_dir)
    buat_grafik_skenario_2(out_dir)
    buat_grafik_skenario_3(out_dir)
    buat_grafik_skenario_4(out_dir)
    print("=" * 72)
    print("  SELESAI! 8 grafik terpisah siap ditempel selebar area teks A4.")
    print("=" * 72)


if __name__ == '__main__':
    main()
