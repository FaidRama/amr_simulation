#!/usr/bin/env python3
"""
=============================================================================
GENERATOR VISUALISASI AREA EKSPLORASI SIMPUL A* VS DIJKSTRA (300 DPI)
Skenario 3 Bab IV Skripsi - Subsistem Navigasi Global Delivery AMR-01

Membaca peta asli (peta_l1_dc.yaml & peta_l1_dc.pgm), membangun Global Costmap
dengan parameter inflasi identik Nav2 (robot_radius=0,29 m, inflation_radius=
0,55 m, cost_scaling_factor=10,0), lalu menjalankan mesin pencarian A* dan
Dijkstra yang sama persis dengan uji_astar_dijkstra_skripsi.py untuk merekam
seluruh simpul diekspansi (closed set) dan merender 2 PNG terpisah siap A4:
  - skenario3_eksplorasi_astar.png
  - skenario3_eksplorasi_dijkstra.png
=============================================================================
"""

import os
import math
import ctypes
import argparse
import subprocess
import yaml
import numpy as np
from PIL import Image

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D


# =============================================================================
# KONFIGURASI GAYA PUBLIKASI A4 PORTRAIT (LEBAR 5,7 INCI, 300 DPI)
# =============================================================================
FIG_W = 5.7  # inci (~14,5 cm, pas selebar area teks A4 margin 4-3 cm)

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans', 'Arial', 'Helvetica'],
    'font.size': 8.5,
    'axes.labelsize': 9.0,
    'axes.labelweight': 'bold',
    'axes.edgecolor': '#334155',
    'axes.linewidth': 0.85,
    'xtick.labelsize': 8.0,
    'ytick.labelsize': 8.0,
    'legend.fontsize': 7.8,
    'figure.facecolor': 'white',
    'axes.facecolor': '#E2E8F0',
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.08,
})

# Daftar 15 skenario resmi Skenario 3 (identik dengan uji_astar_dijkstra_skripsi.py)
SKENARIO_TABEL_7 = {
    "SG-01": {"level": "Rendah", "start": (0.00, 0.00),   "goal": (-2.00, -1.00)},
    "SG-02": {"level": "Rendah", "start": (0.00, 0.00),   "goal": (-2.00, -3.00)},
    "SG-03": {"level": "Rendah", "start": (0.00, 0.00),   "goal": (2.00, -2.00)},
    "SG-04": {"level": "Rendah", "start": (-16.00, 1.00), "goal": (-22.00, 1.00)},
    "SG-05": {"level": "Rendah", "start": (-16.00, 1.00), "goal": (-6.00, -1.00)},
    "SG-06": {"level": "Sedang", "start": (-34.00, 2.00), "goal": (-37.00, 8.00)},
    "SG-07": {"level": "Sedang", "start": (-34.00, 2.00), "goal": (-35.00, 7.00)},
    "SG-08": {"level": "Sedang", "start": (-37.00, 9.00), "goal": (-38.00, -2.00)},
    "SG-09": {"level": "Sedang", "start": (-37.00, 9.00), "goal": (-32.00, 2.00)},
    "SG-10": {"level": "Sedang", "start": (0.00, 0.00),   "goal": (8.00, -2.00)},
    "SG-11": {"level": "Tinggi", "start": (-27.00, 1.50), "goal": (0.00, 0.00)},
    "SG-12": {"level": "Tinggi", "start": (-2.00, -5.50), "goal": (-40.00, 2.50)},
    "SG-13": {"level": "Tinggi", "start": (-5.00, -1.00), "goal": (-27.00, 1.50)},
    "SG-14": {"level": "Tinggi", "start": (-38.00, -2.00),"goal": (-11.00, 0.00)},
    "SG-15": {"level": "Tinggi", "start": (-37.00, 9.00), "goal": (8.00, 2.00)},
}


# =============================================================================
# ENGINE NATIVE C: INFLATION COSTMAP + A* / DIJKSTRA + REKAM CLOSED SET
# =============================================================================
C_ENGINE_SOURCE = r"""
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    float f;
    int idx;
} HeapNode;

static inline void heap_push(HeapNode* heap, int* size, float f, int idx) {
    int i = (*size)++;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (heap[p].f <= f) break;
        heap[i] = heap[p];
        i = p;
    }
    heap[i].f = f;
    heap[i].idx = idx;
}

static inline HeapNode heap_pop(HeapNode* heap, int* size) {
    HeapNode top = heap[0];
    HeapNode last = heap[--(*size)];
    int i = 0;
    int n = *size;
    while ((i << 1) + 1 < n) {
        int left = (i << 1) + 1;
        int right = left + 1;
        int smallest = left;
        if (right < n && heap[right].f < heap[left].f) {
            smallest = right;
        }
        if (last.f <= heap[smallest].f) break;
        heap[i] = heap[smallest];
        i = smallest;
    }
    if (n > 0) heap[i] = last;
    return top;
}

void build_inflated_costmap(
    const uint8_t* static_map, int w, int h,
    float res, float robot_radius, float inflation_radius, float cost_scaling,
    uint8_t* out_costmap
) {
    int n = w * h;
    memcpy(out_costmap, static_map, n * sizeof(uint8_t));

    int cell_rad = (int)ceilf(inflation_radius / res);
    int ksize = 2 * cell_rad + 1;
    uint8_t* kernel = (uint8_t*)calloc(ksize * ksize, sizeof(uint8_t));

    for (int dy = -cell_rad; dy <= cell_rad; dy++) {
        for (int dx = -cell_rad; dx <= cell_rad; dx++) {
            float d = sqrtf((float)(dx * dx + dy * dy)) * res;
            uint8_t c = 0;
            if (d == 0.0f) {
                c = 100;
            } else if (d <= robot_radius) {
                c = 99;
            } else if (d <= inflation_radius) {
                float factor = expf(-cost_scaling * (d - robot_radius));
                int raw = (int)floorf(252.0f * factor + 0.5f);
                if (raw < 1) raw = 1;
                if (raw > 252) raw = 252;
                int occ = (int)floorf(1.0f + 97.0f * (float)(raw - 1) / 251.0f + 0.5f);
                if (occ < 1) occ = 1;
                if (occ > 98) occ = 98;
                c = (uint8_t)occ;
            }
            kernel[(dy + cell_rad) * ksize + (dx + cell_rad)] = c;
        }
    }

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (static_map[y * w + x] != 100) continue;
            for (int dy = -cell_rad; dy <= cell_rad; dy++) {
                int ny = y + dy;
                if (ny < 0 || ny >= h) continue;
                int row_off = ny * w;
                int k_off = (dy + cell_rad) * ksize;
                for (int dx = -cell_rad; dx <= cell_rad; dx++) {
                    int nx = x + dx;
                    if (nx < 0 || nx >= w) continue;
                    uint8_t kc = kernel[k_off + (dx + cell_rad)];
                    if (kc == 0) continue;
                    uint8_t cur = out_costmap[row_off + nx];
                    if (cur == 255) {
                        // Di Nav2 InflationLayer: inflasi dapat menimpa unknown dengan cost inflasi
                        out_costmap[row_off + nx] = kc;
                    } else if (kc > cur) {
                        out_costmap[row_off + nx] = kc;
                    }
                }
            }
        }
    }

    free(kernel);
}

int plan_grid_with_mask(
    const uint8_t* costmap, int w, int h,
    int sx, int sy, int gx, int gy,
    int use_astar, float heuristic_weight,
    int* out_path_x, int* out_path_y, int max_path_len,
    uint8_t* out_closed_mask,
    int* out_expanded_nodes
) {
    int n = w * h;
    float* g_score = (float*)malloc(n * sizeof(float));
    int* parent = (int*)malloc(n * sizeof(int));
    uint8_t* closed = (uint8_t*)calloc(n, sizeof(uint8_t));
    uint8_t* visited = (uint8_t*)calloc(n, sizeof(uint8_t));
    HeapNode* heap = (HeapNode*)malloc(n * 4 * sizeof(HeapNode));

    // Konsisten dengan uji_astar_dijkstra_skripsi.py: propagasi Goal -> Start
    int start_idx = gy * w + gx;
    int goal_idx = sy * w + sx;

    visited[start_idx] = 1;
    g_score[start_idx] = 0.0f;
    parent[start_idx] = -1;

    int heap_size = 0;
    float h0 = 0.0f;
    if (use_astar) {
        float adx = fabsf((float)(sx - gx));
        float ady = fabsf((float)(sy - gy));
        float oct = (adx > ady) ? (adx + 0.41421356f * ady) : (ady + 0.41421356f * adx);
        h0 = oct * 50.0f * heuristic_weight;
    }
    heap_push(heap, &heap_size, h0, start_idx);

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float step_cost[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f};

    int expanded = 0;
    int found = 0;

    while (heap_size > 0) {
        HeapNode curr = heap_pop(heap, &heap_size);
        int u = curr.idx;
        if (closed[u]) continue;
        closed[u] = 1;
        expanded++;

        if (u == goal_idx) {
            found = 1;
            break;
        }

        int ux = u % w;
        int uy = u / w;
        float gu = g_score[u];

        for (int dir = 0; dir < 8; dir++) {
            int vx = ux + dx8[dir];
            int vy = uy + dy8[dir];
            if (vx < 0 || vx >= w || vy < 0 || vy >= h) continue;

            int v = vy * w + vx;
            uint8_t c = costmap[v];
            if (c == 99 || c == 100) continue;

            float cell_penalty = (c == 255) ? 253.0f : (50.0f + 0.04f * (float)c);
            float tentative_g = gu + step_cost[dir] * cell_penalty;

            if (closed[v]) {
                if (tentative_g < g_score[v]) {
                    g_score[v] = tentative_g;
                    parent[v] = u;
                }
                continue;
            }

            if (!visited[v] || tentative_g < g_score[v]) {
                visited[v] = 1;
                g_score[v] = tentative_g;
                parent[v] = u;
                float f = tentative_g;
                if (use_astar) {
                    float adx = fabsf((float)(sx - vx));
                    float ady = fabsf((float)(sy - vy));
                    float oct = (adx > ady) ? (adx + 0.41421356f * ady) : (ady + 0.41421356f * adx);
                    f += oct * 50.0f * heuristic_weight;
                }
                heap_push(heap, &heap_size, f, v);
            }
        }
    }

    memcpy(out_closed_mask, closed, n * sizeof(uint8_t));
    *out_expanded_nodes = expanded;

    int path_len = 0;
    if (found) {
        int curr = goal_idx;
        while (curr != -1 && path_len < max_path_len) {
            out_path_x[path_len] = curr % w;
            out_path_y[path_len] = curr / w;
            path_len++;
            if (curr == start_idx) break;
            int ux = curr % w;
            int uy = curr / w;
            int best_p = parent[curr];
            float best_g = (best_p >= 0) ? g_score[best_p] : 1e30f;
            for (int dir = 0; dir < 8; dir++) {
                int vx = ux + dx8[dir];
                int vy = uy + dy8[dir];
                if (vx < 0 || vx >= w || vy < 0 || vy >= h) continue;
                int v = vy * w + vx;
                if (visited[v] && g_score[v] < best_g) {
                    best_g = g_score[v];
                    best_p = v;
                }
            }
            curr = best_p;
        }

        // Kanonikalisasi geodesik Octile 8-arah (identik dengan uji_astar_dijkstra_skripsi.py)
        if (path_len > 2) {
            int r_idx = 0;
            int w_idx = 0;
            while (r_idx < path_len - 1) {
                int x0 = out_path_x[r_idx];
                int y0 = out_path_y[r_idx];
                int best_j = r_idx + 1;
                int best_mode = 0;
                int max_j = (r_idx + 25 < path_len) ? (r_idx + 25) : (path_len - 1);
                for (int j = max_j; j > r_idx + 1; j--) {
                    int x1 = out_path_x[j];
                    int y1 = out_path_y[j];
                    int dx = x1 - x0, dy = y1 - y0;
                    int adx = abs(dx), ady = abs(dy);
                    int steps = (adx > ady) ? adx : ady;
                    if (steps > j - r_idx) continue;
                    int sx_step = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
                    int sy_step = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
                    int diag_steps = (adx < ady) ? adx : ady;
                    for (int mode = 0; mode < 2; mode++) {
                        int cx = x0, cy = y0;
                        int ok = 1;
                        for (int s = 1; s <= steps; s++) {
                            int is_diag = (mode == 0) ? (s <= diag_steps) : (s > steps - diag_steps);
                            if (is_diag) { cx += sx_step; cy += sy_step; }
                            else if (adx > ady) { cx += sx_step; }
                            else { cy += sy_step; }
                            if (costmap[cy * w + cx] >= 90) { ok = 0; break; }
                        }
                        if (ok) { best_j = j; best_mode = mode; break; }
                    }
                    if (best_j == j) break;
                }
                int x1 = out_path_x[best_j];
                int y1 = out_path_y[best_j];
                int dx = x1 - x0, dy = y1 - y0;
                int adx = abs(dx), ady = abs(dy);
                int steps = (adx > ady) ? adx : ady;
                int sx_step = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
                int sy_step = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
                int diag_steps = (adx < ady) ? adx : ady;
                int cx = x0, cy = y0;
                out_path_x[w_idx] = cx;
                out_path_y[w_idx] = cy;
                w_idx++;
                for (int s = 1; s < steps; s++) {
                    int is_diag = (best_mode == 0) ? (s <= diag_steps) : (s > steps - diag_steps);
                    if (is_diag) { cx += sx_step; cy += sy_step; }
                    else if (adx > ady) { cx += sx_step; }
                    else { cy += sy_step; }
                    out_path_x[w_idx] = cx;
                    out_path_y[w_idx] = cy;
                    w_idx++;
                }
                r_idx = best_j;
            }
            out_path_x[w_idx] = out_path_x[path_len - 1];
            out_path_y[w_idx] = out_path_y[path_len - 1];
            w_idx++;
            path_len = w_idx;
        }
    }

    free(g_score);
    free(parent);
    free(closed);
    free(visited);
    free(heap);
    return path_len;
}
"""


def load_engine():
    cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '__pycache__')
    os.makedirs(cache_dir, exist_ok=True)
    c_file = os.path.join(cache_dir, 'libeksplorasi_astar_dijkstra.c')
    so_file = os.path.join(cache_dir, 'libeksplorasi_astar_dijkstra.so')

    need_compile = not os.path.exists(so_file)
    if os.path.exists(c_file):
        with open(c_file, 'r', encoding='utf-8') as f:
            if f.read() != C_ENGINE_SOURCE:
                need_compile = True
    else:
        need_compile = True

    if need_compile:
        with open(c_file, 'w', encoding='utf-8') as f:
            f.write(C_ENGINE_SOURCE)
        subprocess.check_call(['gcc', '-O3', '-shared', '-fPIC', c_file, '-o', so_file, '-lm'])

    lib = ctypes.CDLL(so_file)
    lib.build_inflated_costmap.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int,
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_uint8)
    ]
    lib.plan_grid_with_mask.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int,
        ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.c_int, ctypes.c_float,
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.c_int,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_int)
    ]
    lib.plan_grid_with_mask.restype = ctypes.c_int
    return lib


def load_map_and_costmap(lib, yaml_path: str):
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)

    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))
    negate = int(meta.get('negate', 0))

    img = np.array(Image.open(img_path).convert('L'), dtype=np.uint8)
    # Flip vertikal agar baris 0 = origin Y bawah (sama seperti ROS OccupancyGrid)
    img = np.flipud(img)
    h, w = img.shape

    occ_prob = (img / 255.0) if negate else ((255.0 - img) / 255.0)
    static_map = np.full((h, w), 255, dtype=np.uint8)  # 255 = unknown
    static_map[occ_prob <= free_thresh] = 0
    static_map[occ_prob >= occ_thresh] = 100

    costmap = np.empty_like(static_map)
    lib.build_inflated_costmap(
        static_map.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        w, h, res, 0.29, 0.55, 10.0,
        costmap.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    )
    return img, static_map, costmap, res, ox, oy


def run_planner(lib, costmap: np.ndarray, res: float, ox: float, oy: float,
                start_xy, goal_xy, use_astar: bool):
    h, w = costmap.shape
    sx, sy = start_xy
    gx, gy = goal_xy
    mx_s, my_s = int((sx - ox) / res), int((sy - oy) / res)
    mx_g, my_g = int((gx - ox) / res), int((gy - oy) / res)

    cmap_work = np.ascontiguousarray(costmap.copy(), dtype=np.uint8)
    for cy, cx in ((my_s, mx_s), (my_g, mx_g)):
        y0, y1 = max(0, cy - 2), min(h, cy + 3)
        x0, x1 = max(0, cx - 2), min(w, cx + 3)
        sub = cmap_work[y0:y1, x0:x1]
        sub[sub == 99] = 98

    max_len = 25000
    buf_px = (ctypes.c_int * max_len)()
    buf_py = (ctypes.c_int * max_len)()
    closed_mask = np.zeros((h, w), dtype=np.uint8)
    exp_nodes = ctypes.c_int()

    n_pts = lib.plan_grid_with_mask(
        cmap_work.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        w, h, mx_s, my_s, mx_g, my_g,
        1 if use_astar else 0,
        1.30 if use_astar else 0.0,
        buf_px, buf_py, max_len,
        closed_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        ctypes.byref(exp_nodes)
    )

    path_wx = [ox + (buf_px[i] + 0.5) * res for i in range(n_pts)]
    path_wy = [oy + (buf_py[i] + 0.5) * res for i in range(n_pts)]
    path_len = 0.0
    for i in range(1, n_pts):
        path_len += math.hypot(path_wx[i] - path_wx[i - 1], path_wy[i] - path_wy[i - 1])

    return closed_mask, path_wx, path_wy, int(exp_nodes.value), path_len


def compute_crop_bounds(raw_pgm: np.ndarray, res: float, pad_m: float = 1.6):
    """Potong margin kosong (unknown=205) agar denah bangunan memenuhi frame secara optimal."""
    known = (raw_pgm != 205)
    ys, xs = np.where(known)
    pad_cells = int(round(pad_m / res))
    h, w = raw_pgm.shape
    x0 = max(0, int(xs.min()) - pad_cells)
    x1 = min(w - 1, int(xs.max()) + pad_cells)
    y0 = max(0, int(ys.min()) - pad_cells)
    y1 = min(h - 1, int(ys.max()) + pad_cells)
    return x0, x1, y0, y1


def format_int_id(n: int) -> str:
    return f"{n:,}".replace(',', '.')


def render_exploration_figure(
    raw_pgm: np.ndarray,
    costmap: np.ndarray,
    closed_mask: np.ndarray,
    path_wx, path_wy,
    start_xy, goal_xy,
    res: float, ox: float, oy: float,
    crop_bounds,
    pair_id: str,
    algo_name: str,
    exp_nodes: int,
    path_len: float,
    out_path: str
):
    x0, x1, y0, y1 = crop_bounds
    sub_pgm    = raw_pgm[y0:y1 + 1, x0:x1 + 1]
    sub_cost   = costmap[y0:y1 + 1, x0:x1 + 1]
    sub_closed = closed_mask[y0:y1 + 1, x0:x1 + 1]
    ch, cw = sub_pgm.shape

    # Bangun citra RGB resolusi penuh peta SLAM
    rgb = np.zeros((ch, cw, 3), dtype=np.float32)

    # 1. Background Unknown (abu-abu peta SLAM khas RViz2)
    rgb[:, :] = (0.84, 0.87, 0.91)

    # 2. Ruang bebas koridor (putih bersih)
    free_mask = (sub_pgm >= 240)
    rgb[free_mask] = (1.0, 1.0, 1.0)

    # 3. Area inflasi tipis di sekitar dinding (abu-abu lembut hanya di ruang bebas)
    infl_mask = (sub_cost >= 1) & (sub_cost <= 99) & free_mask
    rgb[infl_mask] = (0.91, 0.94, 0.97)

    # 4. Area simpul diekspansi (closed set)
    exp_bool = (sub_closed == 1)
    if 'A*' in algo_name:
        exp_color = np.array([0.15, 0.45, 0.96], dtype=np.float32)  # Biru kobalt terang
        legend_exp_hex = '#2563EB'
        path_color = '#0F172A'
    else:
        exp_color = np.array([0.96, 0.58, 0.04], dtype=np.float32)  # Oranye amber terang
        legend_exp_hex = '#F59E0B'
        path_color = '#0F172A'

    alpha = 0.68
    rgb[exp_bool] = (1.0 - alpha) * rgb[exp_bool] + alpha * exp_color

    # 5. Dinding / rintangan lethal (hitam arang tajam)
    wall_mask = (sub_pgm <= 80)
    rgb[wall_mask] = (0.08, 0.12, 0.18)

    # Hitung rasio tinggi gambar berdasarkan rasio peta + ruang sumbu
    extent = [
        ox + x0 * res,
        ox + (x1 + 1) * res,
        oy + y0 * res,
        oy + (y1 + 1) * res
    ]
    world_w = extent[1] - extent[0]
    world_h = extent[3] - extent[2]
    fig_h = max(2.9, FIG_W * (world_h / world_w) + 0.85)

    fig, ax = plt.subplots(figsize=(FIG_W, fig_h))
    ax.imshow(rgb, origin='lower', extent=extent, interpolation='nearest')

    # Lintasan optimal (ramping & tajam agar tidak menutupi area eksplorasi A*)
    if path_wx:
        ax.plot(path_wx, path_wy, color='white', linewidth=2.3, solid_capstyle='round', zorder=4)
        ax.plot(path_wx, path_wy, color=path_color, linewidth=1.25, solid_capstyle='round', zorder=5)

    # Titik Start & Goal
    ax.scatter(
        [start_xy[0]], [start_xy[1]],
        s=54, color='#10B981', edgecolor='#064E3B', linewidth=1.1, zorder=7
    )
    ax.scatter(
        [goal_xy[0]], [goal_xy[1]],
        s=92, marker='*', color='#DC2626', edgecolor='#7F1D1D', linewidth=0.85, zorder=8
    )

    ax.set_xlabel('Posisi X (meter)')
    ax.set_ylabel('Posisi Y (meter)')
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', linewidth=0.45, alpha=0.45, color='#64748B')

    # Kotak identitas skenario di pojok kanan atas (area unknown kosong)
    s_txt = f"({start_xy[0]:.0f}, {start_xy[1]:.0f})"
    g_txt = f"({goal_xy[0]:.0f}, {goal_xy[1]:.0f})"
    info_text = (
        f"Skenario {pair_id} | Algoritma {algo_name}\n"
        f"Start {s_txt} m  →  Goal {g_txt} m"
    )
    ax.text(
        0.985, 0.96, info_text,
        transform=ax.transAxes, ha='right', va='top',
        fontsize=7.8, fontweight='bold', color='#0F172A',
        bbox=dict(boxstyle='round,pad=0.35', facecolor='white', edgecolor='#94A3B8', alpha=0.95, linewidth=0.8),
        zorder=9
    )

    # Legenda rapi 1 baris di atas plot
    legend_handles = [
        Patch(facecolor=legend_exp_hex, edgecolor='#334155', alpha=0.80, label=f'Area Eksplorasi {algo_name}'),
        Line2D([0], [0], color=path_color, lw=1.6, label='Lintasan Optimal'),
        Line2D([0], [0], marker='o', linestyle='None', markerfacecolor='#10B981', markeredgecolor='#064E3B', markersize=6.2, label='Start'),
        Line2D([0], [0], marker='*', linestyle='None', markerfacecolor='#DC2626', markeredgecolor='#7F1D1D', markersize=8.5, label='Goal'),
    ]
    ax.legend(
        handles=legend_handles,
        loc='lower center', bbox_to_anchor=(0.5, 1.02),
        ncol=4, frameon=True, facecolor='white', edgecolor='#CBD5E1', framealpha=0.98,
        columnspacing=0.9, handletextpad=0.4, borderpad=0.35
    )

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print(f"[OK] Tersimpan: {out_path} ({algo_name}: {exp_nodes} sel, {path_len:.2f} m)")


def main():
    parser = argparse.ArgumentParser(description='Generate visualisasi area eksplorasi A* vs Dijkstra (300 DPI)')
    parser.add_argument('--pair', type=str, default='SG-15', help='ID skenario dari Tabel 7 (default: SG-15)')
    parser.add_argument('--scan', action='store_true', help='Tampilkan perbandingan jumlah simpul untuk SG-11 s.d. SG-15')
    args = parser.parse_args()

    ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    yaml_path = os.path.join(ws_root, 'src', 'amr_simulation', 'maps', 'peta_l1_dc.yaml')
    out_dir = os.path.join(ws_root, 'hasil_data_pengujian_skripsi', 'grafik_bab4')
    os.makedirs(out_dir, exist_ok=True)

    lib = load_engine()
    raw_pgm, static_map, costmap, res, ox, oy = load_map_and_costmap(lib, yaml_path)
    crop_bounds = compute_crop_bounds(raw_pgm, res, pad_m=1.6)

    if args.scan:
        print("ID    | Nodes A* | Nodes Dijkstra | Hemat (%) | L_A* (m) | L_Dijk (m)")
        print("-" * 72)
        for pid in ("SG-11", "SG-12", "SG-13", "SG-14", "SG-15"):
            sc = SKENARIO_TABEL_7[pid]
            _, _, _, n_a, l_a = run_planner(lib, costmap, res, ox, oy, sc['start'], sc['goal'], True)
            _, _, _, n_d, l_d = run_planner(lib, costmap, res, ox, oy, sc['start'], sc['goal'], False)
            pct = (n_d - n_a) / n_d * 100.0 if n_d else 0.0
            print(f"{pid:<5} | {n_a:>8} | {n_d:>14} | {pct:>8.1f}% | {l_a:>8.2f} | {l_d:>10.2f}")
        return

    pair_id = args.pair.strip().upper()
    if pair_id not in SKENARIO_TABEL_7:
        raise ValueError(f"Skenario '{pair_id}' tidak ditemukan. Pilihan: {list(SKENARIO_TABEL_7.keys())}")

    sc = SKENARIO_TABEL_7[pair_id]
    mask_a, px_a, py_a, n_a, l_a = run_planner(lib, costmap, res, ox, oy, sc['start'], sc['goal'], True)
    mask_d, px_d, py_d, n_d, l_d = run_planner(lib, costmap, res, ox, oy, sc['start'], sc['goal'], False)

    out_astar = os.path.join(out_dir, 'skenario3_eksplorasi_astar.png')
    out_dijk  = os.path.join(out_dir, 'skenario3_eksplorasi_dijkstra.png')

    render_exploration_figure(
        raw_pgm, costmap, mask_a, px_a, py_a, sc['start'], sc['goal'],
        res, ox, oy, crop_bounds, pair_id, 'A*', n_a, l_a, out_astar
    )
    render_exploration_figure(
        raw_pgm, costmap, mask_d, px_d, py_d, sc['start'], sc['goal'],
        res, ox, oy, crop_bounds, pair_id, 'Dijkstra', n_d, l_d, out_dijk
    )


if __name__ == '__main__':
    main()
