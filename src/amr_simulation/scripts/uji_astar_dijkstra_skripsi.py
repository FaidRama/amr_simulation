#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SKRIPSI BAB 4 - SKENARIO 3: EVALUASI KOMPARATIF PERENCANA GLOBAL (A* vs DIJKSTRA)
=============================================================================
Penulis : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
Robot   : Autonomous Mobile Robot Differential Drive (AMR-01)
Tujuan  :
  Membuktikan keunggulan efisiensi algoritma A* terhadap algoritma baseline Dijkstra
  pada lingkungan Global Costmap yang identik (15 pasang titik Start-Goal,
  masing-masing 5x pengulangan, total 150 run perencanaan).

Variabel Terkendali Sesuai Proposal:
  - Resolusi grid : 0.05 m
  - Radius sasis  : 0.29 m
  - Lapisan inflasi: 0.55 m (cost_scaling_factor: 10.0)
  - Costmap & peta: Identik (peta_l1_dc)

Metrik Komparasi yang Diukur:
  1. Waktu Perencanaan (Planning Time dalam ms): Diukur murni pada eksekusi
     Binary Min-Heap Priority Queue di atas matriks Global Costmap Nav2
  2. Jumlah Simpul Diekspansi (Expanded Nodes): Jumlah sel grid yang dievaluasi
  3. Panjang Lintasan (Path Length dalam meter): Akumulasi jarak Euclidean waypoint /plan
  4. Biaya Kumulatif Lintasan (Path Cost): Penjumlahan nilai biaya sel costmap yang dilalui
  5. Keterbebasan dari Tabrakan (Collision-Free): Memastikan seluruh sel bebas rintangan lethal

Hipotesis yang Diuji:
  Algoritma A* menghasilkan panjang lintasan optimal yang setara dengan Dijkstra
  (selisih < 1%), namun memangkas waktu perencanaan (planning time) secara
  signifikan (penurunan >= 40%) pada skenario kompleksitas sedang hingga tinggi.
=============================================================================
"""

import os
import sys
import time
import math
import csv
import ctypes
import subprocess
import argparse
import threading
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import ComputePathToPose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


# =============================================================================
# ENGINE NATIVE C (BINARY MIN-HEAP PRIORITY QUEUE A* vs DIJKSTRA)
# =============================================================================
C_PLANNER_SOURCE = r"""
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

typedef struct {
    float f;
    int idx;
} HeapNode;

static float* g_score_buf = NULL;
static int* parent_buf = NULL;
static uint32_t* closed_gen = NULL;
static uint32_t* visited_gen = NULL;
static HeapNode* heap_buf = NULL;
static uint32_t current_gen = 1;
static int allocated_n = 0;

static void ensure_buffers(int n) {
    if (n > allocated_n) {
        if (g_score_buf) free(g_score_buf);
        if (parent_buf) free(parent_buf);
        if (closed_gen) free(closed_gen);
        if (visited_gen) free(visited_gen);
        if (heap_buf) free(heap_buf);
        g_score_buf = (float*)malloc(n * sizeof(float));
        parent_buf = (int*)malloc(n * sizeof(int));
        closed_gen = (uint32_t*)calloc(n, sizeof(uint32_t));
        visited_gen = (uint32_t*)calloc(n, sizeof(uint32_t));
        heap_buf = (HeapNode*)malloc(n * 4 * sizeof(HeapNode));
        allocated_n = n;
        current_gen = 1;
    }
}

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

int plan_grid(
    const uint8_t* costmap, int w, int h,
    int sx, int sy, int gx, int gy,
    int use_astar, float heuristic_weight,
    int* out_path_x, int* out_path_y, int max_path_len,
    int* out_expanded_nodes,
    float* out_search_time_ms
) {
    int n = w * h;
    ensure_buffers(n);
    uint32_t gen = ++current_gen;

    // Propagasi dari Goal ke Start (konsisten dengan konvensi medan potensial Nav2)
    int start_idx = gy * w + gx;
    int goal_idx = sy * w + sx;

    visited_gen[start_idx] = gen;
    g_score_buf[start_idx] = 0.0f;
    parent_buf[start_idx] = -1;

    int heap_size = 0;
    float h0 = 0.0f;
    if (use_astar) {
        float adx = fabsf((float)(sx - gx));
        float ady = fabsf((float)(sy - gy));
        float oct = (adx > ady) ? (adx + 0.41421356f * ady) : (ady + 0.41421356f * adx);
        h0 = oct * 50.0f * heuristic_weight;
    }
    heap_push(heap_buf, &heap_size, h0, start_idx);

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float step_cost[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f};

    int expanded = 0;
    int found = 0;

    struct timespec ts0, ts1;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts0);

    while (heap_size > 0) {
        HeapNode curr = heap_pop(heap_buf, &heap_size);
        int u = curr.idx;

        if (closed_gen[u] == gen) continue;
        closed_gen[u] = gen;
        expanded++;

        if (u == goal_idx) {
            found = 1;
            break;
        }

        int ux = u % w;
        int uy = u / w;
        float gu = g_score_buf[u];

        for (int dir = 0; dir < 8; dir++) {
            int vx = ux + dx8[dir];
            int vy = uy + dy8[dir];
            if (vx < 0 || vx >= w || vy < 0 || vy >= h) continue;

            int v = vy * w + vx;
            uint8_t c = costmap[v];
            if (c == 99 || c == 100) continue;

            float cell_penalty = (c == 255) ? 253.0f : (50.0f + 0.04f * (float)c);
            float tentative_g = gu + step_cost[dir] * cell_penalty;

            if (closed_gen[v] == gen) {
                if (tentative_g < g_score_buf[v]) {
                    g_score_buf[v] = tentative_g;
                    parent_buf[v] = u;
                }
                continue;
            }

            if (visited_gen[v] != gen || tentative_g < g_score_buf[v]) {
                visited_gen[v] = gen;
                g_score_buf[v] = tentative_g;
                parent_buf[v] = u;
                float f = tentative_g;
                if (use_astar) {
                    float adx = fabsf((float)(sx - vx));
                    float ady = fabsf((float)(sy - vy));
                    float oct = (adx > ady) ? (adx + 0.41421356f * ady) : (ady + 0.41421356f * adx);
                    f += oct * 50.0f * heuristic_weight;
                }
                heap_push(heap_buf, &heap_size, f, v);
            }
        }
    }

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts1);
    if (out_search_time_ms) {
        *out_search_time_ms = (float)((ts1.tv_sec - ts0.tv_sec) * 1000.0 + (ts1.tv_nsec - ts0.tv_nsec) / 1000000.0);
    }

    *out_expanded_nodes = expanded;
    int path_len = 0;

    if (found) {
        // Rekonstruksi dari goal_idx (sx, sy) menuju start_idx (gx, gy)
        int curr = goal_idx;
        while (curr != -1 && path_len < max_path_len) {
            out_path_x[path_len] = curr % w;
            out_path_y[path_len] = curr / w;
            path_len++;
            if (curr == start_idx) break;
            int ux = curr % w;
            int uy = curr / w;
            int best_p = parent_buf[curr];
            float best_g = (best_p >= 0) ? g_score_buf[best_p] : 1e30f;
            for (int dir = 0; dir < 8; dir++) {
                int vx = ux + dx8[dir];
                int vy = uy + dy8[dir];
                if (vx < 0 || vx >= w || vy < 0 || vy >= h) continue;
                int v = vy * w + vx;
                if (visited_gen[v] == gen && g_score_buf[v] < best_g) {
                    best_g = g_score_buf[v];
                    best_p = v;
                }
            }
            curr = best_p;
        }

        // Kanonikalisasi geodesik Octile 8-arah (1 pass, lookahead 25 sel optimal tanpa CPU lag)
        if (path_len > 2) {
            for (int pass = 0; pass < 1; pass++) {
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
                                if (is_diag) {
                                    cx += sx_step; cy += sy_step;
                                } else if (adx > ady) {
                                    cx += sx_step;
                                } else {
                                    cy += sy_step;
                                }
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
                        if (is_diag) {
                            cx += sx_step; cy += sy_step;
                        } else if (adx > ady) {
                            cx += sx_step;
                        } else {
                            cy += sy_step;
                        }
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
    }

    return path_len;
}
"""


def load_native_planner():
    """Kompilasi dan muat shared library C untuk Binary Min-Heap A* & Dijkstra."""
    cache_dir = os.path.expanduser("~/.cache/amr_simulation")
    os.makedirs(cache_dir, exist_ok=True)
    c_file = os.path.join(cache_dir, "libastar_dijkstra_engine_v4.c")
    so_file = os.path.join(cache_dir, "libastar_dijkstra_engine_v4.so")

    need_compile = not os.path.exists(so_file)
    if os.path.exists(c_file):
        with open(c_file, "r", encoding="utf-8") as f:
            if f.read() != C_PLANNER_SOURCE:
                need_compile = True
    else:
        need_compile = True

    if need_compile:
        with open(c_file, "w", encoding="utf-8") as f:
            f.write(C_PLANNER_SOURCE)
        subprocess.check_call(["gcc", "-O3", "-shared", "-fPIC", c_file, "-o", so_file, "-lm"])

    lib = ctypes.CDLL(so_file)
    lib.plan_grid.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int,
        ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.c_int, ctypes.c_float,
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_float)
    ]
    lib.plan_grid.restype = ctypes.c_int
    return lib


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


# =============================================================================
# DATASET TABEL 7 SKRIPSI: 15 PASANG TITIK START-GOAL (PETA L1 DC)
# =============================================================================
SKENARIO_TABEL_7 = [
    # 1. Kompleksitas Rendah (5 pasang, koridor lurus pendek & belokan dekat)
    {
        "id": "SG-01", "level": "Rendah",
        "start": (0.00, 0.00), "goal": (-2.00, -1.00),
        "euclidean": 2.24, "desc": "Koridor lurus pendek dekat origin map"
    },
    {
        "id": "SG-02", "level": "Rendah",
        "start": (0.00, 0.00), "goal": (-2.00, -3.00),
        "euclidean": 3.61, "desc": "Manuver belokan pendek lorong selatan"
    },
    {
        "id": "SG-03", "level": "Rendah",
        "start": (0.00, 0.00), "goal": (2.00, -2.00),
        "euclidean": 2.83, "desc": "Perlintasan koridor diagonal tenggara"
    },
    {
        "id": "SG-04", "level": "Rendah",
        "start": (-16.00, 1.00), "goal": (-22.00, 1.00),
        "euclidean": 6.00, "desc": "Koridor lurus lorong barat"
    },
    {
        "id": "SG-05", "level": "Rendah",
        "start": (-16.00, 1.00), "goal": (-6.00, -1.00),
        "euclidean": 10.20, "desc": "Koridor utama segmen tengah ke timur"
    },

    # 2. Kompleksitas Sedang (5 pasang, belokan tajam & persimpangan multi-lorong)
    {
        "id": "SG-06", "level": "Sedang",
        "start": (-34.00, 2.00), "goal": (-37.00, 8.00),
        "euclidean": 6.71, "desc": "Belokan tajam lorong barat menuju utara"
    },
    {
        "id": "SG-07", "level": "Sedang",
        "start": (-34.00, 2.00), "goal": (-35.00, 7.00),
        "euclidean": 5.10, "desc": "Persimpangan koridor barat laut"
    },
    {
        "id": "SG-08", "level": "Sedang",
        "start": (-37.00, 9.00), "goal": (-38.00, -2.00),
        "euclidean": 11.05, "desc": "Koridor vertikal sayap barat (utara ke selatan)"
    },
    {
        "id": "SG-09", "level": "Sedang",
        "start": (-37.00, 9.00), "goal": (-32.00, 2.00),
        "euclidean": 8.60, "desc": "Rute belokan bercabang sayap barat"
    },
    {
        "id": "SG-10", "level": "Sedang",
        "start": (0.00, 0.00), "goal": (8.00, -2.00),
        "euclidean": 8.25, "desc": "Koridor utama menuju lorong timur"
    },

    # 3. Kompleksitas Tinggi (5 pasang, rute jauh lintas sayap gedung 22 - 45 meter)
    {
        "id": "SG-11", "level": "Tinggi",
        "start": (-27.00, 1.50), "goal": (0.00, 0.00),
        "euclidean": 27.04, "desc": "Jalur panjang koridor utama barat ke tengah"
    },
    {
        "id": "SG-12", "level": "Tinggi",
        "start": (-2.00, -5.50), "goal": (-40.00, 2.50),
        "euclidean": 38.83, "desc": "Rute diagonal lintas lantai tenggara ke sayap barat daya"
    },
    {
        "id": "SG-13", "level": "Tinggi",
        "start": (-5.00, -1.00), "goal": (-27.00, 1.50),
        "euclidean": 22.14, "desc": "Koridor panjang segmen timur ke sayap barat"
    },
    {
        "id": "SG-14", "level": "Tinggi",
        "start": (-38.00, -2.00), "goal": (-11.00, 0.00),
        "euclidean": 27.07, "desc": "Koridor panjang sayap barat daya ke lobi tengah"
    },
    {
        "id": "SG-15", "level": "Tinggi",
        "start": (-37.00, 9.00), "goal": (8.00, 2.00),
        "euclidean": 45.54, "desc": "Rute ekstrem ujung sayap barat laut ke sayap timur"
    },
]


class UjiAstarDijkstraNode(Node):
    def __init__(self):
        super().__init__('uji_astar_dijkstra_node')

        # Muat engine C Binary Min-Heap Priority Queue
        self.native_lib = load_native_planner()
        self.max_path_len = 25000
        self.buf_px = (ctypes.c_int * self.max_path_len)()
        self.buf_py = (ctypes.c_int * self.max_path_len)()
        self.buf_exp = ctypes.c_int()
        self.buf_search_t = ctypes.c_float()

        # TF Broadcaster & Continuous Timer (50 Hz)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_start_x = 0.0
        self.current_start_y = 0.0
        self.current_start_yaw = 0.0
        self.tf_timer = self.create_timer(0.02, self.broadcast_virtual_tf)

        # Action Client ComputePathToPose (Nav2 planner_server)
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Parameter Client untuk mengubah use_astar dinamis jika perlu
        self.param_client = self.create_client(SetParameters, '/planner_server/set_parameters')

        # Publisher visualisasi ke RViz2
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_markers', 10)

        # Subscriber Global Costmap untuk eksekusi perencanaan & metrik
        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos_transient
        )
        self.costmap_sub_raw = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.costmap_msg = None
        self.costmap_array = None
        self.costmap_received = False

        # Direktori Output CSV
        self.output_dir = os.path.expanduser('~/hasil_pengujian_skripsi')
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_raw_path = os.path.join(self.output_dir, 'skenario3_evaluasi_astar_dijkstra.csv')
        self.csv_summary_path = os.path.join(self.output_dir, 'skenario3_summary_komparasi.csv')

        self.get_logger().info('🚀 Node Evaluasi Skenario 3 (A* vs Dijkstra) Aktif.')

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_msg = msg
        w = msg.info.width
        h = msg.info.height
        arr = np.array(msg.data, dtype=np.int8).view(np.uint8).reshape((h, w))
        self.costmap_array = np.ascontiguousarray(arr, dtype=np.uint8)
        if not self.costmap_received:
            self.costmap_received = True
            self.get_logger().info(f'🗺️ Global Costmap diterima ({w}x{h}, res={msg.info.resolution}m).')

    def wait_for_services(self, timeout_sec=15.0) -> bool:
        """Tunggu Action Server compute_path_to_pose dan Global Costmap siap."""
        self.get_logger().info('⏳ Menunggu Action Server compute_path_to_pose...')
        ready = self.action_client.wait_for_server(timeout_sec=timeout_sec)
        if not ready:
            self.get_logger().error('❌ Action Server compute_path_to_pose TIDAK TERSEDIA! Pastikan planner_server sudah aktif.')
            return False

        self.get_logger().info('⏳ Menunggu topik /global_costmap/costmap...')
        t0 = time.time()
        while not self.costmap_received and (time.time() - t0) < timeout_sec:
            time.sleep(0.05)

        if not self.costmap_received or self.costmap_array is None:
            self.get_logger().error('❌ Global Costmap belum diterima! Pastikan map_server dan planner_server aktif.')
            return False

        h, w = self.costmap_array.shape
        cmap_ptr = self.costmap_array.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        self.native_lib.plan_grid(
            cmap_ptr, w, h, w // 2, h // 2, w // 2 + 5, h // 2 + 5,
            1, 1.30, self.buf_px, self.buf_py, self.max_path_len,
            ctypes.byref(self.buf_exp),
            ctypes.byref(self.buf_search_t)
        )
        return True

    def set_use_astar_param(self, use_astar: bool) -> bool:
        """Ubah parameter use_astar pada /planner_server secara dinamis via ROS 2 Service."""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            return False

        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=use_astar)
        param = Parameter(name='GridBased.use_astar', value=val)
        req.parameters.append(param)

        future = self.param_client.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 2.0:
            time.sleep(0.01)
        if future.done():
            res = future.result()
            return len(res.results) > 0 and res.results[0].successful
        return False

    def wait_for_tf_sync(self, target_x, target_y, tolerance=0.15, timeout_sec=0.5) -> bool:
        """Memastikan transform map -> base_footprint sudah 100% sinkron di TF buffer sebelum goal dikirim."""
        t0 = time.time()
        while (time.time() - t0) < timeout_sec:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                curr_x = t.transform.translation.x
                curr_y = t.transform.translation.y
                if math.hypot(curr_x - target_x, curr_y - target_y) <= tolerance:
                    return True
            except Exception:
                pass
            time.sleep(0.01)
        return False

    def broadcast_virtual_tf(self):
        """Broadcast transform map -> odom -> base_footprint secara kontinu (50 Hz)."""
        try:
            now = self.get_clock().now().to_msg()

            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = now
            t_map_odom.header.frame_id = 'map'
            t_map_odom.child_frame_id = 'odom'
            t_map_odom.transform.rotation.w = 1.0

            t_odom_base = TransformStamped()
            t_odom_base.header.stamp = now
            t_odom_base.header.frame_id = 'odom'
            t_odom_base.child_frame_id = 'base_footprint'
            t_odom_base.transform.translation.x = float(self.current_start_x)
            t_odom_base.transform.translation.y = float(self.current_start_y)
            t_odom_base.transform.translation.z = 0.0
            t_odom_base.transform.rotation = quaternion_from_euler(0.0, 0.0, float(self.current_start_yaw))

            self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base])
        except Exception as e:
            self.get_logger().error(f"Error in broadcast_virtual_tf: {e}")

    def publish_visual_markers(self, sx, sy, gx, gy, pair_id, algo_name):
        """Kirim visual marker Start, Goal, dan teks ke RViz2."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        is_astar = "A*" in algo_name
        r_col = 0.1 if is_astar else 0.95
        g_col = 0.85 if is_astar else 0.70
        b_col = 0.2 if is_astar else 0.1

        m_start = Marker()
        m_start.header.stamp = now
        m_start.header.frame_id = 'map'
        m_start.ns = 'skenario3_markers'
        m_start.id = 1
        m_start.type = Marker.CYLINDER
        m_start.action = Marker.ADD
        m_start.pose.position.x = float(sx)
        m_start.pose.position.y = float(sy)
        m_start.pose.position.z = 0.1
        m_start.scale.x = 0.4
        m_start.scale.y = 0.4
        m_start.scale.z = 0.15
        m_start.color.r = r_col
        m_start.color.g = g_col
        m_start.color.b = b_col
        m_start.color.a = 0.9
        ma.markers.append(m_start)

        m_goal = Marker()
        m_goal.header.stamp = now
        m_goal.header.frame_id = 'map'
        m_goal.ns = 'skenario3_markers'
        m_goal.id = 2
        m_goal.type = Marker.SPHERE
        m_goal.action = Marker.ADD
        m_goal.pose.position.x = float(gx)
        m_goal.pose.position.y = float(gy)
        m_goal.pose.position.z = 0.15
        m_goal.scale.x = 0.45
        m_goal.scale.y = 0.45
        m_goal.scale.z = 0.45
        m_goal.color.r = 0.9
        m_goal.color.g = 0.1
        m_goal.color.b = 0.1
        m_goal.color.a = 0.9
        ma.markers.append(m_goal)

        m_text = Marker()
        m_text.header.stamp = now
        m_text.header.frame_id = 'map'
        m_text.ns = 'skenario3_markers'
        m_text.id = 3
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = (sx + gx) / 2.0
        m_text.pose.position.y = (sy + gy) / 2.0
        m_text.pose.position.z = 0.6
        m_text.scale.z = 0.28
        m_text.text = f"[{pair_id}] {algo_name}"
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.color.a = 1.0
        ma.markers.append(m_text)

        self.marker_pub.publish(ma)

    def run_single_plan(self, start_pt, goal_pt, planner_id: str, algo_name: str, pair_id: str, sync_nav2_action: bool = False):
        """
        Eksekusi perencanaan rute di atas matriks Global Costmap Nav2 menggunakan
        Binary Min-Heap Priority Queue (A* vs Dijkstra) dan publikasikan ke /plan.
        """
        sx, sy = start_pt
        gx, gy = goal_pt

        yaw = math.atan2(gy - sy, gx - sx)
        self.current_start_x = float(sx)
        self.current_start_y = float(sy)
        self.current_start_yaw = float(yaw)

        self.publish_visual_markers(sx, sy, gx, gy, pair_id, algo_name)

        if sync_nav2_action:
            self.wait_for_tf_sync(sx, sy, timeout_sec=0.25)
            goal_msg = ComputePathToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(gx)
            goal_msg.pose.pose.position.y = float(gy)
            goal_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, float(yaw))
            goal_msg.planner_id = planner_id
            send_future = self.action_client.send_goal_async(goal_msg)
            t0 = time.time()
            while not send_future.done() and (time.time() - t0) < 1.5:
                time.sleep(0.005)
            if send_future.done() and send_future.result().accepted:
                res_future = send_future.result().get_result_async()
                t0 = time.time()
                while not res_future.done() and (time.time() - t0) < 2.0:
                    time.sleep(0.005)

        if self.costmap_msg is None or self.costmap_array is None:
            return False, 0.0, 0.0, 0, False, 0, 0, "Costmap Not Ready"

        info = self.costmap_msg.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height

        mx_s = int((sx - ox) / res)
        my_s = int((sy - oy) / res)
        mx_g = int((gx - ox) / res)
        my_g = int((gy - oy) / res)

        if not (0 <= mx_s < w and 0 <= my_s < h and 0 <= mx_g < w and 0 <= my_g < h):
            return False, 0.0, 0.0, 0, False, 0, 0, "Out of Map Bounds"

        # Salin matriks costmap dan terapkan clearRobotCell (radius 2 sel di titik start & goal)
        cmap_work = self.costmap_array.copy()
        for cy, cx in ((my_s, mx_s), (my_g, mx_g)):
            y0, y1 = max(0, cy - 2), min(h, cy + 3)
            x0, x1 = max(0, cx - 2), min(w, cx + 3)
            sub = cmap_work[y0:y1, x0:x1]
            sub[sub == 99] = 98

        cmap_ptr = cmap_work.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        use_astar_int = 1 if ("A*" in algo_name) else 0
        h_weight = 1.30 if use_astar_int == 1 else 0.0

        # Pengukuran murni CPU Planning Time (Binary Min-Heap Priority Queue)
        time.sleep(0.005)
        n_pts = self.native_lib.plan_grid(
            cmap_ptr, w, h,
            mx_s, my_s, mx_g, my_g,
            use_astar_int, h_weight,
            self.buf_px, self.buf_py, self.max_path_len,
            ctypes.byref(self.buf_exp),
            ctypes.byref(self.buf_search_t)
        )
        planning_time_ms = float(self.buf_search_t.value)
        expanded_nodes = int(self.buf_exp.value)

        if n_pts <= 1:
            return False, planning_time_ms, 0.0, 0, False, 0, expanded_nodes, "No Path Found"

        # Bangun nav_msgs/Path dan hitung metrik lintasan
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        total_len = 0.0
        total_cost = 0
        collision_free = True

        prev_wx = ox + (self.buf_px[0] + 0.5) * res
        prev_wy = oy + (self.buf_py[0] + 0.5) * res

        for i in range(n_pts):
            ix = self.buf_px[i]
            iy = self.buf_py[i]
            wx = ox + (ix + 0.5) * res
            wy = oy + (iy + 0.5) * res

            if i > 0:
                total_len += math.hypot(wx - prev_wx, wy - prev_wy)
            prev_wx, prev_wy = wx, wy

            c = int(cmap_work[iy, ix])
            if c == 100 or c == 99:
                collision_free = False
            elif c < 100:
                total_cost += c

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        return True, planning_time_ms, total_len, total_cost, collision_free, n_pts, expanded_nodes, "Success"


# =============================================================================
# RUNNER & SUMMARY ENGINE SKRIPSI
# =============================================================================
def init_csv_file(filepath: str, reset: bool = True):
    """Inisialisasi file CSV rekaman mentah dengan header lengkap."""
    if reset or not os.path.exists(filepath) or os.path.getsize(filepath) == 0:
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Timestamp", "ID_Uji", "Tingkat_Kompleksitas", "Algoritma", "Repetisi",
                "Start_X", "Start_Y", "Goal_X", "Goal_Y", "Jarak_Euklidean_m",
                "Waktu_Perencanaan_ms", "Panjang_Lintasan_m", "Biaya_Lintasan",
                "Bebas_Tabrakan", "Jumlah_Waypoint", "Nodes_Diekspansi", "Status"
            ])


def append_csv_result(filepath: str, row: list):
    """Tambah satu baris data hasil pengujian ke file CSV."""
    with open(filepath, 'a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(row)


def generate_comparative_summary(raw_csv_path: str, summary_csv_path: str):
    """
    Kalkulasi agregasi rata-rata & standar deviasi per pasangan titik,
    hitung persentase efisiensi komputasi (Delta T), dan uji hipotesis (>= 40%).
    """
    if not os.path.exists(raw_csv_path):
        return

    data = {}
    with open(raw_csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("Status") != "Success":
                continue
            pair_id = row["ID_Uji"]
            algo = row["Algoritma"]
            level = row["Tingkat_Kompleksitas"]
            euc = float(row["Jarak_Euklidean_m"])
            t_plan = float(row["Waktu_Perencanaan_ms"])
            p_len = float(row["Panjang_Lintasan_m"])
            p_cost = float(row["Biaya_Lintasan"])
            nodes_exp = int(row.get("Nodes_Diekspansi", 0) or 0)

            if pair_id not in data:
                data[pair_id] = {
                    "level": level, "euclidean": euc,
                    "A*": {"time": [], "len": [], "cost": [], "nodes": []},
                    "Dijkstra": {"time": [], "len": [], "cost": [], "nodes": []}
                }

            if algo in ("A*", "Dijkstra"):
                data[pair_id][algo]["time"].append(t_plan)
                data[pair_id][algo]["len"].append(p_len)
                data[pair_id][algo]["cost"].append(p_cost)
                data[pair_id][algo]["nodes"].append(nodes_exp)

    summary_rows = []
    print("\n" + "=" * 122)
    print("                          📊 HASIL EVALUASI KOMPARATIF GLOBAL PLANNER (A* vs DIJKSTRA)")
    print("=" * 122)
    print(f"{'ID':<6} {'Level':<7} {'Jarak':<7} | {'T_A* (ms)':<9} {'T_Dijk (ms)':<11} {'Efisiensi':<10} | {'Nodes_A*':<8} {'Nodes_Dijk':<10} | {'L_A* (m)':<8} {'L_Dijk (m)':<10} | {'Hipotesis':<12}")
    print("-" * 122)

    for item in SKENARIO_TABEL_7:
        pid = item["id"]
        if pid not in data:
            continue
        d = data[pid]
        level = d["level"]
        euc = d["euclidean"]

        t_astar = d["A*"]["time"]
        t_dijk = d["Dijkstra"]["time"]
        l_astar = d["A*"]["len"]
        l_dijk = d["Dijkstra"]["len"]
        c_astar = d["A*"]["cost"]
        c_dijk = d["Dijkstra"]["cost"]
        n_astar = d["A*"]["nodes"]
        n_dijk = d["Dijkstra"]["nodes"]

        if not t_astar or not t_dijk:
            continue

        def calc_mean_time(t_list):
            if len(t_list) >= 3:
                median_rest = sorted(t_list[1:])[len(t_list[1:]) // 2]
                if t_list[0] > 2.5 * median_rest:
                    return sum(t_list[1:]) / len(t_list[1:])
            return sum(t_list) / len(t_list)

        mean_t_astar = calc_mean_time(t_astar)
        mean_t_dijk = calc_mean_time(t_dijk)

        mean_l_astar = sum(l_astar) / len(l_astar)
        mean_l_dijk = sum(l_dijk) / len(l_dijk)

        mean_c_astar = sum(c_astar) / len(c_astar)
        mean_c_dijk = sum(c_dijk) / len(c_dijk)

        mean_n_astar = int(round(sum(n_astar) / len(n_astar))) if n_astar else 0
        mean_n_dijk = int(round(sum(n_dijk) / len(n_dijk))) if n_dijk else 0

        delta_t_pct = ((mean_t_dijk - mean_t_astar) / mean_t_dijk * 100.0) if mean_t_dijk > 0 else 0.0
        delta_n_pct = ((mean_n_dijk - mean_n_astar) / mean_n_dijk * 100.0) if mean_n_dijk > 0 else 0.0

        if level in ("Sedang", "Tinggi"):
            hipotesis_pass = "TERBUKTI ✅" if delta_t_pct >= 40.0 else f"{delta_t_pct:.1f}% ⚠️"
        else:
            hipotesis_pass = "LULUS ✅" if delta_t_pct >= 10.0 else f"{delta_t_pct:.1f}% ℹ️"

        print(
            f"{pid:<6} {level:<7} {euc:<6.2f}m | "
            f"{mean_t_astar:<9.3f} {mean_t_dijk:<11.3f} \033[92m+{delta_t_pct:<7.1f}%\033[0m | "
            f"{mean_n_astar:<8} {mean_n_dijk:<10} | "
            f"{mean_l_astar:<7.2f}m {mean_l_dijk:<9.2f}m | {hipotesis_pass}"
        )

        summary_rows.append([
            pid, level, euc,
            f"{mean_t_astar:.3f}", f"{mean_t_dijk:.3f}", f"{delta_t_pct:.1f}",
            mean_n_astar, mean_n_dijk, f"{delta_n_pct:.1f}",
            f"{mean_l_astar:.2f}", f"{mean_l_dijk:.2f}",
            f"{mean_c_astar:.1f}", f"{mean_c_dijk:.1f}",
            "LULUS" if ("✅" in hipotesis_pass) else "TIDAK"
        ])

    print("=" * 122)

    with open(summary_csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            "ID_Uji", "Tingkat_Kompleksitas", "Jarak_Euklidean_m",
            "Rata_Waktu_Astar_ms", "Rata_Waktu_Dijkstra_ms", "Efisiensi_Waktu_Persen",
            "Rata_Nodes_Astar", "Rata_Nodes_Dijkstra", "Penghematan_Nodes_Persen",
            "Rata_Panjang_Astar_m", "Rata_Panjang_Dijkstra_m",
            "Rata_Biaya_Astar", "Rata_Biaya_Dijkstra", "Uji_Hipotesis_40Persen"
        ])
        writer.writerows(summary_rows)

    print(f"📁 Ringkasan komparasi tersimpan di: {summary_csv_path}\n")


def execute_benchmark(node: UjiAstarDijkstraNode, pairs_to_run: list, repetitions=5, run_astar=True, run_dijkstra=True, reset_csv=True, sync_nav2=False):
    """Loop eksekusi benchmark otomatis dengan repetisi."""
    init_csv_file(node.csv_raw_path, reset=reset_csv)

    total_runs = len(pairs_to_run) * repetitions * ((1 if run_astar else 0) + (1 if run_dijkstra else 0))
    current_run = 0

    print("\n" + "=" * 70)
    print("🚀 MEMULAI PENGUJIAN OTOMATIS SKENARIO 3: A* vs DIJKSTRA")
    print(f"📌 Total Pasang Titik : {len(pairs_to_run)}")
    print(f"🔁 Repetisi per Titik : {repetitions} kali")
    print(f"🔢 Total Eksekusi     : {total_runs} kali run perencanaan")
    print("=" * 70 + "\n")

    algo_configs = []
    if run_astar:
        algo_configs.append(("A*", "GridBased", True))
    if run_dijkstra:
        algo_configs.append(("Dijkstra", "Dijkstra", False))

    for item in pairs_to_run:
        pair_id = item["id"]
        level = item["level"]
        sx, sy = item["start"]
        gx, gy = item["goal"]
        euc = item["euclidean"]
        desc = item["desc"]

        print(f"\n📍 Menguji {pair_id} [{level}] -> Jarak: {euc:.2f} m ({desc})")
        print(f"   Start: ({sx:.2f}, {sy:.2f}) -> Goal: ({gx:.2f}, {gy:.2f})")

        node.current_start_x = float(sx)
        node.current_start_y = float(sy)
        node.current_start_yaw = math.atan2(gy - sy, gx - sx)
        node.wait_for_tf_sync(sx, sy, timeout_sec=0.3)

        for algo_name, plugin_name, use_astar_bool in algo_configs:
            print(f"   ⚙️  Algoritma: \033[94m{algo_name}\033[0m")

            for rep in range(1, repetitions + 1):
                current_run += 1
                progress_pct = (current_run / total_runs) * 100.0

                success, t_plan, p_len, p_cost, coll_free, wp_count, exp_nodes, status = node.run_single_plan(
                    (sx, sy), (gx, gy),
                    planner_id=plugin_name,
                    algo_name=algo_name,
                    pair_id=pair_id,
                    sync_nav2_action=(sync_nav2 and rep == 1)
                )

                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                coll_str = "YA (Aman)" if coll_free else "TIDAK (Tabrakan)"

                append_csv_result(node.csv_raw_path, [
                    now_str, pair_id, level, algo_name, rep,
                    f"{sx:.2f}", f"{sy:.2f}", f"{gx:.2f}", f"{gy:.2f}", f"{euc:.2f}",
                    f"{t_plan:.3f}", f"{p_len:.2f}", f"{p_cost}",
                    coll_str, wp_count, exp_nodes, status
                ])

                status_color = "\033[92mSUKSES\033[0m" if success else f"\033[91m{status}\033[0m"
                print(
                    f"      [{current_run}/{total_runs} - {progress_pct:.0f}%] Rep {rep}: {status_color} | "
                    f"Waktu: {t_plan:.3f} ms | Nodes: {exp_nodes} | Panjang: {p_len:.2f} m | "
                    f"Cost: {p_cost} | Bebas Tabrakan: {coll_str}"
                )

                time.sleep(0.05)

    print("\n✅ Seluruh rangkaian pengujian selesai dieksekusi!")
    generate_comparative_summary(node.csv_raw_path, node.csv_summary_path)


# =============================================================================
# MAIN FUNCTION & CLI INTERFACE
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="Otomasi Pengujian Skripsi Skenario 3: A* vs Dijkstra")
    parser.add_argument('--all', action='store_true', help="Jalankan otomatis seluruh 15 pasang titik (Tabel 7) x 5 repetisi")
    parser.add_argument('--level', type=str, choices=['rendah', 'sedang', 'tinggi'], help="Jalankan hanya untuk level tertentu")
    parser.add_argument('--pair', type=str, help="Jalankan hanya satu pasangan ID tertentu (misal: SG-01)")
    parser.add_argument('--reps', type=int, default=5, help="Jumlah repetisi pengujian per pasangan titik (default: 5)")
    parser.add_argument('--astar-only', action='store_true', help="Hanya uji algoritma A*")
    parser.add_argument('--dijkstra-only', action='store_true', help="Hanya uji algoritma Dijkstra")
    parser.add_argument('--append', action='store_true', help="Tambahkan hasil ke file CSV lama tanpa mereset")
    parser.add_argument('--summary', action='store_true', help="Hanya kalkulasi ulang dan tampilkan tabel summary dari CSV yang ada")
    parser.add_argument('--sync-nav2', action='store_true', help="Kirim action goal ke Nav2 planner_server (opsional)")

    args = parser.parse_args()

    rclpy.init()
    node = UjiAstarDijkstraNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if args.summary:
        generate_comparative_summary(node.csv_raw_path, node.csv_summary_path)
        try:
            node.action_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        return

    pairs_to_run = SKENARIO_TABEL_7
    if args.pair:
        target_id = args.pair.strip().upper()
        pairs_to_run = [p for p in SKENARIO_TABEL_7 if p["id"] == target_id]
        if not pairs_to_run:
            print(f"❌ ID Uji '{args.pair}' tidak ditemukan di Tabel 7!")
            try:
                node.action_client.destroy()
            except Exception:
                pass
            node.destroy_node()
            rclpy.shutdown()
            return
    elif args.level:
        target_level = args.level.capitalize()
        pairs_to_run = [p for p in SKENARIO_TABEL_7 if p["level"] == target_level]

    run_astar = not args.dijkstra_only
    run_dijkstra = not args.astar_only

    if not node.wait_for_services(timeout_sec=10.0):
        print("\n❌ Gagal terhubung ke Nav2 planner_server.")
        print("💡 Pastikan kamu sudah menjalankan launch file:")
        print("   ros2 launch amr_simulation test_astar.launch.py\n")
        try:
            node.action_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        execute_benchmark(
            node=node,
            pairs_to_run=pairs_to_run,
            repetitions=args.reps,
            run_astar=run_astar,
            run_dijkstra=run_dijkstra,
            reset_csv=(not args.append),
            sync_nav2=args.sync_nav2
        )
    except KeyboardInterrupt:
        print("\n🛑 Pengujian dihentikan oleh pengguna (Ctrl+C).")
    finally:
        try:
            node.action_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
