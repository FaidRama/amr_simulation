/* =================================================================
   AMR-01 // CONTROL STUDIO — Main Application (SPA Router & Theme Engine)
   Handles page navigation, state management, theme system, and API calls.
   ================================================================= */

// ====================== CURATED THEME REGISTRY ======================
const STUDIO_THEMES = [
    {
        id: 'nordic-slate',
        name: 'Nordic Slate',
        desc: 'Soft Eye-Care Dark',
        bg: '#11151f',
        card: '#181e2c',
        accent: '#3b82f6',
        secondary: '#38bdf8',
        success: '#10b981'
    },
    {
        id: 'tokyo-night',
        name: 'Tokyo Night',
        desc: 'Deep Indigo & Lavender',
        bg: '#16161e',
        card: '#1a1b26',
        accent: '#7aa2f7',
        secondary: '#bb9af7',
        success: '#73daca'
    },
    {
        id: 'catppuccin-mocha',
        name: 'Catppuccin Mocha',
        desc: 'Warm Cozy Espresso',
        bg: '#181825',
        card: '#1e1e2e',
        accent: '#89b4fa',
        secondary: '#cba6f7',
        success: '#a6e3a1'
    },
    {
        id: 'emerald-forest',
        name: 'Emerald Forest',
        desc: 'Calm Botanical Pine',
        bg: '#0d1514',
        card: '#13201e',
        accent: '#10b981',
        secondary: '#34d399',
        success: '#2dd4bf'
    },
    {
        id: 'tactical-amber',
        name: 'Tactical Amber',
        desc: 'Avionics Carbon & Gold',
        bg: '#121311',
        card: '#191b18',
        accent: '#f59e0b',
        secondary: '#fbbf24',
        success: '#22c55e'
    },
    {
        id: 'swiss-daylight',
        name: 'Swiss Daylight',
        desc: 'Clean Industrial Light',
        bg: '#f1f5f9',
        card: '#ffffff',
        accent: '#2563eb',
        secondary: '#0ea5e9',
        success: '#059669'
    }
];

function getTheme() {
    const saved = localStorage.getItem('amr_theme') || (APP.settings && APP.settings.theme) || 'nordic-slate';
    return STUDIO_THEMES.some(t => t.id === saved) ? saved : 'nordic-slate';
}

function applyTheme(themeId, silent = false) {
    const themeObj = STUDIO_THEMES.find(t => t.id === themeId) || STUDIO_THEMES[0];
    document.documentElement.setAttribute('data-theme', themeObj.id);
    localStorage.setItem('amr_theme', themeObj.id);

    if (APP.settings) {
        APP.settings.theme = themeObj.id;
    }

    // Update topbar label
    const topLabel = document.getElementById('topbar-theme-label');
    if (topLabel) {
        topLabel.textContent = themeObj.name;
    }

    // Update active state on theme cards if on Config page
    document.querySelectorAll('.theme-card').forEach(card => {
        card.classList.toggle('active', card.dataset.themeId === themeObj.id);
    });

    // Redraw canvases if visible so colors match new theme immediately
    if (typeof drawSpeedHistoryChart === 'function') {
        drawSpeedHistoryChart();
    }
    if (typeof window.redrawJoystick === 'function') {
        window.redrawJoystick();
    }

    if (!silent) {
        showToast(`Tema diterapkan: ${themeObj.name}`, 'info');
    }
}

function cycleTheme() {
    const current = getTheme();
    const idx = STUDIO_THEMES.findIndex(t => t.id === current);
    const next = STUDIO_THEMES[(idx + 1) % STUDIO_THEMES.length];
    applyTheme(next.id, false);
}

window.STUDIO_THEMES = STUDIO_THEMES;
window.getTheme = getTheme;
window.applyTheme = applyTheme;
window.cycleTheme = cycleTheme;

// ====================== STATE ======================
const APP = {
    currentPage: 'home',
    connected: false,
    pose: { x: 0, y: 0, yaw: 0 },
    delivery: { status: 'idle', current_phase: 'idle' },
    humanStatus: 'NO_HUMAN',
    savedPoints: [],
    homePosition: { x: 0, y: 0, yaw: 0 },
    settings: {},
    taskQueue: [],
    selectingFor: null,
    pendingPickup: null,
    pendingDelivery: null,
    statusPollTimer: null,
};

// ====================== API HELPERS ======================
async function apiGet(url) {
    try {
        const res = await fetch(url);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return await res.json();
    } catch (e) {
        console.error(`API GET ${url}:`, e);
        return null;
    }
}

async function apiPost(url, data) {
    try {
        const res = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data)
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return await res.json();
    } catch (e) {
        console.error(`API POST ${url}:`, e);
        return null;
    }
}

// ====================== TOAST ======================
function showToast(message, type = 'info') {
    const container = document.getElementById('toast-container');
    if (!container) return;
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    const iconName = type === 'success' ? 'checkCircle' : (type === 'error' ? 'xCircle' : 'info');
    toast.innerHTML = `<span class="icon-inline">${getIcon(iconName, 16)}</span><span>${message}</span>`;
    container.appendChild(toast);
    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transform = 'translateY(8px)';
        toast.style.transition = '0.25s ease';
        setTimeout(() => toast.remove(), 250);
    }, 3000);
}

// ====================== MODAL ======================
function showModal(title, body, actions) {
    document.getElementById('modal-title').textContent = title;
    document.getElementById('modal-body').innerHTML = body;
    const actionsEl = document.getElementById('modal-actions');
    actionsEl.innerHTML = '';
    actions.forEach(a => {
        const btn = document.createElement('button');
        btn.className = `btn ${a.class || 'btn-outline'}`;
        btn.textContent = a.label;
        btn.onclick = () => {
            document.getElementById('modal-overlay').classList.add('hidden');
            if (a.action) a.action();
        };
        actionsEl.appendChild(btn);
    });
    document.getElementById('modal-overlay').classList.remove('hidden');
}

function hideModal() {
    document.getElementById('modal-overlay').classList.add('hidden');
}

function closeModal(event) {
    if (event.target.id === 'modal-overlay') {
        hideModal();
    }
}

// ====================== NAVIGATION ======================
function navigateTo(page) {
    APP.currentPage = page;
    const content = document.getElementById('app-content');
    const backBtn = document.getElementById('btn-back');
    const titleEl = document.getElementById('page-title');

    if (page === 'home') {
        backBtn.classList.add('hidden');
        titleEl.textContent = 'Mission Overview';
    } else {
        backBtn.classList.remove('hidden');
        const titles = {
            delivery: 'Delivery Dispatch',
            monitoring: 'Cockpit Telemetry',
            camera: 'Optical Camera Stream',
            history: 'Mission Logs',
            joystick: 'Manual Teleop',
            settings: 'System Configuration'
        };
        titleEl.textContent = titles[page] || page;
    }

    document.querySelectorAll('#app-sidebar .nav-item').forEach(item => {
        item.classList.toggle('active', item.dataset.page === page);
    });
    document.querySelectorAll('#mobile-nav .mobile-nav-item').forEach(item => {
        item.classList.toggle('active', item.dataset.page === page);
    });

    if (APP.statusPollTimer) {
        clearInterval(APP.statusPollTimer);
        APP.statusPollTimer = null;
    }
    if (typeof stopAllMapLoops === 'function') {
        stopAllMapLoops();
    }

    const renderers = {
        home: renderHome,
        delivery: renderDelivery,
        monitoring: renderMonitoring,
        camera: renderCamera,
        history: renderHistory,
        joystick: renderJoystick,
        settings: renderSettings
    };

    if (renderers[page]) {
        content.innerHTML = '';
        content.appendChild(renderers[page]());
    }

    if (typeof handleDeliveryStatusUpdate === 'function' && APP.delivery) {
        handleDeliveryStatusUpdate(APP.delivery);
    }
}

// ====================== PAGE: HOME (MISSION & TELEMETRY OVERVIEW) ======================
function renderHome() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    const statusText = APP.delivery.status === 'running'
        ? `TASK ${(APP.delivery.current_task_index || 0) + 1}/${APP.delivery.total_tasks || 0}`
        : (APP.delivery.status === 'idle' ? 'STANDBY' : String(APP.delivery.status).toUpperCase());

    const phaseLabels = {
        idle: 'Siap Menerima Tugas',
        navigating_pickup: 'Menuju Titik Pickup',
        waiting_pickup: 'Menunggu Muat Barang',
        at_pickup: 'Tiba di Pickup',
        navigating_delivery: 'Menuju Titik Tujuan',
        waiting_delivery: 'Menunggu Ambil Barang',
        at_delivery: 'Tiba di Tujuan',
        returning_home: 'Kembali ke Home Base'
    };

    const yawDeg = ((APP.pose.yaw || 0) * 180 / Math.PI).toFixed(1);
    const humanReadable = APP.humanStatus === 'NO_HUMAN' ? 'CLEAR (AMAN)' : APP.humanStatus;
    const activeTasksCount = (APP.delivery.status === 'running' && APP.delivery.total_tasks)
        ? APP.delivery.total_tasks
        : APP.taskQueue.length;

    div.innerHTML = `
        <div class="studio-overview-header">
            <div>
                <div class="studio-title-tag">${getIcon('terminal', 13)} SYS // OVERVIEW</div>
                <h1>Mission & Telemetry Overview</h1>
                <p>Pusat kendali utama, peta SLAM real-time, dan direktori operasi unit AMR-01</p>
            </div>
            <div class="studio-header-actions">
                <button class="btn btn-outline btn-sm" onclick="navigateTo('monitoring')">
                    ${getIcon('activity', 15)} Buka Cockpit
                </button>
                <button class="btn btn-primary btn-sm" onclick="navigateTo('delivery')">
                    ${getIcon('plus', 15)} Dispatch Baru
                </button>
            </div>
        </div>

        <!-- 4-Card KPI Telemetry Strip -->
        <div class="overview-kpi-grid">
            <div class="kpi-card">
                <div class="kpi-top">
                    <span>01 // STATUS MISI</span>
                    ${getIcon('activity', 14)}
                </div>
                <div class="kpi-main" id="home-status">${statusText}</div>
                <div class="kpi-sub">
                    <span>FASE</span>
                    <span class="text-accent" id="home-phase">${phaseLabels[APP.delivery.current_phase] || APP.delivery.current_phase}</span>
                </div>
            </div>

            <div class="kpi-card">
                <div class="kpi-top">
                    <span>02 // POSE (X, Y)</span>
                    ${getIcon('crosshair', 14)}
                </div>
                <div class="kpi-main" id="home-pos">${APP.pose.x.toFixed(2)}, ${APP.pose.y.toFixed(2)}</div>
                <div class="kpi-sub">
                    <span>HEADING YAW</span>
                    <span class="text-accent" id="home-yaw">${yawDeg}°</span>
                </div>
            </div>

            <div class="kpi-card">
                <div class="kpi-top">
                    <span>03 // AI SAFETY</span>
                    ${getIcon('shield', 14)}
                </div>
                <div class="kpi-main text-emerald" id="home-human">${humanReadable}</div>
                <div class="kpi-sub">
                    <span>SENSOR</span>
                    <span class="text-emerald">LIDAR + VISION OK</span>
                </div>
            </div>

            <div class="kpi-card">
                <div class="kpi-top">
                    <span>04 // TASK QUEUE</span>
                    ${getIcon('package', 14)}
                </div>
                <div class="kpi-main"><span id="home-queue-count">${activeTasksCount}</span> <span style="font-size:0.75rem; color:var(--text-muted); font-weight:500;">TASK</span></div>
                <div class="kpi-sub">
                    <span>WAYPOINTS</span>
                    <span>${APP.savedPoints.length} Tersimpan</span>
                </div>
            </div>
        </div>

        <!-- Main 2-Column Overview Grid -->
        <div class="overview-main-grid">
            <!-- LEFT: Live SLAM Map Preview & 1-Click Quick Dispatch -->
            <div class="studio-panel">
                <div class="studio-panel-header">
                    <div class="studio-panel-title">
                        ${getIcon('map', 15)} LIVE 2D SLAM RADAR & QUICK DISPATCH
                    </div>
                    <button class="btn btn-outline btn-sm" style="padding:3px 9px; font-size:0.68rem;" onclick="navigateTo('delivery')">
                        Buka Full Dispatch ${getIcon('chevronRight', 13)}
                    </button>
                </div>
                <div class="studio-panel-body">
                    <div class="map-container" style="margin-bottom: 12px;">
                        <canvas id="home-map-canvas"></canvas>
                        <div class="map-legend">
                            <span class="legend-pickup">Pickup</span>
                            <span class="legend-delivery">Delivery</span>
                            <span class="legend-robot">AMR-01</span>
                        </div>
                    </div>
                    <div style="display:flex; align-items:center; justify-content:space-between; margin-bottom:6px;">
                        <span class="input-label">1-Click Quick Dispatch Target</span>
                        <span class="mono" style="font-size:0.66rem; color:var(--text-muted);">Klik untuk kirim langsung</span>
                    </div>
                    <div class="quick-dispatch-grid" id="home-dispatch-grid"></div>
                </div>
            </div>

            <!-- RIGHT: Studio Control Modules -->
            <div class="studio-panel">
                <div class="studio-panel-header">
                    <div class="studio-panel-title">
                        ${getIcon('layers', 15)} DIREKTORI MODUL KONTROL
                    </div>
                    <span class="mono" style="font-size:0.66rem; color:var(--text-muted);">6 MODUL</span>
                </div>
                <div class="studio-panel-body">
                    <div class="module-list">
                        <div class="module-row" onclick="navigateTo('monitoring')">
                            <span class="module-index">01</span>
                            <div class="module-icon-box">${getIcon('activity', 18)}</div>
                            <div class="module-info">
                                <div class="module-name">Cockpit Telemetry</div>
                                <div class="module-desc">Dual viewport (Kamera + Peta), speedometer, & grafik kecepatan</div>
                            </div>
                            <span class="module-arrow">${getIcon('chevronRight', 16)}</span>
                        </div>

                        <div class="module-row" onclick="navigateTo('delivery')">
                            <span class="module-index">02</span>
                            <div class="module-icon-box">${getIcon('delivery', 18)}</div>
                            <div class="module-info">
                                <div class="module-name">Delivery Dispatch</div>
                                <div class="module-desc">Susun antrean pickup & tujuan pengiriman multi-waypoint</div>
                            </div>
                            <span class="module-arrow">${getIcon('chevronRight', 16)}</span>
                        </div>

                        <div class="module-row" onclick="navigateTo('camera')">
                            <span class="module-index">03</span>
                            <div class="module-icon-box">${getIcon('video', 18)}</div>
                            <div class="module-info">
                                <div class="module-name">Optical Camera Stream</div>
                                <div class="module-desc">Live feed kamera robot, status deteksi manusia, & tangkapan layar</div>
                            </div>
                            <span class="module-arrow">${getIcon('chevronRight', 16)}</span>
                        </div>

                        <div class="module-row" onclick="navigateTo('joystick')">
                            <span class="module-index">04</span>
                            <div class="module-icon-box">${getIcon('compass', 18)}</div>
                            <div class="module-info">
                                <div class="module-name">Manual Teleop</div>
                                <div class="module-desc">Kendali langsung via virtual radar joystick & tombol darurat</div>
                            </div>
                            <span class="module-arrow">${getIcon('chevronRight', 16)}</span>
                        </div>

                        <div class="module-row" onclick="navigateTo('history')">
                            <span class="module-index">05</span>
                            <div class="module-icon-box">${getIcon('clock', 18)}</div>
                            <div class="module-info">
                                <div class="module-name">Mission Logs</div>
                                <div class="module-desc">Tabel catatan waktu dan status riwayat pengiriman selesai</div>
                            </div>
                            <span class="module-arrow">${getIcon('chevronRight', 16)}</span>
                        </div>

                        <div class="module-row" onclick="navigateTo('settings')">
                            <span class="module-index">06</span>
                            <div class="module-icon-box">${getIcon('sliders', 18)}</div>
                            <div class="module-info">
                                <div class="module-name">System Configuration & Themes</div>
                                <div class="module-desc">Pilih tema warna studio, batas kecepatan, audio TTS, & waypoint</div>
                            </div>
                            <span class="module-arrow">${getIcon('chevronRight', 16)}</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        if (typeof initMonitorMap === 'function') {
            initMonitorMap('home-map-canvas');
        }
        if (typeof renderQuickDispatchChips === 'function') {
            renderQuickDispatchChips('home-dispatch-grid');
        }
    }, 50);

    return div;
}

// ====================== PAGE: DELIVERY CONTROL ======================
function renderDelivery() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="studio-two-col">
            <!-- LEFT COLUMN: Interactive SLAM Map & Active Mission Control -->
            <div>
                <div class="page-section">
                    <div class="section-title">
                        <span>01 // PETA SLAM & TITIK TUJUAN</span>
                        <span>KLIK PETA UNTUK SET TITIK</span>
                    </div>
                    <div class="map-container" id="delivery-map-container">
                        <canvas id="delivery-map-canvas"></canvas>
                        <div class="map-legend">
                            <span class="legend-pickup">Pickup</span>
                            <span class="legend-delivery">Delivery</span>
                            <span class="legend-robot">AMR-01</span>
                        </div>
                    </div>
                </div>

                <div class="page-section hidden" id="confirm-action-section"></div>

                <div class="page-section hidden" id="mission-control-section">
                    <div class="section-title">02 // KONTROL MISI AKTIF</div>
                    <div class="panel">
                        <div style="display:flex; align-items:center; justify-content:space-between; margin-bottom:8px;">
                            <span class="input-label">STATUS EKSEKUSI</span>
                            <span class="status-badge" id="mission-status-badge">IDLE</span>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill" id="mission-progress" style="width:0%"></div>
                        </div>
                        <p class="mono" style="font-size:0.72rem; color:var(--text-secondary); margin-top:6px;" id="mission-progress-text"></p>
                        <div class="btn-group mt-12">
                            <button class="btn btn-warning btn-sm" onclick="controlMission('pause')" id="btn-pause">${getIcon('pause', 14)} Pause</button>
                            <button class="btn btn-primary btn-sm" onclick="controlMission('resume')" id="btn-resume">${getIcon('play', 14)} Resume</button>
                            <button class="btn btn-danger btn-sm" onclick="controlMission('cancel')">${getIcon('x', 14)} Cancel</button>
                        </div>
                    </div>
                </div>
            </div>

            <!-- RIGHT COLUMN: Task Builder & Queue -->
            <div>
                <div class="page-section">
                    <div class="section-title">03 // KONFIGURASI TASK BARU</div>
                    <div class="panel">
                        <p style="font-size:0.76rem; color:var(--text-secondary); margin-bottom:12px;">
                            Pilih titik pickup & tujuan dari daftar waypoint atau klik langsung pada peta 2D.
                        </p>

                        <div class="mb-12">
                            <div class="input-label mb-8 icon-inline">${getIcon('pickup', 14)} Titik Pickup</div>
                            <div class="waypoint-chips" id="pickup-chips"></div>
                            <div id="pickup-selected" class="mono" style="font-size:0.75rem; color:var(--accent-bright); font-weight:600; min-height:18px;"></div>
                        </div>

                        <div class="mb-12">
                            <div class="input-label mb-8 icon-inline">${getIcon('delivery_point', 14)} Titik Delivery</div>
                            <div class="waypoint-chips" id="delivery-chips"></div>
                            <div id="delivery-selected" class="mono" style="font-size:0.75rem; color:var(--success); font-weight:600; min-height:18px;"></div>
                        </div>

                        <div style="border-top:1px solid var(--border-subtle); padding-top:12px; margin-top:8px;">
                            <div class="input-label mb-8">Input Koordinat Manual (Meter)</div>
                            <div class="input-row mb-8">
                                <div class="input-group" style="margin-bottom:0">
                                    <input class="input-field" id="manual-x" type="number" step="0.1" placeholder="Koordinat X">
                                </div>
                                <div class="input-group" style="margin-bottom:0">
                                    <input class="input-field" id="manual-y" type="number" step="0.1" placeholder="Koordinat Y">
                                </div>
                            </div>
                            <div class="btn-group mb-8">
                                <button class="btn btn-outline btn-sm" onclick="setManualAsPickup()">Set Pickup</button>
                                <button class="btn btn-outline btn-sm" onclick="setManualAsDelivery()">Set Delivery</button>
                            </div>
                        </div>

                        <button class="btn btn-primary btn-block mt-8" onclick="addTaskToQueue()">
                            ${getIcon('plus', 15)} Tambah ke Antrean
                        </button>
                    </div>
                </div>

                <div class="page-section">
                    <div class="section-title">
                        <span>04 // ANTREAN PENGIRIMAN</span>
                        <span id="task-count">${APP.taskQueue.length} TASK</span>
                    </div>
                    <div class="panel">
                        <ul class="task-list" id="task-list"></ul>
                        <div class="btn-group mt-12" id="delivery-actions">
                            <button class="btn btn-success btn-block" onclick="startDelivery()" id="btn-start-delivery">
                                ${getIcon('play', 15)} Eksekusi Misi
                            </button>
                            <button class="btn btn-outline" onclick="clearTaskQueue()">Reset</button>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        renderWaypointChips();
        renderTaskList();
        updateMissionControlVisibility();
        initDeliveryMap();
        startDeliveryPolling();
    }, 50);

    return div;
}

function renderWaypointChips() {
    const pickupContainer = document.getElementById('pickup-chips');
    const deliveryContainer = document.getElementById('delivery-chips');
    if (!pickupContainer || !deliveryContainer) return;

    pickupContainer.innerHTML = '';
    deliveryContainer.innerHTML = '';

    APP.savedPoints.forEach(p => {
        const chip = document.createElement('span');
        chip.className = `wp-chip type-${p.type}`;
        chip.textContent = p.name;

        if (p.type === 'pickup') {
            chip.onclick = () => selectPredefinedPoint(p, 'pickup');
            pickupContainer.appendChild(chip);
        } else {
            chip.onclick = () => selectPredefinedPoint(p, 'delivery');
            deliveryContainer.appendChild(chip);
        }
    });
}

function selectPredefinedPoint(point, role) {
    if (role === 'pickup') {
        APP.pendingPickup = { name: point.name, x: point.x, y: point.y };
        document.getElementById('pickup-selected').innerHTML = `${getIcon('check', 13)} ${point.name} [${point.x}, ${point.y}]`;
        document.querySelectorAll('#pickup-chips .wp-chip').forEach(c =>
            c.classList.toggle('selected', c.textContent === point.name));
    } else {
        APP.pendingDelivery = { name: point.name, x: point.x, y: point.y };
        document.getElementById('delivery-selected').innerHTML = `${getIcon('check', 13)} ${point.name} [${point.x}, ${point.y}]`;
        document.querySelectorAll('#delivery-chips .wp-chip').forEach(c =>
            c.classList.toggle('selected', c.textContent === point.name));
    }
}

function setManualAsPickup() {
    const x = parseFloat(document.getElementById('manual-x').value);
    const y = parseFloat(document.getElementById('manual-y').value);
    if (isNaN(x) || isNaN(y)) { showToast('Masukkan koordinat X dan Y', 'error'); return; }
    APP.pendingPickup = { name: `(${x}, ${y})`, x, y };
    document.getElementById('pickup-selected').innerHTML = `${getIcon('check', 13)} Pickup: [${x}, ${y}]`;
    showToast('Pickup di-set ke koordinat manual', 'info');
}

function setManualAsDelivery() {
    const x = parseFloat(document.getElementById('manual-x').value);
    const y = parseFloat(document.getElementById('manual-y').value);
    if (isNaN(x) || isNaN(y)) { showToast('Masukkan koordinat X dan Y', 'error'); return; }
    APP.pendingDelivery = { name: `(${x}, ${y})`, x, y };
    document.getElementById('delivery-selected').innerHTML = `${getIcon('check', 13)} Delivery: [${x}, ${y}]`;
    showToast('Delivery di-set ke koordinat manual', 'info');
}

function addTaskToQueue() {
    if (!APP.pendingPickup) { showToast('Pilih titik pickup terlebih dahulu', 'error'); return; }
    if (!APP.pendingDelivery) { showToast('Pilih titik delivery terlebih dahulu', 'error'); return; }

    APP.taskQueue.push({
        pickup: { ...APP.pendingPickup },
        delivery: { ...APP.pendingDelivery }
    });

    APP.pendingPickup = null;
    APP.pendingDelivery = null;

    document.getElementById('pickup-selected').textContent = '';
    document.getElementById('delivery-selected').textContent = '';
    document.querySelectorAll('.wp-chip').forEach(c => c.classList.remove('selected'));

    renderTaskList();
    showToast(`Task ditambahkan (${APP.taskQueue.length} dalam antrean)`, 'success');
}

function removeTask(index) {
    APP.taskQueue.splice(index, 1);
    renderTaskList();
}

function clearTaskQueue() {
    APP.taskQueue = [];
    renderTaskList();
    showToast('Antrean dikosongkan', 'info');
}

function renderTaskList() {
    const list = document.getElementById('task-list');
    const countEl = document.getElementById('task-count');
    if (!list) return;
    if (countEl) countEl.textContent = `${APP.taskQueue.length} TASK`;

    if (APP.taskQueue.length === 0) {
        list.innerHTML = '<li class="history-empty">Antrean kosong. Tambahkan pasangan titik pickup & delivery.</li>';
        return;
    }

    list.innerHTML = APP.taskQueue.map((task, i) => `
        <li class="task-item">
            <span class="task-number">${String(i + 1).padStart(2, '0')}</span>
            <div class="task-info">
                <div class="task-route">${task.pickup.name} → ${task.delivery.name}</div>
                <div class="task-status-text">QUEUED // MENUNGGU EKSEKUSI</div>
            </div>
            <button class="task-remove" onclick="removeTask(${i})">${getIcon('x', 14)}</button>
        </li>
    `).join('');
}

async function startDelivery() {
    if (APP.taskQueue.length === 0) {
        showToast('Tambahkan minimal 1 task ke antrean', 'error');
        return;
    }

    showModal(
        'Eksekusi Misi Pengiriman?',
        `<p>Unit AMR-01 akan mengeksekusi <strong>${APP.taskQueue.length} task</strong> secara berurutan.</p>
         <p style="margin-top:8px; font-size:0.75rem; color:var(--text-muted);">
         Pastikan jalur navigasi bebas dari hambatan statis sebelum memulai.</p>`,
        [
            { label: 'Batal', class: 'btn-outline' },
            {
                label: 'Mulai Sekarang',
                class: 'btn-success',
                action: async () => {
                    const result = await apiPost('/api/delivery/start', { tasks: APP.taskQueue });
                    if (result && result.success) {
                        showToast('Misi pengiriman dimulai!', 'success');
                        updateMissionControlVisibility();
                    } else {
                        showToast('Gagal memulai misi', 'error');
                    }
                }
            }
        ]
    );
}

async function controlMission(command) {
    const result = await apiPost('/api/delivery/control', { command });
    if (result && result.success) {
        const labels = { pause: 'Misi dijeda', resume: 'Misi dilanjutkan', cancel: 'Misi dibatalkan', skip: 'Task dilewati' };
        showToast(labels[command] || command, command === 'cancel' ? 'error' : 'info');
        if (command === 'cancel') {
            _modalDismissedForCurrentPhase = false;
            hideArrivalModal();
            hideGlobalMissionBanner();
        }
    }
}

function updateMissionControlVisibility() {
    const section = document.getElementById('mission-control-section');
    if (!section) return;
    const isActive = ['running', 'paused'].includes(APP.delivery.status);
    section.classList.toggle('hidden', !isActive);
}

let _lastNotifiedPhase = null;
let _modalDismissedForCurrentPhase = false;

// ====================== GLOBAL DELIVERY & ARRIVAL HANDLER ======================
function handleDeliveryStatusUpdate(d) {
    if (!d) return;
    APP.delivery = d;

    const isWaiting = (d.current_phase === 'waiting_pickup' || d.current_phase === 'waiting_delivery') && d.status === 'running';

    if (isWaiting) {
        const isNewPhase = (_lastNotifiedPhase !== d.current_phase);
        if (isNewPhase) {
            _lastNotifiedPhase = d.current_phase;
            _modalDismissedForCurrentPhase = false;
            notifyArrival(d.current_phase, d);
            if (APP.currentPage !== 'delivery') {
                showArrivalModal(d);
            }
        }

        if (APP.currentPage !== 'delivery') {
            updateGlobalMissionBanner(d);
            if (!_modalDismissedForCurrentPhase) {
                showArrivalModal(d);
            }
        } else {
            hideGlobalMissionBanner();
            hideArrivalModal();
        }

        if (APP.currentPage === 'delivery') {
            updateInPageConfirmCard(d);
        }
    } else {
        _lastNotifiedPhase = null;
        _modalDismissedForCurrentPhase = false;
        hideArrivalModal();
        hideGlobalMissionBanner();
        if (APP.currentPage === 'delivery') {
            const sec = document.getElementById('confirm-action-section');
            if (sec) {
                sec.classList.add('hidden');
                sec.innerHTML = '';
            }
        }
    }
}

function showArrivalModal(d) {
    const overlay = document.getElementById('arrival-modal-overlay');
    const box = document.getElementById('arrival-modal-box');
    const iconEl = document.getElementById('arrival-modal-icon');
    const titleEl = document.getElementById('arrival-modal-title');
    const descEl = document.getElementById('arrival-modal-desc');
    const actionsEl = document.getElementById('arrival-modal-actions');
    if (!overlay || !box) return;

    const isPickup = (d.current_phase === 'waiting_pickup');
    const taskIdx = d.current_task_index || 0;
    const taskInfo = d.tasks && d.tasks[taskIdx] ? d.tasks[taskIdx] : null;
    const pointName = isPickup
        ? (taskInfo && taskInfo.pickup ? taskInfo.pickup.name : 'Titik Pickup')
        : (taskInfo && taskInfo.delivery ? taskInfo.delivery.name : 'Titik Tujuan');

    box.className = isPickup ? 'arrival-box-pickup' : 'arrival-box-delivery';
    if (iconEl) iconEl.innerHTML = isPickup ? getIcon('box', 28) : getIcon('checkCircle', 28);

    if (titleEl) {
        titleEl.textContent = isPickup
            ? `Tiba di Pickup: ${pointName}`
            : `Tiba di Tujuan: ${pointName}`;
    }

    if (descEl) {
        descEl.textContent = isPickup
            ? 'Unit AMR-01 telah tiba di titik pickup dan siap dimuat. Muat barang ke kompartemen lalu tekan konfirmasi.'
            : 'Unit AMR-01 telah tiba di titik tujuan. Silakan ambil barang lalu tekan konfirmasi penyelesaian.';
    }

    const cmd = isPickup ? 'confirm_pickup' : 'confirm_delivery';
    const btnLabel = isPickup ? 'Konfirmasi Barang Dimuat' : 'Konfirmasi Barang Diambil';
    const btnClass = isPickup ? 'btn-confirm-pickup' : 'btn-confirm-delivery';

    if (actionsEl) {
        actionsEl.innerHTML = `
            <button class="btn btn-confirm ${btnClass}" onclick="confirmActionFromModal('${cmd}')">
                ${getIcon('check', 18)} ${btnLabel}
            </button>
            <button class="btn btn-secondary" onclick="goToDeliveryPageFromModal()">
                ${getIcon('delivery', 16)} Buka Halaman Dispatch
            </button>
            <button class="btn btn-outline btn-sm" onclick="dismissArrivalModal()">
                ${getIcon('x', 14)} Sembunyikan Sementara
            </button>
        `;
    }

    overlay.classList.remove('hidden');
}

function hideArrivalModal() {
    const overlay = document.getElementById('arrival-modal-overlay');
    if (overlay) overlay.classList.add('hidden');
}

function dismissArrivalModal() {
    _modalDismissedForCurrentPhase = true;
    hideArrivalModal();
}

function closeArrivalModalOnBackdrop(event) {
    if (event.target.id === 'arrival-modal-overlay') {
        dismissArrivalModal();
    }
}

function updateGlobalMissionBanner(d) {
    const banner = document.getElementById('global-mission-banner');
    const iconEl = document.getElementById('banner-icon');
    const titleEl = document.getElementById('banner-title');
    const subtitleEl = document.getElementById('banner-subtitle');
    const btnEl = document.getElementById('banner-btn');
    if (!banner) return;

    const isPickup = (d.current_phase === 'waiting_pickup');
    const taskIdx = d.current_task_index || 0;
    const taskInfo = d.tasks && d.tasks[taskIdx] ? d.tasks[taskIdx] : null;
    const pointName = isPickup
        ? (taskInfo && taskInfo.pickup ? taskInfo.pickup.name : 'Pickup')
        : (taskInfo && taskInfo.delivery ? taskInfo.delivery.name : 'Delivery');

    banner.className = isPickup ? 'banner-pickup' : 'banner-delivery';
    if (iconEl) iconEl.innerHTML = isPickup ? getIcon('box', 20) : getIcon('checkCircle', 20);
    if (titleEl) titleEl.textContent = `AMR-01 Tiba di ${pointName}`;
    if (subtitleEl) subtitleEl.textContent = isPickup ? 'WAITING_PICKUP // Konfirmasi muat barang' : 'WAITING_DELIVERY // Konfirmasi ambil barang';
    if (btnEl) {
        btnEl.innerHTML = `${getIcon('check', 14)} ${isPickup ? 'Konfirmasi Muat' : 'Konfirmasi Ambil'}`;
    }

    banner.classList.remove('hidden');
}

function hideGlobalMissionBanner() {
    const banner = document.getElementById('global-mission-banner');
    if (banner) banner.classList.add('hidden');
}

function onBannerClick() {
    if (APP.delivery && (APP.delivery.current_phase === 'waiting_pickup' || APP.delivery.current_phase === 'waiting_delivery')) {
        _modalDismissedForCurrentPhase = false;
        showArrivalModal(APP.delivery);
    }
}

async function onBannerBtnClick() {
    if (APP.delivery && APP.delivery.status === 'running') {
        const cmd = APP.delivery.current_phase === 'waiting_pickup' ? 'confirm_pickup' : 'confirm_delivery';
        await confirmActionFromModal(cmd);
    }
}

async function confirmActionFromModal(command) {
    hideArrivalModal();
    hideGlobalMissionBanner();
    _modalDismissedForCurrentPhase = false;
    await confirmAction(command);
}

function goToDeliveryPageFromModal() {
    hideArrivalModal();
    navigateTo('delivery');
}

function updateInPageConfirmCard(d) {
    const confirmSection = document.getElementById('confirm-action-section');
    if (!confirmSection) return;

    const isPickup = (d.current_phase === 'waiting_pickup');
    const taskIdx = d.current_task_index || 0;
    const taskInfo = d.tasks && d.tasks[taskIdx] ? d.tasks[taskIdx] : null;

    if (isPickup) {
        const pickupName = taskInfo && taskInfo.pickup ? taskInfo.pickup.name : 'Pickup';
        confirmSection.classList.remove('hidden');
        confirmSection.innerHTML = `
            <div class="confirm-card confirm-pickup">
                <div class="confirm-icon">${getIcon('box', 32)}</div>
                <div class="confirm-title">AMR-01 Tiba di ${pickupName}</div>
                <div class="confirm-desc">Silakan muat barang ke unit robot, kemudian tekan tombol konfirmasi di bawah untuk melanjutkan rute.</div>
                <button class="btn btn-confirm btn-confirm-pickup" onclick="confirmAction('confirm_pickup')">
                    ${getIcon('check', 18)} Konfirmasi Barang Dimuat
                </button>
            </div>
        `;
    } else {
        const deliveryName = taskInfo && taskInfo.delivery ? taskInfo.delivery.name : 'Delivery';
        confirmSection.classList.remove('hidden');
        confirmSection.innerHTML = `
            <div class="confirm-card confirm-delivery">
                <div class="confirm-icon">${getIcon('checkCircle', 32)}</div>
                <div class="confirm-title">AMR-01 Tiba di ${deliveryName}</div>
                <div class="confirm-desc">Silakan ambil barang dari unit robot, kemudian tekan tombol konfirmasi untuk menyelesaikan task.</div>
                <button class="btn btn-confirm btn-confirm-delivery" onclick="confirmAction('confirm_delivery')">
                    ${getIcon('check', 18)} Konfirmasi Barang Diambil
                </button>
            </div>
        `;
    }
}

function updateMissionUI() {
    const badge = document.getElementById('mission-status-badge');
    const progress = document.getElementById('mission-progress');
    const progressText = document.getElementById('mission-progress-text');

    if (!badge) return;

    const d = APP.delivery;
    badge.textContent = (d.status || 'idle').toUpperCase();
    badge.className = `status-badge status-${d.status || 'idle'}`;

    if (d.total_tasks > 0) {
        const pct = Math.round((d.completed_tasks / d.total_tasks) * 100);
        if (progress) progress.style.width = `${pct}%`;

        const phaseLabels = {
            navigating_pickup: 'Menuju titik pickup...',
            at_pickup: 'Tiba di pickup',
            waiting_pickup: 'Menunggu barang dimuat',
            navigating_delivery: 'Mengantar barang...',
            at_delivery: 'Tiba di delivery',
            waiting_delivery: 'Menunggu barang diambil',
            returning_home: 'Kembali ke titik awal...',
            idle: 'Selesai'
        };
        if (progressText) {
            progressText.textContent = `TASK ${(d.current_task_index || 0) + 1}/${d.total_tasks} // ${phaseLabels[d.current_phase] || d.current_phase}`;
        }
    }

    handleDeliveryStatusUpdate(d);
    updateMissionControlVisibility();
}

// ====================== AUDIO & NOTIFICATION SYSTEM ======================
function getSoundMode() {
    return localStorage.getItem('amr_sound_mode') || (APP.settings && APP.settings.arrival_sound_mode) || 'human';
}

function setSoundMode(mode) {
    localStorage.setItem('amr_sound_mode', mode);
    if (APP.settings) {
        APP.settings.arrival_sound_mode = mode;
    }
}

function unlockAudioContext() {
    try {
        const AudioContextClass = window.AudioContext || window.webkitAudioContext;
        if (AudioContextClass) {
            const tempCtx = new AudioContextClass();
            if (tempCtx.state === 'suspended') {
                tempCtx.resume();
            }
        }
        if ('speechSynthesis' in window) {
            window.speechSynthesis.getVoices();
        }
    } catch (e) {}
    document.removeEventListener('click', unlockAudioContext);
    document.removeEventListener('touchstart', unlockAudioContext);
}
document.addEventListener('click', unlockAudioContext);
document.addEventListener('touchstart', unlockAudioContext);

function playChimeSound(type = 'waiting_pickup') {
    try {
        const AudioContextClass = window.AudioContext || window.webkitAudioContext;
        if (!AudioContextClass) return;
        const ctx = new AudioContextClass();
        if (ctx.state === 'suspended') {
            ctx.resume();
        }

        const now = ctx.currentTime;
        const notes = type === 'waiting_pickup'
            ? [659.25, 880.0]
            : [523.25, 783.99, 1046.5];

        notes.forEach((freq, idx) => {
            const osc = ctx.createOscillator();
            const gain = ctx.createGain();
            osc.type = 'sine';
            osc.frequency.setValueAtTime(freq, now + idx * 0.16);

            gain.gain.setValueAtTime(0.001, now + idx * 0.16);
            gain.gain.linearRampToValueAtTime(0.25, now + idx * 0.16 + 0.03);
            gain.gain.exponentialRampToValueAtTime(0.001, now + idx * 0.16 + 0.45);

            osc.connect(gain);
            gain.connect(ctx.destination);

            osc.start(now + idx * 0.16);
            osc.stop(now + idx * 0.16 + 0.5);
        });
    } catch (e) {
        console.warn('AudioContext error:', e);
    }
}

let _currentAudio = null;

function speakHumanVoice(text) {
    if (!text) return;

    try {
        if (_currentAudio) {
            _currentAudio.pause();
            _currentAudio.currentTime = 0;
            _currentAudio = null;
        }

        const audioUrl = `/api/tts?text=${encodeURIComponent(text)}`;
        const audio = new Audio(audioUrl);
        _currentAudio = audio;

        audio.play().catch(err => {
            console.warn('Gagal memutar audio /api/tts, mencoba fallback Web Speech:', err);
            fallbackWebSpeech(text);
        });
    } catch (e) {
        console.warn('Error saat memutar audio TTS:', e);
        fallbackWebSpeech(text);
    }
}

function fallbackWebSpeech(text) {
    if (!('speechSynthesis' in window)) return;
    try {
        window.speechSynthesis.cancel();
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.lang = 'id-ID';
        utterance.rate = 0.95;
        const voices = window.speechSynthesis.getVoices();
        const idVoice = voices.find(v => v.lang === 'id-ID' || v.lang.startsWith('id'));
        if (idVoice) utterance.voice = idVoice;
        window.speechSynthesis.speak(utterance);
    } catch (e) {}
}

if ('speechSynthesis' in window) {
    window.speechSynthesis.onvoiceschanged = () => {
        try { window.speechSynthesis.getVoices(); } catch (e) {}
    };
}

function notifyArrival(phase, deliveryData) {
    if (navigator.vibrate) {
        navigator.vibrate([200, 100, 200, 100, 200]);
    }

    const d = deliveryData || APP.delivery;
    const isPickup = (phase === 'waiting_pickup');
    const taskIdx = (d && d.current_task_index) || 0;
    const taskInfo = (d && d.tasks && d.tasks[taskIdx]) ? d.tasks[taskIdx] : null;
    const pointName = isPickup
        ? (taskInfo && taskInfo.pickup ? taskInfo.pickup.name : '')
        : (taskInfo && taskInfo.delivery ? taskInfo.delivery.name : '');

    const speechText = isPickup
        ? (pointName ? `Robot sudah tiba di titik pickup ${pointName}. Silakan muat barang.` : 'Robot sudah sampai di titik pickup. Silakan muat barang.')
        : (pointName ? `Robot sudah tiba di titik tujuan ${pointName}. Silakan ambil barang.` : 'Robot sudah sampai di titik tujuan. Silakan ambil barang.');

    const toastLabel = isPickup
        ? (pointName ? `Tiba di pickup: ${pointName}. Silakan muat barang.` : 'Tiba di pickup. Silakan muat barang.')
        : (pointName ? `Tiba di tujuan: ${pointName}. Silakan ambil barang.` : 'Tiba di tujuan. Silakan ambil barang.');

    showToast(toastLabel, 'success');

    const mode = getSoundMode();
    if (mode === 'mute') {
        return;
    } else if (mode === 'chime') {
        playChimeSound(phase);
    } else if (mode === 'human') {
        speakHumanVoice(speechText);
    } else if (mode === 'both') {
        playChimeSound(phase);
        setTimeout(() => {
            speakHumanVoice(speechText);
        }, 550);
    }
}

function testArrivalSound(phase) {
    const fakeData = {
        current_task_index: 0,
        tasks: [{
            pickup: { name: 'Rak 1A' },
            delivery: { name: 'Meja 2' }
        }]
    };
    notifyArrival(phase, fakeData);
}

async function confirmAction(command) {
    const result = await apiPost('/api/delivery/control', { command });
    if (result && result.success) {
        const label = command === 'confirm_pickup' ? 'Pickup dikonfirmasi! Melanjutkan rute.' : 'Delivery dikonfirmasi! Task selesai.';
        showToast(label, 'success');
        _modalDismissedForCurrentPhase = false;
        hideArrivalModal();
        hideGlobalMissionBanner();
        const sec = document.getElementById('confirm-action-section');
        if (sec) {
            sec.classList.add('hidden');
            sec.innerHTML = '';
        }
        const data = await apiGet('/api/status');
        if (data && data.delivery) {
            handleDeliveryStatusUpdate(data.delivery);
            if (APP.currentPage === 'delivery') {
                updateMissionUI();
            }
        }
    } else {
        showToast('Gagal mengirim konfirmasi', 'error');
    }
}

function startDeliveryPolling() {
    APP.statusPollTimer = setInterval(async () => {
        const data = await apiGet('/api/status');
        if (data) {
            APP.delivery = data.delivery || APP.delivery;
            updateMissionUI();
        }
    }, 1000);
}

window.onBannerClick = onBannerClick;
window.onBannerBtnClick = onBannerBtnClick;
window.closeArrivalModalOnBackdrop = closeArrivalModalOnBackdrop;
window.confirmActionFromModal = confirmActionFromModal;
window.goToDeliveryPageFromModal = goToDeliveryPageFromModal;
window.dismissArrivalModal = dismissArrivalModal;
window.hideArrivalModal = hideArrivalModal;
window.handleDeliveryStatusUpdate = handleDeliveryStatusUpdate;

// ====================== PAGE: MONITORING (COCKPIT) ======================
function renderMonitoring() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="cockpit-header">
            <div>
                <div class="studio-title-tag">${getIcon('activity', 13)} SYS // TELEMETRY COCKPIT</div>
                <h1 class="cockpit-header-title">Cockpit Telemetry & Visual Stream</h1>
                <p class="cockpit-header-subtitle">Pemantauan optik, peta SLAM, kinematika kecepatan, dan kontrol misi real-time</p>
            </div>
            <div style="display:flex; align-items:center; gap:10px;">
                <span class="status-badge status-idle" id="cockpit-status-badge">STANDBY</span>
            </div>
        </div>

        <div class="cockpit-grid">
            <!-- LEFT COLUMN: VIEWPORT & LOWER ROW -->
            <div style="display:flex; flex-direction:column; gap:16px;">
                <!-- Card 1: Visual Monitoring (Hero Widget) -->
                <div class="cockpit-card viewport-card">
                    <div class="cockpit-card-header">
                        <div class="cockpit-card-title">
                            ${getIcon('video', 16)} 01 // VISUAL VIEWPORT
                        </div>
                        <div class="viewport-tabs">
                            <button class="viewport-tab-btn active" data-mode="camera" onclick="switchViewportMode('camera')">
                                ${getIcon('video', 13)} Kamera
                            </button>
                            <button class="viewport-tab-btn" data-mode="map" onclick="switchViewportMode('map')">
                                ${getIcon('map', 13)} Peta 2D
                            </button>
                            <button class="viewport-tab-btn" data-mode="dual" onclick="switchViewportMode('dual')">
                                ${getIcon('layers', 13)} Split View
                            </button>
                        </div>
                    </div>

                    <!-- Viewport Container -->
                    <div class="viewport-display" id="cockpit-viewport">
                        <!-- View 1: Camera Live -->
                        <div class="viewport-view active" id="view-camera">
                            <img id="cockpit-cam-img" class="cockpit-cam-img" src="/api/camera/stream" alt="Live Kamera"
                                 onerror="this.style.display='none'; document.getElementById('cockpit-cam-offline').style.display='flex';">
                            <div id="cockpit-cam-offline" class="flex-center mono" style="display:none; width:100%; height:100%; color:var(--text-muted); font-size:0.78rem; flex-direction:column; gap:8px;">
                                ${getIcon('video', 32)}
                                STREAM KAMERA OFFLINE
                            </div>
                            <div class="viewport-hud">
                                <div class="viewport-hud-left">
                                    <span class="hud-badge hud-badge-rec"><span class="rec-dot"></span>CAM_01 // LIVE</span>
                                    <span class="hud-badge hud-badge-human" id="hud-human-badge">AI: AMAN</span>
                                </div>
                            </div>
                            <div class="viewport-tools">
                                <button class="tool-btn" onclick="captureScreenshot()" title="Ambil Screenshot">${getIcon('camera', 15)}</button>
                            </div>
                        </div>

                        <!-- View 2: 2D Map -->
                        <div class="viewport-view" id="view-map">
                            <div class="map-container" style="width:100%; height:100%; position:relative; border:none;">
                                <canvas id="monitor-map-canvas"></canvas>
                                <div class="map-legend">
                                    <span class="legend-pickup">Pickup</span>
                                    <span class="legend-delivery">Delivery</span>
                                    <span class="legend-robot">AMR-01</span>
                                </div>
                            </div>
                            <div class="viewport-tools">
                                <button class="tool-btn" onclick="resetMapCenter()" title="Pusatkan ke Robot">${getIcon('crosshair', 15)}</button>
                            </div>
                        </div>

                        <!-- View 3: Dual Split (Kamera + Map side by side) -->
                        <div class="viewport-split" id="view-split">
                            <div class="viewport-split-pane">
                                <img id="split-cam-img" class="cockpit-cam-img" src="/api/camera/stream" alt="Live Kamera">
                                <div class="viewport-hud">
                                    <span class="hud-badge hud-badge-rec"><span class="rec-dot"></span>CAM_01</span>
                                </div>
                            </div>
                            <div class="viewport-split-pane">
                                <div class="map-container" style="width:100%; height:100%; position:relative; border:none;">
                                    <canvas id="split-map-canvas"></canvas>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Lower Row: Telemetry Chart & Quick Dispatch -->
                <div class="cockpit-lower-grid">
                    <!-- Card 3: Telemetri Kecepatan -->
                    <div class="cockpit-card">
                        <div class="cockpit-card-header">
                            <div class="cockpit-card-title">
                                ${getIcon('activity', 16)} 02 // VELOCITY PROFILE
                            </div>
                            <span class="mono" style="font-size:0.65rem; color:var(--text-muted);">30 SAMPLES</span>
                        </div>
                        <div class="telemetry-chart-container">
                            <canvas id="speed-history-canvas"></canvas>
                        </div>
                        <div class="telemetry-meta">
                            <div>AVG: <b id="tele-avg-speed">0.00 m/s</b></div>
                            <div>ODO: <b id="tele-odometer">0.0 m</b></div>
                            <div>TIME: <b id="tele-elapsed">0s</b></div>
                        </div>
                    </div>

                    <!-- Card 4: Quick Dispatch Titik Meja -->
                    <div class="cockpit-card">
                        <div class="cockpit-card-header">
                            <div class="cockpit-card-title">
                                ${getIcon('delivery_point', 16)} 03 // QUICK DISPATCH
                            </div>
                            <span class="mono" style="font-size:0.65rem; color:var(--accent-bright);">1-CLICK</span>
                        </div>
                        <div class="quick-dispatch-grid" id="cockpit-dispatch-grid"></div>
                    </div>
                </div>
            </div>

            <!-- RIGHT COLUMN: SPEEDOMETER, MISSION STATUS, SAFETY -->
            <div style="display:flex; flex-direction:column; gap:16px;">
                <!-- Card 2: Kecepatan & Pose Robot -->
                <div class="cockpit-card gauge-card">
                    <div class="cockpit-card-header" style="width:100%;">
                        <div class="cockpit-card-title">
                            ${getIcon('gauge', 16)} 04 // KINEMATICS & POSE
                        </div>
                        <span class="mono" style="font-size:0.65rem; color:var(--text-muted);" id="speed-limit-tag">MAX 0.5 m/s</span>
                    </div>

                    <!-- Speedometer Arc Gauge -->
                    <div class="speed-gauge-wrap">
                        <svg class="speed-gauge-svg" viewBox="0 0 220 140">
                            <path d="M 25 115 A 85 85 0 0 1 195 115" fill="none" stroke="var(--border)" stroke-width="10" stroke-linecap="round"/>
                            <path id="cockpit-gauge-path" d="M 25 115 A 85 85 0 0 1 195 115" fill="none" stroke="var(--accent)" stroke-width="10" stroke-linecap="round"
                                  stroke-dasharray="267" stroke-dashoffset="267" style="transition: stroke-dashoffset 0.2s ease, stroke 0.25s ease;"/>
                        </svg>
                        <div class="gauge-readout">
                            <div class="gauge-val" id="cockpit-speed-val">0.00</div>
                            <div class="gauge-subtext">LINEAR VELOCITY (m/s)</div>
                            <div class="angular-pill" id="cockpit-angular-val">
                                ${getIcon('compass', 12)} 0.00 rad/s
                            </div>
                        </div>
                    </div>

                    <!-- Coordinates Grid -->
                    <div class="cockpit-pose-grid">
                        <div class="pose-cell">
                            <div class="pose-cell-label">POS X</div>
                            <div class="pose-cell-val pose-cell-accent" id="mon-x">0.000</div>
                        </div>
                        <div class="pose-cell">
                            <div class="pose-cell-label">POS Y</div>
                            <div class="pose-cell-val pose-cell-accent" id="mon-y">0.000</div>
                        </div>
                        <div class="pose-cell">
                            <div class="pose-cell-label">YAW</div>
                            <div class="pose-cell-val pose-cell-warning" id="mon-yaw">0.0°</div>
                        </div>
                    </div>

                    <!-- Distance status -->
                    <div class="distance-bar">
                        <div class="dist-item">TARGET: <span id="mon-dist-target">—</span></div>
                        <div class="dist-item">HOME: <span id="mon-dist-home">—</span></div>
                    </div>
                </div>

                <!-- Card 5: Misi Aktif & Kontrol Cepat -->
                <div class="cockpit-card">
                    <div class="cockpit-card-header">
                        <div class="cockpit-card-title">
                            ${getIcon('delivery', 16)} 05 // ACTIVE MISSION
                        </div>
                        <span class="status-badge status-idle" id="cockpit-mission-badge">IDLE</span>
                    </div>
                    <div class="mission-mini-card">
                        <div class="mission-phase-badge" id="cockpit-phase-badge">
                            <span>PHASE</span>
                            <b id="cockpit-phase-text">Standby</b>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill" id="cockpit-progress-fill" style="width:0%"></div>
                        </div>
                        <div class="mono" style="display:flex; justify-content:space-between; font-size:0.68rem; color:var(--text-muted);">
                            <span id="cockpit-task-counter">QUEUE: 0</span>
                            <span class="text-accent" id="cockpit-target-info">—</span>
                        </div>

                        <div id="cockpit-confirm-btn-wrap" class="hidden" style="margin-top:4px;">
                            <button id="cockpit-confirm-btn" class="btn btn-confirm btn-block" onclick="onCockpitConfirmArrival()">
                                ${getIcon('check', 16)} Konfirmasi Kedatangan
                            </button>
                        </div>

                        <div class="cockpit-ctrl-row">
                            <button class="btn btn-warning" onclick="controlMission('pause')">${getIcon('pause', 13)} Pause</button>
                            <button class="btn btn-primary" onclick="controlMission('resume')">${getIcon('play', 13)} Resume</button>
                            <button class="btn btn-danger" onclick="controlMission('cancel')">${getIcon('x', 13)} Batal</button>
                        </div>
                    </div>
                </div>

                <!-- Card 6: Sistem Keamanan & AI Vision -->
                <div class="cockpit-card">
                    <div class="cockpit-card-header">
                        <div class="cockpit-card-title">
                            ${getIcon('shield', 16)} 06 // SUBSYSTEM HEALTH
                        </div>
                        <span class="mono" style="font-size:0.65rem; color:var(--success);">NOMINAL</span>
                    </div>
                    <div class="nodes-grid">
                        <div class="node-item">
                            <span class="node-name">AI_VISION</span>
                            <span class="node-status" id="cockpit-human-status" style="color:var(--success);">Aman</span>
                        </div>
                        <div class="node-item">
                            <span class="node-name">LIDAR_SCAN</span>
                            <span class="node-status node-status-ok" id="cockpit-lidar-status">OK</span>
                        </div>
                        <div class="node-item">
                            <span class="node-name">NAV2_STACK</span>
                            <span class="node-status node-status-ok">ACTIVE</span>
                        </div>
                        <div class="node-item">
                            <span class="node-name">E_STOP_BUS</span>
                            <span class="node-status text-accent">ARMED</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        if (typeof initCockpit === 'function') {
            initCockpit();
        }
        startMonitoringPolling();
    }, 50);

    return div;
}

// ====================== PAGE: CAMERA ======================
function renderCamera() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="studio-two-col">
            <div class="page-section">
                <div class="section-title">
                    <span>01 // OPTICAL CAMERA FEED</span>
                    <span>/api/camera/stream</span>
                </div>
                <div class="camera-frame">
                    <img id="camera-feed" src="/api/camera/stream" alt="Kamera AMR"
                         onerror="this.style.display='none'; document.getElementById('cam-offline').style.display='flex';">
                    <div class="camera-overlay">
                        <div class="camera-badge"><span class="rec-dot"></span>CAM_01 // LIVE</div>
                    </div>
                    <div id="cam-offline" class="flex-center mono" style="display:none; min-height:280px; color:var(--text-muted); font-size:0.8rem; flex-direction:column; gap:8px;">
                        ${getIcon('video', 32)}
                        STREAM KAMERA TIDAK TERSEDIA
                    </div>
                </div>
            </div>

            <div>
                <div class="page-section">
                    <div class="section-title">02 // VISION & POSE READOUT</div>
                    <div class="panel">
                        <div class="monitor-grid">
                            <div class="monitor-card">
                                <div class="mon-label">Human AI Status</div>
                                <div class="mon-value mon-success" id="cam-human">NO_HUMAN</div>
                            </div>
                            <div class="monitor-card">
                                <div class="mon-label">Koordinat (X, Y)</div>
                                <div class="mon-value mon-accent" id="cam-pos">${APP.pose.x.toFixed(2)}, ${APP.pose.y.toFixed(2)}</div>
                            </div>
                        </div>
                        <button class="btn btn-outline btn-block mt-12" onclick="captureScreenshot()">
                            ${getIcon('camera', 16)} Ambil Snapshot Frame
                        </button>
                    </div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        startCameraPolling();
    }, 50);

    return div;
}

function captureScreenshot() {
    window.open('/api/camera/snapshot', '_blank');
    showToast('Snapshot frame dibuka di tab baru', 'success');
}

function startCameraPolling() {
    APP.statusPollTimer = setInterval(async () => {
        const data = await apiGet('/api/status');
        if (data) {
            if (data.delivery && typeof handleDeliveryStatusUpdate === 'function') {
                handleDeliveryStatusUpdate(data.delivery);
            }
            const camHuman = document.getElementById('cam-human');
            const camPos = document.getElementById('cam-pos');
            if (camHuman) camHuman.textContent = data.human_status || '—';
            if (camPos && data.pose) camPos.textContent = `${data.pose.x.toFixed(2)}, ${data.pose.y.toFixed(2)}`;
        }
    }, 1000);
}

// ====================== PAGE: HISTORY ======================
function renderHistory() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="studio-overview-header">
            <div>
                <div class="studio-title-tag">${getIcon('clock', 13)} SYS // MISSION LOGS</div>
                <h1>Riwayat Pengiriman</h1>
                <p>Log eksekusi tugas pengiriman unit AMR-01 beserta status penyelesaian</p>
            </div>
            <button class="btn btn-danger btn-sm" onclick="clearHistory()">
                ${getIcon('trash', 14)} Bersihkan Log
            </button>
        </div>

        <div class="page-section">
            <div class="panel" style="padding:0; overflow:hidden;" id="history-content">
                <div class="history-empty">Memuat log pengiriman...</div>
            </div>
        </div>
    `;

    setTimeout(loadHistory, 50);
    return div;
}

async function loadHistory() {
    const container = document.getElementById('history-content');
    if (!container) return;

    const data = await apiGet('/api/history');
    if (!data || data.length === 0) {
        container.innerHTML = '<div class="history-empty">BELUM ADA DATA LOG PENGIRIMAN</div>';
        return;
    }

    container.innerHTML = `
        <div style="overflow-x:auto;">
        <table class="history-table">
            <thead>
                <tr>
                    <th>TIMESTAMP</th>
                    <th>PICKUP ORIGIN</th>
                    <th>DELIVERY TARGET</th>
                    <th>STATUS</th>
                </tr>
            </thead>
            <tbody>
                ${data.map(r => {
                    const time = r.timestamp ? new Date(r.timestamp).toLocaleString('id-ID', {
                        day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit'
                    }) : '-';
                    const statusClass = r.status === 'completed' ? 'status-completed' :
                                       r.status === 'failed' ? 'status-failed' : 'status-paused';
                    return `<tr>
                        <td class="mono">${time}</td>
                        <td class="mono" style="color:var(--text-primary); font-weight:600;">${r.pickup_name}</td>
                        <td class="mono" style="color:var(--text-primary); font-weight:600;">${r.delivery_name}</td>
                        <td><span class="status-badge ${statusClass}">${r.status}</span></td>
                    </tr>`;
                }).join('')}
            </tbody>
        </table>
        </div>
    `;
}

async function clearHistory() {
    showModal('Bersihkan Log Riwayat?', 'Seluruh catatan riwayat pengiriman akan dihapus secara permanen.', [
        { label: 'Batal', class: 'btn-outline' },
        {
            label: 'Hapus Semua', class: 'btn-danger',
            action: async () => {
                await apiPost('/api/history/clear', {});
                loadHistory();
                showToast('Log riwayat dibersihkan', 'info');
            }
        }
    ]);
}

// ====================== PAGE: JOYSTICK (MANUAL TELEOP) ======================
function renderJoystick() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="studio-two-col">
            <div class="page-section">
                <div class="section-title">
                    <span>01 // FORWARD OPTICAL STREAM</span>
                    <span>LIVE TELEOP VIEW</span>
                </div>
                <div class="camera-frame">
                    <img src="/api/camera/stream" alt="Kamera AMR"
                         onerror="this.style.display='none'; this.nextElementSibling.style.display='none'; this.parentElement.querySelector('.teleop-cam-offline').style.display='flex';">
                    <div class="camera-overlay">
                        <div class="camera-badge"><span class="rec-dot"></span>TELEOP_CAM // LIVE</div>
                    </div>
                    <div class="teleop-cam-offline flex-center mono" style="display:none; min-height:240px; color:var(--text-muted); font-size:0.78rem; flex-direction:column; gap:8px;">
                        ${getIcon('video', 28)} STREAM KAMERA TIDAK TERSEDIA
                    </div>
                </div>
            </div>

            <div class="page-section">
                <div class="section-title">
                    <span>02 // MANUAL VECTOR CONTROLLER</span>
                    <span>/cmd_vel</span>
                </div>
                <div class="panel">
                    <div class="joystick-container">
                        <div class="joystick-canvas-wrap">
                            <canvas id="joystick-canvas" width="240" height="240"></canvas>
                        </div>
                        <div class="joystick-info">
                            <div class="vel-display">
                                <div class="vel-label">LINEAR X (m/s)</div>
                                <div class="vel-value" id="vel-linear">0.00</div>
                            </div>
                            <div class="vel-display">
                                <div class="vel-label">ANGULAR Z (rad/s)</div>
                                <div class="vel-value text-accent" id="vel-angular">0.00</div>
                            </div>
                        </div>
                        <button class="estop-btn mt-8" onclick="emergencyStop()">
                            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><rect x="6" y="6" width="12" height="12" rx="1"/></svg>
                            EMERGENCY STOP
                        </button>
                    </div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        initJoystick();
    }, 50);

    return div;
}

async function emergencyStop() {
    await apiPost('/api/cmd_vel', { linear: 0, angular: 0 });
    await apiPost('/api/delivery/control', { command: 'cancel' });
    _modalDismissedForCurrentPhase = false;
    hideArrivalModal();
    hideGlobalMissionBanner();
    showToast('EMERGENCY STOP DIAKTIFKAN!', 'error');
}

// ====================== PAGE: SETTINGS (CONFIG & THEMES) ======================
function renderSettings() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    const s = APP.settings || {};
    const currentSoundMode = getSoundMode();
    const activeTheme = getTheme();

    const themeCardsHtml = STUDIO_THEMES.map(t => `
        <button type="button" class="theme-card ${t.id === activeTheme ? 'active' : ''}"
                data-theme-id="${t.id}" onclick="applyTheme('${t.id}')">
            <div class="theme-preview-box" style="background:${t.bg};">
                <div class="theme-preview-top">
                    <div class="theme-preview-dots">
                        <span style="background:${t.accent}"></span>
                        <span style="background:${t.secondary}"></span>
                        <span style="background:${t.success}"></span>
                    </div>
                    <span style="width:18px; height:6px; border-radius:2px; background:${t.card}; border:1px solid rgba(128,128,128,0.3);"></span>
                </div>
                <div class="theme-preview-bar" style="background:linear-gradient(90deg, ${t.accent}, ${t.secondary});"></div>
            </div>
            <div class="theme-card-meta">
                <div>
                    <div class="theme-card-name">${t.name}</div>
                    <div class="theme-card-desc">${t.desc}</div>
                </div>
                <span class="theme-check-badge">${getIcon('check', 11)}</span>
            </div>
        </button>
    `).join('');

    div.innerHTML = `
        <!-- FULL-WIDTH SECTION 01: VISUAL THEME PICKER -->
        <div class="page-section">
            <div class="section-title">
                <span>01 // TEMA & PEWARNAAN STUDIO</span>
                <span>LIVE PREVIEW</span>
            </div>
            <div class="panel">
                <div style="display:flex; align-items:center; justify-content:space-between; flex-wrap:wrap; gap:8px; margin-bottom:10px;">
                    <div>
                        <div class="setting-label">Palet Warna Antarmuka</div>
                        <div class="setting-desc">Pilih tema warna yang paling nyaman di mata untuk kondisi pencahayaan ruangan Anda</div>
                    </div>
                </div>
                <div class="theme-picker-grid">
                    ${themeCardsHtml}
                </div>
            </div>
        </div>

        <div class="studio-two-col">
            <div>
                <div class="page-section">
                    <div class="section-title">02 // PARAMETER KINEMATIKA</div>
                    <div class="panel">
                        <div class="setting-row">
                            <div>
                                <div class="setting-label">Batas Kecepatan Linear</div>
                                <div class="setting-desc">Kecepatan translasi maksimum maju/mundur (m/s)</div>
                            </div>
                            <input class="setting-input" id="set-max-linear" type="number"
                                   step="0.1" min="0.1" max="2.0" value="${s.max_linear_speed || 0.7}">
                        </div>
                        <div class="setting-row">
                            <div>
                                <div class="setting-label">Batas Kecepatan Angular</div>
                                <div class="setting-desc">Kecepatan rotasi yaw maksimum (rad/s)</div>
                            </div>
                            <input class="setting-input" id="set-max-angular" type="number"
                                   step="0.1" min="0.1" max="3.0" value="${s.max_angular_speed || 0.9}">
                        </div>
                        <div class="setting-row">
                            <div>
                                <div class="setting-label">Durasi Tunggu Waypoint</div>
                                <div class="setting-desc">Jeda waktu berhenti di setiap titik (detik)</div>
                            </div>
                            <input class="setting-input" id="set-pause-duration" type="number"
                                   step="1" min="0" max="30" value="${s.waypoint_pause_duration || 3}">
                        </div>
                        <button class="btn btn-primary btn-block mt-12" onclick="saveSettings()">
                            ${getIcon('save', 15)} Simpan Konfigurasi
                        </button>
                    </div>
                </div>

                <div class="page-section">
                    <div class="section-title">03 // SISTEM AUDIO & NOTIFIKASI</div>
                    <div class="panel">
                        <div class="setting-row">
                            <div>
                                <div class="setting-label">Mode Audio Kedatangan</div>
                                <div class="setting-desc">Output suara saat unit tiba di pickup/delivery</div>
                            </div>
                            <select class="input-field" id="set-sound-mode" onchange="setSoundMode(this.value)"
                                    style="width: auto; min-width: 190px; cursor: pointer;">
                                <option value="human" ${currentSoundMode === 'human' ? 'selected' : ''}>Voice TTS (Suara Manusia)</option>
                                <option value="chime" ${currentSoundMode === 'chime' ? 'selected' : ''}>Synth Chime (Nada Teknis)</option>
                                <option value="both" ${currentSoundMode === 'both' ? 'selected' : ''}>Hybrid (Chime + Voice TTS)</option>
                                <option value="mute" ${currentSoundMode === 'mute' ? 'selected' : ''}>Silent (Nonaktif)</option>
                            </select>
                        </div>
                        <div class="setting-row" style="flex-wrap: wrap;">
                            <div>
                                <div class="setting-label">Diagnostik Output Audio</div>
                                <div class="setting-desc">Uji sampel pengumuman suara di perangkat ini</div>
                            </div>
                            <div style="display: flex; gap: 6px; flex-wrap: wrap;">
                                <button type="button" class="btn btn-outline btn-sm" onclick="testArrivalSound('waiting_pickup')">
                                    ${getIcon('volume', 13)} Uji Pickup
                                </button>
                                <button type="button" class="btn btn-outline btn-sm" onclick="testArrivalSound('waiting_delivery')">
                                    ${getIcon('volume', 13)} Uji Delivery
                                </button>
                                <button type="button" class="btn btn-outline btn-sm" onclick="playChimeSound('waiting_pickup')">
                                    ${getIcon('bell', 13)} Uji Chime
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>

            <div>
                <div class="page-section">
                    <div class="section-title">
                        <span>04 // DATABASE WAYPOINT</span>
                        <span>${APP.savedPoints.length} TITIK</span>
                    </div>
                    <div class="panel">
                        <div id="saved-points-list"></div>
                        <button class="btn btn-outline btn-block btn-sm mt-12" onclick="showAddPointModal()">
                            ${getIcon('plus', 14)} Registrasi Titik Baru
                        </button>
                    </div>
                </div>

                <div class="page-section">
                    <div class="section-title">05 // INFORMASI SISTEM ROS2</div>
                    <div class="panel">
                        <div class="setting-row">
                            <div class="setting-label">ROS2 Distribution</div>
                            <div class="mono text-accent" style="font-weight:600; font-size:0.78rem;">FOXY FITZROY</div>
                        </div>
                        <div class="setting-row">
                            <div class="setting-label">Status Bridge</div>
                            <div class="mono" id="sys-connection" style="font-weight:600; font-size:0.78rem;">—</div>
                        </div>
                        <div class="setting-row">
                            <div class="setting-label">Stream Kamera</div>
                            <div class="mono" id="sys-camera" style="font-weight:600; font-size:0.78rem;">—</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        renderSavedPointsList();
        updateSystemInfo();
    }, 50);

    return div;
}

function renderSavedPointsList() {
    const container = document.getElementById('saved-points-list');
    if (!container) return;

    if (APP.savedPoints.length === 0) {
        container.innerHTML = '<div class="history-empty">Belum ada titik koordinat tersimpan.</div>';
        return;
    }

    container.innerHTML = APP.savedPoints.map(p => `
        <div class="setting-row">
            <div>
                <div class="setting-label">${p.name}</div>
                <div class="setting-desc mono icon-inline">
                    ${p.type === 'pickup' ? getIcon('pickup', 12) + ' PICKUP' : getIcon('delivery_point', 12) + ' DELIVERY'}
                    // X: ${p.x}, Y: ${p.y}
                </div>
            </div>
            <button class="btn btn-outline btn-sm" style="padding:5px 9px;"
                    onclick="deletePoint('${p.name}')">${getIcon('trash', 13)}</button>
        </div>
    `).join('');
}

async function saveSettings() {
    const soundMode = document.getElementById('set-sound-mode')?.value || getSoundMode();
    setSoundMode(soundMode);

    const data = {
        max_linear_speed: parseFloat(document.getElementById('set-max-linear').value) || 0.7,
        max_angular_speed: parseFloat(document.getElementById('set-max-angular').value) || 0.9,
        waypoint_pause_duration: parseInt(document.getElementById('set-pause-duration').value) || 3,
        arrival_sound_mode: soundMode,
        theme: getTheme()
    };
    const result = await apiPost('/api/settings', data);
    if (result && result.success) {
        APP.settings = result.settings;
        showToast('Konfigurasi & tema disimpan', 'success');
    } else {
        showToast('Konfigurasi lokal disimpan', 'info');
    }
}

function showAddPointModal() {
    document.getElementById('modal-title').textContent = 'Registrasi Waypoint Baru';
    document.getElementById('modal-body').innerHTML = `
        <div class="input-group">
            <div class="input-label">Nama Identifier</div>
            <input class="input-field" id="new-point-name" placeholder="Contoh: Rak C1">
        </div>
        <div class="input-row">
            <div class="input-group">
                <div class="input-label">Koordinat X (m)</div>
                <input class="input-field" id="new-point-x" type="number" step="0.1" placeholder="0.0">
            </div>
            <div class="input-group">
                <div class="input-label">Koordinat Y (m)</div>
                <input class="input-field" id="new-point-y" type="number" step="0.1" placeholder="0.0">
            </div>
        </div>
        <div class="input-group">
            <div class="input-label">Kategori Titik</div>
            <select class="input-field" id="new-point-type">
                <option value="pickup">Pickup (Titik Muat)</option>
                <option value="delivery">Delivery (Titik Tujuan)</option>
            </select>
        </div>
    `;
    const actionsEl = document.getElementById('modal-actions');
    actionsEl.innerHTML = '';

    const btnCancel = document.createElement('button');
    btnCancel.className = 'btn btn-outline';
    btnCancel.textContent = 'Batal';
    btnCancel.onclick = hideModal;

    const btnSave = document.createElement('button');
    btnSave.className = 'btn btn-primary';
    btnSave.textContent = 'Simpan';
    btnSave.onclick = async () => {
        const name = document.getElementById('new-point-name').value.trim();
        const x = parseFloat(document.getElementById('new-point-x').value);
        const y = parseFloat(document.getElementById('new-point-y').value);
        const type = document.getElementById('new-point-type').value;

        if (!name) { showToast('Nama titik wajib diisi', 'error'); return; }
        if (isNaN(x) || isNaN(y)) { showToast('Koordinat tidak valid', 'error'); return; }

        const result = await apiPost('/api/saved_points', { name, x, y, type });
        if (result && result.success) {
            APP.savedPoints.push(result.point);
            renderSavedPointsList();
            hideModal();
            showToast(`Waypoint "${name}" ditambahkan`, 'success');
        }
    };

    actionsEl.appendChild(btnCancel);
    actionsEl.appendChild(btnSave);
    document.getElementById('modal-overlay').classList.remove('hidden');
}

async function deletePoint(name) {
    showModal('Hapus Waypoint?', `Hapus titik <strong>"${name}"</strong> dari konfigurasi?`, [
        { label: 'Batal', class: 'btn-outline' },
        {
            label: 'Hapus', class: 'btn-danger',
            action: async () => {
                await apiPost('/api/saved_points/delete', { name });
                APP.savedPoints = APP.savedPoints.filter(p => p.name !== name);
                renderSavedPointsList();
                showToast(`"${name}" dihapus`, 'info');
            }
        }
    ]);
}

async function updateSystemInfo() {
    const data = await apiGet('/api/status');
    const connEl = document.getElementById('sys-connection');
    const camEl = document.getElementById('sys-camera');
    if (connEl) {
        connEl.textContent = data ? 'CONNECTED' : 'DISCONNECTED';
        connEl.style.color = data ? 'var(--success)' : 'var(--danger)';
    }
    if (camEl) {
        try {
            const res = await fetch('/api/camera/snapshot');
            camEl.textContent = res.ok ? 'ONLINE' : 'OFFLINE';
            camEl.style.color = res.ok ? 'var(--success)' : 'var(--text-muted)';
        } catch {
            camEl.textContent = 'OFFLINE';
            camEl.style.color = 'var(--text-muted)';
        }
    }
}

// ====================== GLOBAL STATUS POLLING ======================
async function globalStatusPoll() {
    const data = await apiGet('/api/status');
    const badge = document.getElementById('connection-badge');

    if (data && data.connected) {
        APP.connected = true;
        APP.pose = data.pose || APP.pose;
        APP.delivery = data.delivery || APP.delivery;
        APP.humanStatus = data.human_status || 'NO_HUMAN';

        if (typeof handleDeliveryStatusUpdate === 'function') {
            handleDeliveryStatusUpdate(APP.delivery);
        }

        if (badge) {
            badge.className = 'badge badge-online';
            badge.querySelector('.badge-text').textContent = 'ROS2 ONLINE';
        }

        const topX = document.getElementById('topbar-pose-x');
        const topY = document.getElementById('topbar-pose-y');
        const topYaw = document.getElementById('topbar-pose-yaw');
        const yawDeg = ((APP.pose.yaw || 0) * 180 / Math.PI).toFixed(1);
        if (topX) topX.textContent = `${APP.pose.x.toFixed(2)}m`;
        if (topY) topY.textContent = `${APP.pose.y.toFixed(2)}m`;
        if (topYaw) topYaw.textContent = `${yawDeg}°`;

        const sideNode = document.getElementById('sidebar-node-state');
        const sideHuman = document.getElementById('sidebar-human-state');
        if (sideNode) {
            sideNode.textContent = APP.delivery.status === 'running' ? 'NAV2_ACTIVE' : 'NAV2_IDLE';
            sideNode.className = APP.delivery.status === 'running' ? 'sys-card-val text-accent' : 'sys-card-val';
        }
        if (sideHuman) {
            const isSafe = APP.humanStatus === 'NO_HUMAN';
            sideHuman.textContent = isSafe ? 'CLEAR' : APP.humanStatus;
            sideHuman.className = isSafe ? 'sys-card-val text-emerald' : 'sys-card-val text-amber';
        }

        if (APP.currentPage === 'home') {
            const homeStatus = document.getElementById('home-status');
            if (homeStatus) {
                const st = APP.delivery.status === 'running'
                    ? `TASK ${(APP.delivery.current_task_index || 0) + 1}/${APP.delivery.total_tasks || 0}`
                    : (APP.delivery.status === 'idle' ? 'STANDBY' : String(APP.delivery.status).toUpperCase());
                homeStatus.textContent = st;
            }

            const phaseLabels = {
                idle: 'Siap Menerima Tugas',
                navigating_pickup: 'Menuju Titik Pickup',
                waiting_pickup: 'Menunggu Muat Barang',
                at_pickup: 'Tiba di Pickup',
                navigating_delivery: 'Menuju Titik Tujuan',
                waiting_delivery: 'Menunggu Ambil Barang',
                at_delivery: 'Tiba di Tujuan',
                returning_home: 'Kembali ke Home Base'
            };
            const homePhase = document.getElementById('home-phase');
            if (homePhase) homePhase.textContent = phaseLabels[APP.delivery.current_phase] || APP.delivery.current_phase;

            const homePos = document.getElementById('home-pos');
            if (homePos) homePos.textContent = `${APP.pose.x.toFixed(2)}, ${APP.pose.y.toFixed(2)}`;

            const homeYaw = document.getElementById('home-yaw');
            if (homeYaw) homeYaw.textContent = `${yawDeg}°`;

            const homeHuman = document.getElementById('home-human');
            if (homeHuman) {
                const isSafe = APP.humanStatus === 'NO_HUMAN';
                homeHuman.textContent = isSafe ? 'CLEAR (AMAN)' : APP.humanStatus;
                homeHuman.className = isSafe ? 'kpi-main text-emerald' : 'kpi-main text-amber';
            }
        }
    } else {
        APP.connected = false;
        if (badge) {
            badge.className = 'badge badge-offline';
            badge.querySelector('.badge-text').textContent = 'OFFLINE';
        }
    }
}

// ====================== NAV ICONS & LIVE CLOCK ======================
function populateNavIcons() {
    if (typeof getIcon !== 'function') return;

    const sidebarMap = {
        's-icon-home': 'home',
        's-icon-monitoring': 'activity',
        's-icon-delivery': 'package',
        's-icon-camera': 'video',
        's-icon-history': 'clock',
        's-icon-joystick': 'compass',
        's-icon-settings': 'sliders'
    };
    Object.entries(sidebarMap).forEach(([id, icon]) => {
        const el = document.getElementById(id);
        if (el) el.innerHTML = getIcon(icon, 17);
    });

    const mobileMap = {
        'm-icon-home': 'home',
        'm-icon-monitoring': 'activity',
        'm-icon-delivery': 'package',
        'm-icon-camera': 'video',
        'm-icon-joystick': 'compass'
    };
    Object.entries(mobileMap).forEach(([id, icon]) => {
        const el = document.getElementById(id);
        if (el) el.innerHTML = getIcon(icon, 18);
    });
}

function startLiveClock() {
    const clockEl = document.getElementById('live-clock');
    if (!clockEl) return;
    const update = () => {
        const now = new Date();
        clockEl.textContent = now.toTimeString().split(' ')[0];
    };
    update();
    setInterval(update, 1000);
}

// ====================== INIT ======================
async function init() {
    populateNavIcons();
    startLiveClock();
    applyTheme(getTheme(), true);

    const pointsData = await apiGet('/api/saved_points');
    if (pointsData) {
        APP.savedPoints = pointsData.points || [];
        APP.homePosition = pointsData.home || { x: 0, y: 0, yaw: 0 };
    }

    const settingsData = await apiGet('/api/settings');
    if (settingsData) {
        APP.settings = settingsData;
        if (!localStorage.getItem('amr_theme') && settingsData.theme) {
            applyTheme(settingsData.theme, true);
        }
    }

    navigateTo('home');

    globalStatusPoll();
    setInterval(globalStatusPoll, 1000);
}

document.addEventListener('DOMContentLoaded', init);
