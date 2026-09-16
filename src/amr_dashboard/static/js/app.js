/* =================================================================
   AMR DELIVERY DASHBOARD — Main Application (SPA Router)
   Handles page navigation, state management, and API calls.
   ================================================================= */

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
    // Delivery task builder
    taskQueue: [],       // [{pickup: {name,x,y}, delivery: {name,x,y}}, ...]
    selectingFor: null,  // 'pickup' or 'delivery'
    pendingPickup: null,
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
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    const icons = {
        success: '✓',
        error: '✕',
        info: 'ℹ'
    };
    toast.innerHTML = `<span>${icons[type] || 'ℹ'}</span><span>${message}</span>`;
    container.appendChild(toast);
    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transform = 'translateY(10px)';
        toast.style.transition = '0.3s ease';
        setTimeout(() => toast.remove(), 300);
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

// ====================== NAVIGATION ======================
function navigateTo(page) {
    APP.currentPage = page;
    const content = document.getElementById('app-content');
    const backBtn = document.getElementById('btn-back');
    const titleEl = document.getElementById('page-title');

    // Update topbar
    if (page === 'home') {
        backBtn.classList.add('hidden');
        titleEl.textContent = 'AMR Dashboard';
    } else {
        backBtn.classList.remove('hidden');
        const titles = {
            delivery: '📦 Kontrol Delivery',
            monitoring: '📡 Monitoring',
            camera: '📷 View Kamera',
            history: '📋 Riwayat Delivery',
            joystick: '🎮 Manual Control',
            settings: '⚙️ Pengaturan'
        };
        titleEl.textContent = titles[page] || page;
    }

    // Stop previous polling
    if (APP.statusPollTimer) {
        clearInterval(APP.statusPollTimer);
        APP.statusPollTimer = null;
    }

    // Render page
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
        content.querySelector('.page-enter')?.classList.add('page-enter');
    }
}

// ====================== PAGE: HOME ======================
function renderHome() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    const statusText = APP.delivery.status === 'running'
        ? `Task ${(APP.delivery.current_task_index || 0) + 1}/${APP.delivery.total_tasks || 0}`
        : (APP.delivery.status === 'idle' ? 'Standby' : APP.delivery.status);

    const phaseLabels = {
        idle: 'Siap',
        navigating_pickup: 'Menuju Pickup',
        at_pickup: 'Di Pickup',
        navigating_delivery: 'Menuju Tujuan',
        at_delivery: 'Di Tujuan'
    };

    div.innerHTML = `
        <div class="home-header">
            <h1>AMR Delivery</h1>
            <p>Kontrol & monitoring robot pengiriman</p>
        </div>

        <div class="home-status-bar">
            <div class="status-chip">
                <div class="chip-label">Status</div>
                <div class="chip-value" id="home-status">${statusText}</div>
            </div>
            <div class="status-chip">
                <div class="chip-label">Fase</div>
                <div class="chip-value" id="home-phase">${phaseLabels[APP.delivery.current_phase] || APP.delivery.current_phase}</div>
            </div>
            <div class="status-chip">
                <div class="chip-label">Posisi</div>
                <div class="chip-value" id="home-pos">${APP.pose.x.toFixed(1)}, ${APP.pose.y.toFixed(1)}</div>
            </div>
        </div>

        <div class="menu-grid">
            <div class="menu-card" onclick="navigateTo('delivery')">
                <div class="card-icon icon-delivery">📦</div>
                <div class="card-title">Kontrol Delivery</div>
                <div class="card-desc">Atur titik pickup & tujuan pengiriman</div>
            </div>
            <div class="menu-card" onclick="navigateTo('monitoring')">
                <div class="card-icon icon-monitor">📡</div>
                <div class="card-title">Monitoring</div>
                <div class="card-desc">Posisi AMR & status real-time</div>
            </div>
            <div class="menu-card" onclick="navigateTo('camera')">
                <div class="card-icon icon-camera">📷</div>
                <div class="card-title">View Kamera</div>
                <div class="card-desc">Live feed kamera robot</div>
            </div>
            <div class="menu-card" onclick="navigateTo('history')">
                <div class="card-icon icon-history">📋</div>
                <div class="card-title">Riwayat</div>
                <div class="card-desc">Log pengiriman sebelumnya</div>
            </div>
            <div class="menu-card" onclick="navigateTo('joystick')">
                <div class="card-icon icon-joystick">🎮</div>
                <div class="card-title">Manual Control</div>
                <div class="card-desc">Joystick virtual & emergency stop</div>
            </div>
            <div class="menu-card" onclick="navigateTo('settings')">
                <div class="card-icon icon-settings">⚙️</div>
                <div class="card-title">Pengaturan</div>
                <div class="card-desc">Konfigurasi robot & waypoints</div>
            </div>
        </div>
    `;

    return div;
}

// ====================== PAGE: DELIVERY CONTROL ======================
function renderDelivery() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="page-section">
            <div class="section-title">Peta & Titik Tujuan</div>
            <div class="map-container" id="delivery-map-container">
                <canvas id="delivery-map-canvas"></canvas>
                <div class="map-legend">
                    <span class="legend-pickup">Pickup</span>
                    <span class="legend-delivery">Delivery</span>
                    <span class="legend-robot">Robot</span>
                </div>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Tambah Task</div>
            <div class="panel">
                <p style="font-size:0.75rem; color:var(--text-secondary); margin-bottom:10px;">
                    Pilih titik pickup, lalu titik delivery. Atau tap langsung di peta.
                </p>

                <div class="mb-8">
                    <div class="input-label mb-8">📍 Titik Pickup</div>
                    <div class="waypoint-chips" id="pickup-chips"></div>
                    <div id="pickup-selected" style="font-size:0.78rem; color:var(--accent); font-weight:600; min-height:20px;"></div>
                </div>

                <div class="mb-12">
                    <div class="input-label mb-8">🎯 Titik Delivery</div>
                    <div class="waypoint-chips" id="delivery-chips"></div>
                    <div id="delivery-selected" style="font-size:0.78rem; color:var(--success); font-weight:600; min-height:20px;"></div>
                </div>

                <div class="input-label mb-8">Atau masukkan koordinat manual:</div>
                <div class="input-row mb-8">
                    <div class="input-group" style="margin-bottom:0">
                        <input class="input-field" id="manual-x" type="number" step="0.1" placeholder="X">
                    </div>
                    <div class="input-group" style="margin-bottom:0">
                        <input class="input-field" id="manual-y" type="number" step="0.1" placeholder="Y">
                    </div>
                </div>
                <div class="btn-group mb-8">
                    <button class="btn btn-outline btn-sm" onclick="setManualAsPickup()">Set Pickup</button>
                    <button class="btn btn-outline btn-sm" onclick="setManualAsDelivery()">Set Delivery</button>
                </div>

                <button class="btn btn-primary btn-block btn-sm mt-8" onclick="addTaskToQueue()">
                    ＋ Tambah ke Antrian
                </button>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Antrian Task (<span id="task-count">${APP.taskQueue.length}</span>)</div>
            <ul class="task-list" id="task-list"></ul>
            <div class="btn-group mt-12" id="delivery-actions">
                <button class="btn btn-success btn-block" onclick="startDelivery()" id="btn-start-delivery">
                    🚀 Mulai Delivery
                </button>
                <button class="btn btn-outline" onclick="clearTaskQueue()">Hapus</button>
            </div>
        </div>

        <div class="page-section" id="mission-control-section" class="hidden">
            <div class="section-title">Kontrol Misi</div>
            <div class="panel">
                <div class="text-center mb-8">
                    <span class="status-badge" id="mission-status-badge">IDLE</span>
                </div>
                <div class="progress-bar">
                    <div class="progress-fill" id="mission-progress" style="width:0%"></div>
                </div>
                <p style="font-size:0.72rem; color:var(--text-secondary); text-align:center;" id="mission-progress-text"></p>
                <div class="btn-group mt-12">
                    <button class="btn btn-warning btn-sm" onclick="controlMission('pause')" id="btn-pause">⏸ Pause</button>
                    <button class="btn btn-primary btn-sm" onclick="controlMission('resume')" id="btn-resume">▶ Resume</button>
                    <button class="btn btn-danger btn-sm" onclick="controlMission('cancel')">✕ Cancel</button>
                </div>
            </div>
        </div>
    `;

    // Render setelah DOM ready
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
        document.getElementById('pickup-selected').textContent = `✓ ${point.name} (${point.x}, ${point.y})`;
        // Highlight chip
        document.querySelectorAll('#pickup-chips .wp-chip').forEach(c =>
            c.classList.toggle('selected', c.textContent === point.name));
    } else {
        APP.pendingDelivery = { name: point.name, x: point.x, y: point.y };
        document.getElementById('delivery-selected').textContent = `✓ ${point.name} (${point.x}, ${point.y})`;
        document.querySelectorAll('#delivery-chips .wp-chip').forEach(c =>
            c.classList.toggle('selected', c.textContent === point.name));
    }
}

function setManualAsPickup() {
    const x = parseFloat(document.getElementById('manual-x').value);
    const y = parseFloat(document.getElementById('manual-y').value);
    if (isNaN(x) || isNaN(y)) { showToast('Masukkan koordinat X dan Y', 'error'); return; }
    APP.pendingPickup = { name: `(${x}, ${y})`, x, y };
    document.getElementById('pickup-selected').textContent = `✓ Pickup: (${x}, ${y})`;
    showToast('Pickup di-set ke koordinat manual', 'info');
}

function setManualAsDelivery() {
    const x = parseFloat(document.getElementById('manual-x').value);
    const y = parseFloat(document.getElementById('manual-y').value);
    if (isNaN(x) || isNaN(y)) { showToast('Masukkan koordinat X dan Y', 'error'); return; }
    APP.pendingDelivery = { name: `(${x}, ${y})`, x, y };
    document.getElementById('delivery-selected').textContent = `✓ Delivery: (${x}, ${y})`;
    showToast('Delivery di-set ke koordinat manual', 'info');
}

function addTaskToQueue() {
    if (!APP.pendingPickup) { showToast('Pilih titik pickup dulu!', 'error'); return; }
    if (!APP.pendingDelivery) { showToast('Pilih titik delivery dulu!', 'error'); return; }

    APP.taskQueue.push({
        pickup: { ...APP.pendingPickup },
        delivery: { ...APP.pendingDelivery }
    });

    APP.pendingPickup = null;
    APP.pendingDelivery = null;

    // Reset UI
    document.getElementById('pickup-selected').textContent = '';
    document.getElementById('delivery-selected').textContent = '';
    document.querySelectorAll('.wp-chip').forEach(c => c.classList.remove('selected'));

    renderTaskList();
    showToast(`Task ditambahkan (total: ${APP.taskQueue.length})`, 'success');
}

function removeTask(index) {
    APP.taskQueue.splice(index, 1);
    renderTaskList();
}

function clearTaskQueue() {
    APP.taskQueue = [];
    renderTaskList();
    showToast('Antrian dihapus', 'info');
}

function renderTaskList() {
    const list = document.getElementById('task-list');
    const countEl = document.getElementById('task-count');
    if (!list) return;
    if (countEl) countEl.textContent = APP.taskQueue.length;

    if (APP.taskQueue.length === 0) {
        list.innerHTML = '<li class="history-empty">Belum ada task. Tambahkan titik pickup & delivery.</li>';
        return;
    }

    list.innerHTML = APP.taskQueue.map((task, i) => `
        <li class="task-item">
            <span class="task-number">${i + 1}</span>
            <div class="task-info">
                <div class="task-route">${task.pickup.name} → ${task.delivery.name}</div>
                <div class="task-status-text">Menunggu</div>
            </div>
            <button class="task-remove" onclick="removeTask(${i})">✕</button>
        </li>
    `).join('');
}

async function startDelivery() {
    if (APP.taskQueue.length === 0) {
        showToast('Tambahkan task dulu!', 'error');
        return;
    }

    showModal(
        'Mulai Delivery?',
        `<p>AMR akan memulai misi dengan <strong>${APP.taskQueue.length} task</strong>.</p>
         <p style="margin-top:8px; font-size:0.75rem; color:var(--text-muted);">
         Robot akan bergerak ke setiap titik pickup, lalu mengantar ke tujuan masing-masing.</p>`,
        [
            { label: 'Batal', class: 'btn-outline' },
            {
                label: '🚀 Mulai!',
                class: 'btn-success',
                action: async () => {
                    const result = await apiPost('/api/delivery/start', { tasks: APP.taskQueue });
                    if (result && result.success) {
                        showToast('Misi dimulai!', 'success');
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
        const labels = { pause: 'Misi di-pause', resume: 'Misi dilanjutkan', cancel: 'Misi dibatalkan', skip: 'Task di-skip' };
        showToast(labels[command] || command, command === 'cancel' ? 'error' : 'info');
    }
}

function updateMissionControlVisibility() {
    const section = document.getElementById('mission-control-section');
    if (!section) return;
    const isActive = ['running', 'paused'].includes(APP.delivery.status);
    section.classList.toggle('hidden', !isActive);
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
        progress.style.width = `${pct}%`;

        const phaseLabels = {
            navigating_pickup: 'Menuju pickup',
            at_pickup: 'Mengambil barang',
            navigating_delivery: 'Mengantar barang',
            at_delivery: 'Menyerahkan barang',
            idle: 'Selesai'
        };
        progressText.textContent = `Task ${(d.current_task_index || 0) + 1}/${d.total_tasks} — ${phaseLabels[d.current_phase] || d.current_phase}`;
    }

    updateMissionControlVisibility();
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

// ====================== PAGE: MONITORING ======================
function renderMonitoring() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="page-section">
            <div class="section-title">Peta Real-time</div>
            <div class="map-container" id="monitor-map-container">
                <canvas id="monitor-map-canvas"></canvas>
                <div class="map-legend">
                    <span class="legend-robot">Robot</span>
                </div>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Status AMR</div>
            <div class="text-center mb-12">
                <span class="status-badge" id="mon-status-badge">IDLE</span>
            </div>
            <div class="progress-bar mb-8">
                <div class="progress-fill" id="mon-progress" style="width:0%"></div>
            </div>
            <p class="text-center text-muted" style="font-size:0.72rem;" id="mon-progress-text"></p>
        </div>

        <div class="page-section">
            <div class="section-title">Data Sensor</div>
            <div class="monitor-grid">
                <div class="monitor-card">
                    <div class="mon-label">Posisi X</div>
                    <div class="mon-value mon-accent" id="mon-x">0.000</div>
                </div>
                <div class="monitor-card">
                    <div class="mon-label">Posisi Y</div>
                    <div class="mon-value mon-accent" id="mon-y">0.000</div>
                </div>
                <div class="monitor-card">
                    <div class="mon-label">Orientasi</div>
                    <div class="mon-value mon-warning" id="mon-yaw">0.0°</div>
                </div>
                <div class="monitor-card">
                    <div class="mon-label">Human Status</div>
                    <div class="mon-value mon-success" id="mon-human">—</div>
                </div>
                <div class="monitor-card">
                    <div class="mon-label">Fase</div>
                    <div class="mon-value" id="mon-phase" style="font-size:0.8rem; color:var(--text-primary);">Idle</div>
                </div>
                <div class="monitor-card">
                    <div class="mon-label">Waktu Misi</div>
                    <div class="mon-value mon-accent" id="mon-elapsed">0s</div>
                </div>
            </div>
        </div>
    `;

    setTimeout(() => {
        initMonitorMap();
        startMonitoringPolling();
    }, 50);

    return div;
}

// ====================== PAGE: CAMERA ======================
function renderCamera() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="page-section">
            <div class="section-title">Live Feed Kamera</div>
            <div class="camera-frame">
                <img id="camera-feed" src="/api/camera/stream" alt="Kamera AMR"
                     onerror="this.style.display='none'; document.getElementById('cam-offline').style.display='flex';">
                <div class="camera-overlay">
                    <div class="camera-badge"><span class="rec-dot"></span>LIVE</div>
                </div>
                <div id="cam-offline" class="flex-center" style="display:none; min-height:250px; color:var(--text-muted); font-size:0.85rem; flex-direction:column; gap:8px;">
                    <span style="font-size:2rem;">📷</span>
                    Kamera tidak tersedia
                </div>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Info</div>
            <div class="panel">
                <div class="monitor-grid">
                    <div class="monitor-card">
                        <div class="mon-label">Human Status</div>
                        <div class="mon-value mon-success" id="cam-human">—</div>
                    </div>
                    <div class="monitor-card">
                        <div class="mon-label">Posisi</div>
                        <div class="mon-value mon-accent" id="cam-pos">—</div>
                    </div>
                </div>
            </div>
        </div>

        <div class="page-section">
            <button class="btn btn-outline btn-block" onclick="captureScreenshot()">📸 Screenshot</button>
        </div>
    `;

    setTimeout(() => {
        startCameraPolling();
    }, 50);

    return div;
}

function captureScreenshot() {
    window.open('/api/camera/snapshot', '_blank');
    showToast('Screenshot diambil', 'success');
}

function startCameraPolling() {
    APP.statusPollTimer = setInterval(async () => {
        const data = await apiGet('/api/status');
        if (data) {
            const camHuman = document.getElementById('cam-human');
            const camPos = document.getElementById('cam-pos');
            if (camHuman) camHuman.textContent = data.human_status || '—';
            if (camPos) camPos.textContent = `${data.pose.x.toFixed(1)}, ${data.pose.y.toFixed(1)}`;
        }
    }, 1500);
}

// ====================== PAGE: HISTORY ======================
function renderHistory() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="page-section">
            <div class="section-title">Riwayat Pengiriman</div>
            <div class="panel" id="history-content">
                <div class="history-empty">Memuat...</div>
            </div>
        </div>
        <div class="page-section">
            <button class="btn btn-danger btn-block btn-sm" onclick="clearHistory()">🗑 Hapus Semua Riwayat</button>
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
        container.innerHTML = '<div class="history-empty">Belum ada riwayat pengiriman.</div>';
        return;
    }

    container.innerHTML = `
        <div style="overflow-x:auto;">
        <table class="history-table">
            <thead>
                <tr>
                    <th>Waktu</th>
                    <th>Dari</th>
                    <th>Ke</th>
                    <th>Status</th>
                </tr>
            </thead>
            <tbody>
                ${data.map(r => {
                    const time = r.timestamp ? new Date(r.timestamp).toLocaleString('id-ID', {
                        day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit'
                    }) : '-';
                    const statusClass = r.status === 'completed' ? 'mon-success' :
                                       r.status === 'failed' ? 'mon-danger' : 'mon-warning';
                    return `<tr>
                        <td>${time}</td>
                        <td>${r.pickup_name}</td>
                        <td>${r.delivery_name}</td>
                        <td><span class="${statusClass}" style="font-weight:700">${r.status}</span></td>
                    </tr>`;
                }).join('')}
            </tbody>
        </table>
        </div>
    `;
}

async function clearHistory() {
    showModal('Hapus Riwayat?', 'Semua riwayat pengiriman akan dihapus permanen.', [
        { label: 'Batal', class: 'btn-outline' },
        {
            label: 'Hapus', class: 'btn-danger',
            action: async () => {
                await apiPost('/api/history/clear', {});
                loadHistory();
                showToast('Riwayat dihapus', 'info');
            }
        }
    ]);
}

// ====================== PAGE: JOYSTICK ======================
function renderJoystick() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    div.innerHTML = `
        <div class="page-section">
            <div class="section-title">Kamera</div>
            <div class="camera-frame" style="max-height:180px; overflow:hidden;">
                <img src="/api/camera/stream" alt="Kamera"
                     style="width:100%; object-fit:cover;"
                     onerror="this.parentElement.innerHTML='<div class=\\'flex-center\\' style=\\'height:120px;color:var(--text-muted);font-size:0.8rem;\\'>📷 Kamera offline</div>'">
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Joystick</div>
            <div class="joystick-container">
                <div class="joystick-canvas-wrap">
                    <canvas id="joystick-canvas" width="220" height="220"></canvas>
                </div>
                <div class="joystick-info">
                    <div class="vel-display">
                        <div class="vel-label">Linear (m/s)</div>
                        <div class="vel-value" id="vel-linear">0.00</div>
                    </div>
                    <div class="vel-display">
                        <div class="vel-label">Angular (rad/s)</div>
                        <div class="vel-value" id="vel-angular">0.00</div>
                    </div>
                </div>
            </div>
        </div>

        <div class="page-section flex-center">
            <button class="estop-btn" onclick="emergencyStop()">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><rect x="6" y="6" width="12" height="12" rx="1"/></svg>
                STOP
            </button>
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
    showToast('EMERGENCY STOP!', 'error');
}

// ====================== PAGE: SETTINGS ======================
function renderSettings() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    const s = APP.settings;

    div.innerHTML = `
        <div class="page-section">
            <div class="section-title">Kecepatan Robot</div>
            <div class="panel">
                <div class="setting-row">
                    <div>
                        <div class="setting-label">Kecepatan Linear Maks</div>
                        <div class="setting-desc">Kecepatan maju/mundur (m/s)</div>
                    </div>
                    <input class="setting-input" id="set-max-linear" type="number"
                           step="0.1" min="0.1" max="2.0" value="${s.max_linear_speed || 0.7}">
                </div>
                <div class="setting-row">
                    <div>
                        <div class="setting-label">Kecepatan Angular Maks</div>
                        <div class="setting-desc">Kecepatan putar (rad/s)</div>
                    </div>
                    <input class="setting-input" id="set-max-angular" type="number"
                           step="0.1" min="0.1" max="3.0" value="${s.max_angular_speed || 0.9}">
                </div>
                <div class="setting-row">
                    <div>
                        <div class="setting-label">Jeda di Waypoint</div>
                        <div class="setting-desc">Waktu tunggu di setiap titik (detik)</div>
                    </div>
                    <input class="setting-input" id="set-pause-duration" type="number"
                           step="1" min="0" max="30" value="${s.waypoint_pause_duration || 3}">
                </div>
                <button class="btn btn-primary btn-block mt-12" onclick="saveSettings()">💾 Simpan Pengaturan</button>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Titik Tersimpan</div>
            <div class="panel" id="saved-points-list"></div>
            <button class="btn btn-outline btn-block btn-sm mt-8" onclick="showAddPointModal()">＋ Tambah Titik Baru</button>
        </div>

        <div class="page-section">
            <div class="section-title">Informasi Sistem</div>
            <div class="panel">
                <div class="setting-row">
                    <div class="setting-label">ROS2 Distro</div>
                    <div style="color:var(--accent); font-weight:600; font-size:0.82rem;">Foxy</div>
                </div>
                <div class="setting-row">
                    <div class="setting-label">Koneksi</div>
                    <div id="sys-connection" style="font-weight:600; font-size:0.82rem;">—</div>
                </div>
                <div class="setting-row">
                    <div class="setting-label">Kamera</div>
                    <div id="sys-camera" style="font-weight:600; font-size:0.82rem;">—</div>
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
        container.innerHTML = '<div class="history-empty">Belum ada titik tersimpan.</div>';
        return;
    }

    container.innerHTML = APP.savedPoints.map(p => `
        <div class="setting-row">
            <div>
                <div class="setting-label">${p.name}</div>
                <div class="setting-desc">${p.type === 'pickup' ? '📍 Pickup' : '🎯 Delivery'} — (${p.x}, ${p.y})</div>
            </div>
            <button class="btn btn-outline btn-sm" style="padding:6px 10px; font-size:0.7rem;"
                    onclick="deletePoint('${p.name}')">✕</button>
        </div>
    `).join('');
}

async function saveSettings() {
    const data = {
        max_linear_speed: parseFloat(document.getElementById('set-max-linear').value) || 0.7,
        max_angular_speed: parseFloat(document.getElementById('set-max-angular').value) || 0.9,
        waypoint_pause_duration: parseInt(document.getElementById('set-pause-duration').value) || 3
    };
    const result = await apiPost('/api/settings', data);
    if (result && result.success) {
        APP.settings = result.settings;
        showToast('Pengaturan disimpan!', 'success');
    } else {
        showToast('Gagal menyimpan', 'error');
    }
}

function showAddPointModal() {
    document.getElementById('modal-title').textContent = 'Tambah Titik Baru';
    document.getElementById('modal-body').innerHTML = `
        <div class="input-group">
            <div class="input-label">Nama Titik</div>
            <input class="input-field" id="new-point-name" placeholder="Contoh: Rak C1">
        </div>
        <div class="input-row">
            <div class="input-group">
                <div class="input-label">X</div>
                <input class="input-field" id="new-point-x" type="number" step="0.1" placeholder="0.0">
            </div>
            <div class="input-group">
                <div class="input-label">Y</div>
                <input class="input-field" id="new-point-y" type="number" step="0.1" placeholder="0.0">
            </div>
        </div>
        <div class="input-group">
            <div class="input-label">Tipe</div>
            <select class="input-field" id="new-point-type">
                <option value="pickup">Pickup</option>
                <option value="delivery">Delivery</option>
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
            showToast(`Titik "${name}" disimpan!`, 'success');
        }
    };

    actionsEl.appendChild(btnCancel);
    actionsEl.appendChild(btnSave);
    document.getElementById('modal-overlay').classList.remove('hidden');
}

async function deletePoint(name) {
    showModal('Hapus Titik?', `Hapus titik <strong>"${name}"</strong>?`, [
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
        connEl.textContent = data ? 'Terhubung' : 'Terputus';
        connEl.style.color = data ? 'var(--success)' : 'var(--danger)';
    }
    if (camEl) {
        // Camera check via snapshot
        try {
            const res = await fetch('/api/camera/snapshot');
            camEl.textContent = res.ok ? 'Aktif' : 'Nonaktif';
            camEl.style.color = res.ok ? 'var(--success)' : 'var(--text-muted)';
        } catch {
            camEl.textContent = 'Nonaktif';
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

        if (badge) {
            badge.className = 'badge badge-online';
            badge.querySelector('.badge-text').textContent = 'Online';
        }

        // Update home page chips if visible
        const homeStatus = document.getElementById('home-status');
        if (homeStatus && APP.currentPage === 'home') {
            const st = APP.delivery.status === 'running'
                ? `Task ${(APP.delivery.current_task_index || 0) + 1}/${APP.delivery.total_tasks || 0}`
                : (APP.delivery.status === 'idle' ? 'Standby' : APP.delivery.status);
            homeStatus.textContent = st;

            const phaseLabels = {
                idle: 'Siap', navigating_pickup: 'Menuju Pickup', at_pickup: 'Di Pickup',
                navigating_delivery: 'Menuju Tujuan', at_delivery: 'Di Tujuan'
            };
            const homePhase = document.getElementById('home-phase');
            if (homePhase) homePhase.textContent = phaseLabels[APP.delivery.current_phase] || APP.delivery.current_phase;
            const homePos = document.getElementById('home-pos');
            if (homePos) homePos.textContent = `${APP.pose.x.toFixed(1)}, ${APP.pose.y.toFixed(1)}`;
        }
    } else {
        APP.connected = false;
        if (badge) {
            badge.className = 'badge badge-offline';
            badge.querySelector('.badge-text').textContent = 'Offline';
        }
    }
}

// ====================== INIT ======================
async function init() {
    // Load saved points
    const pointsData = await apiGet('/api/saved_points');
    if (pointsData) {
        APP.savedPoints = pointsData.points || [];
        APP.homePosition = pointsData.home || { x: 0, y: 0, yaw: 0 };
    }

    // Load settings
    const settingsData = await apiGet('/api/settings');
    if (settingsData) {
        APP.settings = settingsData;
    }

    // Render home page
    navigateTo('home');

    // Start global polling
    globalStatusPoll();
    setInterval(globalStatusPoll, 2000);
}

// Start when DOM ready
document.addEventListener('DOMContentLoaded', init);
