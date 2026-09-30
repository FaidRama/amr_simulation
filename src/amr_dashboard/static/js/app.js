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
    const iconName = type === 'success' ? 'checkCircle' : (type === 'error' ? 'xCircle' : 'info');
    toast.innerHTML = `<span class="icon-inline">${getIcon(iconName, 18)}</span><span>${message}</span>`;
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

    // Update topbar
    if (page === 'home') {
        backBtn.classList.add('hidden');
        titleEl.textContent = 'AMR Dashboard';
    } else {
        backBtn.classList.remove('hidden');
        const titles = {
            delivery: 'Kontrol Delivery',
            monitoring: 'Pusat Pemantauan',
            camera: 'View Kamera',
            history: 'Riwayat Delivery',
            joystick: 'Manual Control',
            settings: 'Pengaturan'
        };
        titleEl.textContent = titles[page] || page;
    }

    // Update active state in sidebar and mobile nav
    document.querySelectorAll('#app-sidebar .nav-item').forEach(item => {
        item.classList.toggle('active', item.dataset.page === page);
    });
    document.querySelectorAll('#mobile-nav .mobile-nav-item').forEach(item => {
        item.classList.toggle('active', item.dataset.page === page);
    });

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

    // Sinkronkan status kedatangan / banner / popup di halaman baru
    if (typeof handleDeliveryStatusUpdate === 'function' && APP.delivery) {
        handleDeliveryStatusUpdate(APP.delivery);
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
        waiting_pickup: 'Menunggu Muat Barang',
        at_pickup: 'Di Pickup',
        navigating_delivery: 'Menuju Tujuan',
        waiting_delivery: 'Menunggu Ambil Barang',
        at_delivery: 'Di Tujuan',
        returning_home: 'Kembali ke Titik Awal'
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
                <div class="card-icon icon-delivery">${getIcon('delivery', 26)}</div>
                <div class="card-title">Kontrol Delivery</div>
                <div class="card-desc">Atur titik pickup & tujuan pengiriman</div>
            </div>
            <div class="menu-card" onclick="navigateTo('monitoring')">
                <div class="card-icon icon-monitor">${getIcon('monitor', 26)}</div>
                <div class="card-title">Monitoring</div>
                <div class="card-desc">Posisi AMR & status real-time</div>
            </div>
            <div class="menu-card" onclick="navigateTo('camera')">
                <div class="card-icon icon-camera">${getIcon('camera', 26)}</div>
                <div class="card-title">View Kamera</div>
                <div class="card-desc">Live feed kamera robot</div>
            </div>
            <div class="menu-card" onclick="navigateTo('history')">
                <div class="card-icon icon-history">${getIcon('history', 26)}</div>
                <div class="card-title">Riwayat</div>
                <div class="card-desc">Log pengiriman sebelumnya</div>
            </div>
            <div class="menu-card" onclick="navigateTo('joystick')">
                <div class="card-icon icon-joystick">${getIcon('joystick', 26)}</div>
                <div class="card-title">Manual Control</div>
                <div class="card-desc">Joystick virtual & emergency stop</div>
            </div>
            <div class="menu-card" onclick="navigateTo('settings')">
                <div class="card-icon icon-settings">${getIcon('settings', 26)}</div>
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
                    <div class="input-label mb-8 icon-inline">${getIcon('pickup', 15)} Titik Pickup</div>
                    <div class="waypoint-chips" id="pickup-chips"></div>
                    <div id="pickup-selected" style="font-size:0.78rem; color:var(--accent); font-weight:600; min-height:20px;"></div>
                </div>

                <div class="mb-12">
                    <div class="input-label mb-8 icon-inline">${getIcon('delivery_point', 15)} Titik Delivery</div>
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
                    ${getIcon('plus', 16)} Tambah ke Antrian
                </button>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Antrian Task (<span id="task-count">${APP.taskQueue.length}</span>)</div>
            <ul class="task-list" id="task-list"></ul>
            <div class="btn-group mt-12" id="delivery-actions">
                <button class="btn btn-success btn-block" onclick="startDelivery()" id="btn-start-delivery">
                    ${getIcon('play', 16)} Mulai Delivery
                </button>
                <button class="btn btn-outline" onclick="clearTaskQueue()">Hapus</button>
            </div>
        </div>

        <div class="page-section hidden" id="mission-control-section">
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
                    <button class="btn btn-warning btn-sm" onclick="controlMission('pause')" id="btn-pause">${getIcon('pause', 14)} Pause</button>
                    <button class="btn btn-primary btn-sm" onclick="controlMission('resume')" id="btn-resume">${getIcon('play', 14)} Resume</button>
                    <button class="btn btn-danger btn-sm" onclick="controlMission('cancel')">${getIcon('x', 14)} Cancel</button>
                </div>
            </div>
        </div>

        <div class="page-section hidden" id="confirm-action-section"></div>
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
        document.getElementById('pickup-selected').innerHTML = `${getIcon('check', 14)} ${point.name} (${point.x}, ${point.y})`;
        // Highlight chip
        document.querySelectorAll('#pickup-chips .wp-chip').forEach(c =>
            c.classList.toggle('selected', c.textContent === point.name));
    } else {
        APP.pendingDelivery = { name: point.name, x: point.x, y: point.y };
        document.getElementById('delivery-selected').innerHTML = `${getIcon('check', 14)} ${point.name} (${point.x}, ${point.y})`;
        document.querySelectorAll('#delivery-chips .wp-chip').forEach(c =>
            c.classList.toggle('selected', c.textContent === point.name));
    }
}

function setManualAsPickup() {
    const x = parseFloat(document.getElementById('manual-x').value);
    const y = parseFloat(document.getElementById('manual-y').value);
    if (isNaN(x) || isNaN(y)) { showToast('Masukkan koordinat X dan Y', 'error'); return; }
    APP.pendingPickup = { name: `(${x}, ${y})`, x, y };
    document.getElementById('pickup-selected').innerHTML = `${getIcon('check', 14)} Pickup: (${x}, ${y})`;
    showToast('Pickup di-set ke koordinat manual', 'info');
}

function setManualAsDelivery() {
    const x = parseFloat(document.getElementById('manual-x').value);
    const y = parseFloat(document.getElementById('manual-y').value);
    if (isNaN(x) || isNaN(y)) { showToast('Masukkan koordinat X dan Y', 'error'); return; }
    APP.pendingDelivery = { name: `(${x}, ${y})`, x, y };
    document.getElementById('delivery-selected').innerHTML = `${getIcon('check', 14)} Delivery: (${x}, ${y})`;
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
            <button class="task-remove" onclick="removeTask(${i})">${getIcon('x', 14)}</button>
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
                label: 'Mulai Sekarang',
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

// Track phase & modal status untuk notifikasi kedatangan
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
            // Jika berada di halaman lain (camera, monitoring, dsb), langsung munculkan modal pop-up konfirmasi
            if (APP.currentPage !== 'delivery') {
                showArrivalModal(d);
            }
        }

        // Tampilkan floating banner di semua halaman selain halaman delivery
        if (APP.currentPage !== 'delivery') {
            updateGlobalMissionBanner(d);
            if (!_modalDismissedForCurrentPhase) {
                showArrivalModal(d);
            }
        } else {
            // Di halaman delivery, sembunyikan modal/banner agar fokus ke card di halaman
            hideGlobalMissionBanner();
            hideArrivalModal();
        }

        // Update in-page card jika sedang di halaman delivery
        if (APP.currentPage === 'delivery') {
            updateInPageConfirmCard(d);
        }
    } else {
        if (!isWaiting) {
            _lastNotifiedPhase = null;
            _modalDismissedForCurrentPhase = false;
        }
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
    if (iconEl) iconEl.innerHTML = isPickup ? getIcon('box', 36) : getIcon('checkCircle', 36);

    if (titleEl) {
        titleEl.textContent = isPickup
            ? `Robot Tiba di Pickup: ${pointName}!`
            : `Robot Tiba di Tujuan: ${pointName}!`;
    }

    if (descEl) {
        descEl.textContent = isPickup
            ? 'Robot sudah sampai di titik pickup dan siap dimuat. Silakan muat barang ke robot lalu tekan konfirmasi.'
            : 'Robot sudah sampai di titik tujuan pengantaran. Silakan ambil barang dari robot lalu tekan konfirmasi.';
    }

    const cmd = isPickup ? 'confirm_pickup' : 'confirm_delivery';
    const btnLabel = isPickup ? 'Barang Sudah Dimuat' : 'Barang Sudah Diambil';
    const btnClass = isPickup ? 'btn-confirm-pickup' : 'btn-confirm-delivery';

    if (actionsEl) {
        actionsEl.innerHTML = `
            <button class="btn btn-confirm ${btnClass}" onclick="confirmActionFromModal('${cmd}')">
                ${getIcon('check', 20)} ${btnLabel}
            </button>
            <button class="btn btn-secondary" onclick="goToDeliveryPageFromModal()" style="display:flex; align-items:center; justify-content:center; gap:8px;">
                ${getIcon('delivery', 18)} Buka Halaman Delivery
            </button>
            <button class="btn btn-outline" onclick="dismissArrivalModal()" style="display:flex; align-items:center; justify-content:center; gap:6px; font-size:0.8rem; opacity:0.8; padding:8px;">
                ${getIcon('x', 14)} Tutup Dialog
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
    if (iconEl) iconEl.innerHTML = isPickup ? getIcon('box', 22) : getIcon('checkCircle', 22);
    if (titleEl) titleEl.textContent = `Tiba di ${pointName}!`;
    if (subtitleEl) subtitleEl.textContent = isPickup ? 'Menunggu konfirmasi muat barang' : 'Menunggu konfirmasi ambil barang';
    if (btnEl) {
        btnEl.innerHTML = `${getIcon('check', 14)} ${isPickup ? 'Muat' : 'Ambil'}`;
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
                <div class="confirm-icon">${getIcon('box', 44)}</div>
                <div class="confirm-title">Robot Tiba di ${pickupName}!</div>
                <div class="confirm-desc">Robot sudah sampai di titik pickup. Tekan tombol di bawah setelah barang dimuat ke robot.</div>
                <button class="btn btn-confirm btn-confirm-pickup" onclick="confirmAction('confirm_pickup')">
                    ${getIcon('check', 20)} Barang Sudah Dimuat
                </button>
            </div>
        `;
    } else {
        const deliveryName = taskInfo && taskInfo.delivery ? taskInfo.delivery.name : 'Delivery';
        confirmSection.classList.remove('hidden');
        confirmSection.innerHTML = `
            <div class="confirm-card confirm-delivery">
                <div class="confirm-icon">${getIcon('checkCircle', 44)}</div>
                <div class="confirm-title">Robot Tiba di ${deliveryName}!</div>
                <div class="confirm-desc">Robot sudah sampai di titik delivery. Tekan tombol di bawah setelah barang diambil.</div>
                <button class="btn btn-confirm btn-confirm-delivery" onclick="confirmAction('confirm_delivery')">
                    ${getIcon('check', 20)} Barang Sudah Diambil
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
            progressText.textContent = `Task ${(d.current_task_index || 0) + 1}/${d.total_tasks} — ${phaseLabels[d.current_phase] || d.current_phase}`;
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

// Browser Autoplay Policy unlocker
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

// Synthesized clean notification chime (Web Audio API)
function playChimeSound(type = 'waiting_pickup') {
    try {
        const AudioContextClass = window.AudioContext || window.webkitAudioContext;
        if (!AudioContextClass) return;
        const ctx = new AudioContextClass();
        if (ctx.state === 'suspended') {
            ctx.resume();
        }

        const now = ctx.currentTime;
        // Pickup: nada ceria 2-tingkat (E5 -> A5)
        // Delivery / chime: nada melodi 3-tingkat selesai (C5 -> G5 -> C6)
        const notes = type === 'waiting_pickup'
            ? [659.25, 880.0]
            : [523.25, 783.99, 1046.5];

        notes.forEach((freq, idx) => {
            const osc = ctx.createOscillator();
            const gain = ctx.createGain();

            osc.type = 'sine';
            osc.frequency.setValueAtTime(freq, now + idx * 0.16);

            // Volume envelope: smooth attack & decay
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

// Human Voice Text-To-Speech (Google Translate TTS dengan fallback ke Web Speech API)
function speakHumanVoice(text) {
    if (!text) return;

    try {
        // Hentikan audio sebelumnya jika masih berputar
        if (_currentAudio) {
            _currentAudio.pause();
            _currentAudio.currentTime = 0;
            _currentAudio = null;
        }

        // Putar audio Google Translate via backend /api/tts
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

// Pre-load voices on voice changed
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

    // Siapkan teks pemberitahuan
    const speechText = isPickup
        ? (pointName ? `Robot sudah tiba di titik pickup ${pointName}. Silakan muat barang.` : 'Robot sudah sampai di titik pickup. Silakan muat barang.')
        : (pointName ? `Robot sudah tiba di titik tujuan ${pointName}. Silakan ambil barang.` : 'Robot sudah sampai di titik tujuan. Silakan ambil barang.');

    const toastLabel = isPickup
        ? (pointName ? `Robot tiba di pickup: ${pointName}! Silakan muat barang.` : 'Robot sudah sampai di pickup! Silakan muat barang.')
        : (pointName ? `Robot tiba di tujuan: ${pointName}! Silakan ambil barang.` : 'Robot sudah sampai di tujuan! Silakan ambil barang.');

    showToast(toastLabel, 'success');

    // Eksekusi audio berdasarkan preferensi user
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
        const label = command === 'confirm_pickup' ? 'Pickup dikonfirmasi! Robot lanjut mengantar.' : 'Delivery dikonfirmasi! Task selesai.';
        showToast(label, 'success');
        _modalDismissedForCurrentPhase = false;
        hideArrivalModal();
        hideGlobalMissionBanner();
        const sec = document.getElementById('confirm-action-section');
        if (sec) {
            sec.classList.add('hidden');
            sec.innerHTML = '';
        }
        // Ambil status terbaru segera
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

// Window bindings untuk event handler global
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
                <h1 class="cockpit-header-title">Pusat Pemantauan Sistem</h1>
                <p class="cockpit-header-subtitle">Monitoring visual, telemetri kecepatan, dan misi AMR secara real-time</p>
            </div>
            <div style="display:flex; align-items:center; gap:10px;">
                <span class="status-badge" id="cockpit-status-badge">STANDBY</span>
            </div>
        </div>

        <div class="cockpit-grid">
            <!-- LEFT COLUMN: VIEWPORT & LOWER ROW -->
            <div style="display:flex; flex-direction:column; gap:18px;">
                <!-- Card 1: Visual Monitoring (Hero Widget) -->
                <div class="cockpit-card viewport-card">
                    <div class="cockpit-card-header">
                        <div class="cockpit-card-title">
                            ${getIcon('camera', 18)} Visual Pemantauan
                        </div>
                        <div class="viewport-tabs">
                            <button class="viewport-tab-btn active" data-mode="camera" onclick="switchViewportMode('camera')">
                                ${getIcon('camera', 14)} Kamera Live
                            </button>
                            <button class="viewport-tab-btn" data-mode="map" onclick="switchViewportMode('map')">
                                ${getIcon('map', 14)} Peta 2D
                            </button>
                            <button class="viewport-tab-btn" data-mode="dual" onclick="switchViewportMode('dual')">
                                ${getIcon('layers', 14)} Dual Split
                            </button>
                        </div>
                    </div>

                    <!-- Viewport Container -->
                    <div class="viewport-display" id="cockpit-viewport">
                        <!-- View 1: Camera Live -->
                        <div class="viewport-view active" id="view-camera">
                            <img id="cockpit-cam-img" class="cockpit-cam-img" src="/api/camera/stream" alt="Live Kamera"
                                 onerror="this.style.display='none'; document.getElementById('cockpit-cam-offline').style.display='flex';">
                            <div id="cockpit-cam-offline" class="flex-center" style="display:none; width:100%; height:100%; color:var(--text-muted); font-size:0.85rem; flex-direction:column; gap:8px;">
                                ${getIcon('camera', 36)}
                                Kamera tidak tersedia
                            </div>
                            <div class="viewport-hud">
                                <div class="viewport-hud-left">
                                    <span class="hud-badge hud-badge-rec"><span class="rec-dot"></span>LIVE</span>
                                    <span class="hud-badge hud-badge-human" id="hud-human-badge">AI: Aman</span>
                                </div>
                            </div>
                            <div class="viewport-tools">
                                <button class="tool-btn" onclick="captureScreenshot()" title="Ambil Screenshot">${getIcon('camera', 16)}</button>
                            </div>
                        </div>

                        <!-- View 2: 2D Map -->
                        <div class="viewport-view" id="view-map">
                            <div class="map-container" style="width:100%; height:100%; position:relative;">
                                <canvas id="monitor-map-canvas"></canvas>
                                <div class="map-legend">
                                    <span class="legend-robot">Robot</span>
                                </div>
                            </div>
                            <div class="viewport-tools">
                                <button class="tool-btn" onclick="resetMapCenter()" title="Pusatkan ke Robot">${getIcon('navigation', 16)}</button>
                            </div>
                        </div>

                        <!-- View 3: Dual Split (Kamera + Map side by side) -->
                        <div class="viewport-split" id="view-split">
                            <div class="viewport-split-pane">
                                <img id="split-cam-img" class="cockpit-cam-img" src="/api/camera/stream" alt="Live Kamera">
                                <div class="viewport-hud">
                                    <span class="hud-badge hud-badge-rec"><span class="rec-dot"></span>LIVE</span>
                                </div>
                            </div>
                            <div class="viewport-split-pane">
                                <div class="map-container" style="width:100%; height:100%; position:relative;">
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
                                ${getIcon('activity', 18)} Grafik Kecepatan
                            </div>
                            <span style="font-size:0.68rem; color:var(--text-muted);">Real-time</span>
                        </div>
                        <div class="telemetry-chart-container">
                            <canvas id="speed-history-canvas"></canvas>
                        </div>
                        <div class="telemetry-meta">
                            <div>Rata-rata: <b id="tele-avg-speed">0.00 m/s</b></div>
                            <div>Odometer: <b id="tele-odometer">0.0 m</b></div>
                            <div>Misi: <b id="tele-elapsed">0s</b></div>
                        </div>
                    </div>

                    <!-- Card 4: Quick Dispatch Titik Meja -->
                    <div class="cockpit-card">
                        <div class="cockpit-card-header">
                            <div class="cockpit-card-title">
                                ${getIcon('delivery_point', 18)} Quick Dispatch Meja
                            </div>
                            <span style="font-size:0.68rem; color:var(--accent);">1-Click Target</span>
                        </div>
                        <div class="quick-dispatch-grid" id="cockpit-dispatch-grid">
                            <!-- Di-render oleh JavaScript -->
                        </div>
                    </div>
                </div>
            </div>

            <!-- RIGHT COLUMN: SPEEDOMETER, MISSION STATUS, SAFETY -->
            <div style="display:flex; flex-direction:column; gap:18px;">
                <!-- Card 2: Kecepatan & Pose Robot (Menggantikan Temperature) -->
                <div class="cockpit-card gauge-card">
                    <div class="cockpit-card-header" style="width:100%;">
                        <div class="cockpit-card-title">
                            ${getIcon('gauge', 18)} Kecepatan & Pose
                        </div>
                        <span style="font-size:0.68rem; color:var(--text-muted);" id="speed-limit-tag">Maks 0.5 m/s</span>
                    </div>

                    <!-- Speedometer Arc Gauge -->
                    <div class="speed-gauge-wrap">
                        <svg class="speed-gauge-svg" viewBox="0 0 220 140">
                            <defs>
                                <linearGradient id="cockpit-gauge-grad" x1="0%" y1="0%" x2="100%" y2="0%">
                                    <stop offset="0%" stop-color="#00d4ff"/>
                                    <stop offset="65%" stop-color="#00ff88"/>
                                    <stop offset="100%" stop-color="#ffaa00"/>
                                </linearGradient>
                            </defs>
                            <path d="M 25 115 A 85 85 0 0 1 195 115" fill="none" stroke="rgba(255,255,255,0.08)" stroke-width="12" stroke-linecap="round"/>
                            <path id="cockpit-gauge-path" d="M 25 115 A 85 85 0 0 1 195 115" fill="none" stroke="url(#cockpit-gauge-grad)" stroke-width="12" stroke-linecap="round"
                                  stroke-dasharray="267" stroke-dashoffset="267" style="transition: stroke-dashoffset 0.25s ease;"/>
                        </svg>
                        <div class="gauge-readout">
                            <div class="gauge-val" id="cockpit-speed-val">0.00</div>
                            <div class="gauge-subtext">Kecepatan Linear (m/s)</div>
                            <div class="angular-pill" id="cockpit-angular-val">
                                ${getIcon('gauge', 12)} 0.00 rad/s
                            </div>
                        </div>
                    </div>

                    <!-- Coordinates Grid -->
                    <div class="cockpit-pose-grid">
                        <div class="pose-cell">
                            <div class="pose-cell-label">Posisi X</div>
                            <div class="pose-cell-val pose-cell-accent" id="mon-x">0.000</div>
                        </div>
                        <div class="pose-cell">
                            <div class="pose-cell-label">Posisi Y</div>
                            <div class="pose-cell-val pose-cell-accent" id="mon-y">0.000</div>
                        </div>
                        <div class="pose-cell">
                            <div class="pose-cell-label">Orientasi</div>
                            <div class="pose-cell-val pose-cell-warning" id="mon-yaw">0.0°</div>
                        </div>
                    </div>

                    <!-- Distance status -->
                    <div class="distance-bar">
                        <div class="dist-item">Ke Target: <span id="mon-dist-target">—</span></div>
                        <div class="dist-item">Ke Home: <span id="mon-dist-home">—</span></div>
                    </div>
                </div>

                <!-- Card 5: Misi Aktif & Kontrol Cepat -->
                <div class="cockpit-card">
                    <div class="cockpit-card-header">
                        <div class="cockpit-card-title">
                            ${getIcon('delivery', 18)} Misi Pengiriman
                        </div>
                        <span class="status-badge" id="cockpit-mission-badge">IDLE</span>
                    </div>
                    <div class="mission-mini-card">
                        <div class="mission-phase-badge" id="cockpit-phase-badge">
                            <span>Fase Saat Ini:</span>
                            <b id="cockpit-phase-text">Standby</b>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill" id="cockpit-progress-fill" style="width:0%"></div>
                        </div>
                        <div style="display:flex; justify-content:space-between; font-size:0.7rem; color:var(--text-muted);">
                            <span id="cockpit-task-counter">Antrean: 0 Task</span>
                            <span id="cockpit-target-info">—</span>
                        </div>

                        <!-- Tombol Konfirmasi Kedatangan Cepat di Cockpit -->
                        <div id="cockpit-confirm-btn-wrap" class="hidden" style="margin-top:6px;">
                            <button id="cockpit-confirm-btn" class="btn btn-confirm btn-block" onclick="onCockpitConfirmArrival()">
                                ${getIcon('check', 18)} Konfirmasi Kedatangan
                            </button>
                        </div>

                        <!-- Quick Mission Controls -->
                        <div class="cockpit-ctrl-row">
                            <button class="btn btn-warning" onclick="controlMission('pause')">${getIcon('pause', 14)} Pause</button>
                            <button class="btn btn-primary" onclick="controlMission('resume')">${getIcon('play', 14)} Resume</button>
                            <button class="btn btn-danger" onclick="controlMission('cancel')">${getIcon('x', 14)} Batal</button>
                        </div>
                    </div>
                </div>

                <!-- Card 6: Sistem Keamanan & AI Vision -->
                <div class="cockpit-card">
                    <div class="cockpit-card-header">
                        <div class="cockpit-card-title">
                            ${getIcon('shield', 18)} Keamanan & Sensor
                        </div>
                        <span style="font-size:0.68rem; color:var(--success);">Proteksi Aktif</span>
                    </div>
                    <div class="nodes-grid">
                        <div class="node-item">
                            <span class="node-name">AI Vision</span>
                            <span class="node-status" id="cockpit-human-status" style="color:var(--success);">Aman</span>
                        </div>
                        <div class="node-item">
                            <span class="node-name">Sensor LiDAR</span>
                            <span class="node-status node-status-ok" id="cockpit-lidar-status">Aman</span>
                        </div>
                        <div class="node-item">
                            <span class="node-name">Nav2 Stack</span>
                            <span class="node-status node-status-ok">Active</span>
                        </div>
                        <div class="node-item">
                            <span class="node-name">E-Stop System</span>
                            <span class="node-status" style="color:var(--accent);">Ready</span>
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
        <div class="page-section">
            <div class="section-title">Live Feed Kamera</div>
            <div class="camera-frame">
                <img id="camera-feed" src="/api/camera/stream" alt="Kamera AMR"
                     onerror="this.style.display='none'; document.getElementById('cam-offline').style.display='flex';">
                <div class="camera-overlay">
                    <div class="camera-badge"><span class="rec-dot"></span>LIVE</div>
                </div>
                <div id="cam-offline" class="flex-center" style="display:none; min-height:250px; color:var(--text-muted); font-size:0.85rem; flex-direction:column; gap:8px;">
                    ${getIcon('camera', 36)}
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
            <button class="btn btn-outline btn-block" onclick="captureScreenshot()">${getIcon('camera', 16)} Screenshot</button>
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
            if (data.delivery && typeof handleDeliveryStatusUpdate === 'function') {
                handleDeliveryStatusUpdate(data.delivery);
            }
            const camHuman = document.getElementById('cam-human');
            const camPos = document.getElementById('cam-pos');
            if (camHuman) camHuman.textContent = data.human_status || '—';
            if (camPos) camPos.textContent = `${data.pose.x.toFixed(1)}, ${data.pose.y.toFixed(1)}`;
        }
    }, 1000);
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
            <button class="btn btn-danger btn-block btn-sm" onclick="clearHistory()">${getIcon('trash', 15)} Hapus Semua Riwayat</button>
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
            <div class="section-title">Kamera Feed</div>
            <div class="camera-frame">
                <img src="/api/camera/stream" alt="Kamera AMR"
                     onerror="this.style.display='none'; this.nextElementSibling.style.display='flex';">
                <div class="camera-overlay">
                    <div class="camera-badge"><span class="rec-dot"></span>LIVE</div>
                </div>
                <div class="flex-center" style="display:none; min-height:160px; color:var(--text-muted); font-size:0.8rem; flex-direction:column; gap:6px;">
                    ${getIcon('camera', 28)} Kamera tidak tersedia
                </div>
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
    _modalDismissedForCurrentPhase = false;
    hideArrivalModal();
    hideGlobalMissionBanner();
    showToast('EMERGENCY STOP!', 'error');
}

// ====================== PAGE: SETTINGS ======================
function renderSettings() {
    const div = document.createElement('div');
    div.className = 'page-enter';

    const s = APP.settings || {};
    const currentSoundMode = getSoundMode();

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
                <button class="btn btn-primary btn-block mt-12" onclick="saveSettings()">${getIcon('save', 16)} Simpan Pengaturan</button>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Notifikasi Suara Kedatangan</div>
            <div class="panel">
                <div class="setting-row">
                    <div>
                        <div class="setting-label">Tipe Suara</div>
                        <div class="setting-desc">Pemberitahuan saat tiba di titik pickup & deliver</div>
                    </div>
                    <select class="input-field" id="set-sound-mode" onchange="setSoundMode(this.value)"
                            style="width: auto; min-width: 195px; padding: 7px 12px; font-size: 0.8rem; cursor: pointer;">
                        <option value="human" ${currentSoundMode === 'human' ? 'selected' : ''}>🗣️ Suara Manusia (TTS)</option>
                        <option value="chime" ${currentSoundMode === 'chime' ? 'selected' : ''}>🔔 Suara Notifikasi Biasa</option>
                        <option value="both" ${currentSoundMode === 'both' ? 'selected' : ''}>🎵 Keduanya (Chime + Suara)</option>
                        <option value="mute" ${currentSoundMode === 'mute' ? 'selected' : ''}>🔇 Senyap (Mute)</option>
                    </select>
                </div>
                <div class="setting-row" style="flex-wrap: wrap; gap: 8px;">
                    <div>
                        <div class="setting-label">Uji Coba Suara</div>
                        <div class="setting-desc">Dengarkan contoh suara notifikasi di browser</div>
                    </div>
                    <div style="display: flex; gap: 6px; flex-wrap: wrap;">
                        <button type="button" class="btn btn-outline btn-sm" onclick="testArrivalSound('waiting_pickup')">🔊 Tes Pickup</button>
                        <button type="button" class="btn btn-outline btn-sm" onclick="testArrivalSound('waiting_delivery')">🔊 Tes Deliver</button>
                        <button type="button" class="btn btn-outline btn-sm" onclick="playChimeSound('waiting_pickup')">🔔 Tes Chime</button>
                    </div>
                </div>
            </div>
        </div>

        <div class="page-section">
            <div class="section-title">Titik Tersimpan</div>
            <div class="panel" id="saved-points-list"></div>
            <button class="btn btn-outline btn-block btn-sm mt-8" onclick="showAddPointModal()">${getIcon('plus', 14)} Tambah Titik Baru</button>
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
                <div class="setting-desc icon-inline">${p.type === 'pickup' ? getIcon('pickup', 13) + ' Pickup' : getIcon('delivery_point', 13) + ' Delivery'} — (${p.x}, ${p.y})</div>
            </div>
            <button class="btn btn-outline btn-sm" style="padding:6px 10px; font-size:0.7rem;"
                    onclick="deletePoint('${p.name}')">${getIcon('trash', 14)}</button>
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
        arrival_sound_mode: soundMode
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

        if (typeof handleDeliveryStatusUpdate === 'function') {
            handleDeliveryStatusUpdate(APP.delivery);
        }

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
                idle: 'Siap', navigating_pickup: 'Menuju Pickup',
                waiting_pickup: 'Menunggu Muat', at_pickup: 'Di Pickup',
                navigating_delivery: 'Menuju Tujuan',
                waiting_delivery: 'Menunggu Ambil', at_delivery: 'Di Tujuan',
                returning_home: 'Kembali ke Titik Awal'
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

// ====================== NAV ICONS & LIVE CLOCK ======================
function populateNavIcons() {
    if (typeof getIcon !== 'function') return;

    // Sidebar icons
    const sidebarMap = {
        's-icon-home': 'home',
        's-icon-monitoring': 'activity',
        's-icon-delivery': 'package',
        's-icon-camera': 'video',
        's-icon-history': 'clock',
        's-icon-joystick': 'compass',
        's-icon-settings': 'settings'
    };
    Object.entries(sidebarMap).forEach(([id, icon]) => {
        const el = document.getElementById(id);
        if (el) el.innerHTML = getIcon(icon, 20);
    });

    // Mobile nav icons
    const mobileMap = {
        'm-icon-home': 'home',
        'm-icon-monitoring': 'activity',
        'm-icon-delivery': 'package',
        'm-icon-camera': 'video',
        'm-icon-joystick': 'compass'
    };
    Object.entries(mobileMap).forEach(([id, icon]) => {
        const el = document.getElementById(id);
        if (el) el.innerHTML = getIcon(icon, 20);
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
    // Populate navigation icons and start real-time header clock
    populateNavIcons();
    startLiveClock();

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
    setInterval(globalStatusPoll, 1000);
}

// Start when DOM ready
document.addEventListener('DOMContentLoaded', init);
