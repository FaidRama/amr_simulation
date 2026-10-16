/* =================================================================
   AMR-01 // CONTROL STUDIO — Comprehensive Monitoring Cockpit Module
   Provides live viewport switching (Camera / 2D Map / Dual),
   Speedometer Arc Gauge, Real-time Speed Chart, Telemetry & Dispatch.
   ================================================================= */

let cockpitViewportMode = 'camera'; // 'camera', 'map', 'dual'
let speedHistory = [];
const MAX_SPEED_HISTORY = 30;
let lastPoseRecord = null;
let lastPoseTimestamp = 0;
let totalDistanceTraveled = 0.0;

// ====================== VIEWPORT SWITCHER ======================
function switchViewportMode(mode) {
    cockpitViewportMode = mode;

    document.querySelectorAll('.viewport-tab-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === mode);
    });

    const camView = document.getElementById('view-camera');
    const mapView = document.getElementById('view-map');
    const splitView = document.getElementById('view-split');

    if (!camView || !mapView || !splitView) return;

    camView.classList.remove('active');
    mapView.classList.remove('active');
    splitView.classList.remove('active');

    if (mode === 'camera') {
        camView.classList.add('active');
        const img = document.getElementById('cockpit-cam-img');
        if (img) {
            img.style.display = 'block';
            img.src = '/api/camera/stream';
        }
    } else if (mode === 'map') {
        mapView.classList.add('active');
        if (typeof initMonitorMap === 'function') {
            initMonitorMap('monitor-map-canvas');
        }
    } else if (mode === 'dual') {
        splitView.classList.add('active');
        const splitImg = document.getElementById('split-cam-img');
        if (splitImg) {
            splitImg.src = '/api/camera/stream';
        }
        if (typeof initMonitorMap === 'function') {
            initMonitorMap('split-map-canvas');
        }
    }
}

function resetMapCenter() {
    showToast('Peta disinkronkan ke posisi AMR-01', 'info');
}

// ====================== SPEEDOMETER GAUGE ======================
function updateSpeedGauge(linearSpeed, angularSpeed) {
    const meterPath = document.getElementById('cockpit-gauge-path');
    const speedValEl = document.getElementById('cockpit-speed-val');
    const angularValEl = document.getElementById('cockpit-angular-val');
    const limitTag = document.getElementById('speed-limit-tag');

    const maxSpeed = (APP.settings && (APP.settings.max_linear_speed || APP.settings.max_linear_vel))
        ? (APP.settings.max_linear_speed || APP.settings.max_linear_vel)
        : 0.7;

    if (limitTag) {
        limitTag.textContent = `MAX ${maxSpeed.toFixed(1)} m/s`;
    }

    const safeLinear = Math.max(0, Math.abs(linearSpeed || 0));
    const safeAngular = angularSpeed || 0;

    if (speedValEl) {
        speedValEl.textContent = safeLinear.toFixed(2);
    }
    if (angularValEl) {
        angularValEl.innerHTML = `${getIcon('compass', 12)} ${safeAngular >= 0 ? '+' : ''}${safeAngular.toFixed(2)} rad/s`;
    }

    if (meterPath) {
        const totalLength = 267;
        const ratio = Math.min(1.0, Math.max(0.0, safeLinear / maxSpeed));
        const offset = totalLength * (1.0 - ratio);
        meterPath.style.strokeDashoffset = offset.toFixed(1);
    }
}

// ====================== REAL-TIME SPEED CHART ======================
function pushSpeedSample(val) {
    speedHistory.push(Math.max(0, val));
    if (speedHistory.length > MAX_SPEED_HISTORY) {
        speedHistory.shift();
    }
    drawSpeedHistoryChart();
}

function drawSpeedHistoryChart() {
    const canvas = document.getElementById('speed-history-canvas');
    if (!canvas) return;

    const styles = getComputedStyle(document.documentElement);
    const accentColor = styles.getPropertyValue('--accent').trim() || '#3b82f6';
    const accentGlow = styles.getPropertyValue('--accent-glow').trim() || 'rgba(59, 130, 246, 0.2)';
    const successColor = styles.getPropertyValue('--success').trim() || '#10b981';
    const mutedColor = styles.getPropertyValue('--text-muted').trim() || '#64748b';
    const gridColor = styles.getPropertyValue('--chart-grid').trim() || 'rgba(148, 163, 184, 0.08)';

    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 300;
    const h = canvas.clientHeight || 120;

    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
    }

    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    // Draw theme-aware technical grid lines
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 1;
    for (let y = 18; y < h; y += 28) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
    }
    for (let x = 40; x < w; x += 50) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
    }

    if (speedHistory.length < 2) {
        ctx.fillStyle = mutedColor;
        ctx.font = '500 10px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText('SAMPLING VELOCITY TELEMETRY...', w / 2, h / 2);
        ctx.restore();
        return;
    }

    const maxSpeed = (APP.settings && (APP.settings.max_linear_speed || APP.settings.max_linear_vel))
        ? (APP.settings.max_linear_speed || APP.settings.max_linear_vel)
        : 0.7;
    const paddingBottom = 8;
    const paddingTop = 10;
    const plotH = h - paddingTop - paddingBottom;
    const stepX = w / (MAX_SPEED_HISTORY - 1);
    const startX = w - (speedHistory.length - 1) * stepX;

    // Theme-aware area gradient
    const fillGrad = ctx.createLinearGradient(0, paddingTop, 0, h);
    fillGrad.addColorStop(0, accentGlow);
    fillGrad.addColorStop(1, 'rgba(0, 0, 0, 0)');

    ctx.beginPath();
    ctx.moveTo(startX, h - paddingBottom);

    for (let i = 0; i < speedHistory.length; i++) {
        const x = startX + i * stepX;
        const norm = Math.min(1.0, Math.max(0.0, speedHistory[i] / maxSpeed));
        const y = h - paddingBottom - (norm * plotH);
        ctx.lineTo(x, y);
    }

    ctx.lineTo(w, h - paddingBottom);
    ctx.closePath();
    ctx.fillStyle = fillGrad;
    ctx.fill();

    // Precision stroke line
    ctx.beginPath();
    for (let i = 0; i < speedHistory.length; i++) {
        const x = startX + i * stepX;
        const norm = Math.min(1.0, Math.max(0.0, speedHistory[i] / maxSpeed));
        const y = h - paddingBottom - (norm * plotH);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = accentColor;
    ctx.lineWidth = 2;
    ctx.stroke();

    // Live indicator dot
    const lastX = w - 3;
    const lastNorm = Math.min(1.0, Math.max(0.0, speedHistory[speedHistory.length - 1] / maxSpeed));
    const lastY = h - paddingBottom - (lastNorm * plotH);
    ctx.beginPath();
    ctx.arc(lastX, lastY, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = successColor;
    ctx.fill();

    ctx.restore();

    const avgEl = document.getElementById('tele-avg-speed');
    if (avgEl && speedHistory.length > 0) {
        const sum = speedHistory.reduce((a, b) => a + b, 0);
        const avg = sum / speedHistory.length;
        avgEl.textContent = `${avg.toFixed(2)} m/s`;
    }
}

// ====================== QUICK DISPATCH ======================
function renderQuickDispatchChips(targetId = 'cockpit-dispatch-grid') {
    const container = document.getElementById(targetId);
    if (!container) return;

    const points = APP.savedPoints || [];
    let html = '';

    const home = APP.homePosition || { x: 0, y: 0, yaw: 0 };
    html += `
        <button class="dispatch-chip" onclick="quickDispatchTo('Home Base', ${home.x}, ${home.y}, ${home.yaw || 0})">
            ${getIcon('home', 14)}
            <span>Home</span>
        </button>
    `;

    points.forEach(pt => {
        const iconName = pt.type === 'pickup' ? 'pickup' : 'delivery_point';
        html += `
            <button class="dispatch-chip" onclick="quickDispatchTo('${pt.name}', ${pt.x}, ${pt.y}, ${pt.yaw || 0})">
                ${getIcon(iconName, 14)}
                <span>${pt.name}</span>
            </button>
        `;
    });

    container.innerHTML = html;
}

async function quickDispatchTo(name, x, y, yaw) {
    if (APP.delivery && APP.delivery.status === 'running') {
        showToast('Misi sedang berjalan. Selesaikan atau batalkan terlebih dahulu.', 'warning');
        return;
    }

    showModal(
        `Quick Dispatch → ${name}`,
        `<p>Kirim unit AMR-01 langsung menuju titik <strong>${name}</strong> <span class="mono">[X: ${x.toFixed(2)}, Y: ${y.toFixed(2)}]</span>?</p>`,
        [
            { label: 'Batal', class: 'btn-outline', action: hideModal },
            {
                label: 'Eksekusi Dispatch',
                class: 'btn-primary',
                action: async () => {
                    const tasks = [{
                        pickup: { name: 'Posisi Saat Ini', x: APP.pose.x, y: APP.pose.y },
                        delivery: { name: name, x: x, y: y, yaw: yaw }
                    }];
                    const res = await apiPost('/api/delivery/start', { tasks });
                    if (res && res.success) {
                        showToast(`AMR-01 bergerak menuju ${name}`, 'success');
                    } else {
                        showToast('Gagal memulai navigasi', 'error');
                    }
                }
            }
        ]
    );
}

async function onCockpitConfirmArrival() {
    if (!APP.delivery) return;
    const cmd = APP.delivery.current_phase === 'waiting_pickup' ? 'confirm_pickup' : 'confirm_delivery';
    await confirmAction(cmd);
}

// ====================== DATA UPDATER FOR COCKPIT ======================
function updateCockpitData(data) {
    if (!data) return;

    const pose = data.pose || APP.pose;
    const d = data.delivery || APP.delivery;
    const human = data.human_status || 'NO_HUMAN';

    let linSpeed = 0.0;
    let angSpeed = 0.0;

    if (data.speed) {
        linSpeed = data.speed.linear || 0.0;
        angSpeed = data.speed.angular || 0.0;
    } else if (data.cmd_vel) {
        linSpeed = data.cmd_vel.linear || 0.0;
        angSpeed = data.cmd_vel.angular || 0.0;
    }

    const now = Date.now();
    if (lastPoseRecord && lastPoseTimestamp > 0) {
        const dt = (now - lastPoseTimestamp) / 1000.0;
        if (dt > 0.05 && dt < 2.0) {
            const dist = Math.hypot(pose.x - lastPoseRecord.x, pose.y - lastPoseRecord.y);
            if (dist > 0.001) {
                totalDistanceTraveled += dist;
                if (Math.abs(linSpeed) < 0.01) {
                    linSpeed = dist / dt;
                }
            }
        }
    }
    lastPoseRecord = { x: pose.x, y: pose.y };
    lastPoseTimestamp = now;

    pushSpeedSample(linSpeed);
    updateSpeedGauge(linSpeed, angSpeed);

    const xEl = document.getElementById('mon-x');
    const yEl = document.getElementById('mon-y');
    const yawEl = document.getElementById('mon-yaw');
    if (xEl) xEl.textContent = pose.x.toFixed(3);
    if (yEl) yEl.textContent = pose.y.toFixed(3);
    if (yawEl) yawEl.textContent = (pose.yaw * 180 / Math.PI).toFixed(1) + '°';

    const home = APP.homePosition || { x: 0, y: 0 };
    const distHome = Math.hypot(pose.x - home.x, pose.y - home.y);
    const homeEl = document.getElementById('mon-dist-home');
    if (homeEl) homeEl.textContent = `${distHome.toFixed(2)}m`;

    const targetEl = document.getElementById('mon-dist-target');
    if (targetEl) {
        if (d.status === 'running' && d.tasks && d.tasks[d.current_task_index || 0]) {
            const curTask = d.tasks[d.current_task_index || 0];
            const targetPt = (d.current_phase === 'navigating_pickup' || d.current_phase === 'waiting_pickup')
                ? curTask.pickup : curTask.delivery;
            const distTarget = Math.hypot(pose.x - targetPt.x, pose.y - targetPt.y);
            targetEl.textContent = `${distTarget.toFixed(2)}m (${targetPt.name})`;
        } else {
            targetEl.textContent = '—';
        }
    }

    const odoEl = document.getElementById('tele-odometer');
    if (odoEl) odoEl.textContent = `${totalDistanceTraveled.toFixed(1)} m`;

    const elapsedEl = document.getElementById('tele-elapsed');
    if (elapsedEl && d.elapsed_time) {
        elapsedEl.textContent = formatElapsed(d.elapsed_time);
    }

    const statusBadge = document.getElementById('cockpit-status-badge');
    if (statusBadge) {
        statusBadge.textContent = (d.status || 'idle').toUpperCase();
        statusBadge.className = `status-badge status-${d.status || 'idle'}`;
    }

    const missionBadge = document.getElementById('cockpit-mission-badge');
    if (missionBadge) {
        missionBadge.textContent = (d.status || 'idle').toUpperCase();
        missionBadge.className = `status-badge status-${d.status || 'idle'}`;
    }

    const phaseLabels = {
        idle: 'Standby / Siap',
        navigating_pickup: 'Menuju Titik Pickup',
        at_pickup: 'Tiba di Titik Pickup',
        waiting_pickup: 'Menunggu Muat Barang',
        navigating_delivery: 'Mengantar ke Tujuan',
        at_delivery: 'Tiba di Titik Tujuan',
        waiting_delivery: 'Menunggu Ambil Barang',
        returning_home: 'Kembali ke Home Base',
    };
    const phaseText = document.getElementById('cockpit-phase-text');
    if (phaseText) {
        phaseText.textContent = phaseLabels[d.current_phase] || d.current_phase || 'Standby';
    }

    const progressFill = document.getElementById('cockpit-progress-fill');
    if (progressFill) {
        const pct = (d.total_tasks > 0) ? Math.round((d.completed_tasks / d.total_tasks) * 100) : 0;
        progressFill.style.width = `${pct}%`;
    }

    const taskCounter = document.getElementById('cockpit-task-counter');
    if (taskCounter) {
        if (d.total_tasks > 0) {
            taskCounter.textContent = `TASK ${(d.current_task_index || 0) + 1}/${d.total_tasks}`;
        } else {
            taskCounter.textContent = 'QUEUE: 0';
        }
    }

    const targetInfo = document.getElementById('cockpit-target-info');
    if (targetInfo) {
        if (d.tasks && d.tasks[d.current_task_index || 0]) {
            const curTask = d.tasks[d.current_task_index || 0];
            targetInfo.textContent = `${curTask.pickup.name} → ${curTask.delivery.name}`;
        } else {
            targetInfo.textContent = '—';
        }
    }

    const confirmWrap = document.getElementById('cockpit-confirm-btn-wrap');
    const confirmBtn = document.getElementById('cockpit-confirm-btn');
    const isWaiting = (d.current_phase === 'waiting_pickup' || d.current_phase === 'waiting_delivery') && d.status === 'running';

    if (confirmWrap && confirmBtn) {
        if (isWaiting) {
            confirmWrap.classList.remove('hidden');
            const isPickup = (d.current_phase === 'waiting_pickup');
            confirmBtn.className = `btn btn-confirm ${isPickup ? 'btn-confirm-pickup' : 'btn-confirm-delivery'} btn-block`;
            confirmBtn.innerHTML = `${getIcon('check', 16)} ${isPickup ? 'Konfirmasi Barang Dimuat' : 'Konfirmasi Barang Diambil'}`;
        } else {
            confirmWrap.classList.add('hidden');
        }
    }

    const humanStatusEl = document.getElementById('cockpit-human-status');
    const hudHumanBadge = document.getElementById('hud-human-badge');
    const humanLabel = getHumanStatusLabel(human);

    if (humanStatusEl) {
        humanStatusEl.textContent = humanLabel;
        humanStatusEl.style.color = (human === 'DISTURBING' || human === 'CALLING') ? 'var(--danger)' :
            (human === 'WALKING' ? 'var(--warning)' : 'var(--success)');
    }
    if (hudHumanBadge) {
        hudHumanBadge.textContent = `AI: ${humanLabel.toUpperCase()}`;
        hudHumanBadge.style.color = (human === 'DISTURBING' || human === 'CALLING') ? 'var(--danger)' :
            (human === 'WALKING' ? 'var(--warning)' : 'var(--success)');
    }
}

function initCockpit() {
    renderQuickDispatchChips('cockpit-dispatch-grid');
    drawSpeedHistoryChart();
    switchViewportMode(cockpitViewportMode || 'camera');
}

function formatElapsed(seconds) {
    if (!seconds || seconds <= 0) return '0s';
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = Math.floor(seconds % 60);
    if (h > 0) return `${h}h ${m}m ${s}s`;
    if (m > 0) return `${m}m ${s}s`;
    return `${s}s`;
}

function getHumanStatusLabel(status) {
    const labels = {
        'NO_HUMAN': 'Clear (Aman)',
        'IDLE': 'Terdeteksi',
        'WALKING': 'Berjalan',
        'DISTURBING': 'Menghalangi!',
        'CALLING': 'Memanggil!',
        'CALIBRATION': 'Kalibrasi...'
    };
    return labels[status] || status;
}

window.switchViewportMode = switchViewportMode;
window.resetMapCenter = resetMapCenter;
window.quickDispatchTo = quickDispatchTo;
window.renderQuickDispatchChips = renderQuickDispatchChips;
window.onCockpitConfirmArrival = onCockpitConfirmArrival;
window.initCockpit = initCockpit;
window.updateCockpitData = updateCockpitData;
window.drawSpeedHistoryChart = drawSpeedHistoryChart;
