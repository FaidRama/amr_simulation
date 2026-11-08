/* =================================================================
   AMR-01 // CONTROL STUDIO — Interactive 2D SLAM Map Module
   Renders the warehouse map with robot position, heading vector,
   waypoints, and supports tap-to-add-waypoint.
   ================================================================= */

// ====================== MAP STATE ======================
const MAP_STATE = {
    image: null,
    metadata: null,
    loaded: false,
    imgWidth: 0,
    imgHeight: 0,
    canvasWidth: 0,
    canvasHeight: 0,
    scale: 1,
    lastLiveRefresh: 0,
    liveSlamImage: null,
    liveSlamMeta: null,
    liveSlamLoaded: false,
    lastSlamFetch: 0,
    slamFetchInFlight: false,
    robotTrail: [],
};

let deliveryMapAnimFrame = null;
let monitorMapAnimFrame = null;
let mappingMapAnimFrame = null;

function stopAllMapLoops() {
    if (deliveryMapAnimFrame) {
        cancelAnimationFrame(deliveryMapAnimFrame);
        deliveryMapAnimFrame = null;
    }
    if (monitorMapAnimFrame) {
        cancelAnimationFrame(monitorMapAnimFrame);
        monitorMapAnimFrame = null;
    }
    if (mappingMapAnimFrame) {
        cancelAnimationFrame(mappingMapAnimFrame);
        mappingMapAnimFrame = null;
    }
}

function clearRobotTrail(showToastMsg = true) {
    MAP_STATE.robotTrail = [];
    if (showToastMsg && typeof showToast === 'function') {
        showToast('Jejak lintasan robot pada peta dibersihkan', 'info');
    }
}
window.clearRobotTrail = clearRobotTrail;

// ====================== LOAD / RELOAD MAP ======================
async function loadMapData(forceReload = false) {
    if (MAP_STATE.loaded && !forceReload) return true;

    try {
        const meta = await apiGet('/api/map/metadata?t=' + Date.now());
        if (!meta) return false;
        MAP_STATE.metadata = meta;

        return new Promise((resolve) => {
            const img = new Image();
            img.onload = () => {
                MAP_STATE.image = img;
                MAP_STATE.imgWidth = img.naturalWidth;
                MAP_STATE.imgHeight = img.naturalHeight;
                MAP_STATE.loaded = true;
                MAP_STATE.lastLiveRefresh = Date.now();
                resolve(true);
            };
            img.onerror = () => resolve(false);
            img.src = '/api/map?t=' + Date.now();
        });
    } catch (e) {
        console.error('Failed to load map:', e);
        return false;
    }
}

async function reloadMapData() {
    MAP_STATE.loaded = false;
    return await loadMapData(true);
}

// ====================== COORDINATE CONVERSION ======================
function worldToPixel(wx, wy) {
    if (!MAP_STATE.metadata) return { px: 0, py: 0 };

    const resolution = MAP_STATE.metadata.resolution || 0.05;
    const origin = MAP_STATE.metadata.origin || [-34.4, -3.16, 0];

    const px = (wx - origin[0]) / resolution;
    const py = MAP_STATE.imgHeight - (wy - origin[1]) / resolution;

    return {
        px: px * MAP_STATE.scale,
        py: py * MAP_STATE.scale
    };
}

function pixelToWorld(px, py) {
    if (!MAP_STATE.metadata) return { wx: 0, wy: 0 };

    const resolution = MAP_STATE.metadata.resolution || 0.05;
    const origin = MAP_STATE.metadata.origin || [-34.4, -3.16, 0];

    const imgPx = px / MAP_STATE.scale;
    const imgPy = py / MAP_STATE.scale;

    const wx = imgPx * resolution + origin[0];
    const wy = (MAP_STATE.imgHeight - imgPy) * resolution + origin[1];

    return { wx: Math.round(wx * 100) / 100, wy: Math.round(wy * 100) / 100 };
}

function screenToWorld(clientX, clientY, canvas, viewport = null) {
    if (!canvas || !MAP_STATE.metadata) return { wx: 0, wy: 0 };
    const rect = canvas.getBoundingClientRect();
    const canvasPx = (clientX - rect.left) * (canvas.width / (rect.width || 1));
    const canvasPy = (clientY - rect.top) * (canvas.height / (rect.height || 1));

    let basePx = canvasPx;
    let basePy = canvasPy;

    if (viewport) {
        basePx = (canvasPx - viewport.panX) / viewport.zoom;
        basePy = (canvasPy - viewport.panY) / viewport.zoom;
    }

    return pixelToWorld(basePx, basePy);
}

// ====================== DRAW FUNCTIONS ======================
function drawMap(canvas, ctx, options = {}) {
    if (!MAP_STATE.loaded || !canvas || !canvas.parentElement) return;

    // Jika sedang mode 'live' (SLAM), refresh gambar peta tiap 2.5 detik di background
    if (MAP_STATE.metadata && MAP_STATE.metadata.map_file === 'live') {
        if (Date.now() - MAP_STATE.lastLiveRefresh > 2500) {
            MAP_STATE.lastLiveRefresh = Date.now();
            loadMapData(true);
        }
    }

    const {
        showRobot = true,
        showWaypoints = false,
        showTaskQueue = false,
        viewport = null,
    } = options;

    const container = canvas.parentElement;
    const isFs = container && container.classList.contains('map-fullscreen');
    const containerWidth = isFs ? window.innerWidth : (container ? (container.clientWidth || 400) : 400);
    const containerHeight = isFs ? window.innerHeight : Math.round(MAP_STATE.imgHeight * (containerWidth / MAP_STATE.imgWidth));

    MAP_STATE.scale = containerWidth / MAP_STATE.imgWidth;
    MAP_STATE.canvasWidth = containerWidth;
    MAP_STATE.canvasHeight = isFs ? containerHeight : Math.round(MAP_STATE.imgHeight * MAP_STATE.scale);

    if (canvas.width !== MAP_STATE.canvasWidth || canvas.height !== MAP_STATE.canvasHeight) {
        canvas.width = MAP_STATE.canvasWidth;
        canvas.height = MAP_STATE.canvasHeight;
    }

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    if (viewport) {
        ctx.translate(viewport.panX, viewport.panY);
        ctx.scale(viewport.zoom, viewport.zoom);
    }

    const zoom = (viewport && viewport.zoom) ? viewport.zoom : 1.0;
    const invZoom = 1.0 / zoom;

    // Draw SLAM occupancy grid image
    ctx.drawImage(MAP_STATE.image, 0, 0, MAP_STATE.canvasWidth, Math.round(MAP_STATE.imgHeight * MAP_STATE.scale));

    // Apply subtle studio dark contrast overlay
    ctx.fillStyle = 'rgba(9, 9, 11, 0.22)';
    ctx.fillRect(0, 0, MAP_STATE.canvasWidth, Math.round(MAP_STATE.imgHeight * MAP_STATE.scale));

    // Draw Home / Docking Station marker
    if (showWaypoints && APP.homePosition) {
        const hPos = worldToPixel(APP.homePosition.x || 0, APP.homePosition.y || 0);
        drawHomeMarker(ctx, hPos.px, hPos.py, zoom);
    }

    // Draw predefined waypoints
    if (showWaypoints && APP.savedPoints) {
        APP.savedPoints.forEach(p => {
            const { px, py } = worldToPixel(p.x, p.y);
            drawWaypointMarker(ctx, px, py, p.type, p.name, zoom);
        });
    }

    // Draw task queue waypoints & route lines
    if (showTaskQueue && APP.taskQueue) {
        APP.taskQueue.forEach((task, i) => {
            const pickup = worldToPixel(task.pickup.x, task.pickup.y);
            const delivery = worldToPixel(task.delivery.x, task.delivery.y);

            // Vector route line between pickup and delivery
            ctx.beginPath();
            ctx.setLineDash([5 * invZoom, 4 * invZoom]);
            ctx.strokeStyle = 'rgba(59, 130, 246, 0.65)';
            ctx.lineWidth = Math.max(1.0, 1.5 * invZoom);
            ctx.moveTo(pickup.px, pickup.py);
            ctx.lineTo(delivery.px, delivery.py);
            ctx.stroke();
            ctx.setLineDash([]);

            drawTaskMarker(ctx, pickup.px, pickup.py, 'pickup', i + 1, zoom);
            drawTaskMarker(ctx, delivery.px, delivery.py, 'delivery', i + 1, zoom);
        });
    }

    // Record & Draw Robot Trajectory Trail
    if (showRobot && typeof recordRobotTrailPoint === 'function') {
        recordRobotTrailPoint(APP.pose.x || 0, APP.pose.y || 0);
        if (MAP_STATE.robotTrail.length > 1) {
            ctx.save();
            ctx.beginPath();
            let prevPt = null;
            MAP_STATE.robotTrail.forEach((pt, idx) => {
                const { px, py } = worldToPixel(pt.x, pt.y);
                if (idx === 0 || !prevPt || Math.hypot(pt.x - prevPt.x, pt.y - prevPt.y) > 1.2) {
                    ctx.moveTo(px, py);
                } else {
                    ctx.lineTo(px, py);
                }
                prevPt = pt;
            });
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.75)';
            ctx.lineWidth = Math.max(1.0, 2.0 * invZoom);
            ctx.stroke();
            ctx.restore();
        }
    }

    // Draw Active Nav2 Navigation Planned Path (/plan)
    if (APP.navPath && APP.navPath.length > 1) {
        ctx.save();
        ctx.beginPath();
        APP.navPath.forEach((pt, idx) => {
            const { px, py } = worldToPixel(pt.x, pt.y);
            if (idx === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.strokeStyle = '#06b6d4'; // Cyan neon navigation path
        ctx.lineWidth = Math.max(1.2, 2.2 * invZoom);
        ctx.setLineDash([5 * invZoom, 3 * invZoom]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
    }

    // Draw robot position & orientation
    if (showRobot) {
        const { px, py } = worldToPixel(APP.pose.x, APP.pose.y);
        drawRobot(ctx, px, py, APP.pose.yaw, zoom);
    }

    ctx.restore();
}

function drawHomeMarker(ctx, x, y, zoom = 1.0) {
    const s = 1.0 / (zoom || 1.0);
    const r = 5.5 * s;

    // Outer subtle halo
    ctx.beginPath();
    ctx.arc(x, y, r + 3 * s, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(168, 85, 247, 0.22)';
    ctx.fill();

    // Core circle
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = '#a855f7';
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = Math.max(0.8, 1.2 * s);
    ctx.stroke();

    // Micro reticle ticks at center
    const ch = 2.2 * s;
    ctx.beginPath();
    ctx.moveTo(x - ch, y);
    ctx.lineTo(x + ch, y);
    ctx.moveTo(x, y - ch);
    ctx.lineTo(x, y + ch);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.75)';
    ctx.lineWidth = Math.max(0.6, 0.8 * s);
    ctx.stroke();

    // Center pinpoint dot
    ctx.beginPath();
    ctx.arc(x, y, 1.2 * s, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // Monospace 'H' icon badge
    ctx.font = `700 ${8 * s}px "JetBrains Mono", monospace`;
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('H', x, y - 0.5 * s);
    ctx.textBaseline = 'alphabetic';
}

function drawWaypointMarker(ctx, x, y, type, label, zoom = 1.0) {
    const s = 1.0 / (zoom || 1.0);
    const color = type === 'pickup' ? '#3b82f6' : '#10b981';
    const radius = 5 * s;

    // Outer subtle technical halo
    ctx.beginPath();
    ctx.arc(x, y, radius + 3 * s, 0, Math.PI * 2);
    ctx.fillStyle = type === 'pickup'
        ? 'rgba(59, 130, 246, 0.22)'
        : 'rgba(16, 185, 129, 0.22)';
    ctx.fill();

    // Crisp marker base circle
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = Math.max(0.8, 1.2 * s);
    ctx.stroke();

    // Precision micro crosshairs (pinpoint titik pasti koordinat)
    const ch = 2.5 * s;
    ctx.beginPath();
    ctx.moveTo(x - ch, y);
    ctx.lineTo(x + ch, y);
    ctx.moveTo(x, y - ch);
    ctx.lineTo(x, y + ch);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.lineWidth = Math.max(0.6, 0.8 * s);
    ctx.stroke();

    // Exact center pinpoint dot
    ctx.beginPath();
    ctx.arc(x, y, 1.3 * s, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // Monospace pill label (tetap proporsional dan rapi di layar saat zoom)
    if (label) {
        const fontSize = 9 * s;
        ctx.font = `600 ${fontSize}px "JetBrains Mono", monospace`;
        const textWidth = ctx.measureText(label).width;
        const padX = 4 * s;
        const boxW = textWidth + padX * 2;
        const boxH = 13 * s;
        const boxX = x - boxW / 2;
        const boxY = y - radius - boxH - 3 * s;

        ctx.fillStyle = 'rgba(9, 9, 11, 0.88)';
        ctx.fillRect(boxX, boxY, boxW, boxH);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.lineWidth = Math.max(0.6, 0.8 * s);
        ctx.strokeRect(boxX, boxY, boxW, boxH);

        ctx.fillStyle = '#f4f4f5';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(label, x, boxY + boxH / 2);
        ctx.textBaseline = 'alphabetic';
    }
}

function drawTaskMarker(ctx, x, y, type, number, zoom = 1.0) {
    const s = 1.0 / (zoom || 1.0);
    const color = type === 'pickup' ? '#3b82f6' : '#10b981';
    const radius = 7.5 * s;

    // Outer halo
    ctx.beginPath();
    ctx.arc(x, y, radius + 2.5 * s, 0, Math.PI * 2);
    ctx.fillStyle = type === 'pickup'
        ? 'rgba(59, 130, 246, 0.22)'
        : 'rgba(16, 185, 129, 0.22)';
    ctx.fill();

    // Core circle
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = Math.max(0.8, 1.2 * s);
    ctx.stroke();

    // Precision center dot
    ctx.beginPath();
    ctx.arc(x, y, 1.2 * s, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // Number text
    ctx.font = `700 ${8 * s}px "JetBrains Mono", monospace`;
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(number, x, y);

    // Subtitle label PKP / DLV
    ctx.font = `700 ${6.5 * s}px "JetBrains Mono", monospace`;
    ctx.fillStyle = color;
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(type === 'pickup' ? 'PKP' : 'DLV', x, y + radius + 7 * s);
}

function drawRobot(ctx, x, y, yaw, zoom = 1.0, customScale = null, overrideRadius = null) {
    const s = 1.0 / (zoom || 1.0);

    // Hitung piksel per meter (PPM) dari resolusi peta SLAM
    let ppm;
    if (overrideRadius !== null) {
        ppm = overrideRadius / 0.30;
    } else {
        const res = (MAP_STATE.metadata && MAP_STATE.metadata.resolution) ? MAP_STATE.metadata.resolution : 0.05;
        const scale = (customScale !== null) ? customScale : (MAP_STATE.scale || 1.0);
        ppm = (1.0 / res) * scale;
    }

    // 1. Lingkaran Batas Aman Nav2 (Radius 0.30m = 30 cm)
    const safetyRadius = 0.30 * ppm;
    const r = Math.max(3.5 * s, safetyRadius);

    // 2. Dimensi Bodi Kotak Sasis Fisik (20 cm x 20 cm -> setengah lebar = 10 cm = 0.10m)
    const boxHalf = 0.10 * ppm;
    const bw = Math.max(2.2 * s, boxHalf);
    const bh = Math.max(2.2 * s, boxHalf);

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yaw);

    // ================= LAYER 1: Lingkaran Batas Aman 30 cm (Nav2 Collision Bubble) =================
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(245, 158, 11, 0.08)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.55)';
    ctx.lineWidth = Math.max(0.75, 1.2 * s);
    ctx.setLineDash([4 * s, 3 * s]);
    ctx.stroke();
    ctx.setLineDash([]);

    // ================= LAYER 2: Bodi Kotak Sasis Fisik Robot (20cm x 20cm) =================
    const cornerR = Math.min(2.5 * s, bw * 0.25);
    ctx.beginPath();
    if (ctx.roundRect) {
        ctx.roundRect(-bw, -bh, bw * 2, bh * 2, cornerR);
    } else {
        ctx.rect(-bw, -bh, bw * 2, bh * 2);
    }
    // Isi bodi gelap kontras dengan border amber tegas
    ctx.fillStyle = 'rgba(24, 24, 27, 0.90)';
    ctx.fill();
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = Math.max(1.0, 1.5 * s);
    ctx.stroke();

    // Aksen Bumper Depan (Arah Hadap Maju +X)
    ctx.beginPath();
    ctx.moveTo(bw, -bh * 0.7);
    ctx.lineTo(bw, bh * 0.7);
    ctx.strokeStyle = '#38bdf8'; // Cyan neon indikator depan
    ctx.lineWidth = Math.max(1.2, 2.0 * s);
    ctx.stroke();

    // ================= LAYER 3: Precision Reticle & Pinpoint Dot di (0,0) =================
    const crosshairLen = Math.min(bw * 0.75, 3.5 * s);
    ctx.beginPath();
    ctx.moveTo(-crosshairLen, 0);
    ctx.lineTo(crosshairLen, 0);
    ctx.moveTo(0, -crosshairLen);
    ctx.lineTo(0, crosshairLen);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.75)';
    ctx.lineWidth = Math.max(0.6, 0.8 * s);
    ctx.stroke();

    // Center pinpoint dot (putih terang berkontras gelap tepat di titik koordinat nyata)
    const dotR = Math.min(1.5 * s, bw * 0.25);
    ctx.beginPath();
    ctx.arc(0, 0, Math.max(1.0 * s, dotR), 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = Math.max(0.6, 0.8 * s);
    ctx.stroke();

    // ================= LAYER 4: Panah Arah Hadap Kompak di Depan Titik Pusat =================
    const arrowStart = Math.max(1.8 * s, bw * 0.25);
    const arrowTip = Math.min(r * 0.85, bw + 5.0 * s);
    const arrowLen = arrowTip - arrowStart;
    const arrowHalfW = Math.min(bh * 0.45, 3.2 * s);

    ctx.beginPath();
    ctx.moveTo(arrowTip, 0);
    ctx.lineTo(arrowStart, -arrowHalfW);
    ctx.lineTo(arrowStart + arrowLen * 0.28, 0); // Lekukan aerodinamis belakang
    ctx.lineTo(arrowStart, arrowHalfW);
    ctx.closePath();
    ctx.fillStyle = '#f59e0b';
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = Math.max(0.6, 0.8 * s);
    ctx.stroke();

    // ================= LAYER 5: Hairline Heading Ray ke Depan Bibir Lingkaran 30 cm =================
    const rayStart = r;
    const rayEnd = r + Math.min(r * 0.35, 5.5 * s);
    ctx.beginPath();
    ctx.moveTo(rayStart, 0);
    ctx.lineTo(rayEnd, 0);
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.85)';
    ctx.lineWidth = Math.max(0.8, 1.2 * s);
    ctx.stroke();

    ctx.restore();
}

// ====================== DELIVERY MAP (ZOOM, PAN & FULLSCREEN) ======================
const DELIVERY_VIEWPORT = {
    zoom: 1.0,
    panX: 0,
    panY: 0,
    isFullscreen: false,
};

function updateDeliveryMapHud(extraText = '') {
    const hud = document.getElementById('delivery-map-cursor-hud');
    if (!hud) return;
    const zoomPct = Math.round(DELIVERY_VIEWPORT.zoom * 100);
    if (extraText) {
        hud.innerHTML = extraText;
    } else {
        hud.innerHTML = `${getIcon('search', 12)} Zoom: <strong>${zoomPct}%</strong> | Scroll: Zoom, Drag: Geser, Klik: Pilih Titik`;
    }
}

function zoomDeliveryMap(factor) {
    const canvas = document.getElementById('delivery-map-canvas');
    if (!canvas) return;
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const oldZoom = DELIVERY_VIEWPORT.zoom;
    const newZoom = Math.max(0.6, Math.min(6.0, oldZoom * factor));
    const baseX = (cx - DELIVERY_VIEWPORT.panX) / oldZoom;
    const baseY = (cy - DELIVERY_VIEWPORT.panY) / oldZoom;
    DELIVERY_VIEWPORT.zoom = newZoom;
    DELIVERY_VIEWPORT.panX = cx - baseX * newZoom;
    DELIVERY_VIEWPORT.panY = cy - baseY * newZoom;
    updateDeliveryMapHud();
}

function resetDeliveryMapView() {
    DELIVERY_VIEWPORT.zoom = 1.0;
    DELIVERY_VIEWPORT.panX = 0;
    DELIVERY_VIEWPORT.panY = 0;
    updateDeliveryMapHud();
    if (typeof showToast === 'function') {
        showToast('Tampilan peta di-reset (100%)', 'info');
    }
}

function toggleDeliveryMapFullscreen() {
    const container = document.getElementById('delivery-map-container');
    if (!container) return;
    const isFs = container.classList.toggle('map-fullscreen');
    DELIVERY_VIEWPORT.isFullscreen = isFs;

    const fsBtn = document.getElementById('btn-delivery-fullscreen');
    if (fsBtn) {
        fsBtn.innerHTML = isFs ? getIcon('minimize', 14) : getIcon('maximize', 14);
        fsBtn.title = isFs ? 'Keluar Layar Penuh (Esc)' : 'Layar Penuh (Full Screen)';
    }

    let exitBanner = document.getElementById('delivery-map-exit-banner');
    if (isFs) {
        if (!exitBanner) {
            exitBanner = document.createElement('div');
            exitBanner.id = 'delivery-map-exit-banner';
            exitBanner.className = 'map-fullscreen-exit-banner';
            exitBanner.innerHTML = `<span style="display:inline-flex;align-items:center;gap:6px;">${getIcon('x', 14)} KELUAR LAYAR PENUH (Esc)</span>`;
            exitBanner.onclick = () => toggleDeliveryMapFullscreen();
            container.appendChild(exitBanner);
        }
        exitBanner.style.display = 'flex';
        if (typeof showToast === 'function') {
            showToast('Mode Layar Penuh Aktif — Tekan Esc untuk keluar', 'info');
        }
    } else {
        if (exitBanner) exitBanner.style.display = 'none';
    }
}

window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
        const container = document.getElementById('delivery-map-container');
        if (container && container.classList.contains('map-fullscreen')) {
            toggleDeliveryMapFullscreen();
        }
    }
});

function openPointModal(wx, wy) {
    const wxNum = Number(wx);
    const wyNum = Number(wy);
    const wxStr = wxNum.toFixed(2);
    const wyStr = wyNum.toFixed(2);

    let distText = '';
    if (window.APP && window.APP.pose && typeof window.APP.pose.x === 'number' && typeof window.APP.pose.y === 'number') {
        const d = Math.hypot(wxNum - window.APP.pose.x, wyNum - window.APP.pose.y);
        distText = `~${d.toFixed(2)}m dari robot`;
    }

    const bodyHtml = `
        <div class="point-modal-container">
            <div class="point-modal-coord-bar">
                <div class="mono" style="font-weight:700; color:var(--text-primary); display:flex; align-items:center; gap:6px;">
                    ${getIcon('mapPin', 14) || getIcon('pickup', 14)} <span>X: <strong style="color:var(--accent-bright);">${wxStr}m</strong>, Y: <strong style="color:var(--accent-bright);">${wyStr}m</strong></span>
                </div>
                ${distText ? `<span class="point-modal-dist-tag mono">${getIcon('navigation', 12)} ${distText}</span>` : ''}
            </div>

            <div style="font-size:0.75rem; color:var(--text-secondary); margin-top:2px; margin-bottom:4px;">
                Pilih tindakan navigasi untuk titik koordinat ini:
            </div>

            <!-- HERO ACTION: Direct 1-Click Dispatch -->
            <div class="point-action-card point-action-hero" onclick="execPointModalAction('direct', ${wxNum}, ${wyNum})" role="button" tabindex="0">
                <div class="point-action-icon">${getIcon('send', 18)}</div>
                <div class="point-action-content">
                    <div class="point-action-title">
                        <span>Langsung Antar ke Sini</span>
                        <span class="point-action-badge">1-Click Dispatch</span>
                    </div>
                    <div class="point-action-desc">Kirim robot secara instan langsung menuju titik koordinat ini tanpa antrean.</div>
                </div>
            </div>

            <!-- WAYPOINT CONFIGURATION (2 COLUMNS) -->
            <div class="point-modal-grid-2">
                <div class="point-action-card point-action-delivery" onclick="execPointModalAction('delivery', ${wxNum}, ${wyNum})" role="button" tabindex="0">
                    <div class="point-action-icon">${getIcon('delivery_point', 18)}</div>
                    <div class="point-action-content">
                        <div class="point-action-title">Set Tujuan (Delivery)</div>
                        <div class="point-action-desc">Tetapkan titik ini sebagai target akhir pengantaran pada form.</div>
                    </div>
                </div>

                <div class="point-action-card point-action-pickup" onclick="execPointModalAction('pickup', ${wxNum}, ${wyNum})" role="button" tabindex="0">
                    <div class="point-action-icon">${getIcon('pickup', 18)}</div>
                    <div class="point-action-content">
                        <div class="point-action-title">Set Titik Muat (Pickup)</div>
                        <div class="point-action-desc">Tetapkan titik ini sebagai lokasi awal pengambilan muatan barang.</div>
                    </div>
                </div>
            </div>

            <!-- LOCALIZATION & SAVED DATABASE (2 COLUMNS) -->
            <div class="point-modal-grid-2">
                <div class="point-action-card point-action-amcl" onclick="execPointModalAction('initialpose', ${wxNum}, ${wyNum})" role="button" tabindex="0">
                    <div class="point-action-icon">${getIcon('crosshair', 18)}</div>
                    <div class="point-action-content">
                        <div class="point-action-title">Posisi Awal (AMCL)</div>
                        <div class="point-action-desc">Kirim /initialpose untuk kalibrasi estimasi posisi robot di peta.</div>
                    </div>
                </div>

                <div class="point-action-card point-action-save" onclick="execPointModalAction('save', ${wxNum}, ${wyNum})" role="button" tabindex="0">
                    <div class="point-action-icon">${getIcon('save', 18)}</div>
                    <div class="point-action-content">
                        <div class="point-action-title">Simpan ke Daftar Lokasi</div>
                        <div class="point-action-desc">Beri nama &amp; simpan titik koordinat ini ke database lokasi.</div>
                    </div>
                </div>
            </div>
        </div>
    `;

    showModal(
        `${getIcon('navigation', 17)} Tetapkan Tindakan Koordinat`,
        bodyHtml,
        [
            {
                label: 'Tutup / Batal',
                class: 'btn-outline',
                action: hideModal
            }
        ],
        'NAV2 // MAP ACTION'
    );
}

window.execPointModalAction = function(action, wx, wy) {
    hideModal();
    const wxNum = Number(wx);
    const wyNum = Number(wy);
    const wxStr = wxNum.toFixed(2);
    const wyStr = wyNum.toFixed(2);

    if (action === 'direct') {
        const point = { name: `Peta (${wxStr}, ${wyStr})`, x: wxNum, y: wyNum };
        if (typeof directDispatchToPoint === 'function') {
            directDispatchToPoint(point);
        } else {
            APP.pendingDelivery = point;
            const el = document.getElementById('delivery-selected');
            if (el) el.innerHTML = `${getIcon('check', 13)} Tujuan: [${wxStr}, ${wyStr}]`;
            showToast(`Tujuan ditetapkan: (${wxStr}, ${wyStr})`, 'info');
        }
    } else if (action === 'delivery') {
        APP.pendingDelivery = { name: `Peta (${wxStr}, ${wyStr})`, x: wxNum, y: wyNum };
        const el = document.getElementById('delivery-selected');
        const roleLabel = (APP.dispatchMode === 'direct') ? 'Tujuan' : 'Delivery';
        if (el) el.innerHTML = `${getIcon('check', 13)} ${roleLabel}: [${wxStr}, ${wyStr}]`;
        showToast(`Tujuan ditetapkan: (${wxStr}, ${wyStr})`, 'info');
    } else if (action === 'pickup') {
        APP.pendingPickup = { name: `Peta (${wxStr}, ${wyStr})`, x: wxNum, y: wyNum };
        const el = document.getElementById('pickup-selected');
        if (el) el.innerHTML = `${getIcon('check', 13)} Pickup: [${wxStr}, ${wyStr}]`;
        showToast(`Pickup ditetapkan: (${wxStr}, ${wyStr})`, 'info');
    } else if (action === 'initialpose') {
        if (typeof openInitialPoseModal === 'function') {
            openInitialPoseModal(wxNum, wyNum);
        } else if (typeof sendInitialPoseToRobot === 'function') {
            sendInitialPoseToRobot(wxNum, wyNum, 0.0, false);
        }
    } else if (action === 'save') {
        if (typeof openSavePointFromMapModal === 'function') {
            openSavePointFromMapModal(wxNum, wyNum);
        }
    }
};

async function initDeliveryMap() {
    const canvas = document.getElementById('delivery-map-canvas');
    if (!canvas) return;

    const ok = await loadMapData();
    if (!ok) {
        const container = canvas.parentElement;
        if (container) {
            container.innerHTML = '<div class="flex-center mono" style="height:160px; color:var(--text-muted); font-size:0.76rem;">DATA PETA SLAM TIDAK TERSEDIA</div>';
        }
        return;
    }

    const ctx = canvas.getContext('2d');
    canvas.style.cursor = 'crosshair';

    if (!canvas._deliveryListenersAttached) {
        canvas._deliveryListenersAttached = true;

        // Zoom via Mouse Wheel
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = canvas.getBoundingClientRect();
            const cursorX = (e.clientX - rect.left) * (canvas.width / (rect.width || 1));
            const cursorY = (e.clientY - rect.top) * (canvas.height / (rect.height || 1));

            const zoomFactor = e.deltaY < 0 ? 1.2 : (1 / 1.2);
            const oldZoom = DELIVERY_VIEWPORT.zoom;
            const newZoom = Math.max(0.6, Math.min(6.0, oldZoom * zoomFactor));

            if (newZoom !== oldZoom) {
                const baseX = (cursorX - DELIVERY_VIEWPORT.panX) / oldZoom;
                const baseY = (cursorY - DELIVERY_VIEWPORT.panY) / oldZoom;
                DELIVERY_VIEWPORT.zoom = newZoom;
                DELIVERY_VIEWPORT.panX = cursorX - baseX * newZoom;
                DELIVERY_VIEWPORT.panY = cursorY - baseY * newZoom;

                const { wx, wy } = screenToWorld(e.clientX, e.clientY, canvas, DELIVERY_VIEWPORT);
                updateDeliveryMapHud(`${getIcon('pickup', 12)} <strong>X=${wx.toFixed(2)}m, Y=${wy.toFixed(2)}m</strong> | ${getIcon('search', 12)} Zoom: <strong>${Math.round(newZoom * 100)}%</strong>`);
            }
        }, { passive: false });

        // Hover Coordinate Display
        canvas.addEventListener('pointermove', (e) => {
            if (isPointerDown) return;
            const { wx, wy } = screenToWorld(e.clientX, e.clientY, canvas, DELIVERY_VIEWPORT);
            const zoomPct = Math.round(DELIVERY_VIEWPORT.zoom * 100);
            updateDeliveryMapHud(`${getIcon('pickup', 12)} Kursor: <strong>X=${wx.toFixed(2)}m, Y=${wy.toFixed(2)}m</strong> | ${getIcon('search', 12)} Zoom: <strong>${zoomPct}%</strong>`);
        });

        canvas.addEventListener('pointerleave', () => {
            if (!isPointerDown) updateDeliveryMapHud();
        });

        // Pan Drag vs Click Selection
        let isPointerDown = false;
        let isDragging = false;
        let startX = 0;
        let startY = 0;
        let lastX = 0;
        let lastY = 0;

        canvas.addEventListener('pointerdown', (e) => {
            if (e.button !== 0 && e.pointerType === 'mouse') return;
            isPointerDown = true;
            isDragging = false;
            startX = e.clientX;
            startY = e.clientY;
            lastX = e.clientX;
            lastY = e.clientY;
            canvas.style.cursor = 'grabbing';
        });

        window.addEventListener('pointermove', (e) => {
            if (!isPointerDown) return;
            const dx = e.clientX - lastX;
            const dy = e.clientY - lastY;
            const dist = Math.hypot(e.clientX - startX, e.clientY - startY);

            if (dist > 4) {
                isDragging = true;
                const rect = canvas.getBoundingClientRect();
                const scaleX = canvas.width / (rect.width || 1);
                const scaleY = canvas.height / (rect.height || 1);
                DELIVERY_VIEWPORT.panX += dx * scaleX;
                DELIVERY_VIEWPORT.panY += dy * scaleY;
            }
            lastX = e.clientX;
            lastY = e.clientY;
        });

        window.addEventListener('pointerup', (e) => {
            if (!isPointerDown) return;
            isPointerDown = false;
            canvas.style.cursor = 'crosshair';

            if (!isDragging) {
                const rect = canvas.getBoundingClientRect();
                if (e.clientX >= rect.left && e.clientX <= rect.right &&
                    e.clientY >= rect.top && e.clientY <= rect.bottom) {
                    const { wx, wy } = screenToWorld(e.clientX, e.clientY, canvas, DELIVERY_VIEWPORT);
                    openPointModal(wx, wy);
                }
            }
            isDragging = false;
        });
    }

    function animate() {
        if (!document.getElementById('delivery-map-canvas')) return;
        drawMap(canvas, ctx, {
            showRobot: true,
            showWaypoints: true,
            showTaskQueue: true,
            viewport: DELIVERY_VIEWPORT,
        });
        deliveryMapAnimFrame = requestAnimationFrame(animate);
    }
    animate();
}

// ====================== MONITOR & OVERVIEW MAP ======================
async function initMonitorMap(canvasId = 'monitor-map-canvas') {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    const ok = await loadMapData();
    if (!ok) {
        const container = canvas.parentElement;
        if (container) {
            container.innerHTML = '<div class="flex-center mono" style="height:160px; color:var(--text-muted); font-size:0.76rem;">DATA PETA SLAM TIDAK TERSEDIA</div>';
        }
        return;
    }

    const ctx = canvas.getContext('2d');

    function animate() {
        if (!document.getElementById(canvasId)) return;
        drawMap(canvas, ctx, {
            showRobot: true,
            showWaypoints: true,
            showTaskQueue: true,
        });
        monitorMapAnimFrame = requestAnimationFrame(animate);
    }
    animate();
}

function startMonitoringPolling() {
    APP.statusPollTimer = setInterval(async () => {
        const data = await apiGet('/api/status');
        if (!data) return;

        APP.pose = data.pose || APP.pose;
        APP.poseSource = data.pose_source || APP.poseSource;
        APP.delivery = data.delivery || APP.delivery;
        APP.humanStatus = data.human_status || 'NO_HUMAN';

        if (typeof handleDeliveryStatusUpdate === 'function') {
            handleDeliveryStatusUpdate(APP.delivery);
        }

        if (typeof updateCockpitData === 'function') {
            updateCockpitData(data);
        }
    }, 500);
}

// ====================== LIVE SLAM MAPPING STUDIO CANVAS ======================
async function loadLiveSlamMapData(force = false) {
    if (force) {
        MAP_STATE.slamFetchInFlight = false;
    }
    if (MAP_STATE.slamFetchInFlight) return MAP_STATE.liveSlamLoaded;
    MAP_STATE.slamFetchInFlight = true;
    try {
        const meta = await apiGet('/api/map/metadata?live=1&t=' + Date.now());
        const isLive = meta && (
            meta.source === 'live_slam' ||
            meta.source === 'live_ros_topic' ||
            meta.map_file === 'live'
        );
        if (!meta || !isLive) {
            MAP_STATE.slamFetchInFlight = false;
            return false;
        }

        return await new Promise((resolve) => {
            const img = new Image();
            img.onload = () => {
                MAP_STATE.liveSlamMeta = meta;
                MAP_STATE.liveSlamImage = img;
                MAP_STATE.liveSlamLoaded = true;
                MAP_STATE.lastSlamFetch = Date.now();
                MAP_STATE.slamFetchInFlight = false;
                APP.hasLiveMap = true;
                resolve(true);
            };
            img.onerror = () => {
                MAP_STATE.slamFetchInFlight = false;
                resolve(false);
            };
            img.src = '/api/map?live=1&t=' + Date.now();
        });
    } catch (e) {
        MAP_STATE.slamFetchInFlight = false;
        return false;
    }
}

function recordRobotTrailPoint(wx, wy) {
    if (typeof wx !== 'number' || typeof wy !== 'number' || isNaN(wx) || isNaN(wy)) return;
    const trail = MAP_STATE.robotTrail;
    if (trail.length === 0) {
        trail.push({ x: wx, y: wy });
        return;
    }
    const last = trail[trail.length - 1];
    const dist = Math.hypot(wx - last.x, wy - last.y);
    // Jika robot melompat/teleport > 1.2m (ganti peta, initial pose, relokasi):
    // Reset jejak agar garis hijau tidak ditarik melintasi seluruh peta
    if (dist > 1.2) {
        MAP_STATE.robotTrail = [{ x: wx, y: wy }];
        return;
    }
    if (dist >= 0.06) {
        trail.push({ x: wx, y: wy });
        if (trail.length > 600) trail.shift();
    }
}

function drawOriginCrosshair(ctx, px, py) {
    ctx.save();
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.55)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(px - 12, py);
    ctx.lineTo(px + 12, py);
    ctx.moveTo(px, py - 12);
    ctx.lineTo(px, py + 12);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.arc(px, py, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#38bdf8';
    ctx.fill();

    ctx.font = '600 8px "JetBrains Mono", monospace';
    ctx.fillStyle = 'rgba(56, 189, 248, 0.9)';
    ctx.textAlign = 'left';
    ctx.fillText('ORIGIN (0,0)', px + 6, py - 5);
    ctx.restore();
}

function drawLiveSlamMap(canvas, ctx) {
    if (!canvas || !canvas.parentElement) return;

    // Record live robot movement trail
    if (APP.pose) {
        recordRobotTrailPoint(APP.pose.x || 0, APP.pose.y || 0);
    }

    // Periodically refresh live SLAM image every 1.0s when live map is available or mapping is active
    if ((APP.hasLiveMap || APP.mappingActive) && (Date.now() - MAP_STATE.lastSlamFetch > 1000)) {
        MAP_STATE.lastSlamFetch = Date.now();
        loadLiveSlamMapData();
    }

    const container = canvas.parentElement;
    const containerWidth = container.clientWidth || 480;

    // CASE 1: Live SLAM OccupancyGrid is available from slam_toolbox
    if (MAP_STATE.liveSlamLoaded && MAP_STATE.liveSlamImage && MAP_STATE.liveSlamMeta) {
        const meta = MAP_STATE.liveSlamMeta;
        const imgW = MAP_STATE.liveSlamImage.naturalWidth || meta.width || 100;
        const imgH = MAP_STATE.liveSlamImage.naturalHeight || meta.height || 100;
        const resolution = meta.resolution || 0.05;
        const origin = meta.origin || [0, 0, 0];

        // Keep reasonable aspect ratio and minimum height so small initial maps look crisp
        const scaleX = containerWidth / imgW;
        const rawHeight = Math.round(imgH * scaleX);
        const canvasW = containerWidth;
        const canvasH = Math.max(300, Math.min(540, rawHeight));

        if (canvas.width !== canvasW || canvas.height !== canvasH) {
            canvas.width = canvasW;
            canvas.height = canvasH;
        }

        // Fill dark studio background
        ctx.fillStyle = '#090d14';
        ctx.fillRect(0, 0, canvasW, canvasH);

        // Fit map centered inside canvas preserving aspect ratio
        const fitScale = Math.min(canvasW / imgW, canvasH / imgH) * 0.94;
        const drawW = imgW * fitScale;
        const drawH = imgH * fitScale;
        const offsetX = (canvasW - drawW) / 2;
        const offsetY = (canvasH - drawH) / 2;

        ctx.save();
        ctx.imageSmoothingEnabled = false; // Crisp occupancy grid cells like RViz
        ctx.drawImage(MAP_STATE.liveSlamImage, offsetX, offsetY, drawW, drawH);
        ctx.restore();

        // Subtle Live Map HUD Tag in top-left corner
        ctx.fillStyle = 'rgba(15, 23, 42, 0.78)';
        ctx.fillRect(10, 10, 210, 26);
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.45)';
        ctx.strokeRect(10, 10, 210, 26);
        ctx.font = '600 10.5px "JetBrains Mono", monospace';
        ctx.fillStyle = '#34d399';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(`● LIVE /map [${imgW}×${imgH} px]`, 18, 23);
        ctx.textBaseline = 'alphabetic';

        const toSlamPixel = (wx, wy) => {
            const px = offsetX + ((wx - origin[0]) / resolution) * fitScale;
            const py = offsetY + (imgH - (wy - origin[1]) / resolution) * fitScale;
            return { px, py };
        };

        // Draw Origin (0,0)
        const origPt = toSlamPixel(0, 0);
        drawOriginCrosshair(ctx, origPt.px, origPt.py);

        // Draw Robot Trajectory Trail
        if (MAP_STATE.robotTrail.length > 1) {
            ctx.save();
            ctx.beginPath();
            let prevPt = null;
            MAP_STATE.robotTrail.forEach((pt, idx) => {
                const { px, py } = toSlamPixel(pt.x, pt.y);
                if (idx === 0 || !prevPt || Math.hypot(pt.x - prevPt.x, pt.y - prevPt.y) > 1.2) {
                    ctx.moveTo(px, py);
                } else {
                    ctx.lineTo(px, py);
                }
                prevPt = pt;
            });
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.78)';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.restore();
        }

        // Draw Live Robot Pose & Heading
        const robotPt = toSlamPixel(APP.pose.x || 0, APP.pose.y || 0);
        drawRobot(ctx, robotPt.px, robotPt.py, APP.pose.yaw || 0, 1.0, fitScale);
        return;
    }

    // CASE 2: Standby Radar Grid (Before /map stream arrives from slam_toolbox)
    const canvasW = containerWidth;
    const canvasH = 340;
    if (canvas.width !== canvasW || canvas.height !== canvasH) {
        canvas.width = canvasW;
        canvas.height = canvasH;
    }

    ctx.fillStyle = '#090d14';
    ctx.fillRect(0, 0, canvasW, canvasH);

    // Draw subtle metric grid (1m = 40px)
    const ppm = 40;
    const cx = canvasW / 2;
    const cy = canvasH / 2;

    ctx.strokeStyle = 'rgba(148, 163, 184, 0.08)';
    ctx.lineWidth = 1;
    for (let x = cx % ppm; x < canvasW; x += ppm) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvasH);
        ctx.stroke();
    }
    for (let y = cy % ppm; y < canvasH; y += ppm) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvasW, y);
        ctx.stroke();
    }

    const toStandbyPixel = (wx, wy) => ({
        px: cx + wx * ppm,
        py: cy - wy * ppm,
    });

    drawOriginCrosshair(ctx, cx, cy);

    // Draw Robot Trail on Standby Grid
    if (MAP_STATE.robotTrail.length > 1) {
        ctx.beginPath();
        let prevPt = null;
        MAP_STATE.robotTrail.forEach((pt, idx) => {
            const { px, py } = toStandbyPixel(pt.x, pt.y);
            if (idx === 0 || !prevPt || Math.hypot(pt.x - prevPt.x, pt.y - prevPt.y) > 1.2) {
                ctx.moveTo(px, py);
            } else {
                ctx.lineTo(px, py);
            }
            prevPt = pt;
        });
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.65)';
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    // Draw Robot on Standby Grid
    const rPt = toStandbyPixel(APP.pose.x || 0, APP.pose.y || 0);
    drawRobot(ctx, rPt.px, rPt.py, APP.pose.yaw || 0, 1.0, null, 0.30 * ppm);

    // Info banner at top of canvas
    ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
    ctx.fillRect(12, 12, canvasW - 24, 38);
    ctx.strokeStyle = 'rgba(59, 130, 246, 0.35)';
    ctx.strokeRect(12, 12, canvasW - 24, 38);

    ctx.font = '600 11px "JetBrains Mono", monospace';
    ctx.fillStyle = '#93c5fd';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const msg = APP.mappingActive
        ? '[MENUNGGU] MENUNGGU FRAME PERTAMA /map DARI SLAM TOOLBOX...'
        : '[STANDBY] SLAM STANDBY — Klik [Mulai SLAM Mapping] untuk membangun peta baru';
    ctx.fillText(msg, canvasW / 2, 31);
    ctx.textBaseline = 'alphabetic';
}

async function initMappingMap(canvasId = 'mapping-map-canvas') {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    await loadLiveSlamMapData();
    const ctx = canvas.getContext('2d');

    function animate() {
        if (!document.getElementById(canvasId)) return;
        drawLiveSlamMap(canvas, ctx);
        mappingMapAnimFrame = requestAnimationFrame(animate);
    }
    animate();
}

window.stopAllMapLoops = stopAllMapLoops;
window.reloadMapData = reloadMapData;
window.initMappingMap = initMappingMap;
window.loadLiveSlamMapData = loadLiveSlamMapData;
window.clearRobotTrail = clearRobotTrail;
window.zoomDeliveryMap = zoomDeliveryMap;
window.resetDeliveryMapView = resetDeliveryMapView;
window.toggleDeliveryMapFullscreen = toggleDeliveryMapFullscreen;
window.screenToWorld = screenToWorld;


