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

function clearRobotTrail() {
    MAP_STATE.robotTrail = [];
    if (typeof showToast === 'function') {
        showToast('Jejak lintasan robot pada peta dibersihkan', 'info');
    }
}

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
    } = options;

    const container = canvas.parentElement;
    const containerWidth = container.clientWidth || 400;
    MAP_STATE.scale = containerWidth / MAP_STATE.imgWidth;
    MAP_STATE.canvasWidth = containerWidth;
    MAP_STATE.canvasHeight = Math.round(MAP_STATE.imgHeight * MAP_STATE.scale);

    if (canvas.width !== MAP_STATE.canvasWidth || canvas.height !== MAP_STATE.canvasHeight) {
        canvas.width = MAP_STATE.canvasWidth;
        canvas.height = MAP_STATE.canvasHeight;
    }

    // Draw SLAM occupancy grid image
    ctx.drawImage(MAP_STATE.image, 0, 0, MAP_STATE.canvasWidth, MAP_STATE.canvasHeight);

    // Apply subtle studio dark contrast overlay
    ctx.fillStyle = 'rgba(9, 9, 11, 0.22)';
    ctx.fillRect(0, 0, MAP_STATE.canvasWidth, MAP_STATE.canvasHeight);

    // Draw Home / Docking Station marker
    if (showWaypoints && APP.homePosition) {
        const hPos = worldToPixel(APP.homePosition.x || 0, APP.homePosition.y || 0);
        drawHomeMarker(ctx, hPos.px, hPos.py);
    }

    // Draw predefined waypoints
    if (showWaypoints && APP.savedPoints) {
        APP.savedPoints.forEach(p => {
            const { px, py } = worldToPixel(p.x, p.y);
            drawWaypointMarker(ctx, px, py, p.type, p.name);
        });
    }

    // Draw task queue waypoints & route lines
    if (showTaskQueue && APP.taskQueue) {
        APP.taskQueue.forEach((task, i) => {
            const pickup = worldToPixel(task.pickup.x, task.pickup.y);
            const delivery = worldToPixel(task.delivery.x, task.delivery.y);

            // Vector route line between pickup and delivery
            ctx.beginPath();
            ctx.setLineDash([5, 4]);
            ctx.strokeStyle = 'rgba(59, 130, 246, 0.65)';
            ctx.lineWidth = 1.5;
            ctx.moveTo(pickup.px, pickup.py);
            ctx.lineTo(delivery.px, delivery.py);
            ctx.stroke();
            ctx.setLineDash([]);

            drawTaskMarker(ctx, pickup.px, pickup.py, 'pickup', i + 1);
            drawTaskMarker(ctx, delivery.px, delivery.py, 'delivery', i + 1);
        });
    }

    // Record & Draw Robot Trajectory Trail
    if (showRobot && typeof recordRobotTrailPoint === 'function') {
        recordRobotTrailPoint(APP.pose.x || 0, APP.pose.y || 0);
        if (MAP_STATE.robotTrail.length > 1) {
            ctx.save();
            ctx.beginPath();
            MAP_STATE.robotTrail.forEach((pt, idx) => {
                const { px, py } = worldToPixel(pt.x, pt.y);
                if (idx === 0) ctx.moveTo(px, py);
                else ctx.lineTo(px, py);
            });
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.75)';
            ctx.lineWidth = 2;
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
        ctx.lineWidth = 2.5;
        ctx.setLineDash([5, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
    }

    // Draw robot position & orientation
    if (showRobot) {
        const { px, py } = worldToPixel(APP.pose.x, APP.pose.y);
        drawRobot(ctx, px, py, APP.pose.yaw);
    }
}

function drawHomeMarker(ctx, x, y) {
    const r = 6;
    ctx.beginPath();
    ctx.arc(x, y, r + 3, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(168, 85, 247, 0.22)';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = '#a855f7';
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.font = '700 8px "JetBrains Mono", monospace';
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('H', x, y);
    ctx.textBaseline = 'alphabetic';
}

function drawWaypointMarker(ctx, x, y, type, label) {
    const color = type === 'pickup' ? '#3b82f6' : '#10b981';
    const radius = 5;

    // Outer technical halo
    ctx.beginPath();
    ctx.arc(x, y, radius + 4, 0, Math.PI * 2);
    ctx.fillStyle = type === 'pickup'
        ? 'rgba(59, 130, 246, 0.2)'
        : 'rgba(16, 185, 129, 0.2)';
    ctx.fill();

    // Crisp center dot
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Monospace pill label
    if (label) {
        ctx.font = '600 9px "JetBrains Mono", monospace';
        const textWidth = ctx.measureText(label).width;
        const padX = 4;
        const boxW = textWidth + padX * 2;
        const boxH = 13;
        const boxX = x - boxW / 2;
        const boxY = y - radius - boxH - 4;

        ctx.fillStyle = 'rgba(9, 9, 11, 0.85)';
        ctx.fillRect(boxX, boxY, boxW, boxH);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 1;
        ctx.strokeRect(boxX, boxY, boxW, boxH);

        ctx.fillStyle = '#f4f4f5';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(label, x, boxY + boxH / 2);
        ctx.textBaseline = 'alphabetic';
    }
}

function drawTaskMarker(ctx, x, y, type, number) {
    const color = type === 'pickup' ? '#3b82f6' : '#10b981';
    const radius = 9;

    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.font = '700 9px "JetBrains Mono", monospace';
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(number, x, y);

    ctx.font = '700 7px "JetBrains Mono", monospace';
    ctx.fillStyle = color;
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(type === 'pickup' ? 'PKP' : 'DLV', x, y + radius + 9);
}

function drawRobot(ctx, x, y, yaw) {
    const size = 8;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yaw);

    // Heading ray line
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(size * 2.4, 0);
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.65)';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Outer radar ring
    ctx.beginPath();
    ctx.arc(0, 0, size + 5, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Robot directional chevron/triangle
    ctx.beginPath();
    ctx.moveTo(size * 1.45, 0);
    ctx.lineTo(-size * 0.9, -size * 0.85);
    ctx.lineTo(-size * 0.5, 0);
    ctx.lineTo(-size * 0.9, size * 0.85);
    ctx.closePath();
    ctx.fillStyle = '#f59e0b';
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    ctx.restore();
}

// ====================== DELIVERY MAP ======================
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

    canvas.onclick = (e) => {
        const rect = canvas.getBoundingClientRect();
        const px = e.clientX - rect.left;
        const py = e.clientY - rect.top;
        const { wx, wy } = pixelToWorld(px, py);

        showModal(
            'Tetapkan Koordinat Peta',
            `<span class="mono">POSISI KLIK: <strong>X=${wx}m, Y=${wy}m</strong></span><br>
             Pilih tindakan untuk titik koordinat ini:`,
            [
                {
                    label: 'Set Pickup', class: 'btn-primary',
                    action: () => {
                        APP.pendingPickup = { name: `Map (${wx}, ${wy})`, x: wx, y: wy };
                        const el = document.getElementById('pickup-selected');
                        if (el) el.innerHTML = `${getIcon('check', 13)} Pickup: [${wx}, ${wy}]`;
                        showToast(`Pickup ditetapkan: (${wx}, ${wy})`, 'info');
                    }
                },
                {
                    label: 'Set Delivery', class: 'btn-success',
                    action: () => {
                        APP.pendingDelivery = { name: `Map (${wx}, ${wy})`, x: wx, y: wy };
                        const el = document.getElementById('delivery-selected');
                        if (el) el.innerHTML = `${getIcon('check', 13)} Delivery: [${wx}, ${wy}]`;
                        showToast(`Delivery ditetapkan: (${wx}, ${wy})`, 'info');
                    }
                },
                {
                    label: '📍 Set Posisi Awal Robot (AMCL)', class: 'btn-warning',
                    action: async () => {
                        if (typeof sendInitialPoseToRobot === 'function') {
                            await sendInitialPoseToRobot(wx, wy, 0.0, false);
                        }
                    }
                },
                {
                    label: '💾 Simpan ke Daftar Lokasi', class: 'btn-secondary',
                    action: () => {
                        if (typeof openSavePointFromMapModal === 'function') {
                            openSavePointFromMapModal(wx, wy);
                        }
                    }
                }
            ]
        );
    };

    function animate() {
        if (!document.getElementById('delivery-map-canvas')) return;
        drawMap(canvas, ctx, {
            showRobot: true,
            showWaypoints: true,
            showTaskQueue: true,
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
async function loadLiveSlamMapData() {
    if (MAP_STATE.slamFetchInFlight) return MAP_STATE.liveSlamLoaded;
    MAP_STATE.slamFetchInFlight = true;
    try {
        const meta = await apiGet('/api/map/metadata?live=1&t=' + Date.now());
        if (!meta || meta.source !== 'live_slam') {
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

    // Periodically refresh live SLAM image every 1.2s when live map is available
    if (APP.hasLiveMap && (Date.now() - MAP_STATE.lastSlamFetch > 1200)) {
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
            MAP_STATE.robotTrail.forEach((pt, idx) => {
                const { px, py } = toSlamPixel(pt.x, pt.y);
                if (idx === 0) ctx.moveTo(px, py);
                else ctx.lineTo(px, py);
            });
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.78)';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.restore();
        }

        // Draw Live Robot Pose & Heading
        const robotPt = toSlamPixel(APP.pose.x || 0, APP.pose.y || 0);
        drawRobot(ctx, robotPt.px, robotPt.py, APP.pose.yaw || 0);
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
        MAP_STATE.robotTrail.forEach((pt, idx) => {
            const { px, py } = toStandbyPixel(pt.x, pt.y);
            if (idx === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.65)';
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    // Draw Robot on Standby Grid
    const rPt = toStandbyPixel(APP.pose.x || 0, APP.pose.y || 0);
    drawRobot(ctx, rPt.px, rPt.py, APP.pose.yaw || 0);

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
        ? '⏳ MENUNGGU FRAME PERTAMA /map DARI SLAM TOOLBOX...'
        : '📡 SLAM STANDBY — Klik [Mulai SLAM Mapping] untuk membangun peta baru';
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


