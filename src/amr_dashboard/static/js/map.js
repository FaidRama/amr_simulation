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
};

let deliveryMapAnimFrame = null;
let monitorMapAnimFrame = null;

function stopAllMapLoops() {
    if (deliveryMapAnimFrame) {
        cancelAnimationFrame(deliveryMapAnimFrame);
        deliveryMapAnimFrame = null;
    }
    if (monitorMapAnimFrame) {
        cancelAnimationFrame(monitorMapAnimFrame);
        monitorMapAnimFrame = null;
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

window.stopAllMapLoops = stopAllMapLoops;
window.reloadMapData = reloadMapData;

