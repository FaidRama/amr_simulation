/* =================================================================
   AMR DASHBOARD — Interactive Map Module
   Renders the warehouse map with robot position, waypoints, and
   supports tap-to-add-waypoint on mobile.
   ================================================================= */

// ====================== MAP STATE ======================
const MAP_STATE = {
    image: null,
    metadata: null,
    loaded: false,
    // Map image dimensions
    imgWidth: 0,
    imgHeight: 0,
    // Canvas dimensions  
    canvasWidth: 0,
    canvasHeight: 0,
    // Scale factor
    scale: 1,
};

// ====================== LOAD MAP ======================
async function loadMapData() {
    if (MAP_STATE.loaded) return true;

    try {
        // Load metadata
        const meta = await apiGet('/api/map/metadata');
        if (!meta) return false;
        MAP_STATE.metadata = meta;

        // Load image
        return new Promise((resolve) => {
            const img = new Image();
            img.onload = () => {
                MAP_STATE.image = img;
                MAP_STATE.imgWidth = img.naturalWidth;
                MAP_STATE.imgHeight = img.naturalHeight;
                MAP_STATE.loaded = true;
                resolve(true);
            };
            img.onerror = () => resolve(false);
            img.src = '/api/map';
        });
    } catch (e) {
        console.error('Failed to load map:', e);
        return false;
    }
}

// ====================== COORDINATE CONVERSION ======================
// World (ROS map frame) → Canvas pixel
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

// Canvas pixel → World (ROS map frame)
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
    if (!MAP_STATE.loaded || !canvas) return;

    const {
        showRobot = true,
        showWaypoints = false,
        showTaskQueue = false,
        interactive = false,
    } = options;

    // Size canvas to container
    const container = canvas.parentElement;
    const containerWidth = container.clientWidth;
    MAP_STATE.scale = containerWidth / MAP_STATE.imgWidth;
    MAP_STATE.canvasWidth = containerWidth;
    MAP_STATE.canvasHeight = Math.round(MAP_STATE.imgHeight * MAP_STATE.scale);

    canvas.width = MAP_STATE.canvasWidth;
    canvas.height = MAP_STATE.canvasHeight;

    // Draw map image
    ctx.drawImage(MAP_STATE.image, 0, 0, MAP_STATE.canvasWidth, MAP_STATE.canvasHeight);

    // Apply slight tint for better contrast
    ctx.fillStyle = 'rgba(6, 10, 24, 0.25)';
    ctx.fillRect(0, 0, MAP_STATE.canvasWidth, MAP_STATE.canvasHeight);

    // Draw predefined waypoints
    if (showWaypoints && APP.savedPoints) {
        APP.savedPoints.forEach(p => {
            const { px, py } = worldToPixel(p.x, p.y);
            drawWaypointMarker(ctx, px, py, p.type, p.name);
        });
    }

    // Draw task queue waypoints
    if (showTaskQueue && APP.taskQueue) {
        APP.taskQueue.forEach((task, i) => {
            // Pickup
            const pickup = worldToPixel(task.pickup.x, task.pickup.y);
            drawTaskMarker(ctx, pickup.px, pickup.py, 'pickup', i + 1);

            // Delivery
            const delivery = worldToPixel(task.delivery.x, task.delivery.y);
            drawTaskMarker(ctx, delivery.px, delivery.py, 'delivery', i + 1);

            // Line between pickup and delivery
            ctx.beginPath();
            ctx.setLineDash([4, 4]);
            ctx.strokeStyle = 'rgba(0, 212, 255, 0.3)';
            ctx.lineWidth = 1;
            ctx.moveTo(pickup.px, pickup.py);
            ctx.lineTo(delivery.px, delivery.py);
            ctx.stroke();
            ctx.setLineDash([]);
        });
    }

    // Draw robot position
    if (showRobot) {
        const { px, py } = worldToPixel(APP.pose.x, APP.pose.y);
        drawRobot(ctx, px, py, APP.pose.yaw);
    }
}

function drawWaypointMarker(ctx, x, y, type, label) {
    const color = type === 'pickup' ? '#00d4ff' : '#00ff88';
    const radius = 5;

    // Outer glow
    ctx.beginPath();
    ctx.arc(x, y, radius + 4, 0, Math.PI * 2);
    ctx.fillStyle = type === 'pickup'
        ? 'rgba(0, 212, 255, 0.15)'
        : 'rgba(0, 255, 136, 0.15)';
    ctx.fill();

    // Dot
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    // Border
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Label
    if (label) {
        ctx.font = '600 9px Inter, sans-serif';
        ctx.fillStyle = '#fff';
        ctx.textAlign = 'center';
        ctx.fillText(label, x, y - radius - 6);
    }
}

function drawTaskMarker(ctx, x, y, type, number) {
    const color = type === 'pickup' ? '#00d4ff' : '#00ff88';
    const radius = 10;

    // Circle with number
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Number
    ctx.font = 'bold 9px Inter, sans-serif';
    ctx.fillStyle = type === 'pickup' ? '#000' : '#000';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(number, x, y);

    // Type label below
    ctx.font = '600 7px Inter, sans-serif';
    ctx.fillStyle = color;
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(type === 'pickup' ? 'P' : 'D', x, y + radius + 9);
}

function drawRobot(ctx, x, y, yaw) {
    const size = 8;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yaw); // Canvas Y is inverted

    // Robot body (triangle pointing forward)
    ctx.beginPath();
    ctx.moveTo(size * 1.5, 0);
    ctx.lineTo(-size, -size);
    ctx.lineTo(-size, size);
    ctx.closePath();
    ctx.fillStyle = '#ff4466';
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Glow ring
    ctx.beginPath();
    ctx.arc(0, 0, size + 5, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255, 68, 102, 0.35)';
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.restore();
}

// ====================== DELIVERY MAP ======================
let deliveryMapAnimFrame = null;

async function initDeliveryMap() {
    const canvas = document.getElementById('delivery-map-canvas');
    if (!canvas) return;

    const ok = await loadMapData();
    if (!ok) {
        const container = canvas.parentElement;
        container.innerHTML = '<div class="flex-center" style="height:150px; color:var(--text-muted); font-size:0.82rem;">Peta tidak tersedia</div>';
        return;
    }

    const ctx = canvas.getContext('2d');

    // Add tap handler for interactive waypoint adding
    canvas.addEventListener('click', (e) => {
        const rect = canvas.getBoundingClientRect();
        const px = e.clientX - rect.left;
        const py = e.clientY - rect.top;
        const { wx, wy } = pixelToWorld(px, py);

        // Ask user what to set (pickup or delivery)
        showModal(
            'Titik di Peta',
            `Koordinat: <strong>(${wx}, ${wy})</strong><br>
             Tetapkan sebagai apa?`,
            [
                {
                    label: '📍 Pickup', class: 'btn-primary',
                    action: () => {
                        APP.pendingPickup = { name: `Peta (${wx}, ${wy})`, x: wx, y: wy };
                        const el = document.getElementById('pickup-selected');
                        if (el) el.textContent = `✓ Pickup: (${wx}, ${wy})`;
                        showToast(`Pickup: (${wx}, ${wy})`, 'info');
                    }
                },
                {
                    label: '🎯 Delivery', class: 'btn-success',
                    action: () => {
                        APP.pendingDelivery = { name: `Peta (${wx}, ${wy})`, x: wx, y: wy };
                        const el = document.getElementById('delivery-selected');
                        if (el) el.textContent = `✓ Delivery: (${wx}, ${wy})`;
                        showToast(`Delivery: (${wx}, ${wy})`, 'info');
                    }
                }
            ]
        );
    });

    // Animation loop
    function animate() {
        drawMap(canvas, ctx, {
            showRobot: true,
            showWaypoints: true,
            showTaskQueue: true,
            interactive: true,
        });
        deliveryMapAnimFrame = requestAnimationFrame(animate);
    }
    animate();
}

// ====================== MONITOR MAP ======================
let monitorMapAnimFrame = null;

async function initMonitorMap() {
    const canvas = document.getElementById('monitor-map-canvas');
    if (!canvas) return;

    const ok = await loadMapData();
    if (!ok) {
        const container = canvas.parentElement;
        container.innerHTML = '<div class="flex-center" style="height:150px; color:var(--text-muted); font-size:0.82rem;">Peta tidak tersedia</div>';
        return;
    }

    const ctx = canvas.getContext('2d');

    function animate() {
        drawMap(canvas, ctx, {
            showRobot: true,
            showWaypoints: true,
            showTaskQueue: false,
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
        APP.delivery = data.delivery || APP.delivery;
        APP.humanStatus = data.human_status || 'NO_HUMAN';

        // Update monitoring UI
        const monX = document.getElementById('mon-x');
        const monY = document.getElementById('mon-y');
        const monYaw = document.getElementById('mon-yaw');
        const monHuman = document.getElementById('mon-human');
        const monPhase = document.getElementById('mon-phase');
        const monElapsed = document.getElementById('mon-elapsed');
        const monBadge = document.getElementById('mon-status-badge');
        const monProgress = document.getElementById('mon-progress');
        const monProgressText = document.getElementById('mon-progress-text');

        if (monX) monX.textContent = APP.pose.x.toFixed(3);
        if (monY) monY.textContent = APP.pose.y.toFixed(3);
        if (monYaw) monYaw.textContent = (APP.pose.yaw * 180 / Math.PI).toFixed(1) + '°';
        if (monHuman) {
            monHuman.textContent = APP.humanStatus;
            monHuman.className = 'mon-value ' + (
                APP.humanStatus === 'DISTURBING' || APP.humanStatus === 'CALLING' ? 'mon-danger' :
                APP.humanStatus === 'WALKING' ? 'mon-warning' : 'mon-success'
            );
        }

        const phaseLabels = {
            idle: 'Idle', navigating_pickup: 'Menuju Pickup', at_pickup: 'Di Pickup',
            navigating_delivery: 'Menuju Tujuan', at_delivery: 'Di Tujuan'
        };
        if (monPhase) monPhase.textContent = phaseLabels[APP.delivery.current_phase] || APP.delivery.current_phase;

        if (monElapsed && APP.delivery.elapsed_time) {
            const sec = Math.floor(APP.delivery.elapsed_time);
            const min = Math.floor(sec / 60);
            monElapsed.textContent = min > 0 ? `${min}m ${sec % 60}s` : `${sec}s`;
        }

        if (monBadge) {
            monBadge.textContent = (APP.delivery.status || 'idle').toUpperCase();
            monBadge.className = `status-badge status-${APP.delivery.status || 'idle'}`;
        }

        if (monProgress && APP.delivery.total_tasks > 0) {
            const pct = Math.round((APP.delivery.completed_tasks / APP.delivery.total_tasks) * 100);
            monProgress.style.width = `${pct}%`;
        }

        if (monProgressText && APP.delivery.total_tasks > 0) {
            monProgressText.textContent =
                `Task ${(APP.delivery.current_task_index || 0) + 1}/${APP.delivery.total_tasks} — ${phaseLabels[APP.delivery.current_phase] || ''}`;
        }
    }, 500);
}
