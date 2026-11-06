/* =================================================================
   AMR-01 // CONTROL STUDIO — Virtual Vector Joystick Module
   Theme-aware precision radar-style touch/mouse joystick.
   ================================================================= */

let joystickActive = false;
let joystickSendTimer = null;

function initJoystick() {
    const canvas = document.getElementById('joystick-canvas');
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const size = canvas.width;
    const center = size / 2;
    const maxRadius = (size / 2) - 22;
    const knobRadius = 26;

    let knobX = center;
    let knobY = center;
    let touching = false;

    function draw() {
        const styles = getComputedStyle(document.documentElement);
        const bgPrimary = styles.getPropertyValue('--bg-primary').trim() || '#11151f';
        const bgTertiary = styles.getPropertyValue('--bg-tertiary').trim() || '#202738';
        const borderColor = styles.getPropertyValue('--border').trim() || '#283147';
        const borderHover = styles.getPropertyValue('--border-hover').trim() || '#3b4866';
        const accentColor = styles.getPropertyValue('--accent').trim() || '#3b82f6';
        const accentBright = styles.getPropertyValue('--accent-bright').trim() || '#60a5fa';
        const accentGlow = styles.getPropertyValue('--accent-glow').trim() || 'rgba(59, 130, 246, 0.16)';
        const mutedColor = styles.getPropertyValue('--text-muted').trim() || '#64748b';

        ctx.clearRect(0, 0, size, size);

        // Outer structural ring
        ctx.beginPath();
        ctx.arc(center, center, maxRadius + 10, 0, Math.PI * 2);
        ctx.fillStyle = bgPrimary;
        ctx.fill();
        ctx.strokeStyle = touching ? accentColor : borderColor;
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Inner concentric radar rings
        for (let i = 1; i <= 3; i++) {
            const r = (maxRadius / 3) * i;
            ctx.beginPath();
            ctx.arc(center, center, r, 0, Math.PI * 2);
            ctx.strokeStyle = borderColor;
            ctx.lineWidth = 1;
            ctx.stroke();
        }

        // Degree ticks on outer perimeter
        for (let deg = 0; deg < 360; deg += 30) {
            const rad = (deg * Math.PI) / 180;
            const r1 = maxRadius + 3;
            const r2 = maxRadius + 8;
            ctx.beginPath();
            ctx.moveTo(center + Math.cos(rad) * r1, center + Math.sin(rad) * r1);
            ctx.lineTo(center + Math.cos(rad) * r2, center + Math.sin(rad) * r2);
            ctx.strokeStyle = deg % 90 === 0 ? mutedColor : borderColor;
            ctx.lineWidth = deg % 90 === 0 ? 1.5 : 1;
            ctx.stroke();
        }

        // Crosshair axes
        ctx.beginPath();
        ctx.moveTo(center, center - maxRadius);
        ctx.lineTo(center, center + maxRadius);
        ctx.moveTo(center - maxRadius, center);
        ctx.lineTo(center + maxRadius, center);
        ctx.strokeStyle = borderColor;
        ctx.lineWidth = 1;
        ctx.stroke();

        // Cardinal directional chevrons
        ctx.fillStyle = mutedColor;
        drawArrow(ctx, center, center - maxRadius + 10, 0);
        drawArrow(ctx, center, center + maxRadius - 10, Math.PI);
        drawArrow(ctx, center - maxRadius + 10, center, -Math.PI / 2);
        drawArrow(ctx, center + maxRadius - 10, center, Math.PI / 2);

        // Active vector line & halo
        if (touching) {
            ctx.beginPath();
            ctx.moveTo(center, center);
            ctx.lineTo(knobX, knobY);
            ctx.strokeStyle = accentColor;
            ctx.lineWidth = 2;
            ctx.stroke();

            ctx.beginPath();
            ctx.arc(knobX, knobY, knobRadius + 6, 0, Math.PI * 2);
            ctx.fillStyle = accentGlow;
            ctx.fill();
        }

        // Tactile Studio Knob
        ctx.beginPath();
        ctx.arc(knobX, knobY, knobRadius, 0, Math.PI * 2);
        ctx.fillStyle = touching ? accentColor : bgTertiary;
        ctx.fill();
        ctx.strokeStyle = touching ? accentBright : borderHover;
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Center precision dot on knob
        ctx.beginPath();
        ctx.arc(knobX, knobY, 4, 0, Math.PI * 2);
        ctx.fillStyle = touching ? '#ffffff' : mutedColor;
        ctx.fill();
    }

    window.redrawJoystick = draw;

    function drawArrow(ctx, x, y, rotation) {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(rotation);
        ctx.beginPath();
        ctx.moveTo(0, -5);
        ctx.lineTo(-4, 2);
        ctx.lineTo(4, 2);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    // Set explicit touch-action and user-select on canvas and container
    canvas.style.touchAction = 'none';
    canvas.style.userSelect = 'none';
    canvas.style.webkitUserSelect = 'none';
    if (canvas.parentElement) {
        canvas.parentElement.style.touchAction = 'none';
        canvas.parentElement.style.userSelect = 'none';
        canvas.parentElement.style.webkitUserSelect = 'none';
    }

    function getRelativePos(e) {
        const rect = canvas.getBoundingClientRect();
        let clientX = e.clientX;
        let clientY = e.clientY;

        if (e.touches && e.touches.length > 0) {
            clientX = e.touches[0].clientX;
            clientY = e.touches[0].clientY;
        } else if (e.changedTouches && e.changedTouches.length > 0) {
            clientX = e.changedTouches[0].clientX;
            clientY = e.changedTouches[0].clientY;
        }

        const scaleX = size / (rect.width || size);
        const scaleY = size / (rect.height || size);

        return {
            x: (clientX - rect.left) * scaleX,
            y: (clientY - rect.top) * scaleY
        };
    }

    function updateKnob(x, y) {
        const dx = x - center;
        const dy = y - center;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist > maxRadius) {
            knobX = center + (dx / dist) * maxRadius;
            knobY = center + (dy / dist) * maxRadius;
        } else {
            knobX = x;
            knobY = y;
        }

        const normalX = (knobX - center) / maxRadius;
        const normalY = (center - knobY) / maxRadius;

        const maxLinear = APP.settings.max_linear_speed || 0.7;
        const maxAngular = APP.settings.max_angular_speed || 0.9;

        const linear = Math.round(normalY * maxLinear * 100) / 100;
        const angular = Math.round(-normalX * maxAngular * 100) / 100;

        const velLinEl = document.getElementById('vel-linear');
        const velAngEl = document.getElementById('vel-angular');
        if (velLinEl) velLinEl.textContent = linear.toFixed(2);
        if (velAngEl) velAngEl.textContent = angular.toFixed(2);

        draw();
    }

    let lastSentLinear = null;
    let lastSentAngular = null;
    let lastSentTime = 0;

    function sendVelocity(force = false) {
        const normalX = (knobX - center) / maxRadius;
        const normalY = (center - knobY) / maxRadius;

        const maxLinear = APP.settings.max_linear_speed || 0.7;
        const maxAngular = APP.settings.max_angular_speed || 0.9;

        const linear = Math.round(normalY * maxLinear * 100) / 100;
        const angular = Math.round(-normalX * maxAngular * 100) / 100;

        const now = Date.now();
        if (!force && linear === lastSentLinear && angular === lastSentAngular && (now - lastSentTime < 140)) {
            return;
        }

        lastSentLinear = linear;
        lastSentAngular = angular;
        lastSentTime = now;

        apiPost('/api/cmd_vel', { linear, angular });
    }

    function startInteraction(e) {
        if (e.cancelable) e.preventDefault();
        touching = true;
        const pos = getRelativePos(e);
        updateKnob(pos.x, pos.y);

        if (joystickSendTimer) clearInterval(joystickSendTimer);
        sendVelocity(true);
        joystickSendTimer = setInterval(() => sendVelocity(false), 120);
    }

    function moveInteraction(e) {
        if (!touching) return;
        if (e.cancelable) e.preventDefault();
        const pos = getRelativePos(e);
        updateKnob(pos.x, pos.y);
    }

    function endInteraction(e) {
        if (!touching) return;
        if (e && e.cancelable) e.preventDefault();
        touching = false;

        knobX = center;
        knobY = center;

        if (joystickSendTimer) {
            clearInterval(joystickSendTimer);
            joystickSendTimer = null;
        }

        sendVelocity(true);

        lastSentLinear = 0;
        lastSentAngular = 0;

        const velLinEl = document.getElementById('vel-linear');
        const velAngEl = document.getElementById('vel-angular');
        if (velLinEl) velLinEl.textContent = '0.00';
        if (velAngEl) velAngEl.textContent = '0.00';

        draw();
    }

    // --- MOBILE TOUCH EVENTS (Native tracking, lancar di HP & Tablet) ---
    canvas.addEventListener('touchstart', (e) => {
        startInteraction(e);
    }, { passive: false });

    canvas.addEventListener('touchmove', (e) => {
        moveInteraction(e);
    }, { passive: false });

    canvas.addEventListener('touchend', (e) => {
        endInteraction(e);
    }, { passive: false });

    canvas.addEventListener('touchcancel', (e) => {
        endInteraction(e);
    }, { passive: false });

    // Tambahan window touch listener agar bila jari ditarik sangat jauh di HP tetap terlacak
    window.addEventListener('touchmove', (e) => {
        if (touching) moveInteraction(e);
    }, { passive: false });

    window.addEventListener('touchend', (e) => {
        if (touching) endInteraction(e);
    }, { passive: false });

    // --- DESKTOP MOUSE EVENTS (Analog Gamepad style: drag bebas keluar kanvas di laptop) ---
    canvas.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return; // Hanya klik kiri mouse
        startInteraction(e);

        function onWindowMouseMove(me) {
            moveInteraction(me);
        }

        function onWindowMouseUp(me) {
            window.removeEventListener('mousemove', onWindowMouseMove);
            window.removeEventListener('mouseup', onWindowMouseUp);
            endInteraction(me);
        }

        window.addEventListener('mousemove', onWindowMouseMove);
        window.addEventListener('mouseup', onWindowMouseUp);
    });

    draw();
}
