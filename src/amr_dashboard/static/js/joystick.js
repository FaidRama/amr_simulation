/* =================================================================
   AMR DASHBOARD — Virtual Joystick Module
   Canvas-based touch/mouse joystick for manual robot control.
   Sends cmd_vel commands to the Flask API.
   ================================================================= */

let joystickActive = false;
let joystickSendTimer = null;

function initJoystick() {
    const canvas = document.getElementById('joystick-canvas');
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const size = canvas.width;
    const center = size / 2;
    const maxRadius = (size / 2) - 20;
    const knobRadius = 28;

    let knobX = center;
    let knobY = center;
    let touching = false;

    function draw() {
        ctx.clearRect(0, 0, size, size);

        // Outer ring (background)
        ctx.beginPath();
        ctx.arc(center, center, maxRadius + 10, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(12, 18, 41, 0.6)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(0, 212, 255, 0.15)';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Inner zone circles
        for (let r = maxRadius; r > 0; r -= maxRadius / 3) {
            ctx.beginPath();
            ctx.arc(center, center, r, 0, Math.PI * 2);
            ctx.strokeStyle = `rgba(0, 212, 255, ${0.06 + (1 - r / maxRadius) * 0.06})`;
            ctx.lineWidth = 1;
            ctx.stroke();
        }

        // Crosshair
        ctx.beginPath();
        ctx.moveTo(center, center - maxRadius);
        ctx.lineTo(center, center + maxRadius);
        ctx.moveTo(center - maxRadius, center);
        ctx.lineTo(center + maxRadius, center);
        ctx.strokeStyle = 'rgba(0, 212, 255, 0.08)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Direction arrows
        const arrowColor = 'rgba(0, 212, 255, 0.2)';
        ctx.fillStyle = arrowColor;
        // Up arrow
        drawArrow(ctx, center, center - maxRadius + 12, 0);
        // Down arrow
        drawArrow(ctx, center, center + maxRadius - 12, Math.PI);
        // Left arrow
        drawArrow(ctx, center - maxRadius + 12, center, -Math.PI / 2);
        // Right arrow
        drawArrow(ctx, center + maxRadius - 12, center, Math.PI / 2);

        // Knob shadow
        if (touching) {
            ctx.beginPath();
            ctx.arc(knobX, knobY, knobRadius + 6, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(0, 212, 255, 0.12)';
            ctx.fill();
        }

        // Knob connection line
        if (touching) {
            ctx.beginPath();
            ctx.moveTo(center, center);
            ctx.lineTo(knobX, knobY);
            ctx.strokeStyle = 'rgba(0, 212, 255, 0.25)';
            ctx.lineWidth = 2;
            ctx.stroke();
        }

        // Knob
        const gradient = ctx.createRadialGradient(knobX, knobY, 0, knobX, knobY, knobRadius);
        if (touching) {
            gradient.addColorStop(0, 'rgba(0, 212, 255, 0.9)');
            gradient.addColorStop(1, 'rgba(0, 150, 200, 0.7)');
        } else {
            gradient.addColorStop(0, 'rgba(0, 212, 255, 0.5)');
            gradient.addColorStop(1, 'rgba(0, 120, 170, 0.35)');
        }
        ctx.beginPath();
        ctx.arc(knobX, knobY, knobRadius, 0, Math.PI * 2);
        ctx.fillStyle = gradient;
        ctx.fill();
        ctx.strokeStyle = touching ? 'rgba(0, 212, 255, 0.8)' : 'rgba(0, 212, 255, 0.3)';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Inner knob highlight
        ctx.beginPath();
        ctx.arc(knobX, knobY, knobRadius * 0.4, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.fill();
    }

    function drawArrow(ctx, x, y, rotation) {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(rotation);
        ctx.beginPath();
        ctx.moveTo(0, -6);
        ctx.lineTo(-5, 3);
        ctx.lineTo(5, 3);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    function getRelativePos(e) {
        const rect = canvas.getBoundingClientRect();
        let clientX, clientY;
        if (e.touches && e.touches.length > 0) {
            clientX = e.touches[0].clientX;
            clientY = e.touches[0].clientY;
        } else {
            clientX = e.clientX;
            clientY = e.clientY;
        }
        return {
            x: clientX - rect.left,
            y: clientY - rect.top
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

        // Calculate velocities
        // Y axis = linear velocity (up = forward = positive)
        // X axis = angular velocity (left = positive = turn left)
        const normalX = (knobX - center) / maxRadius;  // -1 to 1
        const normalY = (center - knobY) / maxRadius;   // -1 to 1 (inverted for Y)

        const maxLinear = APP.settings.max_linear_speed || 0.7;
        const maxAngular = APP.settings.max_angular_speed || 0.9;

        const linear = Math.round(normalY * maxLinear * 100) / 100;
        const angular = Math.round(-normalX * maxAngular * 100) / 100;

        // Update display
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
        // Jangan kirim request HTTP berulang jika nilai velocity sama, kecuali force=true atau sudah 250ms
        if (!force && linear === lastSentLinear && angular === lastSentAngular && (now - lastSentTime < 250)) {
            return;
        }

        lastSentLinear = linear;
        lastSentAngular = angular;
        lastSentTime = now;

        apiPost('/api/cmd_vel', { linear, angular });
    }

    function startTouch(e) {
        e.preventDefault();
        touching = true;
        const pos = getRelativePos(e);
        updateKnob(pos.x, pos.y);

        if (joystickSendTimer) clearInterval(joystickSendTimer);
        sendVelocity(true);
        joystickSendTimer = setInterval(() => sendVelocity(false), 120);
    }

    function moveTouch(e) {
        e.preventDefault();
        if (!touching) return;
        const pos = getRelativePos(e);
        updateKnob(pos.x, pos.y);
    }

    function endTouch(e) {
        e.preventDefault();
        touching = false;
        knobX = center;
        knobY = center;

        // Stop sending velocity
        if (joystickSendTimer) {
            clearInterval(joystickSendTimer);
            joystickSendTimer = null;
        }

        // Send zero velocity immediately
        sendVelocity(true);

        lastSentLinear = 0;
        lastSentAngular = 0;

        const velLinEl = document.getElementById('vel-linear');
        const velAngEl = document.getElementById('vel-angular');
        if (velLinEl) velLinEl.textContent = '0.00';
        if (velAngEl) velAngEl.textContent = '0.00';

        draw();
    }

    // Touch events (mobile)
    canvas.addEventListener('touchstart', startTouch, { passive: false });
    canvas.addEventListener('touchmove', moveTouch, { passive: false });
    canvas.addEventListener('touchend', endTouch, { passive: false });
    canvas.addEventListener('touchcancel', endTouch, { passive: false });

    // Mouse events (desktop fallback)
    canvas.addEventListener('mousedown', startTouch);
    canvas.addEventListener('mousemove', moveTouch);
    canvas.addEventListener('mouseup', endTouch);
    canvas.addEventListener('mouseleave', endTouch);

    // Initial draw
    draw();
}
