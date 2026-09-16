/* =================================================================
   AMR DASHBOARD — Monitoring Module
   Additional monitoring utilities (placeholder for future expansion).
   The main monitoring logic is already in app.js and map.js.
   This module provides utility functions for monitoring.
   ================================================================= */

// Format elapsed time nicely
function formatElapsed(seconds) {
    if (!seconds || seconds <= 0) return '0s';
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = Math.floor(seconds % 60);
    if (h > 0) return `${h}j ${m}m ${s}d`;
    if (m > 0) return `${m}m ${s}d`;
    return `${s}d`;
}

// Get status color class
function getStatusColor(status) {
    const colors = {
        idle: 'mon-accent',
        running: 'mon-accent',
        paused: 'mon-warning',
        completed: 'mon-success',
        failed: 'mon-danger',
        cancelled: 'mon-danger'
    };
    return colors[status] || 'mon-accent';
}

// Get phase label in Indonesian
function getPhaseLabel(phase) {
    const labels = {
        idle: 'Siap',
        navigating_pickup: 'Menuju Pickup',
        at_pickup: 'Mengambil Barang',
        navigating_delivery: 'Menuju Tujuan',
        at_delivery: 'Menyerahkan Barang'
    };
    return labels[phase] || phase;
}

// Get human status label
function getHumanStatusLabel(status) {
    const labels = {
        'NO_HUMAN': 'Aman',
        'IDLE': 'Manusia (Diam)',
        'WALKING': 'Manusia (Berjalan)',
        'DISTURBING': '⚠ Menghalangi!',
        'CALLING': '✋ Memanggil',
        'CALIBRATION': 'Kalibrasi...'
    };
    return labels[status] || status;
}

// Vibrate phone on important events
function vibrateAlert(pattern) {
    if ('vibrate' in navigator) {
        navigator.vibrate(pattern || [100, 50, 100]);
    }
}

// Check if status changed and notify
let lastDeliveryStatus = 'idle';

function checkStatusAlerts(newStatus) {
    if (newStatus === lastDeliveryStatus) return;

    if (newStatus === 'completed' && lastDeliveryStatus === 'running') {
        showToast('🎉 Misi selesai! Semua task berhasil.', 'success');
        vibrateAlert([200, 100, 200, 100, 200]);
    } else if (newStatus === 'cancelled') {
        showToast('Misi dibatalkan.', 'error');
        vibrateAlert([300]);
    }

    lastDeliveryStatus = newStatus;
}
