#!/usr/bin/env python3
"""
=============================================================================
AMR ROBOT VOICE & BLUETOOTH WATCHDOG NODE
Untuk Speaker Bluetooth (Acome, JBL, dsb.) di Jetson AGX Xavier / Linux.
=============================================================================
Fitur Utama:
  1. Auto-Reconnect Bluetooth Watchdog:
     - Memantau koneksi speaker bluetooth secara berkala di background thread.
     - Jika speaker mati/hidup lagi atau sinyal sempat putus, otomatis `bluetoothctl connect`.
     - Otomatis mengarahkan PulseAudio Default Sink ke speaker bluetooth.
  2. Non-blocking & Anti-Conflict Queue:
     - Semua proses suara diproses di antrean terpisah (worker thread).
     - Jika speaker mati atau audio error, Nav2 / kontrol robot TIDAK AKAN PERNAH terblokir/hang.
  3. Safe-Stop Obstacle Voice ("Permisi, robot mau lewat"):
     - Memantau topic `/human_state`.
     - Jika state `SAFE_STOP` berlangsung lebih lama dari batas waktu (default 10 detik,
       dapat dikonfigurasi via dashboard), robot otomatis bersuara meminta jalan.
  4. ROS 2 Topics:
     - Sub `/robot_voice/say`    (std_msgs/String): Ucapkan teks bebas (Google TTS / Offline)
     - Sub `/robot_voice/play`   (std_msgs/String): Play preset ('arrival', 'pickup', 'obstacle', 'chime')
     - Sub `/robot_voice/config` (std_msgs/String): Update konfigurasi realtime dari dashboard
     - Pub `/robot_voice/status` (std_msgs/String): Status Bluetooth & konfigurasi aktif (JSON)
  5. CLI & Mode Interaktif untuk pengujian cepat di terminal.
=============================================================================
"""

import os
import sys
import time
import math
import wave
import queue
import struct
import shutil
import hashlib
import json
import argparse
import threading
import subprocess
import urllib.parse
import urllib.request

# Folder cache audio
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
DASHBOARD_AUDIO_DIR = os.path.join(WS_DIR, 'src', 'amr_dashboard', 'static', 'audio_cache')
FALLBACK_CACHE_DIR = os.path.expanduser('~/.cache/amr_voice')
CONFIG_FILE_PATH = os.path.join(WS_DIR, 'src', 'amr_simulation', 'config', 'voice_config.json')

CHIME_WAV_PATH = '/tmp/amr_chime.wav'

DEFAULT_CONFIG = {
    "target_mac": "0E:21:D5:E9:AB:5F",
    "target_name": "Acome A1",
    "auto_reconnect": True,
    "reconnect_interval_s": 5,
    "obstacle_voice_delay_s": 10,
    "obstacle_voice_repeat_s": 12,
    "obstacle_phrase": "Permisi, robot mau lewat. Mohon beri jalan.",
    "arrival_phrase": "Deliver tiba. Robot sudah sampai di titik tujuan, silakan ambil barang.",
    "pickup_phrase": "Pickup tiba. Robot sudah sampai di titik pickup, silakan muat barang.",
    "home_phrase": "Misi selesai. Robot kembali ke stasiun pangkalan."
}


def load_voice_config():
    """Muat konfigurasi dari file JSON jika ada."""
    config = dict(DEFAULT_CONFIG)
    paths_to_check = [
        CONFIG_FILE_PATH,
        os.path.expanduser('~/deliver_amr/src/amr_simulation/config/voice_config.json'),
        os.path.expanduser('~/.config/amr_voice/voice_config.json')
    ]
    for p in paths_to_check:
        if os.path.isfile(p):
            try:
                with open(p, 'r') as f:
                    loaded = json.load(f)
                    config.update(loaded)
                break
            except Exception:
                pass
    return config


def save_voice_config(config_dict):
    """Simpan konfigurasi ke file JSON."""
    paths_to_save = [CONFIG_FILE_PATH, os.path.expanduser('~/.config/amr_voice/voice_config.json')]
    for p in paths_to_save:
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, 'w') as f:
                json.dump(config_dict, f, indent=2)
            return True
        except Exception:
            pass
    return False


def get_cache_dir():
    """Tentukan folder cache audio yang valid."""
    if os.path.isdir(DASHBOARD_AUDIO_DIR) and os.access(DASHBOARD_AUDIO_DIR, os.W_OK):
        return DASHBOARD_AUDIO_DIR
    os.makedirs(FALLBACK_CACHE_DIR, exist_ok=True)
    return FALLBACK_CACHE_DIR


def generate_chime_wav(filepath=CHIME_WAV_PATH):
    """Sintesis suara ding-dong 2 nada (E6 -> A5) murni dengan python wave/struct."""
    sample_rate = 44100
    tones = [(1318.5, 0.22, 0.7), (880.0, 0.55, 0.85)]
    frames = []

    for freq, duration, vol in tones:
        n_samples = int(sample_rate * duration)
        for i in range(n_samples):
            t = float(i) / sample_rate
            decay = math.exp(-3.8 * (float(i) / n_samples))
            val = int(vol * 32767.0 * decay * math.sin(2.0 * math.pi * freq * t))
            frames.append(struct.pack('<h', max(-32768, min(32767, val))))

    try:
        with wave.open(filepath, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(b''.join(frames))
    except Exception:
        pass

    return filepath


def get_pulse_env():
    """Environment untuk berkomunikasi dengan PulseAudio user jetson (UID 1000)."""
    env = dict(os.environ)
    for uid in ['1000', '1001']:
        user_run = f'/run/user/{uid}'
        pulse_sock = f'{user_run}/pulse/native'
        if os.path.exists(pulse_sock):
            env['XDG_RUNTIME_DIR'] = user_run
            env['PULSE_SERVER'] = f'unix:{pulse_sock}'
            break
    if 'USER' not in env:
        env['USER'] = 'jetson'
    if 'HOME' not in env:
        env['HOME'] = '/home/jetson'
    return env


def route_audio_to_bluetooth(mac_address=None):
    """Pastikan default sink PulseAudio diarahkan ke Bluetooth speaker dan tidak di-mute."""
    env = get_pulse_env()
    try:
        res = subprocess.run(['pactl', 'list', 'sinks', 'short'],
                             env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        bt_sinks = []
        if res.returncode == 0:
            for line in res.stdout.strip().splitlines():
                parts = line.split('\t')
                sname = parts[1] if len(parts) > 1 else parts[0]
                if 'bluez' in sname.lower():
                    bt_sinks.append(sname)

        target_sink = None
        if mac_address:
            mac_clean = mac_address.replace(':', '_').lower()
            for s in bt_sinks:
                if mac_clean in s.lower():
                    target_sink = s
                    break
        if not target_sink and bt_sinks:
            target_sink = bt_sinks[0]

        if target_sink:
            subprocess.run(['pactl', 'set-default-sink', target_sink],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(['pactl', 'set-sink-mute', target_sink, '0'],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(['pactl', 'set-sink-volume', target_sink, '85%'],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            return target_sink
    except Exception:
        pass
    return None


def check_pulseaudio_status():
    """Cek status PulseAudio dan apakah Bluetooth Sink terdeteksi."""
    info = {
        'pulseaudio_running': False,
        'default_sink': '',
        'has_bluetooth_sink': False,
        'bluetooth_sinks': []
    }
    env = get_pulse_env()
    try:
        res = subprocess.run(['pactl', 'list', 'sinks', 'short'],
                             env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        if res.returncode == 0:
            info['pulseaudio_running'] = True
            lines = res.stdout.strip().splitlines()
            for line in lines:
                parts = line.split('\t')
                sink_name = parts[1] if len(parts) > 1 else parts[0]
                if 'bluez' in sink_name.lower():
                    info['has_bluetooth_sink'] = True
                    info['bluetooth_sinks'].append(sink_name)

        res_info = subprocess.run(['pactl', 'info'],
                                  env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        if res_info.returncode == 0:
            for line in res_info.stdout.splitlines():
                if 'Default Sink:' in line:
                    info['default_sink'] = line.split('Default Sink:')[-1].strip()
    except Exception:
        pass

    return info


def is_bluetooth_device_connected(mac_address):
    """Cek apakah perangkat MAC tertentu terkoneksi di BlueZ / PulseAudio."""
    if not mac_address:
        return False
    # Cek via bluetoothctl info
    try:
        res = subprocess.run(['bluetoothctl', 'info', mac_address],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        if res.returncode == 0 and 'Connected: yes' in res.stdout:
            return True
    except Exception:
        pass

    # Cek via pactl sink
    mac_clean = mac_address.replace(':', '_').lower()
    pa_status = check_pulseaudio_status()
    for s in pa_status['bluetooth_sinks']:
        if mac_clean in s.lower():
            return True

    return False


def play_audio_file(filepath):
    """Putar file audio (wav atau mp3) secara aman dengan batas timeout langsung ke speaker Bluetooth."""
    if not os.path.exists(filepath):
        return False

    env = get_pulse_env()
    target_sink = route_audio_to_bluetooth()

    ext = os.path.splitext(filepath)[1].lower()

    # Prioritas WAV
    if ext == '.wav':
        if shutil.which('paplay'):
            try:
                cmd = ['paplay']
                if target_sink:
                    cmd += ['-d', target_sink]
                cmd.append(filepath)
                res = subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6)
                if res.returncode == 0:
                    return True
            except Exception:
                pass
        if shutil.which('aplay'):
            try:
                res = subprocess.run(['aplay', '-q', filepath], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6)
                if res.returncode == 0:
                    return True
            except Exception:
                pass

    # Prioritas MP3
    if shutil.which('mpg123'):
        try:
            cmd = ['mpg123', '-q']
            if target_sink:
                cmd += ['-a', target_sink]
            cmd.append(filepath)
            res = subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6)
            if res.returncode == 0:
                return True
        except Exception:
            pass
    if shutil.which('ffmpeg') and shutil.which('paplay'):
        try:
            wav_tmp = f"/tmp/amr_play_{int(time.time()*1000)}.wav"
            subprocess.run(['ffmpeg', '-y', '-i', filepath, '-ar', '44100', '-ac', '2', wav_tmp],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=4)
            if os.path.exists(wav_tmp):
                cmd = ['paplay']
                if target_sink:
                    cmd += ['-d', target_sink]
                cmd.append(wav_tmp)
                res = subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6)
                try:
                    os.remove(wav_tmp)
                except Exception:
                    pass
                if res.returncode == 0:
                    return True
        except Exception:
            pass
    if shutil.which('ffplay'):
        try:
            res = subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', filepath],
                                 env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6)
            if res.returncode == 0:
                return True
        except Exception:
            pass

    return False


def get_tts_audio_path(text):
    """Ambil audio TTS MP3 (cek cache lokal dulu, jika tidak ada baru unduh)."""
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    cache_dir = get_cache_dir()
    cache_file = os.path.join(cache_dir, f"{text_hash}.mp3")

    if os.path.exists(cache_file) and os.path.getsize(cache_file) > 0:
        return cache_file

    try:
        url = 'https://translate.google.com/translate_tts?ie=UTF-8&tl=id&client=tw-ob&q=' + urllib.parse.quote(text)
        req = urllib.request.Request(
            url,
            headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        )
        with urllib.request.urlopen(req, timeout=4) as resp:
            data = resp.read()
            with open(cache_file, 'wb') as f:
                f.write(data)
        return cache_file
    except Exception as e:
        return None


def speak(text, play_chime_first=True, config=None):
    """Keluarkan suara robot: bunyikan chime lalu ucapkan kalimat secara aman."""
    cfg = config or load_voice_config()
    text_to_speak = text.strip()

    # Cek preset phrase
    preset_map = {
        'arrival': cfg.get('arrival_phrase', DEFAULT_CONFIG['arrival_phrase']),
        'deliver': cfg.get('arrival_phrase', DEFAULT_CONFIG['arrival_phrase']),
        'delivery': cfg.get('arrival_phrase', DEFAULT_CONFIG['arrival_phrase']),
        'pickup': cfg.get('pickup_phrase', DEFAULT_CONFIG['pickup_phrase']),
        'obstacle': cfg.get('obstacle_phrase', DEFAULT_CONFIG['obstacle_phrase']),
        'home': cfg.get('home_phrase', DEFAULT_CONFIG['home_phrase']),
    }

    if text_to_speak.lower() in preset_map:
        text_to_speak = preset_map[text_to_speak.lower()]

    print(f"📢 [ROBOT SPEAK]: \"{text_to_speak}\"")

    if play_chime_first:
        chime_file = generate_chime_wav()
        play_audio_file(chime_file)
        time.sleep(0.15)

    # 1. Coba Google TTS (online / cache lokal)
    audio_file = get_tts_audio_path(text_to_speak)
    if audio_file and os.path.exists(audio_file):
        played = play_audio_file(audio_file)
        if played:
            return True

    # 2. Fallback offline: espeak jika tersedia
    if shutil.which('espeak'):
        try:
            subprocess.run(['espeak', '-v', 'id', text_to_speak],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            return True
        except Exception:
            pass

    # 3. Fallback offline: spd-say jika tersedia
    if shutil.which('spd-say'):
        try:
            subprocess.run(['spd-say', '-l', 'id', text_to_speak],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            return True
        except Exception:
            pass

    return False


# =============================================================================
# BLUETOOTH WATCHDOG THREAD (Auto-Reconnect)
# =============================================================================
class BluetoothWatchdog(threading.Thread):
    """Thread latar belakang yang memantau dan mengkoneksikan ulang speaker Bluetooth otomatis."""
    def __init__(self, config_getter):
        super().__init__(daemon=True)
        self.config_getter = config_getter
        self.running = True
        self.last_status = False
        self.connected_mac = None

    def run(self):
        while self.running:
            cfg = self.config_getter()
            target_mac = cfg.get('target_mac', '').strip()
            auto_recon = cfg.get('auto_reconnect', True)
            interval = cfg.get('reconnect_interval_s', 5)

            if target_mac and auto_recon:
                connected = is_bluetooth_device_connected(target_mac)
                self.last_status = connected

                if not connected:
                    # Coba konek ulang secara silent & non-blocking
                    try:
                        res = subprocess.run(['bluetoothctl', 'connect', target_mac],
                                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=4)
                        if 'Connection successful' in res.stdout or 'Connected: yes' in res.stdout:
                            self.last_status = True
                            self.connected_mac = target_mac
                            print(f"\n[BluetoothWatchdog] 🔵 Speaker {target_mac} BERHASIL terkoneksi otomatis! ✅")
                            # Set default sink
                            self._set_default_sink(target_mac)
                    except Exception:
                        pass
                else:
                    self.connected_mac = target_mac
                    self._set_default_sink(target_mac)

            time.sleep(max(2, interval))

    def _set_default_sink(self, mac_address):
        """Setel sink Bluetooth sebagai default sink PulseAudio jika belum."""
        try:
            mac_clean = mac_address.replace(':', '_')
            target_sink = f"bluez_sink.{mac_clean}.a2dp_sink"
            pa_status = check_pulseaudio_status()
            if target_sink in pa_status['bluetooth_sinks'] and pa_status['default_sink'] != target_sink:
                subprocess.run(['pactl', 'set-default-sink', target_sink],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
                subprocess.run(['pactl', 'set-sink-volume', target_sink, '80%'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
        except Exception:
            pass

    def stop(self):
        self.running = False


# =============================================================================
# ROS 2 NODE INTEGRATION
# =============================================================================
def run_ros2_node():
    """Jalankan sebagai ROS 2 Node yang melayani perintah suara AMR dan Safe Stop timer."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("❌ ROS 2 (rclpy) tidak terdeteksi di lingkungan ini.")
        print("   Pastikan sudah 'source /opt/ros/foxy/setup.bash'")
        return

    rclpy.init()

    class RobotVoiceNode(Node):
        def __init__(self):
            super().__init__('robot_voice_node')
            self.config = load_voice_config()

            # Antrean suara (worker thread) agar tidak pernah memblokir ROS executor
            self.speech_queue = queue.Queue()
            self.worker_thread = threading.Thread(target=self._speech_worker, daemon=True)
            self.worker_thread.start()

            # Watchdog auto-reconnect bluetooth di thread terpisah
            self.watchdog = BluetoothWatchdog(lambda: self.config)
            self.watchdog.start()

            # Tracking SAFE_STOP
            self.safe_stop_start_time = None
            self.last_obstacle_spoken_time = 0.0

            # === SUBSCRIBERS ===
            self.sub_say = self.create_subscription(
                String, '/robot_voice/say', self.callback_say, 10
            )
            self.sub_play = self.create_subscription(
                String, '/robot_voice/play', self.callback_play, 10
            )
            self.sub_config = self.create_subscription(
                String, '/robot_voice/config', self.callback_config, 10
            )
            self.sub_human = self.create_subscription(
                String, '/human_state', self.callback_human_state, 10
            )

            # === PUBLISHER ===
            self.status_pub = self.create_publisher(String, '/robot_voice/status', 10)
            self.status_timer = self.create_timer(2.0, self.publish_status)

            self.get_logger().info("=" * 60)
            self.get_logger().info("🔊 [robot_voice_node] AKTIF!")
            self.get_logger().info(f"   - Target Speaker: {self.config.get('target_name')} ({self.config.get('target_mac')})")
            self.get_logger().info(f"   - Obstacle Voice Delay: {self.config.get('obstacle_voice_delay_s')}s")
            self.get_logger().info("   - Auto-reconnect Watchdog: AKTIF")
            self.get_logger().info("=" * 60)

        def _speech_worker(self):
            """Worker thread yang memproses antrean pemutaran suara secara berurutan."""
            while rclpy.ok():
                try:
                    item = self.speech_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                mode, payload = item
                try:
                    if mode == 'chime':
                        play_audio_file(generate_chime_wav())
                    elif mode == 'speak':
                        speak(payload, play_chime_first=True, config=self.config)
                except Exception as e:
                    self.get_logger().warn(f"Error pada pemutaran audio: {e}")
                finally:
                    self.speech_queue.task_done()

        def callback_say(self, msg):
            text = msg.data.strip()
            if text:
                self.get_logger().info(f"Antrean ucapan: '{text}'")
                self.speech_queue.put(('speak', text))

        def callback_play(self, msg):
            cmd = msg.data.strip().lower()
            if cmd == 'chime':
                self.speech_queue.put(('chime', None))
            else:
                self.speech_queue.put(('speak', cmd))

        def callback_config(self, msg):
            """Update konfigurasi realtime dari topic /robot_voice/config."""
            try:
                new_data = json.loads(msg.data)
                self.config.update(new_data)
                save_voice_config(self.config)
                self.get_logger().info(f"⚙️ Konfigurasi diperbarui: delay={self.config.get('obstacle_voice_delay_s')}s, target={self.config.get('target_mac')}")
                self.publish_status()
            except Exception as e:
                self.get_logger().warn(f"Gagal memproses konfigurasi baru: {e}")

        def callback_human_state(self, msg):
            """Pantau status SAFE_STOP dan picu suara jika terhalang melebihi batas waktu."""
            state = msg.data.strip().upper()
            now = time.time()

            if state == 'SAFE_STOP':
                if self.safe_stop_start_time is None:
                    self.safe_stop_start_time = now
                else:
                    duration = now - self.safe_stop_start_time
                    delay_threshold = float(self.config.get('obstacle_voice_delay_s', 10.0))
                    repeat_interval = float(self.config.get('obstacle_voice_repeat_s', 12.0))

                    if duration >= delay_threshold and (now - self.last_obstacle_spoken_time) >= repeat_interval:
                        phrase = self.config.get('obstacle_phrase', DEFAULT_CONFIG['obstacle_phrase'])
                        self.get_logger().info(f"🚶 [SAFE_STOP {duration:.1f}s >= {delay_threshold}s] Mengucapkan peringatan halangan!")
                        self.speech_queue.put(('speak', phrase))
                        self.last_obstacle_spoken_time = now
            else:
                # Reset jika sudah bukan SAFE_STOP (jalan sudah bersih)
                if self.safe_stop_start_time is not None:
                    self.safe_stop_start_time = None

        def publish_status(self):
            """Publish status koneksi Bluetooth dan setting suara untuk dashboard."""
            is_connected = is_bluetooth_device_connected(self.config.get('target_mac'))
            status = {
                'connected': is_connected,
                'target_mac': self.config.get('target_mac'),
                'target_name': self.config.get('target_name'),
                'auto_reconnect': self.config.get('auto_reconnect', True),
                'obstacle_voice_delay_s': self.config.get('obstacle_voice_delay_s', 10),
                'obstacle_phrase': self.config.get('obstacle_phrase'),
                'queue_size': self.speech_queue.qsize(),
                'timestamp': time.time()
            }
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)

    node = RobotVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.watchdog.stop()
        node.destroy_node()
        rclpy.shutdown()


# =============================================================================
# MENU INTERAKTIF
# =============================================================================
def interactive_menu():
    """Menu interaktif pengujian di terminal Jetson."""
    config = load_voice_config()
    while True:
        print("\n" + "=" * 58)
        print("  🔊 PENGUJIAN SUARA ROBOT & BLUETOOTH WATCHDOG")
        print("=" * 58)
        print("  1. 🔔 Bunyikan Chime / Ding-dong (WAV Offline)")
        print("  2. 🎯 Suara: 'Deliver tiba. Silakan ambil barang.'")
        print("  3. 📦 Suara: 'Pickup tiba. Silakan muat barang.'")
        print("  4. 🚶 Suara: 'Permisi, robot mau lewat.'")
        print("  5. 💬 Ucapkan Kalimat Custom (Ketik bebas)")
        print("  6. 📊 Periksa Status Bluetooth & Audio Sink")
        print("  7. 🔗 Hubungkan Ulang Bluetooth Speaker Manual")
        print("  8. 🤖 Jalankan sebagai ROS 2 Voice & Watchdog Node")
        print("  q. Keluar")
        print("=" * 58)

        try:
            choice = input("👉 Pilih menu (1-8 / q): ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nKeluar.")
            break

        if choice == '1':
            print("🔔 Membunyikan Chime...")
            chime = generate_chime_wav()
            play_audio_file(chime)
        elif choice == '2':
            speak(config.get('arrival_phrase', DEFAULT_CONFIG['arrival_phrase']), config=config)
        elif choice == '3':
            speak(config.get('pickup_phrase', DEFAULT_CONFIG['pickup_phrase']), config=config)
        elif choice == '4':
            speak(config.get('obstacle_phrase', DEFAULT_CONFIG['obstacle_phrase']), config=config)
        elif choice == '5':
            try:
                custom_text = input("✍️ Masukkan kalimat yang ingin diucapkan robot: ").strip()
                if custom_text:
                    speak(custom_text, config=config)
            except (KeyboardInterrupt, EOFError):
                pass
        elif choice == '6':
            check_pulseaudio_status()
            target_mac = config.get('target_mac', '')
            conn = is_bluetooth_device_connected(target_mac)
            status_str = "TERHUBUNG ✅" if conn else "TERPUTUS ❌"
            print(f"🎯 Target Speaker: {config.get('target_name')} ({target_mac}) -> {status_str}")
        elif choice == '7':
            target_mac = config.get('target_mac', '')
            print(f"🔄 Menghubungkan ke {target_mac}...")
            subprocess.run(['bluetoothctl', 'connect', target_mac])
        elif choice == '8':
            print("🤖 Memulai ROS 2 Node Voice & Watchdog...")
            run_ros2_node()
        elif choice in ('q', 'exit', 'quit'):
            print("Selesai.")
            break
        else:
            print("⚠️ Pilihan tidak valid.")


def main():
    parser = argparse.ArgumentParser(description="Tool & Node Suara Robot AMR & Bluetooth Watchdog")
    parser.add_argument('--say', type=str, help="Ucapkan kalimat langsung")
    parser.add_argument('--arrival', action='store_true', help="Bunyikan suara tiba di tujuan")
    parser.add_argument('--pickup', action='store_true', help="Bunyikan suara tiba di pickup")
    parser.add_argument('--obstacle', action='store_true', help="Bunyikan suara permisi robot mau lewat")
    parser.add_argument('--chime', action='store_true', help="Bunyikan ding-dong chime saja")
    parser.add_argument('--status', action='store_true', help="Cek status bluetooth audio sink")
    parser.add_argument('--node', action='store_true', help="Jalankan sebagai ROS 2 Voice & Watchdog Node")
    parser.add_argument('--connect', type=str, help="Koneksikan ke MAC address bluetooth tertentu")

    args = parser.parse_args()
    config = load_voice_config()

    if args.status:
        check_pulseaudio_status()
        conn = is_bluetooth_device_connected(config.get('target_mac'))
        print(f"Status Speaker {config.get('target_mac')}: {'TERHUBUNG ✅' if conn else 'TERPUTUS ❌'}")
    elif args.chime:
        play_audio_file(generate_chime_wav())
    elif args.arrival:
        speak(config.get('arrival_phrase', DEFAULT_CONFIG['arrival_phrase']), config=config)
    elif args.pickup:
        speak(config.get('pickup_phrase', DEFAULT_CONFIG['pickup_phrase']), config=config)
    elif args.obstacle:
        speak(config.get('obstacle_phrase', DEFAULT_CONFIG['obstacle_phrase']), config=config)
    elif args.say:
        speak(args.say, config=config)
    elif args.connect:
        subprocess.run(['bluetoothctl', 'connect', args.connect])
    elif args.node:
        run_ros2_node()
    else:
        interactive_menu()


if __name__ == '__main__':
    main()
