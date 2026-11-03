#!/usr/bin/env python3
"""
=============================================================================
TEST & SUARA ROBOT (AMR VOICE NOTIFIER)
Untuk Speaker Bluetooth (Acome, JBL, dsb.) di Jetson AGX Xavier / Linux.
=============================================================================
Fitur:
  1. Cek status koneksi Bluetooth Speaker & PulseAudio Sink secara otomatis.
  2. Sintesis bunyi Chime/Ding-Dong WAV murni (100% offline, tanpa dependencies).
  3. Pemutaran suara Text-to-Speech (Google TTS Bahasa Indonesia & Cache lokal).
  4. Mode CLI interaktif atau argumen (--say, --arrival, --pickup, --chime).
  5. Mode ROS 2 Node (--node) yang mendengarkan topic /robot_voice/say dan /robot_voice/play.
=============================================================================
"""

import os
import sys
import time
import math
import wave
import struct
import shutil
import hashlib
import argparse
import subprocess
import urllib.parse
import urllib.request

# Folder cache audio
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
DASHBOARD_AUDIO_DIR = os.path.join(WS_DIR, 'src', 'amr_dashboard', 'static', 'audio_cache')
FALLBACK_CACHE_DIR = os.path.expanduser('~/.cache/amr_voice')

CHIME_WAV_PATH = '/tmp/amr_chime.wav'

PRESET_PHRASES = {
    'arrival': "Robot sudah sampai di titik tujuan. Silakan ambil barang.",
    'pickup': "Robot sudah sampai di titik pickup. Silakan muat barang.",
    'obstacle': "Permisi, ada halangan di depan robot. Mohon beri jalan.",
    'moving': "Robot mulai bergerak menuju lokasi berikutnya.",
    'home': "Misi selesai. Robot kembali ke stasiun pangkalan."
}


def get_cache_dir():
    """Tentukan folder cache audio yang valid."""
    if os.path.isdir(DASHBOARD_AUDIO_DIR) and os.access(DASHBOARD_AUDIO_DIR, os.W_OK):
        return DASHBOARD_AUDIO_DIR
    os.makedirs(FALLBACK_CACHE_DIR, exist_ok=True)
    return FALLBACK_CACHE_DIR


def generate_chime_wav(filepath=CHIME_WAV_PATH):
    """Sintesis suara ding-dong 2 nada (E6 -> A5) murni dengan python wave/struct."""
    sample_rate = 44100
    # Nada 1: E6 (1318.5 Hz, 0.22s), Nada 2: A5 (880.0 Hz, 0.5s)
    tones = [(1318.5, 0.22, 0.7), (880.0, 0.55, 0.85)]
    frames = []

    for freq, duration, vol in tones:
        n_samples = int(sample_rate * duration)
        for i in range(n_samples):
            t = float(i) / sample_rate
            # Decay envelope eksponensial lembut
            decay = math.exp(-3.8 * (float(i) / n_samples))
            val = int(vol * 32767.0 * decay * math.sin(2.0 * math.pi * freq * t))
            frames.append(struct.pack('<h', max(-32768, min(32767, val))))

    with wave.open(filepath, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))

    return filepath


def check_pulseaudio_status():
    """Cek status PulseAudio dan apakah Bluetooth Sink terdeteksi."""
    info = {
        'pulseaudio_running': False,
        'default_sink': '',
        'has_bluetooth_sink': False,
        'bluetooth_sinks': []
    }

    try:
        res = subprocess.run(['pactl', 'list', 'sinks', 'short'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3)
        if res.returncode == 0:
            info['pulseaudio_running'] = True
            lines = res.stdout.strip().splitlines()
            for line in lines:
                parts = line.split('\t')
                sink_name = parts[1] if len(parts) > 1 else parts[0]
                if 'bluez' in sink_name.lower():
                    info['has_bluetooth_sink'] = True
                    info['bluetooth_sinks'].append(sink_name)

        # Cek default sink
        res_info = subprocess.run(['pactl', 'info'],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3)
        if res_info.returncode == 0:
            for line in res_info.stdout.splitlines():
                if 'Default Sink:' in line:
                    info['default_sink'] = line.split('Default Sink:')[-1].strip()
    except Exception:
        pass

    return info


def print_audio_status():
    """Cetak ringkasan status Bluetooth & Audio ke terminal."""
    status = check_pulseaudio_status()
    print("=" * 65)
    print(" 📡 STATUS AUDIO SISTEM JETSON")
    print("=" * 65)
    if not status['pulseaudio_running']:
        print(" ❌ PulseAudio TIDAK aktif atau pactl tidak ditemukan.")
        print("    Jalankan: pulseaudio --start")
        return False

    print(" ✅ PulseAudio Server: AKTIF")
    print(f" 🎯 Default Sink     : {status['default_sink'] or 'None'}")

    if status['has_bluetooth_sink']:
        print(" 🔵 Bluetooth Sink   : TERDETEKSI! ✅")
        for s in status['bluetooth_sinks']:
            is_def = " (DEFAULT)" if s == status['default_sink'] else ""
            print(f"    - {s}{is_def}")
    else:
        print(" ⚠️ Bluetooth Sink   : BELUM TERHUBUNG.")
        print("    Petunjuk: Hubungkan speaker Acome via bluetoothctl terlebih dahulu.")
    print("=" * 65)
    return status['has_bluetooth_sink']


def play_audio_file(filepath):
    """Putar file audio (wav atau mp3) menggunakan alat yang tersedia."""
    if not os.path.exists(filepath):
        print(f"❌ File tidak ditemukan: {filepath}")
        return False

    ext = os.path.splitext(filepath)[1].lower()

    # Prioritas pemutar untuk WAV
    if ext == '.wav':
        if shutil.which('paplay'):
            res = subprocess.run(['paplay', filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if res.returncode == 0:
                return True
        if shutil.which('aplay'):
            res = subprocess.run(['aplay', '-q', filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if res.returncode == 0:
                return True

    # Jika MP3
    if shutil.which('mpg123'):
        res = subprocess.run(['mpg123', '-q', filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if res.returncode == 0:
            return True
    if shutil.which('ffplay'):
        res = subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if res.returncode == 0:
            return True
    if shutil.which('cvlc'):
        res = subprocess.run(['cvlc', '--play-and-exit', filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if res.returncode == 0:
            return True
    if shutil.which('paplay'):
        res = subprocess.run(['paplay', filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if res.returncode == 0:
            return True

    print("⚠️ Belum ada pemutar MP3 CLI terpasang!")
    print("   Silakan instal mpg123 yang ringan & instan:")
    print("   >>> sudo apt update && sudo apt install -y mpg123 <<<")
    return False


def get_tts_audio_path(text):
    """Ambil audio TTS MP3 (cek cache lokal dulu, jika tidak ada baru unduh)."""
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    cache_dir = get_cache_dir()
    cache_file = os.path.join(cache_dir, f"{text_hash}.mp3")

    if os.path.exists(cache_file) and os.path.getsize(cache_file) > 0:
        return cache_file

    # Unduh dari Google TTS
    try:
        url = 'https://translate.google.com/translate_tts?ie=UTF-8&tl=id&client=tw-ob&q=' + urllib.parse.quote(text)
        req = urllib.request.Request(
            url,
            headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = resp.read()
            with open(cache_file, 'wb') as f:
                f.write(data)
        return cache_file
    except Exception as e:
        print(f"⚠️ Gagal mengunduh Google TTS (mungkin offline): {e}")
        return None


def speak(text, play_chime_first=True):
    """Keluarkan suara robot: bunyikan chime lalu ucapkan kalimat."""
    print(f"\n📢 [ROBOT SPEAK]: \"{text}\"")

    if play_chime_first:
        chime_file = generate_chime_wav()
        play_audio_file(chime_file)
        time.sleep(0.2)

    # Cek apakah teks cocok dengan preset
    text_to_speak = text
    for key, phrase in PRESET_PHRASES.items():
        if text.strip().lower() == key:
            text_to_speak = phrase
            break

    audio_file = get_tts_audio_path(text_to_speak)
    if audio_file and os.path.exists(audio_file):
        play_audio_file(audio_file)
    else:
        print("⚠️ Suara TTS tidak dapat diputar. Bunyi Chime tetap berbunyi.")


# =============================================================================
# ROS 2 NODE INTEGRATION
# =============================================================================
def run_ros2_node():
    """Jalankan sebagai ROS 2 Node yang melayani perintah suara AMR."""
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
            self.sub_say = self.create_subscription(
                String,
                '/robot_voice/say',
                self.callback_say,
                10
            )
            self.sub_play = self.create_subscription(
                String,
                '/robot_voice/play',
                self.callback_play,
                10
            )
            self.get_logger().info("🔊 [robot_voice_node] Aktif!")
            self.get_logger().info("   - Subscribed to: /robot_voice/say (std_msgs/String)")
            self.get_logger().info("   - Subscribed to: /robot_voice/play (std_msgs/String) ['arrival', 'pickup', 'chime']")

        def callback_say(self, msg):
            text = msg.data.strip()
            if text:
                self.get_logger().info(f"Mengucapkan: '{text}'")
                speak(text, play_chime_first=True)

        def callback_play(self, msg):
            cmd = msg.data.strip().lower()
            if cmd == 'chime':
                play_audio_file(generate_chime_wav())
            elif cmd in PRESET_PHRASES:
                speak(PRESET_PHRASES[cmd], play_chime_first=True)
            else:
                speak(cmd, play_chime_first=True)

    node = RobotVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# =============================================================================
# MENU INTERAKTIF
# =============================================================================
def interactive_menu():
    """Menu interaktif pengujian di terminal Jetson."""
    while True:
        print("\n" + "=" * 55)
        print("  🔊 PENGUJIAN SUARA ROBOT (SPEAKER BLUETOOTH)")
        print("=" * 55)
        print("  1. 🔔 Bunyikan Chime / Ding-dong (WAV Offline)")
        print("  2. 🎯 Suara: 'Robot sampai di titik tujuan (ambil barang)'")
        print("  3. 📦 Suara: 'Robot sampai di titik pickup (muat barang)'")
        print("  4. 🚶 Suara: 'Permisi, robot mau lewat'")
        print("  5. 💬 Ucapkan Kalimat Custom (Ketik bebas)")
        print("  6. 📊 Periksa Status Bluetooth & Audio Sink")
        print("  7. 🤖 Jalankan sebagai ROS 2 Node Listener")
        print("  q. Keluar")
        print("=" * 55)

        try:
            choice = input("👉 Pilih menu (1-7 / q): ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nKeluar.")
            break

        if choice == '1':
            print("🔔 Membunyikan Chime...")
            chime = generate_chime_wav()
            play_audio_file(chime)
        elif choice == '2':
            speak(PRESET_PHRASES['arrival'])
        elif choice == '3':
            speak(PRESET_PHRASES['pickup'])
        elif choice == '4':
            speak(PRESET_PHRASES['obstacle'])
        elif choice == '5':
            try:
                custom_text = input("✍️ Masukkan kalimat yang ingin diucapkan robot: ").strip()
                if custom_text:
                    speak(custom_text)
            except (KeyboardInterrupt, EOFError):
                pass
        elif choice == '6':
            print_audio_status()
        elif choice == '7':
            print("🤖 Memulai ROS 2 Node...")
            run_ros2_node()
        elif choice in ('q', 'exit', 'quit'):
            print("Selesai.")
            break
        else:
            print("⚠️ Pilihan tidak valid.")


def main():
    parser = argparse.ArgumentParser(description="Tool & Node Pengujian Suara Robot AMR")
    parser.add_argument('--say', type=str, help="Ucapkan kalimat langsung")
    parser.add_argument('--arrival', action='store_true', help="Bunyikan suara tiba di tujuan")
    parser.add_argument('--pickup', action='store_true', help="Bunyikan suara tiba di pickup")
    parser.add_argument('--chime', action='store_true', help="Bunyikan ding-dong chime saja")
    parser.add_argument('--status', action='store_true', help="Cek status bluetooth audio sink")
    parser.add_argument('--node', action='store_true', help="Jalankan sebagai ROS 2 Voice Node")

    args = parser.parse_args()

    if args.status:
        print_audio_status()
    elif args.chime:
        play_audio_file(generate_chime_wav())
    elif args.arrival:
        speak(PRESET_PHRASES['arrival'])
    elif args.pickup:
        speak(PRESET_PHRASES['pickup'])
    elif args.say:
        speak(args.say)
    elif args.node:
        run_ros2_node()
    else:
        # Tampilkan status terlebih dahulu lalu masuk menu
        print_audio_status()
        interactive_menu()


if __name__ == '__main__':
    main()
