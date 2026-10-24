import serial
import time

SERIAL_PORT = '/dev/arduino'
BAUD_RATE = 115200
OUTPUT_FILE = 'step_respon_60rpm.csv'

print(f"Menghubungkan ke {SERIAL_PORT}...")
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Jeda inisialisasi serial
    ser.reset_input_buffer()
except Exception as e:
    print(f"Error: {e}")
    exit(1)

print(f"Merekam ke {OUTPUT_FILE}. Tekan Ctrl+C jika ingin berhenti manual.\n")

with open(OUTPUT_FILE, mode='w', encoding='utf-8') as f:
    f.write("Time_s,Target_RPM,RPM_L,RPM_R,PWM_L,PWM_R\n")
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    f.write(line + "\n")
                    f.flush()
                    print(line)
    except KeyboardInterrupt:
        print("\nPerekaman dihentikan pengguna.")
    finally:
        ser.close()
        print(f"File tersimpan di {OUTPUT_FILE}")
