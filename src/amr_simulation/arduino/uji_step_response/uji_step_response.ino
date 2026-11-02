/* =========================================================================
 * STEP RESPONSE OPEN-LOOP TEST DENGAN BASELINE (Ts = 20 ms / 50 Hz)
 * Disesuaikan khusus untuk hardware AMR (Arduino Uno, BTS7960, PG36)
 * =========================================================================
 * Skenario Pengujian Sistem Dinamis Motor (Sesuai Teori Identifikasi Sistem):
 *   - 0.0s - 1.0s : Diam (PWM 0)
 *   - 1.0s - 3.0s : Baseline rolling konstan (PWM 40) -> Melewati deadzone
 *   - 3.0s - 6.0s : Step Jump (PWM 140) -> Area identifikasi dinamis (+100 PWM)
 *   - > 6.0s      : Stop Total (PWM 0) & Pengujian Selesai
 *
 * Pin Hardware AMR:
 *   - Motor Kiri : RPWM = 5, LPWM = 6, REN = 7, LEN = 8
 *   - Motor Kanan: RPWM = 9, LPWM = 10, REN = 11, LEN = 12 (Inverted fisik)
 *   - Enkoder L  : A = 2 (INT0), B = 4
 *   - Enkoder R  : A = 3 (INT1), B = A1 (Digital In)
 *
 * Fitur Keamanan:
 *   - SAAT BARU UPLOAD/MENYALA: MOTOR 100% DIAM (STANDBY).
 *   - HANYA MULAI jika menerima sinyal '1' / Enter dari ROS 2 / Serial.
 *   - Ketik '0' di Serial kapan saja untuk STOP DARURAT seketika.
 * ========================================================================= */

#include <Arduino.h>

// --- 1. PIN MOTOR DRIVER (BTS7960) ---
const byte PIN_RPWM_L = 5;
const byte PIN_LPWM_L = 6;
const byte PIN_REN_L  = 7;
const byte PIN_LEN_L  = 8;

const byte PIN_RPWM_R = 9;
const byte PIN_LPWM_R = 10;
const byte PIN_REN_R  = 11;
const byte PIN_LEN_R  = 12;

// --- 2. PIN ENCODER PG36 ---
const byte PIN_ENC_A_L = 2;   // INT0
const byte PIN_ENC_B_L = 4;
const byte PIN_ENC_A_R = 3;   // INT1
const byte PIN_ENCB_R = A1;  // Pin Analog A1

// --- 3. SPESIFIKASI MOTOR ---
const float TICKS_PER_REV = 350.7; // PG36 rasio 50.1:1, 7 PPR

// --- 4. SAMPLING RATE TEPAT (50 Hz / Ts = 20 ms) ---
const unsigned long TS_US = 20000; // 20.000 mikrodetik = 20 ms

// State Pengujian
enum TestState {
  STATE_STANDBY,
  STATE_RUNNING,
  STATE_FINISHED
};

TestState currentState = STATE_STANDBY;

volatile long count_ticks_L = 0;
volatile long count_ticks_R = 0;

unsigned long prev_time_us       = 0;
unsigned long start_experiment_ms = 0;
unsigned long prev_standby_msg_ms = 0;

// Filter EMA untuk menghaluskan pembacaan RPM diskrit
const float ALPHA = 0.35;
float filtered_rpm_L = 0.0;
float filtered_rpm_R = 0.0;

// --- INTERRUPT SERVICE ROUTINES (ISR) ---
void isrEncoderL() {
  if (digitalRead(PIN_ENC_B_L) == HIGH) {
    count_ticks_L++;
  } else {
    count_ticks_L--;
  }
}

void isrEncoderR() {
  // Motor kanan terbalik secara fisik di sasis
  if (digitalRead(PIN_ENCB_R) == HIGH) {
    count_ticks_R--;
  } else {
    count_ticks_R++;
  }
}

// --- PENGGERAK MOTOR SESUAI ARAH FISIK AMR ---
void setMotorPWM(int pwmL, int pwmR) {
  pwmL = constrain(pwmL, 0, 255);
  pwmR = constrain(pwmR, 0, 255);

  // Motor Kiri Maju: RPWM aktif, LPWM nol
  if (pwmL == 0) {
    analogWrite(PIN_RPWM_L, 0);
    analogWrite(PIN_LPWM_L, 0);
  } else {
    analogWrite(PIN_RPWM_L, pwmL);
    analogWrite(PIN_LPWM_L, 0);
  }

  // Motor Kanan Maju (Inverted fisik): RPWM nol, LPWM aktif
  if (pwmR == 0) {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, 0);
  } else {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, pwmR);
  }
}

void setup() {
  Serial.begin(115200);

  // Inisialisasi Pin Driver
  pinMode(PIN_RPWM_L, OUTPUT);
  pinMode(PIN_LPWM_L, OUTPUT);
  pinMode(PIN_REN_L,  OUTPUT);
  pinMode(PIN_LEN_L,  OUTPUT);

  pinMode(PIN_RPWM_R, OUTPUT);
  pinMode(PIN_LPWM_R, OUTPUT);
  pinMode(PIN_REN_R,  OUTPUT);
  pinMode(PIN_LEN_R,  OUTPUT);

  // Aktifkan Driver BTS7960
  digitalWrite(PIN_REN_L, HIGH);
  digitalWrite(PIN_LEN_L, HIGH);
  digitalWrite(PIN_REN_R, HIGH);
  digitalWrite(PIN_LEN_R, HIGH);

  // Inisialisasi Pin Enkoder
  pinMode(PIN_ENC_A_L, INPUT_PULLUP);
  pinMode(PIN_ENC_B_L, INPUT_PULLUP);
  pinMode(PIN_ENC_A_R, INPUT_PULLUP);
  pinMode(PIN_ENCB_R, INPUT_PULLUP);

  // Pasang Interupsi
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_L), isrEncoderL, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_R), isrEncoderR, FALLING);

  // PASTIKAN MOTOR DIAM MUTLAK DI AWAL
  setMotorPWM(0, 0);
  currentState = STATE_STANDBY;

  Serial.println(F("# ====================================================="));
  Serial.println(F("# SYSTEM READY: ARDUINO STEP RESPONSE TEST (Ts = 20ms)"));
  Serial.println(F("# STATUS: STANDBY (MOTOR DIAM AMAN)"));
  Serial.println(F("# Skenario: 0-1s (PWM 0) -> 1-3s (PWM 40) -> 3-6s (PWM 140) -> Stop"));
  Serial.println(F("# Buka alat pemantauan ROS 2 terlebih dahulu."));
  Serial.println(F("# Kirim '1' atau tekan [ENTER] jika sudah siap MULAI."));
  Serial.println(F("# ====================================================="));
}

void loop() {
  unsigned long now_ms = millis();

  // --- 0. BACA PERINTAH DARI SERIAL / ROS 2 ---
  if (Serial.available() > 0) {
    char c = Serial.read();

    // STOP Darurat Kapan Saja
    if (c == '0' || c == 's' || c == 'S' || c == ' ') {
      setMotorPWM(0, 0);
      currentState = STATE_STANDBY;
      Serial.println(F("# [STOP] Pengujian dihentikan secara manual."));
      return;
    }

    // MULAI Pengujian ('1' atau 'g' atau Enter)
    if (c == '1' || c == 'g' || c == 'G' || c == '\n') {
      if (currentState != STATE_RUNNING) {
        currentState = STATE_RUNNING;
        noInterrupts();
        count_ticks_L = 0;
        count_ticks_R = 0;
        interrupts();

        filtered_rpm_L = 0.0;
        filtered_rpm_R = 0.0;
        start_experiment_ms = millis();
        prev_time_us = micros();

        Serial.println(F("# ====================================================="));
        Serial.println(F("# MULAI PENGUJIAN STEP RESPONSE OPEN-LOOP"));
        Serial.println(F("# Format: t_sec,pwm_input,rpm_L,rpm_R,ticks_L,ticks_R"));
        Serial.println(F("# ====================================================="));
        return;
      }
    }
  }

  // --- STATE STANDBY ---
  if (currentState == STATE_STANDBY) {
    setMotorPWM(0, 0);
    if (now_ms - prev_standby_msg_ms >= 3000) {
      prev_standby_msg_ms = now_ms;
      Serial.println(F("# [STANDBY] Motor diam. Kirim '1' untuk MULAI Step Response..."));
    }
    return;
  }

  // --- STATE FINISHED ---
  if (currentState == STATE_FINISHED) {
    setMotorPWM(0, 0);
    return;
  }

  // --- STATE RUNNING (50 Hz / Ts = 20.000 us) ---
  unsigned long now_us = micros();
  if (now_us - prev_time_us >= TS_US) {
    prev_time_us += TS_US;

    float t_sec = (millis() - start_experiment_ms) / 1000.0;
    int current_pwm = 0;

    // Profil Sinyal Uji Tangga (Step Test dengan Baseline)
    if (t_sec < 1.0) {
      current_pwm = 0;     // 0-1s: Diam
    } else if (t_sec < 3.0) {
      current_pwm = 40;    // 1-3s: Baseline konstan di atas deadzone
    } else if (t_sec < 6.0) {
      current_pwm = 140;   // 3-6s: Step Jump +100 unit PWM (Identifikasi Dinamis)
    } else {
      // > 6.0s: Stop Selesai
      setMotorPWM(0, 0);
      currentState = STATE_FINISHED;
      Serial.println(F("# ====================================================="));
      Serial.println(F("# PENGUJIAN SELESAI: Data 6 detik lengkap terekam."));
      Serial.println(F("# ====================================================="));
      return;
    }

    setMotorPWM(current_pwm, current_pwm);

    // Ambil nilai ticks secara atomik
    noInterrupts();
    long ticks_L = count_ticks_L;
    long ticks_R = count_ticks_R;
    count_ticks_L = 0;
    count_ticks_R = 0;
    interrupts();

    // Hitung RPM pada Ts = 20 ms (0.02 detik)
    // RPM = (|ticks| / 350.7) * (60.0 / 0.02) = |ticks| * 8.5543
    float raw_rpm_L = ((float)abs(ticks_L) / TICKS_PER_REV) * 3000.0;
    float raw_rpm_R = ((float)abs(ticks_R) / TICKS_PER_REV) * 3000.0;

    // Filter EMA untuk grafik halus identifikasi
    if (t_sec < 1.0) {
      filtered_rpm_L = 0.0;
      filtered_rpm_R = 0.0;
    } else {
      filtered_rpm_L = (ALPHA * raw_rpm_L) + ((1.0 - ALPHA) * filtered_rpm_L);
      filtered_rpm_R = (ALPHA * raw_rpm_R) + ((1.0 - ALPHA) * filtered_rpm_R);
    }

    // Output Telemetri CSV Tepat 50 Hz
    Serial.print(t_sec, 3);
    Serial.print(",");
    Serial.print(current_pwm);
    Serial.print(",");
    Serial.print(filtered_rpm_L, 2);
    Serial.print(",");
    Serial.print(filtered_rpm_R, 2);
    Serial.print(",");
    Serial.print(ticks_L);
    Serial.print(",");
    Serial.println(ticks_R);
  }
}
