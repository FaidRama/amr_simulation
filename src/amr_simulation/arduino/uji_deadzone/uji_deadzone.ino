/* =========================================================================
 * PENGUJIAN DEADZONE MOTOR DC PG36 + BTS7960 DENGAN BEBAN ROBOT
 * Disesuaikan khusus untuk hardware AMR (Arduino Uno, BTS7960, PG36)
 * =========================================================================
 * Fitur Keamanan:
 *   - SAAT BARU MENYALA/UPLOAD: MOTOR 100% DIAM (STANDBY).
 *   - Motor HANYA AKAN JALAN jika menerima perintah '1' atau 'g' atau Enter.
 *   - Kapan pun ketik '0' atau [SPASI] di Serial, motor LANGSUNG STOP DARURAT.
 *   - Otomatis berhenti jika kedua roda sudah bergerak lancar.
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

// --- 4. STATE PENGUJIAN ---
enum TestState {
  STATE_STANDBY,   // Diam, menunggu perintah mulai dari user
  STATE_RUNNING,   // Sedang melakukan ramp test PWM
  STATE_FINISHED   // Pengujian selesai, motor dimatikan
};

TestState currentState = STATE_STANDBY;

// Variabel Waktu & Kontrol
volatile long count_ticks_L = 0;
volatile long count_ticks_R = 0;

unsigned long prev_time_sample  = 0;
unsigned long prev_time_ramp    = 0;
unsigned long prev_time_standby = 0;

const unsigned long TS_MS      = 50;   // Interval sampling kecepatan (50 ms / 20 Hz)
const unsigned long T_RAMP_MS  = 250;  // Naikkan 1 PWM tiap 250 ms (halus & stabil)

int current_pwm = 0;
const int MAX_TEST_PWM = 120; // Batas aman pengujian gesek

int deadzone_pwm_L = 0;
int deadzone_pwm_R = 0;

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

  // Motor Kanan Maju (Inverted): RPWM nol, LPWM aktif
  if (pwmR == 0) {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, 0);
  } else {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, pwmR);
  }
}

void cetakHasilDeadzone() {
  Serial.println(F("# ====================================================="));
  Serial.println(F("# RANGKUMAN HASIL DEADZONE (MINIMUM PWM UNTUK BERGERAK):"));
  Serial.print(F("# -> Motor Kiri  : PWM = "));
  Serial.println(deadzone_pwm_L > 0 ? String(deadzone_pwm_L) : F("Belum bergerak"));
  Serial.print(F("# -> Motor Kanan : PWM = "));
  Serial.println(deadzone_pwm_R > 0 ? String(deadzone_pwm_R) : F("Belum bergerak"));
  Serial.println(F("# ====================================================="));
  Serial.println(F("# Ketik '1' untuk uji ulang, atau '0' untuk reset."));
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
  Serial.println(F("# SYSTEM READY: ARDUINO UJI DEADZONE MOTOR PG36"));
  Serial.println(F("# STATUS: STANDBY (MOTOR DIAM AMAN)"));
  Serial.println(F("# Buka alat pemantauan Anda terlebih dahulu (rqt_plot / ROS 2)."));
  Serial.println(F("# Kirim '1' atau tekan [ENTER] jika sudah siap MULAI."));
  Serial.println(F("# ====================================================="));
}

void loop() {
  unsigned long now = millis();

  // --- 0. BACA PERINTAH DARI SERIAL ---
  if (Serial.available() > 0) {
    char c = Serial.read();

    // Perintah STOP Darurat Kapan Saja
    if (c == '0' || c == 's' || c == 'S' || c == ' ') {
      setMotorPWM(0, 0);
      currentState = STATE_STANDBY;
      current_pwm = 0;
      Serial.println(F("# [STOP] Pengujian dihentikan / Reset ke Standby."));
      return;
    }

    // Perintah MULAI Pengujian ('1' atau 'g' atau 'G')
    if (c == '1' || c == 'g' || c == 'G' || c == '\n') {
      if (currentState != STATE_RUNNING) {
        currentState = STATE_RUNNING;
        current_pwm = 0;
        deadzone_pwm_L = 0;
        deadzone_pwm_R = 0;
        noInterrupts();
        count_ticks_L = 0;
        count_ticks_R = 0;
        interrupts();

        prev_time_ramp   = now;
        prev_time_sample = now;

        Serial.println(F("# ====================================================="));
        Serial.println(F("# MULAI PENGUJIAN: PWM RAMPING DARI 0"));
        Serial.println(F("# Format: Waktu_ms,PWM,Ticks_L,Ticks_R,RPM_L,RPM_R"));
        Serial.println(F("# Ketik '0' di Serial untuk STOP DARURAT kapan saja."));
        Serial.println(F("# ====================================================="));
        return;
      }
    }
  }

  // --- STATE 1: STANDBY (MENUNGGU PERINTAH MULAI) ---
  if (currentState == STATE_STANDBY) {
    setMotorPWM(0, 0);
    // Cetak status heartbeat tiap 3 detik agar user tahu koneksi serial aktif
    if (now - prev_time_standby >= 3000) {
      prev_time_standby = now;
      Serial.println(F("# [STANDBY] Motor diam. Kirim '1' untuk MULAI uji deadzone..."));
    }
    return;
  }

  // --- STATE 2: FINISHED (SELESAI) ---
  if (currentState == STATE_FINISHED) {
    setMotorPWM(0, 0);
    return;
  }

  // --- STATE 3: RUNNING (SEDANG PENGUJIAN) ---
  // 1. Naikkan PWM secara perlahan (+1 tiap 250ms)
  if (now - prev_time_ramp >= T_RAMP_MS) {
    prev_time_ramp = now;

    if (current_pwm <= MAX_TEST_PWM) {
      setMotorPWM(current_pwm, current_pwm);
      current_pwm++;
    } else {
      currentState = STATE_FINISHED;
      setMotorPWM(0, 0);
      Serial.println(F("# PENGUJIAN SELESAI: Mencapai batas PWM maksimal."));
      cetakHasilDeadzone();
      return;
    }
  }

  // 2. Sampling Kecepatan & Cetak Telemetri (20 Hz)
  if (now - prev_time_sample >= TS_MS) {
    unsigned long dt = now - prev_time_sample;
    prev_time_sample = now;

    noInterrupts();
    long ticks_L = count_ticks_L;
    long ticks_R = count_ticks_R;
    count_ticks_L = 0;
    count_ticks_R = 0;
    interrupts();

    float dt_sec = dt / 1000.0;
    float rpm_L = ((float)abs(ticks_L) / TICKS_PER_REV) * (60.0 / dt_sec);
    float rpm_R = ((float)abs(ticks_R) / TICKS_PER_REV) * (60.0 / dt_sec);

    // Rekam PWM saat pertama kali roda pecah gesekan (Breakaway: RPM > 1.5)
    if (deadzone_pwm_L == 0 && rpm_L > 1.5) deadzone_pwm_L = current_pwm;
    if (deadzone_pwm_R == 0 && rpm_R > 1.5) deadzone_pwm_R = current_pwm;

    // Auto-Stop: Jika kedua roda sudah bergerak lancar, berhenti otomatis agar aman
    if (deadzone_pwm_L > 0 && deadzone_pwm_R > 0 && current_pwm > (max(deadzone_pwm_L, deadzone_pwm_R) + 20)) {
      currentState = STATE_FINISHED;
      setMotorPWM(0, 0);
      Serial.println(F("# PENGUJIAN SELESAI: Kedua roda telah sukses bergerak!"));
      cetakHasilDeadzone();
      return;
    }

    // Output Telemetri CSV (Bisa dibaca rqt_plot / Serial Plotter)
    Serial.print(now);
    Serial.print(",");
    Serial.print(current_pwm);
    Serial.print(",");
    Serial.print(ticks_L);
    Serial.print(",");
    Serial.print(ticks_R);
    Serial.print(",");
    Serial.print(rpm_L, 2);
    Serial.print(",");
    Serial.println(rpm_R, 2);
  }
}
