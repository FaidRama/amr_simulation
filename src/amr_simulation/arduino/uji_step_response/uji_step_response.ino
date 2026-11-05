/* =========================================================================
 * STEP RESPONSE OPEN-LOOP TEST DENGAN BASELINE (Ts = 20 ms / 50 Hz)
 * Disesuaikan khusus untuk identifikasi sistem dinamis motor DC PG36 + BTS7960
 * =========================================================================
 * Catatan Penting Identifikasi Sistem:
 *   - RAW DATA ENCODER: Filter EMA DIHAPUS agar tidak menimbulkan fasa lag /
 *     keterlambatan respon semu (time constant delay) pada grafik identifikasi.
 *   - Nilai keluaran serial adalah nilai mentah: raw_rpm_L dan raw_rpm_R.
 *   - Skenario Pengujian:
 *       * 0.0s - 1.0s : Diam (PWM 0) -> Baseline nol
 *       * 1.0s - 3.0s : Baseline rolling konstan (PWM Base, default 40)
 *       * 3.0s - 6.0s : Step Jump target (PWM Step: 100, 120, 140, 160, 180, dll)
 *       * > 6.0s      : Stop Total (PWM 0) & Kembali ke STANDBY aman
 *   - Fleksibilitas Input Pengguna:
 *       * '1' atau Enter : Mulai dengan parameter saat ini (Default: 40 -> 140)
 *       * '2' : Step 40 -> 100 PWM (+60 PWM)
 *       * '3' : Step 40 -> 120 PWM (+80 PWM)
 *       * '4' : Step 40 -> 140 PWM (+100 PWM - Standar)
 *       * '5' : Step 40 -> 160 PWM (+120 PWM)
 *       * '6' : Step 40 -> 180 PWM (+140 PWM)
 *       * Custom Input: Kirim "base,step" misal "40,100" atau "40,160"
 *       * '0' atau 's' : STOP DARURAT KAPAN SAJA
 *
 * Pin Hardware AMR:
 *   - Motor Kiri : RPWM = 5, LPWM = 6, REN = 7, LEN = 8
 *   - Motor Kanan: RPWM = 9, LPWM = 10, REN = 11, LEN = 12 (Inverted fisik)
 *   - Enkoder L  : A = 2 (INT0), B = 4
 *   - Enkoder R  : A = 3 (INT1), B = A1 (Digital In)
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
const byte PIN_ENCB_R = A1;   // Pin Analog A1 sebagai input digital

// --- 3. SPESIFIKASI MOTOR ---
const float TICKS_PER_REV = 350.7; // PG36 rasio 50.1:1, 7 PPR

// --- 4. SAMPLING RATE TEPAT (50 Hz / Ts = 20 ms) ---
const unsigned long TS_US = 20000; // 20.000 mikrodetik = 20 ms

// State Pengujian
enum TestState {
  STATE_STANDBY,
  STATE_RUNNING
};

TestState currentState = STATE_STANDBY;

// Parameter Uji Step Dinamis (Dapat diubah via Serial / User Input)
int base_pwm = 40;   // Nilai rolling baseline (melewati deadzone)
int step_pwm = 140;  // Nilai step jump target

volatile long count_ticks_L = 0;
volatile long count_ticks_R = 0;

unsigned long prev_time_us        = 0;
unsigned long start_experiment_ms = 0;
unsigned long prev_standby_msg_ms = 0;

// Buffer input perintah Serial
char cmd_buffer[32];
byte cmd_index = 0;

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

// Fungsi Mulai Uji Step Response
void startStepTest(int new_base, int new_step) {
  base_pwm = constrain(new_base, 0, 255);
  step_pwm = constrain(new_step, 0, 255);

  // Reset encoder ticks secara atomik
  noInterrupts();
  count_ticks_L = 0;
  count_ticks_R = 0;
  interrupts();

  start_experiment_ms = millis();
  prev_time_us = micros();
  currentState = STATE_RUNNING;

  Serial.println(F("# ====================================================="));
  Serial.println(F("# MULAI PENGUJIAN STEP RESPONSE OPEN-LOOP (RAW DATA)"));
  Serial.print(F("# Parameter: Base PWM = "));
  Serial.print(base_pwm);
  Serial.print(F(" (t=1-3s) -> Step PWM = "));
  Serial.print(step_pwm);
  Serial.print(F(" (t=3-6s) | Delta U = "));
  Serial.println(step_pwm - base_pwm);
  Serial.println(F("# Sinyal Kecepatan: MURNI / RAW (Tanpa Filter EMA)"));
  Serial.println(F("# Format: t_sec,pwm_input,raw_rpm_L,raw_rpm_R,ticks_L,ticks_R"));
  Serial.println(F("# ====================================================="));
}

// Menampilkan panduan Serial
void printHelpMenu() {
  Serial.println(F("# ====================================================="));
  Serial.println(F("# UJI STEP RESPONSE MOTOR DC PG36 (Ts = 20ms / 50 Hz)"));
  Serial.print(F("# STATUS: STANDBY | Setting: Base = "));
  Serial.print(base_pwm);
  Serial.print(F(", Step = "));
  Serial.println(step_pwm);
  Serial.println(F("# Menu Pilihan:"));
  Serial.println(F("#   [1] atau [ENTER] : Mulai setting saat ini"));
  Serial.println(F("#   [2] : Step 40 -> 100 PWM (Delta = +60)"));
  Serial.println(F("#   [3] : Step 40 -> 120 PWM (Delta = +80)"));
  Serial.println(F("#   [4] : Step 40 -> 140 PWM (Delta = +100 - Standar)"));
  Serial.println(F("#   [5] : Step 40 -> 160 PWM (Delta = +120)"));
  Serial.println(F("#   [6] : Step 40 -> 180 PWM (Delta = +140)"));
  Serial.println(F("#   Custom: Ketik 'base,step' contoh: 40,120 atau 40 180"));
  Serial.println(F("#   [0] atau 's'     : STOP DARURAT seketika"));
  Serial.println(F("# ====================================================="));
}

// Parser perintah Serial
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // STOP Darurat seketika hanya jika motor sedang berputar (STATE_RUNNING)
    if (currentState == STATE_RUNNING && (c == '0' || c == 's' || c == 'S' || c == ' ')) {
      setMotorPWM(0, 0);
      currentState = STATE_STANDBY;
      cmd_index = 0;
      Serial.println(F("# [STOP] Pengujian dihentikan secara manual. Motor diam aman."));
      return;
    }

    if (c == '\n' || c == '\r') {
      if (cmd_index == 0) {
        // Tombol Enter ditekan langsung saat STANDBY -> mulai dengan setting saat ini
        if (currentState != STATE_RUNNING) {
          startStepTest(base_pwm, step_pwm);
        }
        return;
      }

      cmd_buffer[cmd_index] = '\0';

      // Trim leading spaces
      char* p = cmd_buffer;
      while (*p == ' ' || *p == '\t') p++;

      // Cek apakah perintah adalah STOP saat STANDBY
      if (strcmp(p, "0") == 0 || strcasecmp(p, "s") == 0 || strcasecmp(p, "stop") == 0) {
        setMotorPWM(0, 0);
        currentState = STATE_STANDBY;
        Serial.println(F("# [STOP] Motor diam aman."));
      } else if (strcmp(p, "1") == 0 || strcasecmp(p, "g") == 0 || strcasecmp(p, "start") == 0) {
        startStepTest(base_pwm, step_pwm);
      } else if (strcmp(p, "2") == 0) {
        startStepTest(40, 100);
      } else if (strcmp(p, "3") == 0) {
        startStepTest(40, 120);
      } else if (strcmp(p, "4") == 0) {
        startStepTest(40, 140);
      } else if (strcmp(p, "5") == 0) {
        startStepTest(40, 160);
      } else if (strcmp(p, "6") == 0) {
        startStepTest(40, 180);
      } else if (strcasecmp(p, "help") == 0 || strcmp(p, "?") == 0) {
        printHelpMenu();
      } else {
        // Coba parsing angka custom "base,step" atau "base step" atau "step"
        while (*p && !isdigit(*p) && *p != '-') p++;
        if (*p) {
          int val1 = atoi(p);
          while (*p && (isdigit(*p) || *p == '-')) p++;
          while (*p && (*p == ',' || *p == ' ' || *p == ';' || *p == '\t')) p++;

          if (*p && isdigit(*p)) {
            int val2 = atoi(p);
            startStepTest(val1, val2);
          } else {
            // Hanya 1 angka diberikan: jika >= 50 maka jadikan step_pwm dengan base 40
            if (val1 >= 50) {
              startStepTest(40, val1);
            } else {
              startStepTest(base_pwm, val1);
            }
          }
        } else {
          Serial.print(F("# [ERROR] Format tidak dikenali: "));
          Serial.println(cmd_buffer);
        }
      }

      cmd_index = 0;
    } else {
      if (cmd_index < sizeof(cmd_buffer) - 1) {
        cmd_buffer[cmd_index++] = c;
      }
    }
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

  // Pasang Interupsi Enkoder
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_L), isrEncoderL, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_R), isrEncoderR, FALLING);

  // PASTIKAN MOTOR DIAM MUTLAK DI AWAL
  setMotorPWM(0, 0);
  currentState = STATE_STANDBY;

  printHelpMenu();
}

void loop() {
  unsigned long now_ms = millis();

  // --- 0. BACA PERINTAH SERIAL / USER INPUT ---
  handleSerialInput();

  // --- STATE STANDBY ---
  if (currentState == STATE_STANDBY) {
    setMotorPWM(0, 0);
    if (now_ms - prev_standby_msg_ms >= 4000) {
      prev_standby_msg_ms = now_ms;
      Serial.print(F("# [STANDBY] Base="));
      Serial.print(base_pwm);
      Serial.print(F(", Step="));
      Serial.print(step_pwm);
      Serial.println(F(". Tekan [ENTER] atau ketik '40,120' untuk MULAI..."));
    }
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
      current_pwm = 0;        // 0-1s: Diam (Baseline nol)
    } else if (t_sec < 3.0) {
      current_pwm = base_pwm; // 1-3s: Baseline konstan melewati deadzone
    } else if (t_sec < 6.0) {
      current_pwm = step_pwm; // 3-6s: Step Jump target dinamis
    } else {
      // > 6.0s: Stop & Selesai
      setMotorPWM(0, 0);
      currentState = STATE_STANDBY; // Kembali ke Standby agar siap untuk pengujian step berikutnya
      Serial.println(F("# ====================================================="));
      Serial.println(F("# PENGUJIAN SELESAI: Data 6 detik lengkap terekam."));
      Serial.println(F("# Motor diam aman. Siap untuk pengujian berikutnya."));
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

    // Hitung RPM murni (RAW) pada Ts = 20 ms (0.02 detik)
    // RPM = (|ticks| / 350.7) * (60.0 / 0.02) = |ticks| * 8.554319
    // PENTING: Tanpa filter EMA agar parameter fungsi alih G(s) akurat tanpa distorsi fasa.
    float raw_rpm_L = ((float)abs(ticks_L) / TICKS_PER_REV) * 3000.0;
    float raw_rpm_R = ((float)abs(ticks_R) / TICKS_PER_REV) * 3000.0;

    // Output Telemetri CSV Tepat 50 Hz
    Serial.print(t_sec, 3);
    Serial.print(",");
    Serial.print(current_pwm);
    Serial.print(",");
    Serial.print(raw_rpm_L, 2);
    Serial.print(",");
    Serial.print(raw_rpm_R, 2);
    Serial.print(",");
    Serial.print(ticks_L);
    Serial.print(",");
    Serial.println(ticks_R);
  }
}
