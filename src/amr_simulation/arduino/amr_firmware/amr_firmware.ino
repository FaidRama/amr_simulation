/*
 * =====================================================================
 * FIRMWARE UTAMA AMR ARDUINO UNO - BTS7960 + PG36 ENCODER
 * Terintegrasi Penuh dengan ROS 2 Bridge (Jetson AGX Xavier)
 * =====================================================================
 * Protokol Serial (115200 baud):
 *   Input  (Jetson -> Arduino):
 *     - "<PWM_KIRI>,<PWM_KANAN>\n"  (Contoh: "120,120\n" atau "-100,100\n")
 *     - "RST\n"                     (Reset hitungan pulsa encoder ke 0)
 *   Output (Arduino -> Jetson):
 *     - "ENC:<PULSA_KIRI>,<PULSA_KANAN>\n" (Dikirim rutin setiap 50ms / 20Hz)
 * =====================================================================
 */

#include <Arduino.h>

// --- PIN DRIVER MOTOR KIRI (BTS7960) ---
const int PIN_L_RPWM = 5;
const int PIN_L_LPWM = 6;
const int PIN_L_REN  = 7;
const int PIN_L_LEN  = 8;

// --- PIN DRIVER MOTOR KANAN (BTS7960) ---
const int PIN_R_RPWM = 9;
const int PIN_R_LPWM = 10;
const int PIN_R_REN  = 11;
const int PIN_R_LEN  = 12;

// --- PIN ENCODER MOTOR PG36 ---
const int PIN_ENC_L_A = 2;   // Hardware Interrupt 0 (INT0)
const int PIN_ENC_L_B = 4;
const int PIN_ENC_R_A = 3;   // Hardware Interrupt 1 (INT1)
const int PIN_ENC_R_B = 13;  // Sesuai referensi (ubah ke A0 jika kabel fisik di A0)

// --- OPSI PEMBALIK ARAH (Ubah ke true jika arah roda/encoder terbalik) ---
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = false;
const bool INVERT_ENC_L   = false;
const bool INVERT_ENC_R   = false;

// --- VARIABEL ENCODER ---
volatile long enc_left_ticks  = 0;
volatile long enc_right_ticks = 0;

// --- BUFFER KOMUNIKASI NON-BLOCKING ---
char serial_buf[32];
uint8_t buf_idx = 0;

// --- SAFETY & TIMING ---
unsigned long last_cmd_time = 0;
const unsigned long TIMEOUT_MS = 500;        // Fail-safe: Stop motor jika >0.5s tanpa perintah

unsigned long last_report_time = 0;
const unsigned long REPORT_INTERVAL_MS = 50; // Kirim odometry setiap 50ms (20 Hz)

// =====================================================================
// INTERRUPT SERVICE ROUTINES (ISR) - Membaca Putaran Roda
// =====================================================================
void isr_encoder_left() {
  bool b_state = digitalRead(PIN_ENC_L_B);
  if (INVERT_ENC_L) b_state = !b_state;

  if (b_state == HIGH) {
    enc_left_ticks++;
  } else {
    enc_left_ticks--;
  }
}

void isr_encoder_right() {
  bool b_state = digitalRead(PIN_ENC_R_B);
  if (INVERT_ENC_R) b_state = !b_state;

  // Motor kanan dipasang berlawanan sisi dengan motor kiri
  if (b_state == HIGH) {
    enc_right_ticks--;
  } else {
    enc_right_ticks++;
  }
}

// =====================================================================
// FUNGSI PENGGERAK MOTOR (BTS7960)
// =====================================================================
void setMotors(int pwm_left, int pwm_right) {
  if (INVERT_MOTOR_L) pwm_left  = -pwm_left;
  if (INVERT_MOTOR_R) pwm_right = -pwm_right;

  pwm_left  = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  // --- KONTROL MOTOR KIRI ---
  if (pwm_left > 0) {
    analogWrite(PIN_L_RPWM, pwm_left);
    analogWrite(PIN_L_LPWM, 0);
  } else if (pwm_left < 0) {
    analogWrite(PIN_L_RPWM, 0);
    analogWrite(PIN_L_LPWM, -pwm_left);
  } else {
    analogWrite(PIN_L_RPWM, 0);
    analogWrite(PIN_L_LPWM, 0);
  }

  // --- KONTROL MOTOR KANAN ---
  if (pwm_right > 0) {
    analogWrite(PIN_R_RPWM, pwm_right);
    analogWrite(PIN_R_LPWM, 0);
  } else if (pwm_right < 0) {
    analogWrite(PIN_R_RPWM, 0);
    analogWrite(PIN_R_LPWM, -pwm_right);
  } else {
    analogWrite(PIN_R_RPWM, 0);
    analogWrite(PIN_R_LPWM, 0);
  }
}

// =====================================================================
// PARSER PERINTAH SERIAL (NON-BLOCKING, 0ms DELAY)
// =====================================================================
void processCommand(char* cmd) {
  // Perintah Reset Encoder dari ROS 2
  if (strncmp(cmd, "RST", 3) == 0) {
    noInterrupts();
    enc_left_ticks = 0;
    enc_right_ticks = 0;
    interrupts();
    setMotors(0, 0);
    return;
  }

  // Perintah PWM: "<PWM_L>,<PWM_R>"
  int pwm_l = 0;
  int pwm_r = 0;
  if (sscanf(cmd, "%d,%d", &pwm_l, &pwm_r) == 2) {
    setMotors(pwm_l, pwm_r);
    last_cmd_time = millis();
  }
}

// =====================================================================
// SETUP SISTEM
// =====================================================================
void setup() {
  Serial.begin(115200);

  // Inisialisasi Pin Driver Motor Kiri
  pinMode(PIN_L_RPWM, OUTPUT);
  pinMode(PIN_L_LPWM, OUTPUT);
  pinMode(PIN_L_REN,  OUTPUT);
  pinMode(PIN_L_LEN,  OUTPUT);
  digitalWrite(PIN_L_REN, HIGH);
  digitalWrite(PIN_L_LEN, HIGH);

  // Inisialisasi Pin Driver Motor Kanan
  pinMode(PIN_R_RPWM, OUTPUT);
  pinMode(PIN_R_LPWM, OUTPUT);
  pinMode(PIN_R_REN,  OUTPUT);
  pinMode(PIN_R_LEN,  OUTPUT);
  digitalWrite(PIN_R_REN, HIGH);
  digitalWrite(PIN_R_LEN, HIGH);

  // Pastikan motor berhenti saat baru nyala
  setMotors(0, 0);

  // Inisialisasi Pin Encoder
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Pasang Hardware Interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_encoder_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_encoder_right, FALLING);

  last_cmd_time = millis();
  last_report_time = millis();
}

// =====================================================================
// LOOP UTAMA
// =====================================================================
void loop() {
  unsigned long now = millis();

  // --- 1. BACA SERIAL SECARA NON-BLOCKING ---
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf_idx > 0) {
        serial_buf[buf_idx] = '\0';
        processCommand(serial_buf);
        buf_idx = 0;
      }
    } else if (buf_idx < sizeof(serial_buf) - 1) {
      serial_buf[buf_idx++] = c;
    }
  }

  // --- 2. SAFETY WATCHDOG TIMEOUT ---
  if (now - last_cmd_time > TIMEOUT_MS) {
    setMotors(0, 0);
  }

  // --- 3. KIRIM DATA ENCODER KE JETSON (20 Hz) ---
  if (now - last_report_time >= REPORT_INTERVAL_MS) {
    last_report_time = now;

    noInterrupts();
    long current_left  = enc_left_ticks;
    long current_right = enc_right_ticks;
    interrupts();

    Serial.print("ENC:");
    Serial.print(current_left);
    Serial.print(",");
    Serial.println(current_right);
  }
}
