/* =========================================================================
 * PENGUJIAN STEP RESPON BERTINGKAT - PI TRIAL & ERROR (STANDALONE USB)
 * POLA PROFIL : 0 -> 60 -> 80 -> 100 -> 0 RPM (OTOMATIS)
 * PARAMETER   : Kiri  (Kp = 2.0000, Ki = 15.0000, Kd = 0.0000)
 *               Kanan (Kp = 1.8000, Ki = 16.0000, Kd = 0.0000)
 * SKRIPSI     : DIVALA ZAHRA OKTAVIA (UNNES 2026)
 * ========================================================================= */

#include <Arduino.h>

// ====================== 1. PIN HARDWARE BTS7960 ===========================
const byte PIN_RPWM_L = 5;
const byte PIN_LPWM_L = 6;
const byte PIN_REN_L  = 7;
const byte PIN_LEN_L  = 8;

const byte PIN_RPWM_R = 9;
const byte PIN_LPWM_R = 10;
const byte PIN_REN_R  = 11;
const byte PIN_LEN_R  = 12;

// Pin Encoder Hall Effect PG36
const byte PIN_ENCA_L = 2;   // INT0
const byte PIN_ENCB_L = 4;
const byte PIN_ENCA_R = 3;   // INT1
const byte PIN_ENCB_R = A1;

// ====================== 2. KONFIGURASI ARAH & FISIK =======================
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = true;
const bool INVERT_ENC_L   = true;
const bool INVERT_ENC_R   = true;

const float CPR_WHEEL = 350.7; // 7 PPR * 50.1 gearbox = 350.7 ticks/rev

// =========================================================================
// 3. PARAMETER KENDALI PI TRIAL & ERROR
// =========================================================================
const float KP_L = 2.0000;
const float KI_L = 15.0000;
const float KD_L = 0.0000;

const float KP_R = 1.8000;
const float KI_R = 16.0000;
const float KD_R = 0.0000;

const unsigned long SAMPLING_MS = 20;
const float DT = SAMPLING_MS / 1000.0; // 0.02 s (50 Hz)

// ====================== 4. VARIABEL OPERASIONAL ==========================
volatile long deltaPulsa_L = 0, deltaPulsa_R = 0;

float rpm_L = 0.0, rpm_R = 0.0;           // RPM terfilter untuk log/grafik
float rpm_ctrl_L = 0.0, rpm_ctrl_R = 0.0; // RPM umpan balik kendali PI
float error_L = 0.0, prev_error_L = 0.0, integral_L = 0.0, pwm_L = 0.0;
float error_R = 0.0, prev_error_R = 0.0, integral_R = 0.0, pwm_R = 0.0;

unsigned long waktuMulai = 0;
unsigned long waktuSebelumnya = 0;
bool testSelesai = false;

// ====================== 5. ISR ENCODER ===================================
void isrL() {
  bool b = digitalRead(PIN_ENCB_L);
  if (INVERT_ENC_L) b = !b;
  if (b == HIGH) deltaPulsa_L++;
  else           deltaPulsa_L--;
}

void isrR() {
  bool b = digitalRead(PIN_ENCB_R);
  if (INVERT_ENC_R) b = !b;
  if (b == HIGH) deltaPulsa_R--;
  else           deltaPulsa_R++;
}

// ====================== 6. DRIVER MOTOR BTS7960 ==========================
void setMotorDriver(byte rpwmPin, byte lpwmPin, float valPWM, bool invert) {
  if (invert) valPWM = -valPWM;
  int pwm = constrain(abs((int)valPWM), 0, 255);
  bool arahMaju = (valPWM > 0);

  if (pwm == 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  } else if (arahMaju) {
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, pwm);
  }
}

void resetState() {
  noInterrupts();
  deltaPulsa_L = 0;
  deltaPulsa_R = 0;
  interrupts();

  rpm_L = 0.0; rpm_R = 0.0;
  rpm_ctrl_L = 0.0; rpm_ctrl_R = 0.0;
  error_L = 0.0; prev_error_L = 0.0; integral_L = 0.0; pwm_L = 0.0;
  error_R = 0.0; prev_error_R = 0.0; integral_R = 0.0; pwm_R = 0.0;

  setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, 0, INVERT_MOTOR_L);
  setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, 0, INVERT_MOTOR_R);

  testSelesai = false;
  waktuMulai = millis();
  waktuSebelumnya = millis();

  Serial.println("Time_s,Target_RPM,Actual_RPM_L,PWM_L,Actual_RPM_R,PWM_R");
}

// ====================== 7. SETUP SISTEM ==================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_RPWM_L, OUTPUT); pinMode(PIN_LPWM_L, OUTPUT);
  pinMode(PIN_REN_L, OUTPUT);  pinMode(PIN_LEN_L, OUTPUT);
  digitalWrite(PIN_REN_L, HIGH); digitalWrite(PIN_LEN_L, HIGH);

  pinMode(PIN_RPWM_R, OUTPUT); pinMode(PIN_LPWM_R, OUTPUT);
  pinMode(PIN_REN_R, OUTPUT);  pinMode(PIN_LEN_R, OUTPUT);
  digitalWrite(PIN_REN_R, HIGH); digitalWrite(PIN_LEN_R, HIGH);

  pinMode(PIN_ENCA_L, INPUT_PULLUP); pinMode(PIN_ENCB_L, INPUT_PULLUP);
  pinMode(PIN_ENCA_R, INPUT_PULLUP); pinMode(PIN_ENCB_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_L), isrL, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_R), isrR, FALLING);

  setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, 0, INVERT_MOTOR_L);
  setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, 0, INVERT_MOTOR_R);

  delay(1000); // Jeda aman 1 detik sebelum mulai
  resetState();
}

// ====================== 8. LOOP KENDALI 50 HZ ============================
void loop() {
  // Ketik 'R' atau 'r' di Serial Monitor jika ingin mengulang tes dari awal
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'R' || c == 'r') {
      resetState();
      return;
    }
  }

  if (testSelesai) return;

  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;
    float t = (waktuSekarang - waktuMulai) / 1000.0;

    // ---------------------------------------------------------------------
    // JADWAL STEP RESPON BERTINGKAT OTOMATIS:
    // 0.0 s -  2.0 s :   0.0 RPM (Kondisi Diam Awal)
    // 2.0 s -  6.0 s :  60.0 RPM (Step 1 - Durasi 4 Detik)
    // 6.0 s - 10.0 s :  80.0 RPM (Step 2 - Durasi 4 Detik)
    // 10.0 s - 14.0 s: 100.0 RPM (Step 3 - Durasi 4 Detik)
    // > 14.0 s       :   0.0 RPM (Berhenti Total)
    // ---------------------------------------------------------------------
    float setpoint = 0.0;
    if (t < 2.0) {
      setpoint = 0.0;
    } else if (t < 6.0) {
      setpoint = 60.0;
    } else if (t < 10.0) {
      setpoint = 80.0;
    } else if (t < 14.0) {
      setpoint = 100.0;
    } else {
      setpoint = 0.0;
    }

    // Ambil Data Pulsa Encoder secara Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    interrupts();

    // Hitung RPM Mentah & Filter EMA
    float rpm_raw_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float rpm_raw_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);

    // Filter EMA untuk Loop Kontrol & Akuisisi Data (alpha = 0.55)
    rpm_ctrl_L = (0.55 * rpm_raw_L) + (0.45 * rpm_ctrl_L);
    rpm_ctrl_R = (0.55 * rpm_raw_R) + (0.45 * rpm_ctrl_R);

    rpm_L = rpm_ctrl_L;
    rpm_R = rpm_ctrl_R;

    // ---------------------------------------------------------------------
    // KOMPUTASI KONTROL PI + ANTI-WINDUP
    // ---------------------------------------------------------------------
    if (setpoint <= 0.05) {
      pwm_L = 0.0; integral_L = 0.0; prev_error_L = 0.0;
      pwm_R = 0.0; integral_R = 0.0; prev_error_R = 0.0;
    } else {
      // Motor Kiri (Kp = 2.0000, Ki = 15.0000, Kd = 0.0000)
      error_L = setpoint - rpm_ctrl_L;
      integral_L = constrain(integral_L + (KI_L * error_L * DT), -255.0, 255.0);
      pwm_L = constrain((KP_L * error_L) + integral_L + (KD_L * (error_L - prev_error_L) / DT), 0.0, 255.0);
      prev_error_L = error_L;

      // Motor Kanan (Kp = 1.8000, Ki = 16.0000, Kd = 0.0000)
      error_R = setpoint - rpm_ctrl_R;
      integral_R = constrain(integral_R + (KI_R * error_R * DT), -255.0, 255.0);
      pwm_R = constrain((KP_R * error_R) + integral_R + (KD_R * (error_R - prev_error_R) / DT), 0.0, 255.0);
      prev_error_R = error_R;
    }

    // Kirim Sinyal PWM ke Driver BTS7960
    setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, pwm_L, INVERT_MOTOR_L);
    setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, pwm_R, INVERT_MOTOR_R);

    // Output Format CSV ke Serial Monitor / Serial Plotter
    Serial.print(t, 2);          Serial.print(",");
    Serial.print(setpoint, 1);   Serial.print(",");
    Serial.print(rpm_L, 2);      Serial.print(",");
    Serial.print((int)pwm_L);    Serial.print(",");
    Serial.print(rpm_R, 2);      Serial.print(",");
    Serial.println((int)pwm_R);

    // Hentikan Pengujian Setelah 15 Detik
    if (t >= 15.0) {
      setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, 0, INVERT_MOTOR_L);
      setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, 0, INVERT_MOTOR_R);
      testSelesai = true;
    }
  }
}
