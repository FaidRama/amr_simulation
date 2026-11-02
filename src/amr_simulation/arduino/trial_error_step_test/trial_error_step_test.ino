/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE - PENGUJIAN STEP RESPON TRIAL & ERROR
 * PARAMETER : Kiri  (Kp = 2.0000 | Ki = 15.0000 | Kd = 0.0000)
 *             Kanan (Kp = 1.8000 | Ki = 16.0000 | Kd = 0.0000)
 * SKRIPSI   : DIVALA ZAHRA OKTAVIA (UNNES 2026)
 * =========================================================================
 * Cara Pakai (Tanpa Benchmark):
 *   - Begitu di-upload / dinyalakan: Otomatis menjalankan Multi-Step
 *     (0 -> 60 -> 80 -> 100 -> 0 RPM) dan mencetak format CSV Skripsi.
 *   - Bisa juga pilih lewat Serial:
 *       Ketik '1' : Uji Step 60 RPM (selama 6 detik)
 *       Ketik '2' : Uji Step 80 RPM (selama 6 detik)
 *       Ketik '3' : Uji Step 100 RPM (selama 6 detik)
 *       Ketik '4' : Uji Bertingkat 60 -> 80 -> 100 RPM
 *       Ketik '0' : STOP Darurat
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

// ====================== 2. OPSI INVERT ARAH RODA =========================
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = true;
const bool INVERT_ENC_L   = true;
const bool INVERT_ENC_R   = true;

// ====================== 3. SPESIFIKASI FISIK ROBOT =======================
const float WHEEL_DIAMETER = 0.127; // 5 inci (0.127 meter)
const float WHEEL_BASE_B   = 0.458; // 45.8 cm
const float CPR_WHEEL      = 350.7; // 7 PPR * 50.1 gearbox = 350.7 ticks/rev

// =========================================================================
// 4. PARAMETER KENDALI TRIAL & ERROR
// =========================================================================
const float KP_L = 2.0000;
const float KI_L = 15.0000;
const float KD_L = 0.0000;

const float KP_R = 1.8000;
const float KI_R = 16.0000;
const float KD_R = 0.0000;

const unsigned long SAMPLING_MS = 20;
const float DT = SAMPLING_MS / 1000.0; // 0.02 s (50 Hz)

// ====================== 5. VARIABEL OPERASIONAL ==========================
volatile long totalPulsa_L = 0, totalPulsa_R = 0;
volatile long deltaPulsa_L = 0, deltaPulsa_R = 0;

float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;

float setpoint_L = 0.0, setpoint_R = 0.0; // Target RPM
float rpm_L = 0.0, rpm_R = 0.0;           // Aktual RPM
float rpm_ctrl_L = 0.0, rpm_ctrl_R = 0.0; // Umpan balik kontrol
float error_L = 0.0, prev_error_L = 0.0, integral_L = 0.0, pwm_L = 0.0;
float error_R = 0.0, prev_error_R = 0.0, integral_R = 0.0, pwm_R = 0.0;

unsigned long waktuSebelumnya = 0;
unsigned long waktuMulaiTes   = 0;

// Mode Uji: 4 = Otomatis Bertingkat (60->80->100), 1 = 60 RPM, 2 = 80 RPM, 3 = 100 RPM, 0 = Stop
int modeUji = 4;
bool sedangUji = true;

// ====================== 6. INVERSE KINEMATICS ============================
void hitungInverseKinematics(float v, float omega) {
  float v_R = v + (WHEEL_BASE_B / 2.0) * omega; // m/s
  float v_L = v - (WHEEL_BASE_B / 2.0) * omega; // m/s

  setpoint_R = (v_R / (PI * WHEEL_DIAMETER)) * 60.0; // RPM
  setpoint_L = (v_L / (PI * WHEEL_DIAMETER)) * 60.0; // RPM
}

// ====================== 7. ISR ENCODER ===================================
void isrL() {
  bool b = digitalRead(PIN_ENCB_L);
  if (INVERT_ENC_L) b = !b;
  if (b == HIGH) {
    totalPulsa_L++; deltaPulsa_L++;
  } else {
    totalPulsa_L--; deltaPulsa_L--;
  }
}

void isrR() {
  bool b = digitalRead(PIN_ENCB_R);
  if (INVERT_ENC_R) b = !b;
  if (b == HIGH) {
    totalPulsa_R--; deltaPulsa_R--;
  } else {
    totalPulsa_R++; deltaPulsa_R++;
  }
}

// ====================== 8. DRIVER MOTOR BTS7960 ==========================
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

void driveMotors(float pwmL, float pwmR) {
  setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, pwmL, INVERT_MOTOR_L);
  setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, pwmR, INVERT_MOTOR_R);
}

// ====================== 9. CETAK HEADER CSV & RESET ======================
void mulaiPengujianBaru(int mode) {
  noInterrupts();
  totalPulsa_L = 0; totalPulsa_R = 0;
  deltaPulsa_L = 0; deltaPulsa_R = 0;
  interrupts();

  odom_x = 0.0; odom_y = 0.0; odom_theta = 0.0;
  setpoint_L = 0.0; setpoint_R = 0.0;
  rpm_L = 0.0; rpm_R = 0.0;
  rpm_ctrl_L = 0.0; rpm_ctrl_R = 0.0;
  error_L = 0.0; prev_error_L = 0.0; integral_L = 0.0; pwm_L = 0.0;
  error_R = 0.0; prev_error_R = 0.0; integral_R = 0.0; pwm_R = 0.0;

  driveMotors(0, 0);
  modeUji = mode;
  sedangUji = (mode != 0);
  waktuMulaiTes = millis();
  waktuSebelumnya = millis();

  if (sedangUji) {
    Serial.println(F("# TUNING_METHOD: PID_MANUAL_TRIAL_ERROR;;;;;;;;"));
    Serial.println(F("# MOTOR_LEFT : Kp=2.0000; Ki=15.0000; Kd=0.0000;;;;;;"));
    Serial.println(F("# MOTOR_RIGHT: Kp=1.8000; Ki=16.0000; Kd=0.0000;;;;;;"));
    Serial.println(F("# WHEEL_DIAMETER: 0.127m; WHEEL_BASE: 0.458m;;;;;;;"));
    Serial.println(F("Time_s;Target_RPM_L;Actual_RPM_L;PWM_L;Error_L;Target_RPM_R;Actual_RPM_R;PWM_R;Error_R"));
  }
}

// ====================== 10. SETUP SISTEM =================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_RPWM_L, OUTPUT);
  pinMode(PIN_LPWM_L, OUTPUT);
  pinMode(PIN_REN_L, OUTPUT);
  pinMode(PIN_LEN_L, OUTPUT);
  digitalWrite(PIN_REN_L, HIGH);
  digitalWrite(PIN_LEN_L, HIGH);

  pinMode(PIN_RPWM_R, OUTPUT);
  pinMode(PIN_LPWM_R, OUTPUT);
  pinMode(PIN_REN_R, OUTPUT);
  pinMode(PIN_LEN_R, OUTPUT);
  digitalWrite(PIN_REN_R, HIGH);
  digitalWrite(PIN_LEN_R, HIGH);

  pinMode(PIN_ENCA_L, INPUT_PULLUP);
  pinMode(PIN_ENCB_L, INPUT_PULLUP);
  pinMode(PIN_ENCA_R, INPUT_PULLUP);
  pinMode(PIN_ENCB_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_L), isrL, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_R), isrR, FALLING);

  driveMotors(0, 0);
  delay(1000); // Tunggu 1 detik stabilisasi awal

  // Langsung mulai Mode 4 (Otomatis Bertingkat 60 -> 80 -> 100 RPM)
  mulaiPengujianBaru(4);
}

// ====================== 11. LOOP KENDALI 50 HZ ===========================
void loop() {
  // A. Baca Pilihan Mode dari Serial (Jika diketik '1', '2', '3', '4', atau '0')
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '1')      mulaiPengujianBaru(1); // Step 60 RPM
    else if (c == '2') mulaiPengujianBaru(2); // Step 80 RPM
    else if (c == '3') mulaiPengujianBaru(3); // Step 100 RPM
    else if (c == '4' || c == 'r' || c == 'R') mulaiPengujianBaru(4); // Multi-Step 60->80->100 RPM
    else if (c == '0') mulaiPengujianBaru(0); // Stop
  }

  if (!sedangUji) return;

  // B. Sampling Loop Diskrit Tiap 20 ms (50 Hz)
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;
    float t = (waktuSekarang - waktuMulaiTes) / 1000.0;

    // Penentuan Target Setpoint RPM Berdasarkan Mode Uji
    float target_rpm = 0.0;
    float durasi_habis = 6.5;

    if (modeUji == 1) {
      target_rpm = (t < 0.5) ? 0.0 : (t <= 6.5 ? 60.0 : 0.0);
      durasi_habis = 6.5;
    } else if (modeUji == 2) {
      target_rpm = (t < 0.5) ? 0.0 : (t <= 6.5 ? 80.0 : 0.0);
      durasi_habis = 6.5;
    } else if (modeUji == 3) {
      target_rpm = (t < 0.5) ? 0.0 : (t <= 6.5 ? 100.0 : 0.0);
      durasi_habis = 6.5;
    } else if (modeUji == 4) {
      if (t < 1.0)       target_rpm = 0.0;
      else if (t < 5.0)  target_rpm = 60.0;
      else if (t < 9.0)  target_rpm = 80.0;
      else if (t < 13.0) target_rpm = 100.0;
      else               target_rpm = 0.0;
      durasi_habis = 13.5;
    }

    setpoint_L = target_rpm;
    setpoint_R = target_rpm;

    // Salin Pulsa Encoder secara Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    interrupts();

    // Hitung RPM Aktual (Filter EMA alpha = 0.55 sesuai model identifikasi)
    float rpm_raw_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float rpm_raw_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);
    rpm_ctrl_L = (0.55 * rpm_raw_L) + (0.45 * rpm_ctrl_L);
    rpm_ctrl_R = (0.55 * rpm_raw_R) + (0.45 * rpm_ctrl_R);

    rpm_L = rpm_ctrl_L;
    rpm_R = rpm_ctrl_R;

    // Hitung Odometri Lokal
    float ds_L = ((float)dC_L / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float ds_R = ((float)dC_R / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float dx_robot = (ds_R + ds_L) / 2.0;
    float dtheta = (ds_R - ds_L) / WHEEL_BASE_B;

    float theta_mid = odom_theta + (dtheta / 2.0);
    odom_theta += dtheta;
    odom_x += dx_robot * cos(theta_mid);
    odom_y += dx_robot * sin(theta_mid);

    // ---------------------------------------------------------------------
    // PERHITUNGAN KONTROL PI TRIAL & ERROR
    // Kiri : Kp = 2.0000, Ki = 15.0000, Kd = 0.0000
    // Kanan: Kp = 1.8000, Ki = 16.0000, Kd = 0.0000
    // ---------------------------------------------------------------------
    // Motor Kiri
    if (fabs(setpoint_L) < 0.05) {
      rpm_ctrl_L = 0.0; rpm_L = 0.0; pwm_L = 0.0;
      error_L = 0.0; integral_L = 0.0; prev_error_L = 0.0;
    } else {
      error_L = setpoint_L - rpm_ctrl_L;
      integral_L = constrain(integral_L + (KI_L * error_L * DT), -255.0, 255.0);
      pwm_L = constrain((KP_L * error_L) + integral_L + (KD_L * (error_L - prev_error_L) / DT), 0.0, 255.0);
      prev_error_L = error_L;
    }

    // Motor Kanan
    if (fabs(setpoint_R) < 0.05) {
      rpm_ctrl_R = 0.0; rpm_R = 0.0; pwm_R = 0.0;
      error_R = 0.0; integral_R = 0.0; prev_error_R = 0.0;
    } else {
      error_R = setpoint_R - rpm_ctrl_R;
      integral_R = constrain(integral_R + (KI_R * error_R * DT), -255.0, 255.0);
      pwm_R = constrain((KP_R * error_R) + integral_R + (KD_R * (error_R - prev_error_R) / DT), 0.0, 255.0);
      prev_error_R = error_R;
    }

    // Output Sinyal ke Driver BTS7960
    driveMotors(pwm_L, pwm_R);

    // Cetak Baris Data CSV (Persis Format CSV Skripsi Anda)
    Serial.print(t, 3);          Serial.print(";");
    Serial.print(setpoint_L, 2); Serial.print(";");
    Serial.print(rpm_L, 2);      Serial.print(";");
    Serial.print((int)pwm_L);    Serial.print(";");
    Serial.print(error_L, 2);    Serial.print(";");
    Serial.print(setpoint_R, 2); Serial.print(";");
    Serial.print(rpm_R, 2);      Serial.print(";");
    Serial.print((int)pwm_R);    Serial.print(";");
    Serial.println(error_R, 2);

    // Berhenti Otomatis Saat Durasi Selesai
    if (t >= durasi_habis) {
      driveMotors(0, 0);
      sedangUji = false;
    }
  }
}