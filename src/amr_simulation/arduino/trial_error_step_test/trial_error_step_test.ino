/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE - PENGUJIAN STEP RESPON 80 RPM (TRIAL & ERROR)
 * KHUSUS PENGUJIAN TUNGGAL: STEP 80 RPM (DURASI 6.5 DETIK)
 * FILTER EMA: ALPHA = 0.25 (Peredaman lonjakan kuantisasi diskrit enkoder 7 PPR)
 * MODE: STANDBY DIAM SAAT UPLOAD (MENUNGGU PERINTAH DARI SERIAL/PYTHON)
 * FITUR: MENCETAK DATA CSV 50 HZ + MENGHITUNG OTOMATIS 5 PARAMETER PERFORMA
 *        (RMSE, Overshoot, Rise Time, Settling Time, Steady-State Error)
 * PARAMETER : Motor Kiri  (Kp = 2.5000 | Ki = 18.0000 | Kd = 0.0000)
 *             Motor Kanan (Kp = 2.8000 | Ki = 18.0000 | Kd = 0.0000)
 * SKRIPSI   : DIVALA ZAHRA OKTAVIA (TEKNIK KOMPUTER UNNES 2026)
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
const float CPR_WHEEL      = 350.7; // 7 PPR * 50.1 gearbox = 350.7 ticks/rev

// =========================================================================
// 4. PARAMETER KENDALI PI MANUAL TRIAL & ERROR
// =========================================================================
const float KP_L = 2.5000;
const float KI_L = 18.0000;
const float KD_L = 0.0000;

const float KP_R = 2.8000;
const float KI_R = 18.0000;
const float KD_R = 0.0000;

const unsigned long SAMPLING_MS = 20;
const float DT = SAMPLING_MS / 1000.0; // 0.02 s (50 Hz)

// TARGET STEP RPM & DURASI KHUSUS 80 RPM
const float TARGET_STEP_RPM = 80.0;
const float T_STEP_MULAI    = 0.5;   // Detik ke-0.5 step diaktifkan
const float DURASI_TOTAL    = 6.5;   // Total durasi pengujian (detik)

// ====================== 5. VARIABEL OPERASIONAL ==========================
volatile long totalPulsa_L = 0, totalPulsa_R = 0;
volatile long deltaPulsa_L = 0, deltaPulsa_R = 0;

float setpoint_L = 0.0, setpoint_R = 0.0; // Target RPM
float rpm_L = 0.0, rpm_R = 0.0;           // Aktual RPM (setelah filter)
float rpm_ctrl_L = 0.0, rpm_ctrl_R = 0.0; // Register filter EMA
float error_L = 0.0, prev_error_L = 0.0, integral_L = 0.0, pwm_L = 0.0;
float error_R = 0.0, prev_error_R = 0.0, integral_R = 0.0, pwm_R = 0.0;

unsigned long waktuSebelumnya = 0;
unsigned long waktuMulaiTes   = 0;
bool sedangUji = false; // Standby diam saat baru dinyalakan
bool stepAktif = false;

// =========================================================================
// 6. STRUKTUR AKUMULATOR 5 PARAMETER PERFORMA ON-CHIP
// =========================================================================
struct PerformaTracker {
  float target;
  float t_step_start;
  float ymax;
  float t_10;
  float t_90;
  float t_settle;
  bool reached_10;
  bool reached_90;
  bool settled;
  
  float sum_error_sq;
  float sum_rpm;
  float sum_pwm;
  unsigned long count_steady;

  void reset(float sp, float t_now) {
    target = sp;
    t_step_start = t_now;
    ymax = 0.0;
    t_10 = 0.0;
    t_90 = 0.0;
    t_settle = 0.0;
    reached_10 = false;
    reached_90 = false;
    settled = false;
    sum_error_sq = 0.0;
    sum_rpm = 0.0;
    sum_pwm = 0.0;
    count_steady = 0;
  }

  void update(float rpm, float pwm, float t_now) {
    if (target <= 1.0) return;
    float t_rel = t_now - t_step_start;

    if (rpm > ymax) ymax = rpm;

    if (!reached_10 && rpm >= (0.10 * target)) {
      t_10 = t_rel;
      reached_10 = true;
    }
    if (!reached_90 && rpm >= (0.90 * target)) {
      t_90 = t_rel;
      reached_90 = true;
    }

    float band = 0.05 * target;
    if (abs(rpm - target) <= band) {
      if (!settled && t_rel >= 0.15) {
        t_settle = t_rel;
        settled = true;
      }
    } else {
      if (t_rel < 1.0) settled = false;
    }

    if (t_rel >= 1.0) {
      float err = target - rpm;
      sum_error_sq += (err * err);
      sum_rpm += rpm;
      sum_pwm += pwm;
      count_steady++;
    }
  }

  float getRMSE() {
    return (count_steady > 0) ? sqrt(sum_error_sq / count_steady) : 0.0;
  }
  float getOvershoot() {
    if (target <= 1.0 || ymax <= target) return 0.0;
    return ((ymax - target) / target) * 100.0;
  }
  float getRiseTime() {
    return (reached_90 && reached_10 && t_90 >= t_10) ? (t_90 - t_10) : 0.0;
  }
  float getSettlingTime() {
    return (settled) ? t_settle : 0.0;
  }
  float getEss() {
    if (count_steady == 0) return 0.0;
    return abs(target - (sum_rpm / count_steady));
  }
  float getSteadyRPM() {
    return (count_steady > 0) ? (sum_rpm / count_steady) : 0.0;
  }
  float getSteadyPWM() {
    return (count_steady > 0) ? (sum_pwm / count_steady) : 0.0;
  }
};

PerformaTracker perf_L;
PerformaTracker perf_R;

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

// ====================== 9. CETAK REKAP HASIL 5 PARAMETER ==================
void cetakRangkumanPerforma() {
  Serial.println(F("\n# ============================================================================"));
  Serial.println(F("# RANGKUMAN 5 PARAMETER EVALUASI PERFORMA STEP RESPON 80 RPM (TRIAL & ERROR)"));
  Serial.println(F("# ============================================================================"));
  Serial.println(F("# Parameter Evaluasi              | Motor Kiri     | Motor Kanan    | Satuan"));
  Serial.println(F("# ----------------------------------------------------------------------------"));
  
  Serial.print(F("# 1. RMSE (Ketepatan Setpoint)   | "));
  Serial.print(perf_L.getRMSE(), 3); Serial.print(F("          | "));
  Serial.print(perf_R.getRMSE(), 3); Serial.println(F("          | RPM"));

  Serial.print(F("# 2. Overshoot (Lonjakan Puncak) | "));
  Serial.print(perf_L.getOvershoot(), 2); Serial.print(F("%        | "));
  Serial.print(perf_R.getOvershoot(), 2); Serial.println(F("%        | %"));

  Serial.print(F("# 3. Rise Time (Waktu Naik)      | "));
  Serial.print(perf_L.getRiseTime(), 3); Serial.print(F("        | "));
  Serial.print(perf_R.getRiseTime(), 3); Serial.println(F("        | detik"));

  Serial.print(F("# 4. Settling Time (Waktu Mantap)| "));
  Serial.print(perf_L.getSettlingTime(), 3); Serial.print(F("        | "));
  Serial.print(perf_R.getSettlingTime(), 3); Serial.println(F("        | detik"));

  Serial.print(F("# 5. Steady-State Error (Ess)    | "));
  Serial.print(perf_L.getEss(), 3); Serial.print(F("          | "));
  Serial.print(perf_R.getEss(), 3); Serial.println(F("          | RPM"));

  Serial.print(F("#    Kecepatan Rata-rata Stabil  | "));
  Serial.print(perf_L.getSteadyRPM(), 2); Serial.print(F("        | "));
  Serial.print(perf_R.getSteadyRPM(), 2); Serial.println(F("        | RPM"));

  Serial.print(F("#    Nilai PWM saat Stabil       | "));
  Serial.print(perf_L.getSteadyPWM(), 1); Serial.print(F("        | "));
  Serial.print(perf_R.getSteadyPWM(), 1); Serial.println(F("        | PWM (0-255)"));
  Serial.println(F("# ============================================================================"));
  Serial.println(F("# PENGUJIAN 80 RPM SELESAI. Silakan copy data CSV atau tabel di atas."));
}

// ====================== 10. CETAK HEADER CSV & RESET =====================
void mulaiPengujian80RPM() {
  noInterrupts();
  totalPulsa_L = 0; totalPulsa_R = 0;
  deltaPulsa_L = 0; deltaPulsa_R = 0;
  interrupts();

  setpoint_L = 0.0; setpoint_R = 0.0;
  rpm_L = 0.0; rpm_R = 0.0;
  rpm_ctrl_L = 0.0; rpm_ctrl_R = 0.0;
  error_L = 0.0; prev_error_L = 0.0; integral_L = 0.0; pwm_L = 0.0;
  error_R = 0.0; prev_error_R = 0.0; integral_R = 0.0; pwm_R = 0.0;

  driveMotors(0, 0);
  sedangUji = true;
  stepAktif = false;
  waktuMulaiTes = millis();
  waktuSebelumnya = millis();

  Serial.println(F("# PENGUJIAN : STEP RESPON 80 RPM (TRIAL & ERROR - FILTER ALPHA 0.25);;;;;;;;"));
  Serial.println(F("# MOTOR_LEFT : Kp=2.5000; Ki=18.0000; Kd=0.0000;;;;;;"));
  Serial.println(F("# MOTOR_RIGHT: Kp=2.8000; Ki=18.0000; Kd=0.0000;;;;;;"));
  Serial.println(F("# TARGET_RPM : 80.0; DURASI: 6.5s;;;;;;;"));
  Serial.println(F("Time_s;Target_RPM_L;Actual_RPM_L;PWM_L;Error_L;Target_RPM_R;Actual_RPM_R;PWM_R;Error_R"));
}

// ====================== 11. SETUP SISTEM =================================
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

  // Standby diam: motor mati total saat upload
  driveMotors(0, 0);
  sedangUji = false;
  stepAktif = false;
  Serial.println(F("# [STANDBY] Robot siap. Menunggu perintah '1' untuk memulai pengujian 80 RPM."));
}

// ====================== 12. LOOP KENDALI 50 HZ ===========================
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '1' || c == 'r' || c == 'R') {
      mulaiPengujian80RPM(); // Mulai tes 80 RPM
    } else if (c == '0') {
      sedangUji = false;
      driveMotors(0, 0);
      Serial.println(F("# [STOP] Pengujian Dihentikan Darurat."));
    }
  }

  if (!sedangUji) return;

  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;
    float t = (waktuSekarang - waktuMulaiTes) / 1000.0;

    // TARGET STEP: 0 RPM selama 0.5s baseline, lalu 80.0 RPM sampai 6.5s
    float target_rpm = 0.0;
    if (t >= T_STEP_MULAI && t <= DURASI_TOTAL) {
      target_rpm = TARGET_STEP_RPM;
      if (!stepAktif) {
        stepAktif = true;
        perf_L.reset(TARGET_STEP_RPM, t);
        perf_R.reset(TARGET_STEP_RPM, t);
      }
    } else {
      target_rpm = 0.0;
    }

    setpoint_L = target_rpm;
    setpoint_R = target_rpm;

    // Salin Pulsa Encoder secara Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    interrupts();

    // Hitung RPM Mentah
    float rpm_raw_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float rpm_raw_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);

    // =====================================================================
    // FILTER LOW-PASS EMA DENGAN ALPHA = 0.25 (Peredaman Lebih Kuat)
    // =====================================================================
    rpm_ctrl_L = (0.25 * rpm_raw_L) + (0.75 * rpm_ctrl_L);
    rpm_ctrl_R = (0.25 * rpm_raw_R) + (0.75 * rpm_ctrl_R);

    rpm_L = rpm_ctrl_L;
    rpm_R = rpm_ctrl_R;

    // Kendali PI Trial & Error
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

    // Output ke Driver BTS7960
    driveMotors(pwm_L, pwm_R);

    // Update akumulator 5 parameter performa
    if (stepAktif && target_rpm > 1.0) {
      perf_L.update(rpm_L, pwm_L, t);
      perf_R.update(rpm_R, pwm_R, t);
    }

    // Cetak Baris CSV
    Serial.print(t, 3);          Serial.print(";");
    Serial.print(setpoint_L, 2); Serial.print(";");
    Serial.print(rpm_L, 2);      Serial.print(";");
    Serial.print((int)pwm_L);    Serial.print(";");
    Serial.print(error_L, 2);    Serial.print(";");
    Serial.print(setpoint_R, 2); Serial.print(";");
    Serial.print(rpm_R, 2);      Serial.print(";");
    Serial.print((int)pwm_R);    Serial.print(";");
    Serial.println(error_R, 2);

    // Saat Selesai (t >= 6.5s)
    if (t >= DURASI_TOTAL) {
      driveMotors(0, 0);
      sedangUji = false;
      cetakRangkumanPerforma();
    }
  }
}