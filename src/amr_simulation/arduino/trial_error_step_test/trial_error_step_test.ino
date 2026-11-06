/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE - PENGUJIAN STEP RESPON TRIAL & ERROR
 * FITUR: MENCETAK DATA CSV + MENGHITUNG OTOMATIS 5 PARAMETER PERFORMA:
 *        (RMSE, Overshoot, Rise Time, Settling Time, Steady-State Error)
 * PARAMETER : Kiri  (Kp = 2.5000 | Ki = 18.0000 | Kd = 0.0000)
 *             Kanan (Kp = 2.8000 | Ki = 18.0000 | Kd = 0.0000)
 * SKRIPSI   : DIVALA ZAHRA OKTAVIA (UNNES 2026)
 * =========================================================================
 * Menu Serial:
 *   '1' : Uji Step 60 RPM (6.5 detik)
 *   '2' : Uji Step 80 RPM (6.5 detik)
 *   '3' : Uji Step 100 RPM (6.5 detik)
 *   '4' : Uji Bertingkat 60 -> 80 -> 100 RPM (13.5 detik)
 *   '0' : Stop Darurat
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
const float KP_L = 2.5000;
const float KI_L = 18.0000;
const float KD_L = 0.0000;

const float KP_R = 2.8000;
const float KI_R = 18.0000;
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

// =========================================================================
// 6. STRUKTUR AKUMULATOR 5 PARAMETER PERFORMA REAL-TIME (HEMAT MEMORI RAM)
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
  
  // Akumulator fase steady-state
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

    // 1. Catat Nilai Puncak untuk Overshoot
    if (rpm > ymax) ymax = rpm;

    // 2. Catat Waktu Naik (Rise Time: 10% s.d 90%)
    if (!reached_10 && rpm >= (0.10 * target)) {
      t_10 = t_rel;
      reached_10 = true;
    }
    if (!reached_90 && rpm >= (0.90 * target)) {
      t_90 = t_rel;
      reached_90 = true;
    }

    // 3. Catat Waktu Mantap (Settling Time: Masuk pita toleransi +-5% dan bertahan)
    float band = 0.05 * target;
    if (abs(rpm - target) <= band) {
      if (!settled && t_rel >= 0.2) {
        t_settle = t_rel;
        settled = true;
      }
    } else {
      // Jika terpental keluar pita sebelum fase steady, perbarui waktu settle
      if (t_rel < 1.0) settled = false;
    }

    // 4 & 5. Akumulasi Steady-State (setelah t_rel >= 1.0 detik) untuk RMSE & Ess
    if (t_rel >= 1.0) {
      float err = target - rpm;
      sum_error_sq += (err * err);
      sum_rpm += rpm;
      sum_pwm += pwm;
      count_steady++;
    }
  }

  float getOvershoot() {
    if (target <= 0.0) return 0.0;
    return max(0.0, ((ymax - target) / target) * 100.0);
  }

  float getRiseTime() {
    if (!reached_90) return 0.0;
    return max(0.0, t_90 - t_10);
  }

  float getSettlingTime() {
    return (settled) ? t_settle : 0.0;
  }

  float getSteadyRPM() {
    return (count_steady > 0) ? (sum_rpm / count_steady) : ymax;
  }

  float getEss() {
    return abs(target - getSteadyRPM());
  }

  float getRMSE() {
    return (count_steady > 0) ? sqrt(sum_error_sq / count_steady) : 0.0;
  }

  float getSteadyPWM() {
    return (count_steady > 0) ? (sum_pwm / count_steady) : 0.0;
  }
};

// Pelacak performa untuk masing-masing tahap kecepatan (60, 80, 100 RPM)
PerformaTracker perf_L[3];
PerformaTracker perf_R[3];
int tahap_aktif = -1;

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
  Serial.println(F("# RANGKUMAN 5 PARAMETER EVALUASI PERFORMA STEP RESPON (TRIAL & ERROR)"));
  Serial.println(F("# ============================================================================"));
  Serial.println(F("# Keterangan Definisi Parameter:"));
  Serial.println(F("# 1. RMSE (RPM)       : Ketepatan kecepatan roda mengikuti setpoint saat stabil."));
  Serial.println(F("# 2. Overshoot (%)    : Lonjakan kecepatan melewati setpoint sebelum kembali stabil."));
  Serial.println(F("# 3. Rise Time (s)    : Lama waktu naik dari kondisi 10% s.d 90% setpoint."));
  Serial.println(F("# 4. Settling Time (s): Lama waktu hingga kecepatan masuk pita toleransi +-5%."));
  Serial.println(F("# 5. Steady-State Err : Selisih antara setpoint dan kecepatan rata-rata saat stabil."));
  Serial.println(F("# ============================================================================"));

  int jml_tahap = (modeUji == 4) ? 3 : 1;
  float sp_list[3] = {60.0, 80.0, 100.0};
  if (modeUji == 1) sp_list[0] = 60.0;
  if (modeUji == 2) sp_list[0] = 80.0;
  if (modeUji == 3) sp_list[0] = 100.0;

  for (int i = 0; i < jml_tahap; i++) {
    Serial.print(F("\n# >>> TAHAP SETPOINT: "));
    Serial.print(sp_list[i], 0);
    Serial.println(F(" RPM <<<"));
    Serial.println(F("# ----------------------------------------------------------------------------"));
    Serial.println(F("# Parameter Evaluasi              | Motor Kiri     | Motor Kanan    | Satuan"));
    Serial.println(F("# ----------------------------------------------------------------------------"));
    
    // 1. RMSE
    Serial.print(F("# 1. RMSE (Ketepatan Setpoint)   | "));
    Serial.print(perf_L[i].getRMSE(), 3); Serial.print(F("          | "));
    Serial.print(perf_R[i].getRMSE(), 3); Serial.println(F("          | RPM"));

    // 2. Overshoot
    Serial.print(F("# 2. Overshoot (Lonjakan Puncak) | "));
    Serial.print(perf_L[i].getOvershoot(), 2); Serial.print(F("%        | "));
    Serial.print(perf_R[i].getOvershoot(), 2); Serial.println(F("%        | %"));

    // 3. Rise Time
    Serial.print(F("# 3. Rise Time (Waktu Naik)      | "));
    Serial.print(perf_L[i].getRiseTime(), 3); Serial.print(F("        | "));
    Serial.print(perf_R[i].getRiseTime(), 3); Serial.println(F("        | detik"));

    // 4. Settling Time
    Serial.print(F("# 4. Settling Time (Waktu Mantap)| "));
    Serial.print(perf_L[i].getSettlingTime(), 3); Serial.print(F("        | "));
    Serial.print(perf_R[i].getSettlingTime(), 3); Serial.println(F("        | detik"));

    // 5. Steady-State Error
    Serial.print(F("# 5. Steady-State Error (Ess)    | "));
    Serial.print(perf_L[i].getEss(), 3); Serial.print(F("          | "));
    Serial.print(perf_R[i].getEss(), 3); Serial.println(F("          | RPM"));

    // Info Pendukung
    Serial.print(F("#    Kecepatan Rata-rata Stabil  | "));
    Serial.print(perf_L[i].getSteadyRPM(), 2); Serial.print(F("        | "));
    Serial.print(perf_R[i].getSteadyRPM(), 2); Serial.println(F("        | RPM"));

    Serial.print(F("#    Nilai PWM saat Stabil       | "));
    Serial.print(perf_L[i].getSteadyPWM(), 1); Serial.print(F("        | "));
    Serial.print(perf_R[i].getSteadyPWM(), 1); Serial.println(F("        | PWM (0-255)"));
    Serial.println(F("# ----------------------------------------------------------------------------"));
  }
  Serial.println(F("# ============================================================================"));
  Serial.println(F("# PENGUJIAN SELESAI. Silakan copy data CSV atau tabel rangkuman di atas."));
}

// ====================== 10. CETAK HEADER CSV & RESET =====================
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
  tahap_aktif = -1;

  if (sedangUji) {
    Serial.println(F("# TUNING_METHOD: PID_MANUAL_TRIAL_ERROR;;;;;;;;"));
    Serial.println(F("# MOTOR_LEFT : Kp=2.5000; Ki=18.0000; Kd=0.0000;;;;;;"));
    Serial.println(F("# MOTOR_RIGHT: Kp=2.8000; Ki=18.0000; Kd=0.0000;;;;;;"));
    Serial.println(F("# WHEEL_DIAMETER: 0.127m; WHEEL_BASE: 0.458m;;;;;;;"));
    Serial.println(F("Time_s;Target_RPM_L;Actual_RPM_L;PWM_L;Error_L;Target_RPM_R;Actual_RPM_R;PWM_R;Error_R"));
  }
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

  driveMotors(0, 0);
  delay(1000);

  // Otomatis mulai Mode 4 (Multi-Step 60 -> 80 -> 100 RPM)
  mulaiPengujianBaru(4);
}

// ====================== 12. LOOP KENDALI 50 HZ ===========================
void loop() {
  // Pilihan Menu dari Serial
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '1')      mulaiPengujianBaru(1); // 60 RPM
    else if (c == '2') mulaiPengujianBaru(2); // 80 RPM
    else if (c == '3') mulaiPengujianBaru(3); // 100 RPM
    else if (c == '4' || c == 'r' || c == 'R') mulaiPengujianBaru(4); // Multi-Step
    else if (c == '0') {
      sedangUji = false;
      driveMotors(0, 0);
      Serial.println(F("# [STOP] Pengujian Dihentikan."));
    }
  }

  if (!sedangUji) return;

  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;
    float t = (waktuSekarang - waktuMulaiTes) / 1000.0;

    float target_rpm = 0.0;
    float durasi_habis = 6.5;
    int idx_tahap = 0;

    if (modeUji == 1) {
      target_rpm = (t < 0.5) ? 0.0 : (t <= 6.5 ? 60.0 : 0.0);
      durasi_habis = 6.5;
      idx_tahap = 0;
    } else if (modeUji == 2) {
      target_rpm = (t < 0.5) ? 0.0 : (t <= 6.5 ? 80.0 : 0.0);
      durasi_habis = 6.5;
      idx_tahap = 0;
    } else if (modeUji == 3) {
      target_rpm = (t < 0.5) ? 0.0 : (t <= 6.5 ? 100.0 : 0.0);
      durasi_habis = 6.5;
      idx_tahap = 0;
    } else if (modeUji == 4) {
      if (t < 1.0) {
        target_rpm = 0.0;
        idx_tahap = -1;
      } else if (t < 5.0) {
        target_rpm = 60.0;
        idx_tahap = 0;
      } else if (t < 9.0) {
        target_rpm = 80.0;
        idx_tahap = 1;
      } else if (t < 13.0) {
        target_rpm = 100.0;
        idx_tahap = 2;
      } else {
        target_rpm = 0.0;
        idx_tahap = -1;
      }
      durasi_habis = 13.5;
    }

    // Inisialisasi tracker jika berpindah tahap kecepatan
    if (idx_tahap >= 0 && idx_tahap != tahap_aktif) {
      tahap_aktif = idx_tahap;
      perf_L[tahap_aktif].reset(target_rpm, t);
      perf_R[tahap_aktif].reset(target_rpm, t);
    }

    setpoint_L = target_rpm;
    setpoint_R = target_rpm;

    // Salin Pulsa Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    interrupts();

    // Hitung RPM Aktual (Filter EMA alpha = 0.55)
    float rpm_raw_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float rpm_raw_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);
    rpm_ctrl_L = (0.55 * rpm_raw_L) + (0.45 * rpm_ctrl_L);
    rpm_ctrl_R = (0.55 * rpm_raw_R) + (0.45 * rpm_ctrl_R);

    rpm_L = rpm_ctrl_L;
    rpm_R = rpm_ctrl_R;

    // ---------------------------------------------------------------------
    // PERHITUNGAN KONTROL PI TRIAL & ERROR
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

    // Output ke Driver BTS7960
    driveMotors(pwm_L, pwm_R);

    // Update akumulator 5 parameter performa secara real-time
    if (tahap_aktif >= 0) {
      perf_L[tahap_aktif].update(rpm_L, pwm_L, t);
      perf_R[tahap_aktif].update(rpm_R, pwm_R, t);
    }

    // Cetak Baris Data CSV
    Serial.print(t, 3);          Serial.print(";");
    Serial.print(setpoint_L, 2); Serial.print(";");
    Serial.print(rpm_L, 2);      Serial.print(";");
    Serial.print((int)pwm_L);    Serial.print(";");
    Serial.print(error_L, 2);    Serial.print(";");
    Serial.print(setpoint_R, 2); Serial.print(";");
    Serial.print(rpm_R, 2);      Serial.print(";");
    Serial.print((int)pwm_R);    Serial.print(";");
    Serial.println(error_R, 2);

    // Saat Selesai: Stop Motor & Cetak Rangkuman Lengkap 5 Parameter
    if (t >= durasi_habis) {
      driveMotors(0, 0);
      sedangUji = false;
      cetakRangkumanPerforma();
    }
  }
}
