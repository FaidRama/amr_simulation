/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE - DUAL MODE (PI-PSO & TRIAL-ERROR)
 * SKRIPSI : DIVALA ZAHRA OKTAVIA (TEKNIK KOMPUTER UNNES 2026)
 * =========================================================================
 * Fitur:
 *   1. Pilihan Mode PI:
 *      - PI-PSO        : Kp_L=2.7247, Ki_L=23.5472 | Kp_R=2.5428, Ki_R=24.1789
 *      - Trial & Error : Kp_L=2.5000, Ki_L=18.0000 | Kp_R=2.8000, Ki_R=18.0000
 *   2. Perintah Uji Step Respon dari Serial:
 *      Format: "STEP,<METODE>,<RPM>" 
 *      Contoh: "STEP,PSO,60" | "STEP,PSO,80" | "STEP,PSO,100"
 *              "STEP,TE,60"  | "STEP,TE,80"  | "STEP,TE,100"
 *   3. Output Streaming 9 Kolom:
 *      Time_s;Target_RPM_L;Actual_RPM_L;PWM_L;Error_L;Target_RPM_R;Actual_RPM_R;PWM_R;Error_R
 *   4. Komputasi 5 Parameter Evaluasi Transien (Peak, Mp, Tr, Ts, Ess, RMSE)
 *   5. Mode Normal ROS 2: "V,<v>,<w>" dan Odometri ("1" s/d "5")
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
const byte PIN_ENCA_L = 2; // INT0
const byte PIN_ENCB_L = 4;
const byte PIN_ENCA_R = 3; // INT1
const byte PIN_ENCB_R = A1;

// ====================== 2. OPSI INVERT ARAH ==============================
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = false;
const bool INVERT_ENC_L   = true;
const bool INVERT_ENC_R   = false;

// ====================== 3. SPESIFIKASI FISIK ROBOT =======================
const float WHEEL_DIAMETER = 0.127; // meter (5 inci = 0.127 m)
const float WHEEL_BASE_B   = 0.458; // meter (45.8 cm)
const float CPR_WHEEL      = 350.7; // 7 PPR * 50.1 gearbox = 350.7 ticks/rev

// ====================== 4. GAIN KENDALI PI ===============================
// Parameter PI-PSO (dari file pso_pi_baru.ino)
const float PSO_KP_L = 2.7247, PSO_KI_L = 23.5472;
const float PSO_KP_R = 2.5428, PSO_KI_R = 24.1789;

// Parameter Trial & Error (dari file trial_error_step_test.ino)
const float TE_KP_L  = 2.5000, TE_KI_L  = 18.0000;
const float TE_KP_R  = 2.8000, TE_KI_R  = 18.0000;

// Variabel Gain Aktif
float active_kp_l = PSO_KP_L, active_ki_l = PSO_KI_L;
float active_kp_r = PSO_KP_R, active_ki_r = PSO_KI_R;
bool modeIsPSO = true;

const unsigned long SAMPLING_MS = 20;
const float DT = SAMPLING_MS / 1000.0; // 0.02 detik (50 Hz)

// ====================== 5. VARIABEL OPERASIONAL & ODOMETRI ===============
volatile long totalPulsa_L = 0, totalPulsa_R = 0;
volatile long deltaPulsa_L = 0, deltaPulsa_R = 0;

float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
float setpoint_L = 0.0, setpoint_R = 0.0;
float rpm_L = 0.0, rpm_R = 0.0;
float rpm_ctrl_L = 0.0, rpm_ctrl_R = 0.0;
float error_L = 0.0, integral_L = 0.0, pwm_L = 0.0;
float error_R = 0.0, integral_R = 0.0, pwm_R = 0.0;

unsigned long waktuSebelumnya = 0;
unsigned long lastSerialCmdTime = 0;
const unsigned long SERIAL_WATCHDOG_TIMEOUT_MS = 600;

// Variabel Mode Pengujian
int modePengujian = 0; // 0=Standby, 1-5=Odometri Mandiri, 10=Uji Step Respon, 99=ROS 2
int stepPersegi = 0;
float targetJarak = 0.0, targetSudut = 0.0;

float step_target_rpm = 60.0;
unsigned long step_start_time = 0;
const float STEP_DURASI_TOTAL = 6.5; // detik
const float STEP_T_START      = 0.5; // detik ke-0.5 step diaktifkan
bool stepAktif = false;

// Buffer Serial Non-Blocking
char serial_buf[64];
uint8_t buf_idx = 0;

// ====================== 6. STRUKTUR EVALUASI TRANSIEN =====================
struct PerformaTracker {
  float target;
  float t_step_start;
  float ymax;
  float t_10, t_90, t_settle;
  bool reached_10, reached_90, settled;
  float sum_error_sq, sum_rpm, sum_pwm;
  unsigned long count_steady;

  void reset(float sp, float t_now) {
    target = sp;
    t_step_start = t_now;
    ymax = 0.0;
    t_10 = 0.0; t_90 = 0.0; t_settle = 0.0;
    reached_10 = false; reached_90 = false; settled = false;
    sum_error_sq = 0.0; sum_rpm = 0.0; sum_pwm = 0.0; count_steady = 0;
  }

  void update(float rpm, float pwm, float t_now) {
    if (target <= 1.0) return;
    float t_rel = t_now - t_step_start;
    if (rpm > ymax) ymax = rpm;

    if (!reached_10 && rpm >= (0.10 * target)) { t_10 = t_rel; reached_10 = true; }
    if (!reached_90 && rpm >= (0.90 * target)) { t_90 = t_rel; reached_90 = true; }

    float band = 0.05 * target;
    if (abs(rpm - target) <= band) {
      if (!settled && t_rel >= 0.15) { t_settle = t_rel; settled = true; }
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

  float getPeak()         { return ymax; }
  float getRMSE()         { return (count_steady > 0) ? sqrt(sum_error_sq / count_steady) : 0.0; }
  float getOvershoot()    { return (target > 1.0 && ymax > target) ? (((ymax - target) / target) * 100.0) : 0.0; }
  float getRiseTime()     { return (reached_90 && reached_10 && t_90 >= t_10) ? (t_90 - t_10) : 0.0; }
  float getSettlingTime() { return (settled) ? t_settle : 0.0; }
  float getEss()          { return (count_steady > 0) ? abs(target - (sum_rpm / count_steady)) : 0.0; }
  float getSteadyRPM()    { return (count_steady > 0) ? (sum_rpm / count_steady) : 0.0; }
} perf_L, perf_R;

// ====================== 7. INVERSE KINEMATICS ============================
void hitungInverseKinematics(float v, float omega) {
  float v_R = v + (WHEEL_BASE_B / 2.0) * omega;
  float v_L = v - (WHEEL_BASE_B / 2.0) * omega;
  setpoint_R = (v_R / (PI * WHEEL_DIAMETER)) * 60.0;
  setpoint_L = (v_L / (PI * WHEEL_DIAMETER)) * 60.0;
}

// ====================== 8. ISR ENCODER ===================================
void isrL() {
  bool b = digitalRead(PIN_ENCB_L);
  if (INVERT_ENC_L) b = !b;
  if (b == HIGH) { totalPulsa_L++; deltaPulsa_L++; }
  else           { totalPulsa_L--; deltaPulsa_L--; }
}

void isrR() {
  bool b = digitalRead(PIN_ENCB_R);
  if (INVERT_ENC_R) b = !b;
  if (b == HIGH) { totalPulsa_R++; deltaPulsa_R++; }
  else           { totalPulsa_R--; deltaPulsa_R--; }
}

// ====================== 9. DRIVER MOTOR BTS7960 ==========================
void setMotorDriver(byte rpwmPin, byte lpwmPin, float valPWM, bool invert) {
  if (invert) valPWM = -valPWM;
  int pwm = constrain(abs((int)valPWM), 0, 255);
  if (pwm == 0) {
    analogWrite(rpwmPin, 0); analogWrite(lpwmPin, 0);
  } else if (valPWM > 0) {
    analogWrite(rpwmPin, pwm); analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0); analogWrite(lpwmPin, pwm);
  }
}

void driveMotors(float pwmL, float pwmR) {
  setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, pwmL, INVERT_MOTOR_L);
  setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, pwmR, INVERT_MOTOR_R);
}

// ====================== 10. PEMILIHAN MODE KENDALI =======================
void setControlMode(bool isPSO) {
  modeIsPSO = isPSO;
  if (modeIsPSO) {
    active_kp_l = PSO_KP_L; active_ki_l = PSO_KI_L;
    active_kp_r = PSO_KP_R; active_ki_r = PSO_KI_R;
    Serial.println(F("# [MODE] Aktif: PI-PSO (Optimasi)"));
  } else {
    active_kp_l = TE_KP_L;  active_ki_l = TE_KI_L;
    active_kp_r = TE_KP_R;  active_ki_r = TE_KI_R;
    Serial.println(F("# [MODE] Aktif: PI Trial & Error (Manual)"));
  }
}

// ====================== 11. CETAK 5 PARAMETER ============================
void cetakRangkumanStep() {
  Serial.println(F("\n# ============================================================================"));
  Serial.print(F("# RANGKUMAN 5 PARAMETER EVALUASI STEP RESPON "));
  Serial.print(step_target_rpm, 0);
  Serial.println(modeIsPSO ? F(" RPM (PI-PSO)") : F(" RPM (TRIAL-ERROR)"));
  Serial.println(F("# ============================================================================"));
  Serial.println(F("# Parameter Evaluasi              | Motor Kiri     | Motor Kanan    | Satuan"));
  Serial.println(F("# ----------------------------------------------------------------------------"));
  Serial.print(F("# 1. Nilai Puncak (Peak RPM)     | "));
  Serial.print(perf_L.getPeak(), 2); Serial.print(F("          | "));
  Serial.print(perf_R.getPeak(), 2); Serial.println(F("          | RPM"));

  Serial.print(F("# 2. Overshoot (Mp)              | "));
  Serial.print(perf_L.getOvershoot(), 2); Serial.print(F("%        | "));
  Serial.print(perf_R.getOvershoot(), 2); Serial.println(F("%        | %"));

  Serial.print(F("# 3. Rise Time (Tr 10%-90%)      | "));
  Serial.print(perf_L.getRiseTime(), 3); Serial.print(F("        | "));
  Serial.print(perf_R.getRiseTime(), 3); Serial.println(F("        | detik"));

  Serial.print(F("# 4. Settling Time (Ts +-5%)     | "));
  Serial.print(perf_L.getSettlingTime(), 3); Serial.print(F("        | "));
  Serial.print(perf_R.getSettlingTime(), 3); Serial.println(F("        | detik"));

  Serial.print(F("# 5. Steady-State Error (Ess)    | "));
  Serial.print(perf_L.getEss(), 2); Serial.print(F("          | "));
  Serial.print(perf_R.getEss(), 2); Serial.println(F("          | RPM"));

  Serial.print(F("#    RMSE                        | "));
  Serial.print(perf_L.getRMSE(), 2); Serial.print(F("          | "));
  Serial.print(perf_R.getRMSE(), 2); Serial.println(F("          | RPM"));
  Serial.println(F("# ============================================================================\n"));
}

// ====================== 12. PARSER PERINTAH SERIAL =======================
void processCommand(char* cmd) {
  // A. Kecepatan dari ROS 2: "V,<v>,<omega>"
  if (cmd[0] == 'V' && cmd[1] == ',') {
    float v = 0.0, w = 0.0;
    if (sscanf(cmd, "V,%f,%f", &v, &w) == 2) {
      modePengujian = 99;
      lastSerialCmdTime = millis();
      hitungInverseKinematics(v, w);
    }
    return;
  }

  // B. Ganti Mode Kontroler: "MODE,PSO" atau "MODE,TE"
  if (strncmp(cmd, "MODE,PSO", 8) == 0) { setControlMode(true); return; }
  if (strncmp(cmd, "MODE,TE", 7) == 0)  { setControlMode(false); return; }

  // C. Pemicuan Uji Step Respon: "STEP,<METODE>,<RPM>"
  if (strncmp(cmd, "STEP", 4) == 0) {
    char metodeStr[8];
    float rpmIn = 60.0;
    if (sscanf(cmd, "STEP,%[^,],%f", metodeStr, &rpmIn) >= 1) {
      if (strncmp(metodeStr, "PSO", 3) == 0) setControlMode(true);
      else if (strncmp(metodeStr, "TE", 2) == 0) setControlMode(false);
      
      step_target_rpm = rpmIn;
      modePengujian = 10;
      stepAktif = false;
      integral_L = 0; integral_R = 0;
      setpoint_L = 0; setpoint_R = 0;
      step_start_time = millis();

      Serial.print(F("# PENGUJIAN : STEP RESPON "));
      Serial.print(step_target_rpm, 0);
      Serial.println(modeIsPSO ? F(" RPM (PI-PSO)") : F(" RPM (TRIAL-ERROR)"));
      Serial.println(F("Time_s;Target_RPM_L;Actual_RPM_L;PWM_L;Error_L;Target_RPM_R;Actual_RPM_R;PWM_R;Error_R"));
    }
    return;
  }

  // D. Reset Odometri: "RST"
  if (strncmp(cmd, "RST", 3) == 0) {
    noInterrupts(); totalPulsa_L = 0; totalPulsa_R = 0; deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
    odom_x = 0; odom_y = 0; odom_theta = 0;
    hitungInverseKinematics(0, 0);
    return;
  }

  // E. Menu Uji Mandiri Odometri ('0' s/d '5')
  if (strlen(cmd) == 1) {
    char m = cmd[0];
    if (m == '1') {
      odom_x = 0; odom_y = 0; odom_theta = 0; targetJarak = 2.0;
      hitungInverseKinematics(0.33, 0.0); modePengujian = 1;
    } else if (m == '2') {
      odom_x = 0; odom_y = 0; odom_theta = 0; targetSudut = 90.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.8); modePengujian = 2;
    } else if (m == '3') {
      odom_x = 0; odom_y = 0; odom_theta = 0; targetSudut = 180.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.8); modePengujian = 3;
    } else if (m == '4') {
      odom_x = 0; odom_y = 0; odom_theta = 0; targetSudut = 360.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.8); modePengujian = 4;
    } else if (m == '5') {
      odom_x = 0; odom_y = 0; odom_theta = 0; stepPersegi = 1; targetJarak = 2.0;
      hitungInverseKinematics(0.33, 0.0); modePengujian = 5;
    } else if (m == '0') {
      modePengujian = 0; hitungInverseKinematics(0.0, 0.0); driveMotors(0, 0);
    }
  }
}

// ====================== 13. SETUP =======================================
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

  driveMotors(0, 0);
  setControlMode(true); // Default awal: PI-PSO
  waktuSebelumnya = millis();
  lastSerialCmdTime = millis();
}

// ====================== 14. LOOP KENDALI 50 HZ ===========================
void loop() {
  // A. Baca Perintah Serial Non-Blocking
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

  // B. Loop Utama 50 Hz (Tiap 20 ms)
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;

    // Salin Pulsa Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    long tot_L = totalPulsa_L; long tot_R = totalPulsa_R;
    interrupts();

    // Filter EMA Alpha 0.25 (Stabil & Mulus)
    float rpm_raw_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float rpm_raw_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);
    rpm_ctrl_L = (0.25 * rpm_raw_L) + (0.75 * rpm_ctrl_L);
    rpm_ctrl_R = (0.25 * rpm_raw_R) + (0.75 * rpm_ctrl_R);
    rpm_L = rpm_ctrl_L;
    rpm_R = rpm_ctrl_R;

    // Odometri Lokal
    float ds_L = ((float)dC_L / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float ds_R = ((float)dC_R / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float dx_robot = (ds_R + ds_L) / 2.0;
    float dtheta   = (ds_R - ds_L) / WHEEL_BASE_B;
    float theta_mid = odom_theta + (dtheta / 2.0);
    odom_theta += dtheta;
    odom_x += dx_robot * cos(theta_mid);
    odom_y += dx_robot * sin(theta_mid);

    // KONDISI 1: JIKA SEDANG UJI STEP SKRIPSI
    if (modePengujian == 10) {
      float t = (waktuSekarang - step_start_time) / 1000.0;
      float target_now = 0.0;

      if (t >= STEP_T_START && t <= STEP_DURASI_TOTAL) {
        target_now = step_target_rpm;
        if (!stepAktif) {
          stepAktif = true;
          perf_L.reset(step_target_rpm, t);
          perf_R.reset(step_target_rpm, t);
        }
      } else {
        target_now = 0.0;
      }
      setpoint_L = target_now;
      setpoint_R = target_now;

      // Update Metrik Transien
      if (stepAktif && target_now > 1.0) {
        perf_L.update(rpm_L, pwm_L, t);
        perf_R.update(rpm_R, pwm_R, t);
      }
    }

    // Failsafe Watchdog ROS 2
    if (modePengujian == 99 && (waktuSekarang - lastSerialCmdTime > SERIAL_WATCHDOG_TIMEOUT_MS)) {
      setpoint_L = 0; setpoint_R = 0; driveMotors(0, 0);
    }

    // Algoritma Kendali PI Roda Kiri
    if (fabs(setpoint_L) < 0.05) {
      rpm_ctrl_L = 0; rpm_L = 0; pwm_L = 0; error_L = 0; integral_L = 0;
    } else {
      error_L = setpoint_L - rpm_ctrl_L;
      integral_L = constrain(integral_L + (active_ki_l * error_L * DT), -255.0, 255.0);
      pwm_L = constrain((active_kp_l * error_L) + integral_L, 0.0, 255.0);
    }

    // Algoritma Kendali PI Roda Kanan
    if (fabs(setpoint_R) < 0.05) {
      rpm_ctrl_R = 0; rpm_R = 0; pwm_R = 0; error_R = 0; integral_R = 0;
    } else {
      error_R = setpoint_R - rpm_ctrl_R;
      integral_R = constrain(integral_R + (active_ki_r * error_R * DT), -255.0, 255.0);
      pwm_R = constrain((active_kp_r * error_R) + integral_R, 0.0, 255.0);
    }

    driveMotors(pwm_L, pwm_R);

    // Output Streaming Data (Khusus Mode 10 Step Response)
    if (modePengujian == 10) {
      float t_cur = (waktuSekarang - step_start_time) / 1000.0;
      Serial.print(t_cur, 3); Serial.print(';');
      Serial.print(setpoint_L, 1); Serial.print(';');
      Serial.print(rpm_L, 2); Serial.print(';');
      Serial.print(pwm_L, 0); Serial.print(';');
      Serial.print(error_L, 2); Serial.print(';');
      Serial.print(setpoint_R, 1); Serial.print(';');
      Serial.print(rpm_R, 2); Serial.print(';');
      Serial.print(pwm_R, 0); Serial.print(';');
      Serial.println(error_R, 2);

      // Selesai Pengujian Step (detik ke-6.5)
      if (t_cur > STEP_DURASI_TOTAL) {
        driveMotors(0, 0);
        modePengujian = 0;
        cetakRangkumanStep();
      }
    } else {
      // Streaming Standar untuk Navigasi ROS 2
      Serial.print(F("ENC:")); Serial.print(tot_L); Serial.print(','); Serial.println(tot_R);
    }
  }
}