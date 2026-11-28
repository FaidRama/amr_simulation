/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE: DUAL MODE (PI-PSO & TRIAL-ERROR)
 * LENGKAP: STEP RESPONSE MOTOR + AUTO-STOP ODOMETRI LURUS, ROTASI & PERSEGI
 * SKRIPSI : DIVALA ZAHRA OKTAVIA (UNNES 2026)
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
const byte PIN_ENCA_L = 2; // Interrupt INT0
const byte PIN_ENCB_L = 4;
const byte PIN_ENCA_R = 3; // Interrupt INT1
const byte PIN_ENCB_R = A1;

// ====================== 2. OPSI INVERT ARAH ==============================
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = true;
const bool INVERT_ENC_L   = true;
const bool INVERT_ENC_R   = true;

// ====================== 3. PARAMETER FISIK ROBOT =========================
const float WHEEL_DIAMETER = 0.125; // meter (5 inci = 0.125 meter)
const float WHEEL_BASE_B   = 0.458; // meter (45.8 cm)
const float CPR_WHEEL      = 356.3; // Ticks per revolusi PG36 (7 PPR * 50.1 gearbox)

// ====================== 4. GAIN KENDALI PI ===============================
// Parameter PI-PSO
const float PSO_KP_L = 2.7247, PSO_KI_L = 23.5472;
const float PSO_KP_R = 2.5428, PSO_KI_R = 24.1789;

// Parameter Trial & Error
const float TE_KP_L  = 2.5000, TE_KI_L  = 18.0000;
const float TE_KP_R  = 2.8000, TE_KI_R  = 18.0000;

// Variabel Gain Aktif di Memori RAM Arduino
float active_kp_l = PSO_KP_L, active_ki_l = PSO_KI_L;
float active_kp_r = PSO_KP_R, active_ki_r = PSO_KI_R;
bool modeIsPSO = true;

const unsigned long SAMPLING_MS = 20;
const float DT = SAMPLING_MS / 1000.0; // 0.02 s (50 Hz)

// ====================== 5. VARIABEL ENCODER & ODOMETRI ===================
volatile long totalPulsa_L = 0, totalPulsa_R = 0;
volatile long deltaPulsa_L = 0, deltaPulsa_R = 0;

float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;

float setpoint_L = 0.0, setpoint_R = 0.0; // Target RPM
float rpm_L = 0.0, rpm_R = 0.0;           // Aktual RPM
float error_L = 0.0, prev_error_L = 0.0, integral_L = 0.0, pwm_L = 0.0;
float error_R = 0.0, prev_error_R = 0.0, integral_R = 0.0, pwm_R = 0.0;

unsigned long waktuSebelumnya = 0;
const unsigned long SERIAL_WATCHDOG_TIMEOUT_MS = 600; // 0.6 detik
unsigned long lastSerialCmdTime = 0;

// State Mode Pengujian
int modePengujian = 0; // 0=Standby, 1=Lurus 2m, 2=Rot 90, 3=Rot 180, 4=Rot 360, 5=Persegi, 10=Step, 99=ROS 2
int stepPersegi = 0;
float targetJarak = 0.0, targetSudut = 0.0;

float step_target_rpm = 60.0;
unsigned long step_start_time = 0;
const float STEP_DURASI_TOTAL = 6.5; // detik
const float STEP_T_START      = 0.5; // detik ke-0.5 step diaktifkan
bool stepAktif = false;

// Buffer Serial Non-Blocking
char serial_buf[48];
uint8_t buf_idx = 0;

// ====================== 6. STRUKTUR EVALUASI 5 PARAMETER =================
struct PerformaTracker {
  float target;
  float t_step_start;
  float ymax;
  float t_10, t_90, t_settle;
  bool reached_10, reached_90, settled;
  float sum_error_sq, sum_rpm;
  unsigned long count_steady;

  void reset(float sp, float t_now) {
    target = sp;
    t_step_start = t_now;
    ymax = 0.0;
    t_10 = 0.0; t_90 = 0.0; t_settle = 0.0;
    reached_10 = false; reached_90 = false; settled = false;
    sum_error_sq = 0.0; sum_rpm = 0.0; count_steady = 0;
  }

  void update(float y, float t_now) {
    if (t_now < t_step_start) return;
    float t_rel = t_now - t_step_start;

    if (y > ymax) ymax = y;

    if (!reached_10 && y >= 0.10 * target) { t_10 = t_rel; reached_10 = true; }
    if (!reached_90 && y >= 0.90 * target) { t_90 = t_rel; reached_90 = true; }

    float band = 0.05 * target;
    if (fabs(y - target) > band) {
      t_settle = t_rel;
      settled = false;
    } else {
      settled = true;
    }

    if (t_rel >= (STEP_DURASI_TOTAL - STEP_T_START - 2.0)) {
      float err = target - y;
      sum_error_sq += (err * err);
      sum_rpm += y;
      count_steady++;
    }
  }

  float getOvershoot() {
    if (target <= 0.0 || ymax <= target) return 0.0;
    return ((ymax - target) / target) * 100.0;
  }
  float getRiseTime() {
    return (reached_90 && reached_10) ? (t_90 - t_10) : 0.0;
  }
  float getSettlingTime() { return t_settle; }
  float getSteadyStateError() {
    if (count_steady == 0) return 0.0;
    float mean_y = sum_rpm / count_steady;
    return fabs(target - mean_y);
  }
  float getRMSE() {
    if (count_steady == 0) return 0.0;
    return sqrt(sum_error_sq / count_steady);
  }
};

PerformaTracker tracker_L, tracker_R;

// ====================== 7. INTERRUPT SERVICE ROUTINES ====================
void isrL() {
  byte b = digitalRead(PIN_ENCB_L);
  if (b == HIGH) {
    totalPulsa_L++;
    deltaPulsa_L++;
  } else {
    totalPulsa_L--;
    deltaPulsa_L--;
  }
}

void isrR() {
  byte b = digitalRead(PIN_ENCB_R);
  if (b == HIGH) {
    totalPulsa_R--;
    deltaPulsa_R--;
  } else {
    totalPulsa_R++;
    deltaPulsa_R++;
  }
}

// ====================== 8. FUNGSI DRIVER BTS7960 =========================
void setMotorKiri(int pwmVal) {
  if (INVERT_MOTOR_L) pwmVal = -pwmVal;
  pwmVal = constrain(pwmVal, -255, 255);
  if (pwmVal > 0) {
    analogWrite(PIN_RPWM_L, pwmVal);
    analogWrite(PIN_LPWM_L, 0);
  } else if (pwmVal < 0) {
    analogWrite(PIN_RPWM_L, 0);
    analogWrite(PIN_LPWM_L, -pwmVal);
  } else {
    analogWrite(PIN_RPWM_L, 0);
    analogWrite(PIN_LPWM_L, 0);
  }
}

void setMotorKanan(int pwmVal) {
  if (INVERT_MOTOR_R) pwmVal = -pwmVal;
  pwmVal = constrain(pwmVal, -255, 255);
  if (pwmVal > 0) {
    analogWrite(PIN_RPWM_R, pwmVal);
    analogWrite(PIN_LPWM_R, 0);
  } else if (pwmVal < 0) {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, -pwmVal);
  } else {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, 0);
  }
}

void driveMotors(int pwmL, int pwmR) {
  setMotorKiri(pwmL);
  setMotorKanan(pwmR);
}

void hitungInverseKinematics(float v_lin, float v_ang) {
  float v_l = v_lin - (v_ang * WHEEL_BASE_B / 2.0);
  float v_r = v_lin + (v_ang * WHEEL_BASE_B / 2.0);
  setpoint_L = (v_l / (PI * WHEEL_DIAMETER)) * 60.0;
  setpoint_R = (v_r / (PI * WHEEL_DIAMETER)) * 60.0;
}

// ====================== 9. CETAK RANGKUMAN STEP ==========================
void cetakRangkumanStep() {
  Serial.println(F("\n# ========================================================================"));
  Serial.print(F("# HASIL EVALUASI 5 PARAMETER RESPON TRANSIEN ("));
  Serial.print(step_target_rpm, 0);
  Serial.print(F(" RPM - "));
  Serial.println(modeIsPSO ? F("PI-PSO)") : F("TRIAL-ERROR)"));
  Serial.println(F("# ========================================================================"));
  Serial.println(F("# Parameter Evaluasi;Motor Kiri;Motor Kanan;Satuan"));

  Serial.print(F("# 1. Nilai Puncak (Peak RPM);"));
  Serial.print(tracker_L.ymax, 2); Serial.print(';');
  Serial.print(tracker_R.ymax, 2); Serial.println(F(";RPM"));

  Serial.print(F("# 2. Overshoot (Mp);"));
  Serial.print(tracker_L.getOvershoot(), 2); Serial.print(F("%;"));
  Serial.print(tracker_R.getOvershoot(), 2); Serial.println(F("%;%"));

  Serial.print(F("# 3. Rise Time (Tr 10%-90%);"));
  Serial.print(tracker_L.getRiseTime(), 3); Serial.print(';');
  Serial.print(tracker_R.getRiseTime(), 3); Serial.println(F(";detik"));

  Serial.print(F("# 4. Settling Time (Ts +-5%);"));
  Serial.print(tracker_L.getSettlingTime(), 3); Serial.print(';');
  Serial.print(tracker_R.getSettlingTime(), 3); Serial.println(F(";detik"));

  Serial.print(F("# 5. Steady-State Error (Ess);"));
  Serial.print(tracker_L.getSteadyStateError(), 2); Serial.print(';');
  Serial.print(tracker_R.getSteadyStateError(), 2); Serial.println(F(";RPM"));

  Serial.print(F("#    - Persentase Galat Ess;"));
  Serial.print((tracker_L.getSteadyStateError() / step_target_rpm) * 100.0, 2); Serial.print(F("%;"));
  Serial.print((tracker_R.getSteadyStateError() / step_target_rpm) * 100.0, 2); Serial.println(F("%;%"));

  Serial.print(F("#    - RMSE Respon;"));
  Serial.print(tracker_L.getRMSE(), 2); Serial.print(';');
  Serial.print(tracker_R.getRMSE(), 2); Serial.println(F(";RPM"));
  Serial.println(F("# ========================================================================\n"));
}

// ====================== 10. PROTOKOL PARSER SERIAL =======================
void prosesPerintahSerial(char *cmd) {
  // A. Perintah Kecepatan Linier & Anguler:
  //    - "V,v_lin,v_ang"   (ROS 2 arduino_bridge.py / Teleop / Nav2)
  //    - "CMD,v_lin,v_ang" (skrip uji mandiri)
  //    Menggunakan strchr + atof karena sscanf("%f") nonaktif secara default di AVR Arduino Uno.
  if (cmd[0] == 'V' || cmd[0] == 'v' || strncmp(cmd, "CMD,", 4) == 0) {
    char *comma1 = strchr(cmd, ',');
    if (comma1 != NULL) {
      float v_lin = atof(comma1 + 1);
      char *comma2 = strchr(comma1 + 1, ',');
      float v_ang = (comma2 != NULL) ? atof(comma2 + 1) : 0.0;
      lastSerialCmdTime = millis();
      modePengujian = 99;
      hitungInverseKinematics(v_lin, v_ang);
    }
    return;
  }

  // B. Ganti Mode: "MODE,PSO" atau "MODE,TE" + Kirim Konfirmasi Balik
  if (strncmp(cmd, "MODE,PSO", 8) == 0) {
    active_kp_l = PSO_KP_L; active_ki_l = PSO_KI_L;
    active_kp_r = PSO_KP_R; active_ki_r = PSO_KI_R;
    modeIsPSO = true;
    Serial.print(F("[ARDUINO CONFIRM] Mode PI-PSO Aktif -> Kp_L=")); Serial.print(active_kp_l, 4);
    Serial.print(F(", Ki_L=")); Serial.print(active_ki_l, 4);
    Serial.print(F(" | Kp_R=")); Serial.print(active_kp_r, 4);
    Serial.print(F(", Ki_R=")); Serial.println(active_ki_r, 4);
    return;
  }
  if (strncmp(cmd, "MODE,TE", 7) == 0) {
    active_kp_l = TE_KP_L; active_ki_l = TE_KI_L;
    active_kp_r = TE_KP_R; active_ki_r = TE_KI_R;
    modeIsPSO = false;
    Serial.print(F("[ARDUINO CONFIRM] Mode TRIAL-ERROR Aktif -> Kp_L=")); Serial.print(active_kp_l, 4);
    Serial.print(F(", Ki_L=")); Serial.print(active_ki_l, 4);
    Serial.print(F(" | Kp_R=")); Serial.print(active_kp_r, 4);
    Serial.print(F(", Ki_R=")); Serial.println(active_ki_r, 4);
    return;
  }

  // C. Pemicuan Uji Step Respon dari Python: "STEP,<METODE>,<RPM>"
  if (strncmp(cmd, "STEP", 4) == 0) {
    char *token = strtok(cmd, ","); // "STEP"
    token = strtok(NULL, ",");       // "PSO" atau "TE"
    if (token != NULL) {
      if (strncmp(token, "PSO", 3) == 0) {
        active_kp_l = PSO_KP_L; active_ki_l = PSO_KI_L;
        active_kp_r = PSO_KP_R; active_ki_r = PSO_KI_R;
        modeIsPSO = true;
      } else {
        active_kp_l = TE_KP_L; active_ki_l = TE_KI_L;
        active_kp_r = TE_KP_R; active_ki_r = TE_KI_R;
        modeIsPSO = false;
      }
    }

    token = strtok(NULL, ",");       // RPM
    if (token != NULL) {
      step_target_rpm = atof(token);
    } else {
      step_target_rpm = 60.0;
    }

    modePengujian = 10;
    stepAktif = false;
    integral_L = 0; integral_R = 0;
    setpoint_L = 0; setpoint_R = 0;
    step_start_time = millis();

    Serial.print(F("# PENGUJIAN : STEP RESPON "));
    Serial.print(step_target_rpm, 0);
    Serial.println(modeIsPSO ? F(" RPM (PI-PSO)") : F(" RPM (TRIAL-ERROR)"));
    Serial.println(F("Time_s;Target_RPM_L;Actual_RPM_L;PWM_L;Error_L;Target_RPM_R;Actual_RPM_R;PWM_R;Error_R"));
    return;
  }

  // D. Reset Odometri: "RST"
  if (strncmp(cmd, "RST", 3) == 0) {
    noInterrupts(); totalPulsa_L = 0; totalPulsa_R = 0; deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
    odom_x = 0; odom_y = 0; odom_theta = 0;
    hitungInverseKinematics(0, 0);
    driveMotors(0, 0);
    return;
  }

  // E. Menu Uji Mandiri Odometri ('0' s/d '5')
  if (strlen(cmd) == 1) {
    char m = cmd[0];
    if (m == '1') {
      noInterrupts(); deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetJarak = 2.0;
      hitungInverseKinematics(0.33, 0.0);
      modePengujian = 1;
    } else if (m == '2') {
      noInterrupts(); deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetSudut = 90.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.35);
      modePengujian = 2;
    } else if (m == '3') {
      noInterrupts(); deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetSudut = 180.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.35);
      modePengujian = 3;
    } else if (m == '4') {
      noInterrupts(); deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetSudut = 360.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.35);
      modePengujian = 4;
    } else if (m == '5') {
      noInterrupts(); deltaPulsa_L = 0; deltaPulsa_R = 0; interrupts();
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetJarak = 2.0;
      stepPersegi = 1;
      hitungInverseKinematics(0.33, 0.0);
      modePengujian = 5;
    } else if (m == '0') {
      modePengujian = 0;
      hitungInverseKinematics(0, 0);
      driveMotors(0, 0);
      setpoint_L = 0; setpoint_R = 0;
    }
  }
}

// ====================== 11. SETUP ARDUINO ================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_RPWM_L, OUTPUT); pinMode(PIN_LPWM_L, OUTPUT);
  pinMode(PIN_REN_L, OUTPUT);  pinMode(PIN_LEN_L, OUTPUT);
  pinMode(PIN_RPWM_R, OUTPUT); pinMode(PIN_LPWM_R, OUTPUT);
  pinMode(PIN_REN_R, OUTPUT);  pinMode(PIN_LEN_R, OUTPUT);

  digitalWrite(PIN_REN_L, HIGH); digitalWrite(PIN_LEN_L, HIGH);
  digitalWrite(PIN_REN_R, HIGH); digitalWrite(PIN_LEN_R, HIGH);

  pinMode(PIN_ENCA_L, INPUT_PULLUP);
  pinMode(PIN_ENCB_L, INPUT_PULLUP);
  pinMode(PIN_ENCA_R, INPUT_PULLUP);
  pinMode(PIN_ENCB_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_L), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_R), isrR, RISING);

  driveMotors(0, 0);

  Serial.println(F("\n# ========================================================================"));
  Serial.println(F("# SISTEM KONTROL MOTOR AMR-01 SIAP (DUAL MODE PI-PSO & TRIAL-ERROR)"));
  Serial.println(F("# Default: PI-PSO | Sampling: 50 Hz | Watchdog: 600 ms"));
  Serial.println(F("# ========================================================================"));
}

// ====================== 12. LOOP UTAMA ===================================
void loop() {
  // 1. Baca Perintah Serial Non-Blocking
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf_idx > 0) {
        serial_buf[buf_idx] = '\0';
        prosesPerintahSerial(serial_buf);
        buf_idx = 0;
      }
    } else {
      if (buf_idx < sizeof(serial_buf) - 1) {
        serial_buf[buf_idx++] = c;
      }
    }
  }

  // 2. Loop Kontrol Periodik 50 Hz (20 ms)
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;

    // Salin Pulsa Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    long tot_L = totalPulsa_L; long tot_R = totalPulsa_R;
    interrupts();

    // Filter EMA Alpha = 0.25 (Mulus Skripsi)
    static float rpm_ctrl_L = 0.0, rpm_ctrl_R = 0.0;
    float rpm_raw_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float rpm_raw_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);
    rpm_ctrl_L = (0.25 * rpm_raw_L) + (0.75 * rpm_ctrl_L);
    rpm_ctrl_R = (0.25 * rpm_raw_R) + (0.75 * rpm_ctrl_R);

    rpm_L = rpm_ctrl_L;
    rpm_R = rpm_ctrl_R;

    // Hitung Odometri Lokal Real-Time
    float ds_L = ((float)dC_L / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float ds_R = ((float)dC_R / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float dx_robot = (ds_R + ds_L) / 2.0;
    float dtheta = (ds_R - ds_L) / WHEEL_BASE_B;

    float theta_mid = odom_theta + (dtheta / 2.0);
    odom_theta += dtheta;
    odom_x += dx_robot * cos(theta_mid);
    odom_y += dx_robot * sin(theta_mid);

    // =====================================================================
    // PENGHENTIAN OTOMATIS MODE UJI ODOMETRI (AUTO-STOP)
    // =====================================================================
    if (modePengujian == 1) {
      // 1. Gerak Lurus 2.0 Meter
      float dist = sqrt(odom_x * odom_x + odom_y * odom_y);
      if (dist >= targetJarak) {
        hitungInverseKinematics(0, 0);
        driveMotors(0, 0);
        setpoint_L = 0; setpoint_R = 0;
        modePengujian = 0;
      }
    } else if (modePengujian >= 2 && modePengujian <= 4) {
      // 2. Rotasi 90, 180, 360 Derajat
      if (abs(odom_theta) >= targetSudut) {
        hitungInverseKinematics(0, 0);
        driveMotors(0, 0);
        setpoint_L = 0; setpoint_R = 0;
        modePengujian = 0;
      }
    } else if (modePengujian == 5) {
      // 3. Persegi Tertutup (4x2m = 8m)
      static float markX = 0, markY = 0, markTheta = 0;
      if (stepPersegi % 2 == 1) {
        float dSegmen = sqrt(pow(odom_x - markX, 2) + pow(odom_y - markY, 2));
        if (dSegmen >= targetJarak) {
          markTheta = odom_theta;
          hitungInverseKinematics(0.0, 0.35); // Belok 90 deg
          stepPersegi++;
        }
      } else {
        if (abs(odom_theta - markTheta) >= (90.0 * PI / 180.0)) {
          markX = odom_x; markY = odom_y;
          if (stepPersegi >= 8) {
            hitungInverseKinematics(0, 0);
            driveMotors(0, 0);
            setpoint_L = 0; setpoint_R = 0;
            modePengujian = 0;
          } else {
            hitungInverseKinematics(0.33, 0.0); // Maju 2m lagi
            stepPersegi++;
          }
        }
      }
    } else if (modePengujian == 10) {
      // 4. Mode Step Test 6.5 Detik
      float t_cur = (waktuSekarang - step_start_time) / 1000.0;
      if (t_cur >= STEP_T_START && !stepAktif) {
        stepAktif = true;
        setpoint_L = step_target_rpm;
        setpoint_R = step_target_rpm;
        tracker_L.reset(step_target_rpm, t_cur);
        tracker_R.reset(step_target_rpm, t_cur);
      }
      if (stepAktif) {
        tracker_L.update(rpm_L, t_cur);
        tracker_R.update(rpm_R, t_cur);
      }
    }

    // FAILSAFE WATCHDOG ROS 2
    if (modePengujian == 99 && (waktuSekarang - lastSerialCmdTime > SERIAL_WATCHDOG_TIMEOUT_MS)) {
      setpoint_L = 0.0; setpoint_R = 0.0;
      integral_L = 0.0; integral_R = 0.0;
      driveMotors(0, 0);
    }

    // Algoritma PI Roda Kiri
    if (fabs(setpoint_L) < 0.05) {
      rpm_ctrl_L = 0.0; rpm_L = 0.0; pwm_L = 0.0;
      integral_L = 0.0; prev_error_L = 0.0;
    } else {
      error_L = setpoint_L - rpm_ctrl_L;
      integral_L = constrain(integral_L + (active_ki_l * error_L * DT), -255.0, 255.0);
      pwm_L = constrain((active_kp_l * error_L) + integral_L, -255.0, 255.0);
      prev_error_L = error_L;
    }

    // Algoritma PI Roda Kanan
    if (fabs(setpoint_R) < 0.05) {
      rpm_ctrl_R = 0.0; rpm_R = 0.0; pwm_R = 0.0;
      integral_R = 0.0; prev_error_R = 0.0;
    } else {
      error_R = setpoint_R - rpm_ctrl_R;
      integral_R = constrain(integral_R + (active_ki_r * error_R * DT), -255.0, 255.0);
      pwm_R = constrain((active_kp_r * error_R) + integral_R, -255.0, 255.0);
      prev_error_R = error_R;
    }

    driveMotors(pwm_L, pwm_R);

    // =====================================================================
    // OUTPUT STREAMING SERIAL
    // =====================================================================
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

      if (t_cur > STEP_DURASI_TOTAL) {
        driveMotors(0, 0);
        setpoint_L = 0; setpoint_R = 0;
        modePengujian = 0;
        cetakRangkumanStep();
      }
    } else {
      // Telemetri Odometri & ROS 2
      Serial.print(F("ENC:")); Serial.print(tot_L); Serial.print(','); Serial.println(tot_R);

      Serial.print(F("LOG:"));
      Serial.print((float)waktuSekarang / 1000.0, 2); Serial.print(',');
      Serial.print(rpm_L, 1); Serial.print(',');
      Serial.print(rpm_R, 1); Serial.print(',');
      Serial.print(pwm_L, 0); Serial.print(',');
      Serial.print(pwm_R, 0); Serial.print(',');
      Serial.print(odom_x, 4); Serial.print(',');
      Serial.print(odom_y, 4); Serial.print(',');
      Serial.println(odom_theta * 180.0 / PI, 2);
    }
  }
}