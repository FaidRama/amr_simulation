/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE: ROS 2 + PID VELOCITY CONTROLLER
 * Integrasi Skripsi Divala Zahra Oktavia & ROS 2 Navigation Stack
 * =========================================================================
 * Format Input Serial (dari Jetson):
 *   1) "V,<v_linear>,<v_angular>\n"  (Contoh: "V,0.30,0.00\n" dari Nav2/Teleop)
 *   2) "1" s/d "5"                   (Menu Uji Mandiri Skripsi: Lurus, Belok,
 * Persegi) 3) "0"                           (Stop Darurat) 4) "RST\n" (Reset
 * Encoder & Odometri)
 *
 * Format Output Serial (ke Jetson):
 *   - "ENC:<pulsa_total_L>,<pulsa_total_R>\n" (Untuk ROS 2 /odom - 20 Hz)
 *   - "LOG:<time>,<rpm_L>,<rpm_R>,<pwm_L>,<pwm_R>,<x>,<y>,<theta>\n" (Untuk
 * Skripsi)
 * ========================================================================= */

#include <Arduino.h>

// ====================== 1. PIN HARDWARE ==================================
const byte PIN_RPWM_L = 5;
const byte PIN_LPWM_L = 6;
const byte PIN_REN_L = 7;
const byte PIN_LEN_L = 8;

const byte PIN_RPWM_R = 9;
const byte PIN_LPWM_R = 10;
const byte PIN_REN_R = 11;
const byte PIN_LEN_R = 12;

const byte PIN_ENCA_L = 2; // Interrupt INT0
const byte PIN_ENCB_L = 4;
const byte PIN_ENCA_R = 3; // Interrupt INT1
const byte PIN_ENCB_R = A1;

// ====================== 2. OPSI INVERT ARAH ==============================
// Sesuaikan dengan hasil validasi fisik roda kamu sebelumnya
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = true;
const bool INVERT_ENC_L = true;
const bool INVERT_ENC_R = true;

// ====================== 3. PARAMETER FISIK ROBOT =========================
const float WHEEL_DIAMETER =
    0.127; // meter (5 inci = 0.127 meter sesuai Proposal Skripsi)
const float WHEEL_BASE_B = 0.458; // meter (45.8 cm sesuai Proposal Skripsi)
const float CPR_WHEEL = 350.7; // Ticks per revolusi PG36 (7 PPR * 50.1 gearbox)

// ====================== 4. PARAMETER KENDALI PID-PSO =====================
const float KP_L = 0.9531, KI_L = 21.0829, KD_L = 0.0000;
const float KP_R = 0.7861, KI_R = 23.2146, KD_R = 0.0000;

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

// Watchdog Failsafe (Anti-Runaway jika Jetson mati / kabel serial lepas)
const unsigned long SERIAL_WATCHDOG_TIMEOUT_MS = 600; // 0.6 detik
unsigned long lastSerialCmdTime = 0;

// State Uji Mandiri Skripsi
int modePengujian = 0;
int stepPersegi = 0;
float targetJarak = 0.0, targetSudut = 0.0;

// Buffer Serial Non-Blocking
char serial_buf[48];
uint8_t buf_idx = 0;

// ====================== 6. INVERSE KINEMATICS ============================
void hitungInverseKinematics(float v, float omega) {
  // v_R = v + (B/2)*w
  // v_L = v - (B/2)*w
  float v_R = v + (WHEEL_BASE_B / 2.0) * omega; // m/s
  float v_L = v - (WHEEL_BASE_B / 2.0) * omega; // m/s

  // RPM = (v / (PI * D)) * 60
  setpoint_R = (v_R / (PI * WHEEL_DIAMETER)) * 60.0;
  setpoint_L = (v_L / (PI * WHEEL_DIAMETER)) * 60.0;
}

// ====================== 7. ISR ENCODER ===================================
void isrL() {
  bool b = digitalRead(PIN_ENCB_L);
  if (INVERT_ENC_L)
    b = !b;
  if (b == HIGH) {
    totalPulsa_L++;
    deltaPulsa_L++;
  } else {
    totalPulsa_L--;
    deltaPulsa_L--;
  }
}

void isrR() {
  bool b = digitalRead(PIN_ENCB_R);
  if (INVERT_ENC_R)
    b = !b;
  if (b == HIGH) {
    totalPulsa_R--;
    deltaPulsa_R--;
  } else {
    totalPulsa_R++;
    deltaPulsa_R++;
  }
}

// ====================== 8. DRIVER MOTOR (BTS7960) ========================
void setMotorDriver(byte rpwmPin, byte lpwmPin, float valPWM, bool invert) {
  if (invert)
    valPWM = -valPWM;
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

// ====================== 9. PARSER SERIAL NON-BLOCKING ====================
void processCommand(char *cmd) {
  // 1. Perintah Kecepatan dari ROS 2: "V,<linear_v>,<angular_w>"
  if (cmd[0] == 'V' || cmd[0] == 'v') {
    lastSerialCmdTime = millis(); // Refresh watchdog heartbeat dari Jetson
    char *comma1 = strchr(cmd, ',');
    if (comma1 != NULL) {
      float v_lin = atof(comma1 + 1);
      char *comma2 = strchr(comma1 + 1, ',');
      float v_ang = 0.0;
      if (comma2 != NULL) {
        v_ang = atof(comma2 + 1);
      }
      modePengujian = 99; // Mode Navigasi ROS 2
      hitungInverseKinematics(v_lin, v_ang);
    }
    return;
  }

  // 2. Perintah Reset
  if (strncmp(cmd, "RST", 3) == 0) {
    noInterrupts();
    totalPulsa_L = 0;
    totalPulsa_R = 0;
    deltaPulsa_L = 0;
    deltaPulsa_R = 0;
    interrupts();
    odom_x = 0;
    odom_y = 0;
    odom_theta = 0;
    hitungInverseKinematics(0, 0);
    return;
  }

  // 3. Menu Uji Mandiri Skripsi (Hanya jika input tepat 1 karakter: '0' s/d
  // '5')
  if (strlen(cmd) == 1) {
    char m = cmd[0];
    if (m == '1') {
      odom_x = 0;
      odom_y = 0;
      odom_theta = 0;
      targetJarak = 2.0;
      hitungInverseKinematics(0.33, 0.0);
      modePengujian = 1;
    } else if (m == '2') {
      odom_x = 0;
      odom_y = 0;
      odom_theta = 0;
      targetSudut = 90.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.8);
      modePengujian = 2;
    } else if (m == '3') {
      odom_x = 0;
      odom_y = 0;
      odom_theta = 0;
      targetSudut = 180.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.8);
      modePengujian = 3;
    } else if (m == '4') {
      odom_x = 0;
      odom_y = 0;
      odom_theta = 0;
      targetSudut = 360.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.8);
      modePengujian = 4;
    } else if (m == '5') {
      odom_x = 0;
      odom_y = 0;
      odom_theta = 0;
      stepPersegi = 1;
      targetJarak = 2.0;
      hitungInverseKinematics(0.33, 0.0);
      modePengujian = 5;
    } else if (m == '0') {
      modePengujian = 0;
      hitungInverseKinematics(0.0, 0.0);
    }
  }
}

// ====================== 10. SETUP =======================================
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
  waktuSebelumnya = millis();
  lastSerialCmdTime = millis();
}

// ====================== 11. LOOP UTAMA (20 Hz) ==========================
void loop() {
  // A. BACA SERIAL NON-BLOCKING
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

  // B. SAMPLING PID & ODOMETRI (50 ms / 20 Hz)
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;

    // Salin Pulsa Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L;
    deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R;
    deltaPulsa_R = 0;
    long tot_L = totalPulsa_L;
    long tot_R = totalPulsa_R;
    interrupts();

    // Hitung RPM Aktual
    rpm_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    rpm_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);

    // Hitung Odometri Lokal
    float ds_L = ((float)dC_L / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float ds_R = ((float)dC_R / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float dx_robot = (ds_R + ds_L) / 2.0;
    float dtheta = (ds_R - ds_L) / WHEEL_BASE_B;

    float theta_mid = odom_theta + (dtheta / 2.0);
    odom_theta += dtheta;
    odom_x += dx_robot * cos(theta_mid);
    odom_y += dx_robot * sin(theta_mid);

    // Penghentian Otomatis Mode Uji Skripsi
    if (modePengujian == 1) { // Lurus
      float dist = sqrt(odom_x * odom_x + odom_y * odom_y);
      if (dist >= targetJarak) {
        hitungInverseKinematics(0, 0);
        modePengujian = 0;
      }
    } else if (modePengujian >= 2 && modePengujian <= 4) { // Rotasi
      if (abs(odom_theta) >= targetSudut) {
        hitungInverseKinematics(0, 0);
        modePengujian = 0;
      }
    } else if (modePengujian == 5) { // Persegi
      static float markX = 0, markY = 0, markTheta = 0;
      if (stepPersegi % 2 == 1) {
        float dSegmen = sqrt(pow(odom_x - markX, 2) + pow(odom_y - markY, 2));
        if (dSegmen >= targetJarak) {
          markTheta = odom_theta;
          hitungInverseKinematics(0.0, 0.8);
          stepPersegi++;
        }
      } else {
        if (abs(odom_theta - markTheta) >= (90.0 * PI / 180.0)) {
          markX = odom_x;
          markY = odom_y;
          if (stepPersegi >= 8) {
            hitungInverseKinematics(0, 0);
            modePengujian = 0;
          } else {
            hitungInverseKinematics(0.33, 0.0);
            stepPersegi++;
          }
        }
      }
    }

    // 0. FAILSAFE WATCHDOG: Jika mode Navigasi ROS 2 (99) dan tidak ada
    // heartbeat > 0.6 detik (Jetson mati/kabel putus)
    if (modePengujian == 99 &&
        (waktuSekarang - lastSerialCmdTime > SERIAL_WATCHDOG_TIMEOUT_MS)) {
      setpoint_L = 0.0;
      setpoint_R = 0.0;
      integral_L = 0.0;
      integral_R = 0.0;
      prev_error_L = 0.0;
      prev_error_R = 0.0;
      pwm_L = 0.0;
      pwm_R = 0.0;
      driveMotors(0, 0);
    }

    // Perhitungan Loop PID
    // Motor Kiri
    if (fabs(setpoint_L) < 0.05) {
      pwm_L = 0.0;
      integral_L = 0.0;
      prev_error_L = 0.0;
    } else {
      error_L = setpoint_L - rpm_L;
      integral_L = constrain(integral_L + (KI_L * error_L * DT), -200.0, 200.0);
      pwm_L = constrain((KP_L * error_L) + integral_L +
                            (KD_L * (error_L - prev_error_L) / DT),
                        -255.0, 255.0);
      prev_error_L = error_L;
    }

    // Motor Kanan
    if (fabs(setpoint_R) < 0.05) {
      pwm_R = 0.0;
      integral_R = 0.0;
      prev_error_R = 0.0;
    } else {
      error_R = setpoint_R - rpm_R;
      integral_R = constrain(integral_R + (KI_R * error_R * DT), -200.0,200.0);
      pwm_R = constrain((KP_R * error_R) + integral_R +
                            (KD_R * (error_R - prev_error_R) / DT),
                        -255.0, 255.0);
      prev_error_R = error_R;
    }

    driveMotors(pwm_L, pwm_R);

    // Output Serial untuk ROS 2 /odom
    Serial.print("ENC:");
    Serial.print(tot_L);
    Serial.print(",");
    Serial.println(tot_R);

    // Output Serial Tambahan untuk Logging Skripsi
    Serial.print("LOG:");
    Serial.print((float)waktuSekarang / 1000.0, 2);
    Serial.print(",");
    Serial.print(rpm_L, 1);
    Serial.print(",");
    Serial.print(rpm_R, 1);
    Serial.print(",");
    Serial.print(pwm_L, 0);
    Serial.print(",");
    Serial.print(pwm_R, 0);
    Serial.print(",");
    Serial.print(odom_x, 3);
    Serial.print(",");
    Serial.print(odom_y, 3);
    Serial.print(",");
    Serial.println(odom_theta * 180.0 / PI, 1);
  }
}