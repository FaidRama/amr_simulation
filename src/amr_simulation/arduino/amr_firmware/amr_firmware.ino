/* =========================================================================
 * FIRMWARE AMR DIFFERENTIAL DRIVE (VERSI FINAL TERINTEGRASI)
 * Penulis: Divala Zahra Oktavia (Sesuai Proposal Skripsi & ROS 2 Bridge)
 * =========================================================================
 * Kompatibilitas Antarmuka:
 *   1) arduino_bridge.py (Jetson AGX Xavier) -> Nav2 / Teleop / Odometry ROS 2
 *   2) Menu Uji Mandiri Skripsi (1: Lurus 2m, 2: Rot 90, 3: Rot 180, 4: Rot 360, 5: Persegi)
 * 
 * Fitur Kendali:
 *   - Dual-loop PI PSO-ITSE Teruji (L: 0.8624 / 20.5540, R: 0.7037 / 22.4384)
 *   - Filter EMA (Alpha 0.35) untuk meredam derau kuantisasi pulsa diskrit
 *   - Clamping Anti-Windup Dinamis (Bebas Deadlock & Anti-Windup)
 *   - Watchdog Heartbeat Failsafe (Anti-Runaway jika ROS 2 / Jetson terputus)
 * ========================================================================= */

#include <Arduino.h>

// ====================== 1. PIN HARDWARE BTS7960 ==========================
const byte PIN_RPWM_L = 5;
const byte PIN_LPWM_L = 6;
const byte PIN_REN_L  = 7;
const byte PIN_LEN_L  = 8;

const byte PIN_RPWM_R = 9;
const byte PIN_LPWM_R = 10;
const byte PIN_REN_R  = 11;
const byte PIN_LEN_R  = 12;

// Pin Enkoder PG36
const byte PIN_ENCA_L = 2; // INT0
const byte PIN_ENCB_L = 4;
const byte PIN_ENCA_R = 3; // INT1
const byte PIN_ENCB_R = A1;

// ====================== 2. KONFIGURASI POLARITAS & ARAH ==================
const bool INVERT_MOTOR_L = false;
const bool INVERT_MOTOR_R = true;
const bool INVERT_ENC_L   = true;
const bool INVERT_ENC_R   = true;

// ====================== 3. PARAMETER FISIK ROBOT =========================
const float WHEEL_DIAMETER = 0.127; // D = 5 inci = 0.127 meter[cite: 5]
const float WHEEL_BASE_B   = 0.458; // B = 45.8 cm = 0.458 meter
const float CPR_WHEEL      = 350.7; // Resolusi efektif PG36 (7 PPR * 50.1 gearbox)[cite: 5]

// ====================== 4. PARAMETER KENDALI PI PSO-ITSE =================
const float KP_L = 0.8624;
const float KI_L = 20.5540;
const float KD_L = 0.0000;

const float KP_R = 0.7037;
const float KI_R = 22.4384;
const float KD_R = 0.0000;

const float ALPHA_EMA = 0.35; // Filter peredam kuantisasi pulsa diskrit

const unsigned long SAMPLING_MS = 50; 
const float DT = SAMPLING_MS / 1000.0; // 0.05 s (20 Hz - Sinkron dengan ROS 2 Bridge)

// ====================== 5. VARIABEL ENCODER & ODOMETRI ===================
volatile long totalPulsa_L = 0, totalPulsa_R = 0;
volatile long deltaPulsa_L = 0, deltaPulsa_R = 0;

float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0; // Pose odometri lokal Arduino

float setpoint_L = 0.0, setpoint_R = 0.0; // Target RPM
float rpm_L = 0.0, rpm_R = 0.0;           // Aktual RPM (Filtered EMA)
float integral_L = 0.0, prev_error_L = 0.0, pwm_L = 0.0;
float integral_R = 0.0, prev_error_R = 0.0, pwm_R = 0.0;

unsigned long waktuSebelumnya = 0;
unsigned long waktuMulai = 0;

// Watchdog Failsafe (Anti-Runaway jika Jetson mati / kabel serial lepas)
const unsigned long SERIAL_WATCHDOG_TIMEOUT_MS = 600; // 0.6 detik
unsigned long lastSerialCmdTime = 0;

// State Machine Menu Uji Mandiri Skripsi
int modePengujian = 0; 
int stepPersegi = 0;
float targetJarak = 0.0, targetSudut = 0.0;
float markX = 0.0, markY = 0.0, markTheta = 0.0;

// Buffer Serial Non-Blocking
char serial_buf[48];
uint8_t buf_idx = 0;

// ====================== 6. INVERSE KINEMATICS (SUB-BAB 2.2.10.5) ==========
void hitungInverseKinematics(float v, float omega) {
  // v_R = v + (B/2)*w
  // v_L = v - (B/2)*w
  float v_R = v + (WHEEL_BASE_B / 2.0) * omega; // m/s[cite: 5]
  float v_L = v - (WHEEL_BASE_B / 2.0) * omega; // m/s[cite: 5]

  // RPM = (v / (PI * D)) * 60
  setpoint_R = (v_R / (PI * WHEEL_DIAMETER)) * 60.0;
  setpoint_L = (v_L / (PI * WHEEL_DIAMETER)) * 60.0;
}

// ====================== 7. ISR ENCODER ===================================
void isrL() {
  bool b = digitalRead(PIN_ENCB_L);
  if (INVERT_ENC_L) b = !b;
  if (b == HIGH) { totalPulsa_L++; deltaPulsa_L++; }
  else           { totalPulsa_L--; deltaPulsa_L--; }
}

void isrR() {
  bool b = digitalRead(PIN_ENCB_R);
  if (INVERT_ENC_R) b = !b;
  if (b == HIGH) { totalPulsa_R--; deltaPulsa_R++; }
  else           { totalPulsa_R++; deltaPulsa_R++; }
}

// ====================== 8. DRIVER MOTOR (BTS7960) ========================
void setMotorDriver(byte rpwmPin, byte lpwmPin, float valPWM, bool invert) {
  if (invert) valPWM = -valPWM;
  int pwm = constrain(abs((int)valPWM), 0, 255);
  bool arahMaju = (valPWM > 0);

  if (pwm == 0) {
    analogWrite(rpwmPin, 0); analogWrite(lpwmPin, 0);
  } else if (arahMaju) {
    analogWrite(rpwmPin, pwm); analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0); analogWrite(lpwmPin, pwm);
  }
}

void driveMotors(float pwmL, float pwmR) {
  setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, pwmL, INVERT_MOTOR_L);
  setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, pwmR, INVERT_MOTOR_R);
}

// ====================== 9. ALGORITMA PI CLAMPING ANTI-WINDUP =============
float computePIClamped(float target, float current, float kp, float ki, float kd, 
                       float &integral, float &prev_err, float dt) {
  if (abs(target) < 0.5) {
    integral = 0.0;
    prev_err = 0.0;
    return 0.0;
  }

  float error = target - current;
  if (abs(error) < 0.2) error = 0.0; // Deadband peredam desis motor

  float P = kp * error;
  float D = kd * ((error - prev_err) / dt);
  prev_err = error;

  float tentative_i = integral + (error * dt);
  float unsaturated_pwm = P + (ki * tentative_i) + D;

  // Clamping anti-windup: integrasi aktif hanya bila aktuator belum jenuh di batas +/- 255
  if (abs(unsaturated_pwm) < 255.0) {
    integral = tentative_i;
  }

  float max_integral = 255.0 / ki;
  integral = constrain(integral, -max_integral, max_integral);

  float I = ki * integral;
  return constrain(P + I + D, -255.0, 255.0);
}

// ====================== 10. PARSER SERIAL NON-BLOCKING ===================
void processCommand(char* cmd) {
  // 1. Perintah Kecepatan dari ROS 2 Nav2: "V,<linear_v>,<angular_w>"
  if (cmd[0] == 'V' || cmd[0] == 'v') {
    lastSerialCmdTime = millis(); // Refresh watchdog heartbeat dari Jetson
    char* comma1 = strchr(cmd, ',');
    if (comma1 != NULL) {
      float v_lin = atof(comma1 + 1);
      char* comma2 = strchr(comma1 + 1, ',');
      float v_ang = 0.0;
      if (comma2 != NULL) {
        v_ang = atof(comma2 + 1);
      }
      modePengujian = 99; // Mode Navigasi Penuh ROS 2 (Teman Anda)
      hitungInverseKinematics(v_lin, v_ang);
    }
    return;
  }

  // 2. Perintah Reset Pose & Encoder
  if (strncmp(cmd, "RST", 3) == 0) {
    noInterrupts();
    totalPulsa_L = 0; totalPulsa_R = 0;
    deltaPulsa_L = 0; deltaPulsa_R = 0;
    interrupts();
    odom_x = 0.0; odom_y = 0.0; odom_theta = 0.0;
    integral_L = 0.0; integral_R = 0.0;
    hitungInverseKinematics(0.0, 0.0);
    return;
  }

  // 3. Menu Uji Mandiri Skripsi (Input 1 Karakter)[cite: 5]
  if (strlen(cmd) == 1) {
    char m = cmd[0];
    if (m == '1') { // Skenario 1: Gerak Lurus 2 Meter[cite: 5]
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetJarak = 2.0;
      hitungInverseKinematics(0.35, 0.0); // v = 0.35 m/s (~53 RPM)
      modePengujian = 1;
    } else if (m == '2') { // Skenario 2: Rotasi di Tempat 90 Derajat[cite: 5]
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetSudut = 90.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.60);
      modePengujian = 2;
    } else if (m == '3') { // Skenario 3: Rotasi di Tempat 180 Derajat[cite: 5]
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetSudut = 180.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.60);
      modePengujian = 3;
    } else if (m == '4') { // Skenario 4: Rotasi di Tempat 360 Derajat[cite: 5]
      odom_x = 0; odom_y = 0; odom_theta = 0;
      targetSudut = 360.0 * (PI / 180.0);
      hitungInverseKinematics(0.0, 0.60);
      modePengujian = 4;
    } else if (m == '5') { // Skenario 5: Lintasan Persegi Tertutup (2m x 2m)[cite: 5]
      odom_x = 0; odom_y = 0; odom_theta = 0;
      stepPersegi = 1;
      targetJarak = 2.0;
      markX = 0; markY = 0; markTheta = 0;
      hitungInverseKinematics(0.35, 0.0);
      modePengujian = 5;
    } else if (m == '0') { // Stop Darurat
      modePengujian = 0;
      hitungInverseKinematics(0.0, 0.0);
    }
  }
}

// ====================== 11. SETUP =======================================
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
  waktuSebelumnya = millis();
  waktuMulai = millis();
  lastSerialCmdTime = millis();
}

// ====================== 12. LOOP UTAMA (20 Hz / 50 ms) ==================
void loop() {
  // A. Pembacaan Serial Non-Blocking
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

  // B. Loop Sampling Kendali & Odometri
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;

    // Salin Pulsa Enkoder Atomik
    noInterrupts();
    long dC_L = deltaPulsa_L; deltaPulsa_L = 0;
    long dC_R = deltaPulsa_R; deltaPulsa_R = 0;
    long tot_L = totalPulsa_L;
    long tot_R = totalPulsa_R;
    interrupts();

    // 1. Kecepatan Putar Aktual & Filter EMA
    float raw_rpm_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    float raw_rpm_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);

    rpm_L = (ALPHA_EMA * raw_rpm_L) + ((1.0 - ALPHA_EMA) * rpm_L);
    rpm_R = (ALPHA_EMA * raw_rpm_R) + ((1.0 - ALPHA_EMA) * rpm_R);

    // 2. Integrasi Odometri Dead-Reckoning (Sub-bab 2.2.11)[cite: 5]
    float ds_L = ((float)dC_L / CPR_WHEEL) * (PI * WHEEL_DIAMETER); // [Persamaan 24][cite: 5]
    float ds_R = ((float)dC_R / CPR_WHEEL) * (PI * WHEEL_DIAMETER); // [Persamaan 23][cite: 5]

    float dx_robot = (ds_R + ds_L) / 2.0;                           // [Persamaan 25][cite: 5]
    float dtheta   = (ds_R - ds_L) / WHEEL_BASE_B;                  // [Persamaan 26][cite: 5]

    float theta_mid = odom_theta + (dtheta / 2.0);
    odom_theta += dtheta;                                           // [Persamaan 31][cite: 5]
    odom_x += dx_robot * cos(theta_mid);                            // [Persamaan 32][cite: 5]
    odom_y += dx_robot * sin(theta_mid);                            // [Persamaan 33][cite: 5]

    // Normalisasi Sudut (-PI s/d PI)
    while (odom_theta > PI)  odom_theta -= (2.0 * PI);
    while (odom_theta < -PI) odom_theta += (2.0 * PI);

    // 3. State Machine Pengujian Mandiri Skripsi
    if (modePengujian == 1) { // Lurus 2 Meter[cite: 5]
      float dist = sqrt(odom_x * odom_x + odom_y * odom_y);
      if (dist >= targetJarak) { 
        hitungInverseKinematics(0, 0); 
        modePengujian = 0; 
      }
    } else if (modePengujian >= 2 && modePengujian <= 4) { // Rotasi di Tempat[cite: 5]
      if (abs(odom_theta) >= targetSudut) { 
        hitungInverseKinematics(0, 0); 
        modePengujian = 0; 
      }
    } else if (modePengujian == 5) { // Persegi Tertutup (4 Sisi)[cite: 5]
      if (stepPersegi % 2 == 1) { // Sisi Lurus 2m
        float dSegmen = sqrt(pow(odom_x - markX, 2) + pow(odom_y - markY, 2));
        if (dSegmen >= targetJarak) {
          markTheta = odom_theta;
          hitungInverseKinematics(0.0, 0.60); // Mulai belok 90 deg
          stepPersegi++;
        }
      } else { // Belok Siku 90 Derajat
        float diffTheta = abs(odom_theta - markTheta);
        if (diffTheta > PI) diffTheta = (2.0 * PI) - diffTheta;

        if (diffTheta >= (90.0 * PI / 180.0)) {
          markX = odom_x; markY = odom_y;
          if (stepPersegi >= 8) { 
            hitungInverseKinematics(0, 0); 
            modePengujian = 0; 
          } else { 
            hitungInverseKinematics(0.35, 0.0); 
            stepPersegi++; 
          }
        }
      }
    }

    // 4. Watchdog Proteksi Anti-Runaway Nav2
    if (modePengujian == 99 && (waktuSekarang - lastSerialCmdTime > SERIAL_WATCHDOG_TIMEOUT_MS)) {
      setpoint_L = 0.0; setpoint_R = 0.0;
      integral_L = 0.0; integral_R = 0.0;
      pwm_L = 0.0; pwm_R = 0.0;
      driveMotors(0, 0);
    }

    // 5. Eksekusi Komputasi PI Clamping PSO-ITSE
    pwm_L = computePIClamped(setpoint_L, rpm_L, KP_L, KI_L, KD_L, integral_L, prev_error_L, DT);
    pwm_R = computePIClamped(setpoint_R, rpm_R, KP_R, KI_R, KD_R, integral_R, prev_error_R, DT);

    driveMotors(pwm_L, pwm_R);

    // 6. Transmisi Data ke ROS 2 (/odom)
    // Sesuai dengan pembacaan 'ENC:' pada arduino_bridge.py
    Serial.print("ENC:");
    Serial.print(tot_L); Serial.print(",");
    Serial.println(tot_R);

    // 7. Transmisi Data Telemetri & Skripsi
    // Sesuai dengan urutan parts[1..4] pada arduino_bridge.py
    // parts[0]=t, parts[1]=rpm_L, parts[2]=rpm_R, parts[3]=pwm_L, parts[4]=pwm_R
    float t_sec = (float)(waktuSekarang - waktuMulai) / 1000.0;
    Serial.print("LOG:");
    Serial.print(t_sec, 2);          Serial.print(","); // parts[0]
    Serial.print(rpm_L, 1);          Serial.print(","); // parts[1] (actual_rpm_l)
    Serial.print(rpm_R, 1);          Serial.print(","); // parts[2] (actual_rpm_r)
    Serial.print((int)pwm_L);        Serial.print(","); // parts[3] (pwm_l)
    Serial.print((int)pwm_R);        Serial.print(","); // parts[4] (pwm_r)
    Serial.print(odom_x, 3);         Serial.print(","); // parts[5] (odom_x)
    Serial.print(odom_y, 3);         Serial.print(","); // parts[6] (odom_y)
    Serial.println(odom_theta * 180.0 / PI, 2);         // parts[7] (odom_theta)
  }
}