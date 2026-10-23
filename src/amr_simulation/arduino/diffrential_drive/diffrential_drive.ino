/* =========================================================================
 * IMPLEMENTASI PID-PSO + INVERSE KINEMATICS + ODOMETRI DIFFERENTIAL DRIVE
 * SESUAI PROPOSAL SKRIPSI DIVALA ZAHRA OKTAVIA (UNNES 2026)
 * ========================================================================= */

// ====================== 1. PIN HARDWARE ==================================
const byte PIN_RPWM_L = 5;
const byte PIN_LPWM_L = 6;
const byte PIN_REN_L  = 7;
const byte PIN_LEN_L  = 8;

const byte PIN_RPWM_R = 9;
const byte PIN_LPWM_R = 10;
const byte PIN_REN_R  = 11;
const byte PIN_LEN_R  = 12;

const byte PIN_ENCA_L = 2; // Interrupt
const byte PIN_ENCB_L = 4;
const byte PIN_ENCA_R = 3; // Interrupt
const byte PIN_ENCB_R = A1;

const bool INVERT_MOTOR_KANAN = true;

// ====================== 2. PARAMETER FISIK ROBOT =========================
const float WHEEL_DIAMETER = 0.127; // D = 5 inci = 0.127 meter (Persamaan 23-24)
const float WHEEL_BASE_B   = 0.458; // 45.8 cm = 0.458 m
const float CPR_WHEEL      = 350.7; // 7 PPR * 50.1

// ====================== 3. PARAMETER PID-PSO =============================
const float KP_L = 3.0, KI_L = 3.0, KD_L = 0.0;
const float KP_R = 3.0, KI_R = 3.0, KD_R = 0.0;

const unsigned long SAMPLING_MS = 50; 
const float DT = SAMPLING_MS / 1000.0; 

// ====================== 4. VARIABEL ODOMETRI & KENDALI ===================
float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
volatile long countPulsa_L = 0, countPulsa_R = 0;

float setpoint_L = 0.0, setpoint_R = 0.0;
float rpm_L = 0.0, rpm_R = 0.0;
float error_L = 0.0, prev_error_L = 0.0, integral_L = 0.0, pwm_L = 0.0;
float error_R = 0.0, prev_error_R = 0.0, integral_R = 0.0, pwm_R = 0.0;
unsigned long waktuSebelumnya = 0;

// State Uji Mandiri
int modePengujian = 0; 
int stepPersegi = 0;
float targetJarak = 0.0, targetSudut = 0.0;

// Watchdog Failsafe (Anti-Runaway jika Jetson mati / kabel serial lepas)
const unsigned long SERIAL_WATCHDOG_TIMEOUT_MS = 500; // 0.5 detik
unsigned long lastSerialCmdTime = 0;

// ====================== 5. FUNGSI INVERSE KINEMATICS =====================
// Menghitung Setpoint RPM Roda dari Kecepatan Linear v (m/s) & Angular w (rad/s)
// Berdasarkan Persamaan (21) & (22) pada Proposal Skripsi Anda
void hitungInverseKinematics(float v, float omega) {
  // v_R = v + (B/2)*w [Persamaan 21]
  // v_L = v - (B/2)*w [Persamaan 22]
  float v_R = v + (WHEEL_BASE_B / 2.0) * omega; // m/s
  float v_L = v - (WHEEL_BASE_B / 2.0) * omega; // m/s

  // Konversi kecepatan linear roda (m/s) ke RPM roda: RPM = (v / (pi * D)) * 60
  setpoint_R = (v_R / (PI * WHEEL_DIAMETER)) * 60.0;
  setpoint_L = (v_L / (PI * WHEEL_DIAMETER)) * 60.0;
}

// ====================== 6. ISR ENCODER ===================================
void isrL() {
  if (digitalRead(PIN_ENCB_L) == HIGH) countPulsa_L++;
  else countPulsa_L--;
}
void isrR() {
  if (digitalRead(PIN_ENCB_R) == HIGH) countPulsa_R++;
  else countPulsa_R--;
}

// ====================== 7. DRIVER MOTOR ==================================
void setMotorDriver(byte rpwmPin, byte lpwmPin, float valPWM, bool invert) {
  int pwm = constrain(abs(valPWM), 0, 255);
  bool arahMaju = (valPWM >= 0);
  if (invert) arahMaju = !arahMaju;

  if (valPWM == 0) {
    analogWrite(rpwmPin, 0); analogWrite(lpwmPin, 0);
  } else if (arahMaju) {
    analogWrite(rpwmPin, pwm); analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0); analogWrite(lpwmPin, pwm);
  }
}

void driveMotors(float pwmL, float pwmR) {
  setMotorDriver(PIN_RPWM_L, PIN_LPWM_L, pwmL, false);
  setMotorDriver(PIN_RPWM_R, PIN_LPWM_R, pwmR, INVERT_MOTOR_KANAN);
}

// ====================== 8. SETUP =========================================
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

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_L), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA_R), isrR, RISING);

  driveMotors(0, 0);
  lastSerialCmdTime = millis();

  Serial.println("==================================================");
  Serial.println("  DIFF-DRIVE AMR: PID-PSO + INVERSE KINEMATICS    ");
  Serial.println("==================================================");
  Serial.println("PILIHAN INPUT:");
  Serial.println("1) Dari DWA Jetson -> Format: V,<linear_v>,<angular_w>");
  Serial.println("   Contoh Maju: V,0.3,0.0 | Belok Kanan: V,0.2,-0.5");
  Serial.println("2) Uji Mandiri Skripsi:");
  Serial.println("   1: Lurus 2m | 2: Putar 90 | 3: Putar 180 | 4: Putar 360 | 5: Persegi 2x2m | 0: Stop");
  Serial.println("==================================================");
  Serial.println("Time(s),RPM_L,RPM_R,PWM_L,PWM_R,X(m),Y(m),Theta(deg)");
  waktuSebelumnya = millis();
}

// ====================== 9. LOOP UTAMA ====================================
void loop() {
  // A. BACA PERINTAH DARI SERIAL (DWA JETSON / SERIAL MONITOR)
  if (Serial.available() > 0) {
    char c = Serial.peek();
    
    // Jika input berawalan 'V' atau 'v', berarti kiriman DWA: V,linear_v,angular_w
    if (c == 'V' || c == 'v') {
      Serial.read(); // buang huruf 'V'
      float v_lin = Serial.parseFloat();
      float v_ang = Serial.parseFloat();
      lastSerialCmdTime = millis(); // Refresh watchdog heartbeat dari Jetson
      modePengujian = 99; // Mode Navigasi Eksternal
      hitungInverseKinematics(v_lin, v_ang);
    } 
    // Jika input angka tunggal: Mode Uji Skenario Mandiri
    else {
      char cmd = Serial.read();
      if (cmd == '1') { // Skenario Lurus 2 Meter
        odom_x = 0; odom_y = 0; odom_theta = 0;
        targetJarak = 2.0;
        hitungInverseKinematics(0.33, 0.0); // v = 0.33 m/s (~50 RPM), w = 0
        modePengujian = 1;
      } else if (cmd == '2') { // Rotasi 90 Deg
        odom_x = 0; odom_y = 0; odom_theta = 0;
        targetSudut = 90.0 * (PI / 180.0);
        hitungInverseKinematics(0.0, 0.8); // v = 0, w = 0.8 rad/s
        modePengujian = 2;
      } else if (cmd == '3') { // Rotasi 180 Deg
        odom_x = 0; odom_y = 0; odom_theta = 0;
        targetSudut = 180.0 * (PI / 180.0);
        hitungInverseKinematics(0.0, 0.8);
        modePengujian = 3;
      } else if (cmd == '4') { // Rotasi 360 Deg
        odom_x = 0; odom_y = 0; odom_theta = 0;
        targetSudut = 360.0 * (PI / 180.0);
        hitungInverseKinematics(0.0, 0.8);
        modePengujian = 4;
      } else if (cmd == '5') { // Persegi 2m x 2m
        odom_x = 0; odom_y = 0; odom_theta = 0;
        stepPersegi = 1; targetJarak = 2.0;
        hitungInverseKinematics(0.33, 0.0);
        modePengujian = 5;
      } else if (cmd == '0') { // Stop
        modePengujian = 0;
        hitungInverseKinematics(0.0, 0.0);
      }
    }
  }

  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuSebelumnya >= SAMPLING_MS) {
    waktuSebelumnya = waktuSekarang;

    // B. Salin Pulsa Atomik
    noInterrupts();
    long dC_L = countPulsa_L; long dC_R = countPulsa_R;
    countPulsa_L = 0; countPulsa_R = 0;
    interrupts();

    // C. Hitung RPM
    rpm_L = ((float)dC_L / CPR_WHEEL) * (60.0 / DT);
    rpm_R = ((float)dC_R / CPR_WHEEL) * (60.0 / DT);

    // D. Hitung Odometri (Forward Kinematics)
    float ds_L = ((float)dC_L / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float ds_R = ((float)dC_R / CPR_WHEEL) * (PI * WHEEL_DIAMETER);
    float dx_robot = (ds_R + ds_L) / 2.0;
    float dtheta   = (ds_R - ds_L) / WHEEL_BASE_B;

    float theta_mid = odom_theta + (dtheta / 2.0);
    odom_theta += dtheta;
    odom_x += dx_robot * cos(theta_mid);
    odom_y += dx_robot * sin(theta_mid);

    // E. Penghentian Otomatis pada Mode Uji
    if (modePengujian == 1) { // Lurus 2m
      float dist = sqrt(odom_x * odom_x + odom_y * odom_y);
      if (dist >= targetJarak) {
        hitungInverseKinematics(0, 0); modePengujian = 0;
      }
    } else if (modePengujian >= 2 && modePengujian <= 4) { // Rotasi
      if (abs(odom_theta) >= targetSudut) {
        hitungInverseKinematics(0, 0); modePengujian = 0;
      }
    } else if (modePengujian == 5) { // Persegi 2m x 2m
      static float markX = 0, markY = 0, markTheta = 0;
      if (stepPersegi % 2 == 1) { // Lurus 2m
        float dSegmen = sqrt(pow(odom_x - markX, 2) + pow(odom_y - markY, 2));
        if (dSegmen >= 2.0) {
          markTheta = odom_theta;
          hitungInverseKinematics(0.0, 0.8); // Belok 90 deg
          stepPersegi++;
        }
      } else { // Belok 90 deg
        if (abs(odom_theta - markTheta) >= (90.0 * PI / 180.0)) {
          markX = odom_x; markY = odom_y;
          if (stepPersegi >= 8) { // Selesai 4 sisi
            hitungInverseKinematics(0, 0); modePengujian = 0;
          } else {
            hitungInverseKinematics(0.33, 0.0); // Lanjut lurus sisi berikutnya
            stepPersegi++;
          }
        }
      }
    }

    // FAILSAFE WATCHDOG: Jika mode Navigasi (99) dan tidak ada heartbeat serial > 0.5 detik (Jetson Ctrl+C / kabel putus)
    if (modePengujian == 99 && (waktuSekarang - lastSerialCmdTime > SERIAL_WATCHDOG_TIMEOUT_MS)) {
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

    // F. Loop PID Motor Kiri & Kanan
    if (fabs(setpoint_L) < 0.05) {
      pwm_L = 0.0;
      integral_L = 0.0;
      prev_error_L = 0.0;
    } else {
      error_L = setpoint_L - rpm_L;
      integral_L = constrain(integral_L + (KI_L * error_L * DT), -100.0, 255.0);
      pwm_L = constrain((KP_L * error_L) + integral_L + (KD_L * (error_L - prev_error_L) / DT), -255, 255);
      prev_error_L = error_L;
    }

    if (fabs(setpoint_R) < 0.05) {
      pwm_R = 0.0;
      integral_R = 0.0;
      prev_error_R = 0.0;
    } else {
      error_R = setpoint_R - rpm_R;
      integral_R = constrain(integral_R + (KI_R * error_R * DT), -100.0, 255.0);
      pwm_R = constrain((KP_R * error_R) + integral_R + (KD_R * (error_R - prev_error_R) / DT), -255, 255);
      prev_error_R = error_R;
    }

    driveMotors(pwm_L, pwm_R);

    // G. Output Serial Data
    Serial.print((float)waktuSekarang / 1000.0, 2); Serial.print(",");
    Serial.print(rpm_L, 1); Serial.print(",");
    Serial.print(rpm_R, 1); Serial.print(",");
    Serial.print(pwm_L, 0); Serial.print(",");
    Serial.print(pwm_R, 0); Serial.print(",");
    Serial.print(odom_x, 3); Serial.print(",");
    Serial.print(odom_y, 3); Serial.print(",");
    Serial.println(odom_theta * 180.0 / PI, 1);
  }
}