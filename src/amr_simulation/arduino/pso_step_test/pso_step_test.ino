// =========================================================================
// PENGUJIAN FISIK MULTI-STEP DINAMIS: 0 -> 60 -> 80 -> 100 -> 60 -> 0 RPM
// METODE: STANDARD INTEGRAL CLAMPING ANTI-WINDUP (ANTI-DEADLOCK)
// DUAL MOTOR DC PG36 + BTS7960 (MOTOR KANAN INVERTED)
// OUTPUT SERIAL: Time_s,Target_RPM,RPM_L,RPM_R,PWM_L,PWM_R
// SAMPLING INTERVAL: 20 ms (50 Hz)
// =========================================================================

// --- 1. Definisi Pin Hardware ---
// BTS7960 Motor Kiri
const int PIN_RPWM_L = 5;    
const int PIN_LPWM_L = 6;    
const int PIN_REN_L  = 7;    
const int PIN_LEN_L  = 8;    

// BTS7960 Motor Kanan
const int PIN_RPWM_R = 9;    
const int PIN_LPWM_R = 10;   
const int PIN_REN_R  = 11;   
const int PIN_LEN_R  = 12;   

// Enkoder PG36 (7 PPR x 50.1:1 Gearbox)
const int PIN_ENC_A_L = 2;   // INT0
const int PIN_ENC_B_L = 4;   // D4
const int PIN_ENC_A_R = 3;   // INT1
const int PIN_ENC_B_R = A1;  // A1 Digital In

// --- 2. Parameter Fisik & PI PSO-ITSE ---
const float PULSES_PER_REV = 350.7; 

const float KP_L = 0.8624;
const float KI_L = 20.5540;
const float KD_L = 0.0000;

const float KP_R = 0.7037;
const float KI_R = 22.4384;
const float KD_R = 0.0000;

const float ALPHA_EMA = 0.35; 

// --- 3. Variabel Operasional ---
volatile long count_pulsa_L = 0;
volatile long count_pulsa_R = 0;

float rpm_filtered_L = 0.0;
float rpm_filtered_R = 0.0;

float integral_L = 0.0;
float integral_R = 0.0;
float prev_error_L = 0.0;
float prev_error_R = 0.0;

unsigned long prev_millis = 0;
unsigned long start_millis = 0;

// ISR Enkoder Kiri
void isr_encoder_L() {
  if (digitalRead(PIN_ENC_B_L) == HIGH) count_pulsa_L++;
  else count_pulsa_L--;
}

// ISR Enkoder Kanan (Inversi arah)
void isr_encoder_R() {
  if (digitalRead(PIN_ENC_B_R) == HIGH) count_pulsa_R--;
  else count_pulsa_R++;
}

// Prototipe Fungsi Kendali PI
int compute_pid_dynamic(float target, float current, float kp, float ki, float kd, float &integral, float &prev_err, float dt);

void setup() {
  Serial.begin(115200);

  // Inisialisasi Enkoder
  pinMode(PIN_ENC_A_L, INPUT_PULLUP);
  pinMode(PIN_ENC_B_L, INPUT_PULLUP);
  pinMode(PIN_ENC_A_R, INPUT_PULLUP);
  pinMode(PIN_ENC_B_R, INPUT_PULLUP);

  // Inisialisasi Driver Motor
  pinMode(PIN_RPWM_L, OUTPUT);
  pinMode(PIN_LPWM_L, OUTPUT);
  pinMode(PIN_REN_L, OUTPUT);
  pinMode(PIN_LEN_L, OUTPUT);

  pinMode(PIN_RPWM_R, OUTPUT);
  pinMode(PIN_LPWM_R, OUTPUT);
  pinMode(PIN_REN_R, OUTPUT);
  pinMode(PIN_LEN_R, OUTPUT);

  // Aktifkan Driver Motor
  digitalWrite(PIN_REN_L, HIGH);
  digitalWrite(PIN_LEN_L, HIGH);
  digitalWrite(PIN_REN_R, HIGH);
  digitalWrite(PIN_LEN_R, HIGH);

  // Pasang Interupsi
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_L), isr_encoder_L, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_R), isr_encoder_R, RISING);

  start_millis = millis();
}

void loop() {
  unsigned long current_millis = millis();

  // Eksekusi Sampling Loop 20 ms (50 Hz)
  if (current_millis - prev_millis >= 20) {
    float dt = (current_millis - prev_millis) / 1000.0;
    prev_millis = current_millis;

    float t = (current_millis - start_millis) / 1000.0;

    // Profil Target Multi-Step
    float setpoint = 0.0;
    if (t < 3.0) {
      setpoint = 60.0;
    } else if (t < 6.0) {
      setpoint = 80.0;
    } else if (t < 9.0) {
      setpoint = 100.0;
    } else if (t < 12.0) {
      setpoint = 60.0;
    } else {
      setpoint = 0.0;
    }

    // Pembacaan Pulsa Atomik
    noInterrupts();
    long pulsa_L = count_pulsa_L;
    long pulsa_R = count_pulsa_R;
    count_pulsa_L = 0;
    count_pulsa_R = 0;
    interrupts();

    // Perhitungan RPM Mentah
    float rpm_raw_L = (abs(pulsa_L) / PULSES_PER_REV) * (60.0 / dt);
    float rpm_raw_R = (abs(pulsa_R) / PULSES_PER_REV) * (60.0 / dt);

    // Filter EMA
    rpm_filtered_L = (ALPHA_EMA * rpm_raw_L) + ((1.0 - ALPHA_EMA) * rpm_filtered_L);
    rpm_filtered_R = (ALPHA_EMA * rpm_raw_R) + ((1.0 - ALPHA_EMA) * rpm_filtered_R);

    // Komputasi Kendali PI
    int pwm_L = compute_pid_dynamic(setpoint, rpm_filtered_L, KP_L, KI_L, KD_L, integral_L, prev_error_L, dt);
    int pwm_R = compute_pid_dynamic(setpoint, rpm_filtered_R, KP_R, KI_R, KD_R, integral_R, prev_error_R, dt);

    // Eksekusi PWM ke BTS7960
    analogWrite(PIN_RPWM_L, pwm_L);
    analogWrite(PIN_LPWM_L, 0);

    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, pwm_R);

    // Output Data Stream CSV
    Serial.print(t, 2);           Serial.print(",");
    Serial.print(setpoint, 1);    Serial.print(",");
    Serial.print(rpm_filtered_L); Serial.print(",");
    Serial.print(rpm_filtered_R); Serial.print(",");
    Serial.print(pwm_L);          Serial.print(",");
    Serial.println(pwm_R);

    // Berhenti Total Setelah Detik ke-14
    if (t >= 14.0) {
      analogWrite(PIN_RPWM_L, 0);
      analogWrite(PIN_LPWM_L, 0);
      analogWrite(PIN_RPWM_R, 0);
      analogWrite(PIN_LPWM_R, 0);
      while (true);
    }
  }
}

// Algoritma PI Anti-Windup Clamping (Bebas Deadlock)
int compute_pid_dynamic(float target, float current, float kp, float ki, float kd, float &integral, float &prev_err, float dt) {
  if (target <= 1.0) {
    integral = 0.0;
    prev_err = 0.0;
    return 0;
  }

  float error = target - current;

  // Deadband redam kuantisasi pulsa diskrit
  if (abs(error) < 0.5) {
    error = 0.0;
  }

  float P = kp * error;
  float D = kd * ((error - prev_err) / dt);
  prev_err = error;

  // Prediksi sinyal kendali tanpa saturasi
  float tentative_i = integral + (error * dt);
  float unsaturated_pwm = P + (ki * tentative_i) + D;

  // Clamping Anti-Windup: Integrasi hanya ditambah saat aktuator belum jenuh
  if (unsaturated_pwm < 255.0 && unsaturated_pwm > 0.0) {
    integral = tentative_i;
  }

  // Batasi akumulasi integral maksimal agar tidak melebihi rentang PWM 8-bit
  float max_integral = 255.0 / ki;
  integral = constrain(integral, 0.0, max_integral);

  float I = ki * integral;

  return constrain((int)(P + I + D), 0, 255);
}