// =========================================================================
// PENGAMBILAN DATA STEP RESPONSE IDENTIFIKASI SISTEM MOTOR DC PG36
// SKRIPSI TEKNIK KOMPUTER - PROFIL STEP LINIER PWM 40 -> 160
// =========================================================================

// --- DEFINISI PIN SESUAI KONFIGURASI WIRING ANDA ---
// 1. Encoder Motor Kiri
const int PIN_ENC_L_A = 2;   // Arduino D2 (External Interrupt INT0)
const int PIN_ENC_L_B = 4;   // Arduino D4

// 2. Encoder Motor Kanan
const int PIN_ENC_R_A = 3;   // Arduino D3 (External Interrupt INT1)
const int PIN_ENC_R_B = A1;  // Arduino A1 (Digunakan sebagai Digital Input)

// 3. Driver BTS7960 Motor Kiri
const int PIN_RPWM_L  = 5;   // Arduino D5 (PWM)
const int PIN_LPWM_L  = 6;   // Arduino D6 (PWM)
const int PIN_REN_L   = 7;   // Arduino D7 (Enable Kiri)
const int PIN_LEN_L   = 8;   // Arduino D8 (Enable Kiri)

// 4. Driver BTS7960 Motor Kanan
const int PIN_RPWM_R  = 9;   // Arduino D9 (PWM)
const int PIN_LPWM_R  = 10;  // Arduino D10 (PWM)
const int PIN_REN_R   = 11;  // Arduino D11 (Enable Kanan)
const int PIN_LEN_R   = 12;  // Arduino D12 (Enable Kanan)

// --- PARAMETER ENKODER & WAKTU EKSEKUSI ---
const float PPR_OUTPUT = 350.7;        // 7 PPR x 50.1 Gearbox = 350.7 Ticks/Putaran
const unsigned long SAMPLING_US = 20000; // 20.000 mikrodetik = 20 ms (50 Hz)

// Variabel Pencacah Pulsa Encoder
volatile long enc_ticks_L = 0;
volatile long enc_ticks_R = 0;

// Variabel Waktu Eksperimen
unsigned long t_start_us = 0;
unsigned long t_last_us  = 0;
bool proses_selesai = false;

// =========================================================================
// ROUTINE INTERRUPT ENCODER (QUADRATURE READING)
// =========================================================================
void ISR_Encoder_Kiri() {
  // Jika saat Channel A RISING, Channel B bernilai HIGH -> Maju (ticks++)
  if (digitalRead(PIN_ENC_L_B) == HIGH) {
    enc_ticks_L++;
  } else {
    enc_ticks_L--;
  }
}

void ISR_Encoder_Kanan() {
  // Motor kanan terbalik secara fisik di sasis (Invert Encoder R)
  if (digitalRead(PIN_ENC_R_B) == HIGH) {
    enc_ticks_R--;
  } else {
    enc_ticks_R++;
  }
}

// =========================================================================
// FUNGSI PENGGERAK MOTOR BTS7960
// =========================================================================
void setMotorKiri(int pwm_val) {
  if (pwm_val > 0) {
    analogWrite(PIN_RPWM_L, pwm_val); // Maju
    analogWrite(PIN_LPWM_L, 0);
  } else if (pwm_val < 0) {
    analogWrite(PIN_RPWM_L, 0);
    analogWrite(PIN_LPWM_L, abs(pwm_val)); // Mundur
  } else {
    analogWrite(PIN_RPWM_L, 0);
    analogWrite(PIN_LPWM_L, 0);
  }
}

void setMotorKanan(int pwm_val) {
  // Motor Kanan (INVERTED): Maju = LPWM aktif, RPWM nol
  if (pwm_val > 0) {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, pwm_val);
  } else if (pwm_val < 0) {
    analogWrite(PIN_RPWM_R, abs(pwm_val));
    analogWrite(PIN_LPWM_R, 0);
  } else {
    analogWrite(PIN_RPWM_R, 0);
    analogWrite(PIN_LPWM_R, 0);
  }
}

void aktifkanDriver(bool state) {
  if (state) {
    digitalWrite(PIN_REN_L, HIGH);
    digitalWrite(PIN_LEN_L, HIGH);
    digitalWrite(PIN_REN_R, HIGH);
    digitalWrite(PIN_LEN_R, HIGH);
  } else {
    digitalWrite(PIN_REN_L, LOW);
    digitalWrite(PIN_LEN_L, LOW);
    digitalWrite(PIN_REN_R, LOW);
    digitalWrite(PIN_LEN_R, LOW);
  }
}

// =========================================================================
// SETUP PERANGKAT KERAS
// =========================================================================
void setup() {
  Serial.begin(115200);

  // Inisialisasi Pin Encoder
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Pasang External Interrupt pada pin D2 dan D3
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), ISR_Encoder_Kiri, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), ISR_Encoder_Kanan, RISING);

  // Inisialisasi Pin Driver BTS7960
  pinMode(PIN_RPWM_L, OUTPUT);
  pinMode(PIN_LPWM_L, OUTPUT);
  pinMode(PIN_REN_L, OUTPUT);
  pinMode(PIN_LEN_L, OUTPUT);

  pinMode(PIN_RPWM_R, OUTPUT);
  pinMode(PIN_LPWM_R, OUTPUT);
  pinMode(PIN_REN_R, OUTPUT);
  pinMode(PIN_LEN_R, OUTPUT);

  // Matikan motor dan aktifkan enable driver
  setMotorKiri(0);
  setMotorKanan(0);
  aktifkanDriver(true);

  // Hitung mundur 5 detik untuk bersiap meletakkan robot di lantai
  Serial.println(F("# Standby... Robot mulai bergerak dalam 5 detik!"));
  for (int i = 5; i > 0; i--) {
    Serial.print(F("# "));
    Serial.println(i);
    delay(1000);
  }

  // Header format CSV
  Serial.println(F("t_sec;pwm_input;ticks_L;ticks_R;rpm_L;rpm_R"));

  t_start_us = micros();
  t_last_us  = t_start_us;
}

// =========================================================================
// LOOP EKSEKUSI PENGAMBILAN DATA (50 Hz / 20 ms)
// =========================================================================
void loop() {
  if (proses_selesai) return;

  unsigned long current_us = micros();

  // Loop diskrit presisi tinggi berbasis micros()
  if (current_us - t_last_us >= SAMPLING_US) {
    float dt = (current_us - t_last_us) / 1000000.0; // detik
    float t_sec = (current_us - t_start_us) / 1000000.0;
    t_last_us = current_us;

    // Ambil dan reset pulsa tick secara atomic
    noInterrupts();
    long delta_ticks_L = enc_ticks_L;
    long delta_ticks_R = enc_ticks_R;
    enc_ticks_L = 0;
    enc_ticks_R = 0;
    interrupts();

    // Hitung Kecepatan RPM Aktual
    float rpm_L = (abs(delta_ticks_L) / PPR_OUTPUT) * (60.0 / dt);
    float rpm_R = (abs(delta_ticks_R) / PPR_OUTPUT) * (60.0 / dt);

    // Jadwal Profil Sinyal PWM (40 -> 160)
    int pwm_target = 0;
    if (t_sec < 1.0) {
      pwm_target = 0;    // Detik 0 s.d 1 : Diam
    } else if (t_sec < 3.0) {
      pwm_target = 40;   // Detik 1 s.d 3 : Garis dasar stabil (lepas zona mati)
    } else if (t_sec <= 6.0) {
      pwm_target = 160;  // Detik 3 s.d 6 : Hentakan step linier
    } else {
      // Selesai pengujian
      setMotorKiri(0);
      setMotorKanan(0);
      aktifkanDriver(false);
      proses_selesai = true;
      Serial.println(F("# PENGUJIAN SELESAI. Silakan copy data CSV di atas."));
      return;
    }

    // Terapkan PWM ke motor fisik
    setMotorKiri(pwm_target);
    setMotorKanan(pwm_target);

    // Cetak Data ke Serial Monitor (Format CSV dengan Delimiter ';')
    Serial.print(t_sec, 3);
    Serial.print(';');
    Serial.print(pwm_target);
    Serial.print(';');
    Serial.print(delta_ticks_L);
    Serial.print(';');
    Serial.print(delta_ticks_R);
    Serial.print(';');
    Serial.print(rpm_L, 2);
    Serial.print(';');
    Serial.println(rpm_R, 2);
  }
}