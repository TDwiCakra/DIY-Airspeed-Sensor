#include <Wire.h>

// ============================================================
// Struktur kalibrasi BMP280
// ============================================================
struct BMP280_Calib {
  uint16_t T1; int16_t T2, T3;
  uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
  int32_t t_fine;
};

// ============================================================
// Kalman Filter — diterapkan terpisah per sensor (sesuai jurnal)
// ============================================================
struct KalmanFilter {
  float estimate;
  float error_estimate = 2.0;
  float error_measure  = 0.5;
  float q              = 0.01;
};

BMP280_Calib cal_pt;   // Sensor 1 (0x76) — Total Pressure
BMP280_Calib cal_ps;   // Sensor 2 (0x77) — Static Pressure + Suhu

KalmanFilter kf_pt;    // Kalman untuk total pressure (Sensor 1)
KalmanFilter kf_ps;    // Kalman untuk static pressure (Sensor 2)

// ============================================================
// Konstanta fisika — Persamaan (3) & (4) jurnal Chen et al. 2024
// ============================================================
const float R_GAS  = 8.314;
const float Ma_AIR = 0.028964;
const float Mv_H2O = 0.018016;
const float Z_COMP = 1.0;
const float K_CAL  = 1.0;
const float xv_DRY = 0.0;

float pressure_offset = 0.0;

// ============================================================
// Fuzzy Filter — parameter membership function
//
//   delta kecil (< DELTA_LO) → noise → smoothing berat (alpha kecil)
//   delta besar (> DELTA_HI) → sinyal nyata → smoothing ringan (alpha besar)
//   di antara keduanya → transisi linear
//
//   Aturan fuzzy:
//     IF delta IS small  THEN alpha = ALPHA_SMOOTH   (dominasi nilai lama)
//     IF delta IS large  THEN alpha = ALPHA_TRACK    (dominasi nilai baru)
//   Defuzzifikasi: weighted average (centroid)
// ============================================================
const float DELTA_LO     = 0.05;  // m/s — batas bawah: dianggap noise
const float DELTA_HI     = 0.40;  // m/s — batas atas: dianggap sinyal nyata
const float ALPHA_SMOOTH = 0.10;  // alpha saat noise (halus, lambat)
const float ALPHA_TRACK  = 0.90;  // alpha saat perubahan nyata (responsif)

float v_fuzzy = 0.0;              // state output fuzzy

const int BUTTON_PIN   = 2;
bool lastButtonState   = HIGH;

// ============================================================
// Fungsi BMP280
// ============================================================
void readCalibrationData(uint8_t addr, BMP280_Calib &c) {
  uint8_t d[24];
  Wire.beginTransmission(addr);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)24);
  for (int i = 0; i < 24; i++) d[i] = Wire.read();
  c.T1 = (d[1]<<8)|d[0]; c.T2 = (d[3]<<8)|d[2]; c.T3 = (d[5]<<8)|d[4];
  c.P1 = (d[7]<<8)|d[6]; c.P2 = (d[9]<<8)|d[8]; c.P3 = (d[11]<<8)|d[10];
  c.P4 = (d[13]<<8)|d[12]; c.P5 = (d[15]<<8)|d[14]; c.P6 = (d[17]<<8)|d[16];
  c.P7 = (d[19]<<8)|d[18]; c.P8 = (d[21]<<8)|d[20]; c.P9 = (d[23]<<8)|d[22];
}

void initSensor(uint8_t addr, BMP280_Calib &c) {
  Wire.beginTransmission(addr);
  Wire.write(0xF4);
  Wire.write(0x27);
  Wire.endTransmission();
  readCalibrationData(addr, c);
}

float readTemperature(uint8_t addr, BMP280_Calib &c, float offset) {
  Wire.beginTransmission(addr);
  Wire.write(0xFA);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)3);
  int32_t adc = ((uint32_t)Wire.read()<<12)|((uint32_t)Wire.read()<<4)|((uint32_t)Wire.read()>>4);
  int32_t v1 = ((((adc>>3) - ((int32_t)c.T1<<1))) * ((int32_t)c.T2)) >> 11;
  int32_t v2 = (((((adc>>4) - ((int32_t)c.T1)) * ((adc>>4) - ((int32_t)c.T1))) >> 12) * ((int32_t)c.T3)) >> 14;
  c.t_fine = v1 + v2;
  return (float)((c.t_fine * 5 + 128) >> 8) / 100.0 + offset;
}

float readPressure(uint8_t addr, BMP280_Calib &c) {
  Wire.beginTransmission(addr);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)3);
  int32_t adc = ((uint32_t)Wire.read()<<12)|((uint32_t)Wire.read()<<4)|((uint32_t)Wire.read()>>4);
  int64_t v1 = (int64_t)c.t_fine - 128000;
  int64_t v2 = v1 * v1 * (int64_t)c.P6;
  v2 = v2 + ((v1 * (int64_t)c.P5) << 17);
  v2 = v2 + ((int64_t)c.P4 << 35);
  v1 = ((v1 * v1 * (int64_t)c.P3) >> 8) + ((v1 * (int64_t)c.P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)c.P1) >> 33;
  if (v1 == 0) return 0;
  int64_t p = 1048576 - adc;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)c.P9) * (p>>13) * (p>>13)) >> 25;
  v2 = (((int64_t)c.P8) * p) >> 19;
  return (float)(((p + v1 + v2) >> 8) + ((int64_t)c.P7 << 4)) / 256.0;
}

// ============================================================
// Kalman Update
// ============================================================
float updateKalman(KalmanFilter &kf, float zk) {
  kf.error_estimate += kf.q;
  float Kk = kf.error_estimate / (kf.error_estimate + kf.error_measure);
  kf.estimate += Kk * (zk - kf.estimate);
  kf.error_estimate = (1.0 - Kk) * kf.error_estimate;
  return kf.estimate;
}

// ============================================================
// Hitung airspeed — Persamaan (4) jurnal Chen et al. 2024
//   v = K · sqrt( 2·Z·R·T·|deltaP| / (Ma·(1-xv·(1-Mv/Ma))·ps) )
// ============================================================
float calculateSpeed(float pt, float ps, float T_c) {
  float deltaP = (pt - ps) - pressure_offset;
  if (abs(deltaP) < 0.5) return 0.0;
  if (ps <= 0.0) return 0.0;
  float T_K = T_c + 273.15;
  float hum = 1.0 - xv_DRY * (1.0 - Mv_H2O / Ma_AIR);
  return K_CAL * sqrt((2.0 * Z_COMP * R_GAS * T_K * abs(deltaP))
                      / (Ma_AIR * hum * ps));  // m/s
}

// ============================================================
// Fuzzy Adaptive Filter
//
//   Membership functions (trapezoid/linear):
//     mu_small(d) = max(0, 1 - d/DELTA_LO)          — noise zone
//     mu_large(d) = max(0, min(1,(d-DELTA_LO)/(DELTA_HI-DELTA_LO))) — signal zone
//
//   Rules & defuzzifikasi (weighted average / centroid):
//     alpha = (mu_small*ALPHA_SMOOTH + mu_large*ALPHA_TRACK)
//             / (mu_small + mu_large)
//
//   Output:
//     v_out = alpha*v_new + (1-alpha)*v_prev
// ============================================================
float fuzzyFilter(float v_new, float v_prev) {
  float delta = abs(v_new - v_prev);

  // Derajat keanggotaan
  float mu_small = max(0.0f, 1.0f - delta / DELTA_LO);
  float mu_large = max(0.0f, min(1.0f, (delta - DELTA_LO) / (DELTA_HI - DELTA_LO)));

  float denom = mu_small + mu_large;
  float alpha;
  if (denom < 1e-6) {
    alpha = ALPHA_SMOOTH;  // default: noise suppression
  } else {
    alpha = (mu_small * ALPHA_SMOOTH + mu_large * ALPHA_TRACK) / denom;
  }

  return alpha * v_new + (1.0 - alpha) * v_prev;
}

// ============================================================
// Kalibrasi / Zeroing
// ============================================================
void doCalibration(int samples) {
  float sum_diff = 0;
  for (int i = 0; i < samples; i++) {
    readTemperature(0x76, cal_pt, 0);
    readTemperature(0x77, cal_ps, 0);
    float pt = readPressure(0x76, cal_pt);
    float ps = readPressure(0x77, cal_ps);
    sum_diff += (pt - ps);
    if (i == 0) {
      kf_pt.estimate      = pt;
      kf_ps.estimate      = ps;
      kf_pt.error_estimate = 2.0;
      kf_ps.error_estimate = 2.0;
    }
    delay(50);
  }
  pressure_offset = sum_diff / (float)samples;
  v_fuzzy = 0.0;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  initSensor(0x76, cal_pt);
  initSensor(0x77, cal_ps);

  Serial.println("Calibrating...");
  doCalibration(20);
  Serial.println("Ready. Airspeed [m/s]");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // Tombol: Re-zero
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    doCalibration(10);
    delay(200);
  }
  lastButtonState = currentButtonState;

  // Sensor 1 (0x76) — Total Pressure
  readTemperature(0x76, cal_pt, -1.5);
  float pt_filt = updateKalman(kf_pt, readPressure(0x76, cal_pt));

  // Sensor 2 (0x77) — Static Pressure + Suhu
  float temp    = readTemperature(0x77, cal_ps, -8.89);
  float ps_filt = updateKalman(kf_ps, readPressure(0x77, cal_ps));

  // Tahap 1: kecepatan dari Kalman pressure (Persamaan 4 jurnal)
  float v_kalman = calculateSpeed(pt_filt, ps_filt, temp);

  // Tahap 2: Fuzzy adaptive filter — adaptif terhadap besar perubahan
  v_fuzzy = fuzzyFilter(v_kalman, v_fuzzy);

  // Output satu nilai — kompatibel Serial Plotter
  Serial.println(v_fuzzy, 2);

  delay(50);
}
