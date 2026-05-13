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
// zk = tekanan terukur; xk = estimasi tekanan presisi
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

const float RHO_AIR   = 1.225;  // kg/m3, densitas udara standar
float pressure_offset = 0.0;    // selisih pt-ps saat diam (zeroing)

const int BUTTON_PIN    = 2;
bool lastButtonState    = HIGH;

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
  Wire.write(0x27);  // normal mode, oversampling x1
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
// Kalman Update — satu langkah rekursif
// ============================================================
float updateKalman(KalmanFilter &kf, float zk) {
  // Prediksi error kovarians
  kf.error_estimate += kf.q;
  // Kalman Gain
  float Kk = kf.error_estimate / (kf.error_estimate + kf.error_measure);
  // Update estimasi state
  kf.estimate += Kk * (zk - kf.estimate);
  // Update error kovarians
  kf.error_estimate = (1.0 - Kk) * kf.error_estimate;
  return kf.estimate;
}

// ============================================================
// Hitung airspeed dari Bernoulli: v = sqrt(2*deltaP / rho)
// deltaP = pt - ps dikurangi offset saat diam
// ============================================================
float calculateSpeed(float pt, float ps) {
  float deltaP = (pt - ps) - pressure_offset;
  if (abs(deltaP) < 0.5) return 0.0;
  return sqrt((2.0 * abs(deltaP)) / RHO_AIR) * 3.6;  // km/h
}

// ============================================================
// Kalibrasi / Zeroing: rekam selisih pt-ps saat angin = 0
// Sekaligus inisialisasi estimasi Kalman dengan nilai awal nyata
// ============================================================
void doCalibration(int samples) {
  float sum_diff = 0;
  for (int i = 0; i < samples; i++) {
    readTemperature(0x76, cal_pt, 0);
    readTemperature(0x77, cal_ps, 0);
    float pt = readPressure(0x76, cal_pt);
    float ps = readPressure(0x77, cal_ps);
    sum_diff += (pt - ps);
    // Primasikan estimasi Kalman di iterasi pertama
    if (i == 0) {
      kf_pt.estimate = pt;
      kf_ps.estimate = ps;
      kf_pt.error_estimate = 2.0;
      kf_ps.error_estimate = 2.0;
    }
    delay(50);
  }
  pressure_offset = sum_diff / (float)samples;
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

  Serial.println("Calibrating... Please wait.");
  doCalibration(20);

  Serial.println("=== System Ready. Tekan tombol untuk Re-zero. ===");
  Serial.println();
  // Header kolom — kompatibel dengan Serial Plotter
  Serial.println("S1_Pt_Raw[Pa] | S1_Pt_Filt[Pa] | S2_Ps_Raw[Pa] | S2_Ps_Filt[Pa] | Temp[C] | V_Raw[km/h] | V_Filt[km/h]");
  Serial.println("-------------------------------------------------------------------------------------------------------");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // --- Tombol: Re-zero ---
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    Serial.println("\n>>> Re-zeroing... <<<");
    doCalibration(10);
    Serial.print("New Offset: "); Serial.println(pressure_offset, 2);
    Serial.println();
    delay(200);
  }
  lastButtonState = currentButtonState;

  // ==========================================================
  // Sensor 1 (0x76) — TOTAL PRESSURE (menghadap aliran udara)
  // ==========================================================
  readTemperature(0x76, cal_pt, -1.5);       // update t_fine agar readPressure akurat
  float pt_raw  = readPressure(0x76, cal_pt);
  float pt_filt = updateKalman(kf_pt, pt_raw);

  // ==========================================================
  // Sensor 2 (0x77) — STATIC PRESSURE + SUHU (tegak lurus aliran)
  // ==========================================================
  float temp    = readTemperature(0x77, cal_ps, -8.89);
  float ps_raw  = readPressure(0x77, cal_ps);
  float ps_filt = updateKalman(kf_ps, ps_raw);

  // ==========================================================
  // Kalkulasi Airspeed — raw (dari tekanan mentah) &
  //                      filtered (dari tekanan ter-Kalman)
  // ==========================================================
  float v_raw  = calculateSpeed(pt_raw,  ps_raw);
  float v_filt = calculateSpeed(pt_filt, ps_filt);

  // ==========================================================
  // Output Serial — data per sensor + kecepatan
  // Format: label:nilai (mudah dibaca, bisa di-plot)
  // ==========================================================
  Serial.print("S1_Pt_Raw:");   Serial.print(pt_raw,  2); Serial.print("\t");
  Serial.print("S1_Pt_Filt:");  Serial.print(pt_filt, 2); Serial.print("\t");
  Serial.print("S2_Ps_Raw:");   Serial.print(ps_raw,  2); Serial.print("\t");
  Serial.print("S2_Ps_Filt:");  Serial.print(ps_filt, 2); Serial.print("\t");
  Serial.print("Temp:");        Serial.print(temp,    2); Serial.print("\t");
  Serial.print("V_Raw:");       Serial.print(v_raw,   2); Serial.print("\t");
  Serial.print("V_Filt:");      Serial.println(v_filt, 2);

  delay(50);
}
