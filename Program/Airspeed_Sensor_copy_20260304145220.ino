#include <Wire.h>

struct BMP280_Calib {
  uint16_t T1; int16_t T2, T3;
  uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
  int32_t t_fine;
};

struct KalmanFilter {
  float estimate       = 0;
  float error_estimate = 2;
  float error_measure  = 0.5;
  float q              = 0.001;
};

#define ADDR_1       0x76
#define ADDR_2       0x77
#define BUTTON_PIN   2
#define DEADBAND_PA  10.0f
#define TEMP_OFF_1  -1.5f
#define TEMP_OFF_2  -8.89f

const float RHO_AIR = 1.225f;

BMP280_Calib cal1, cal2;
KalmanFilter kf;
float pressure_offset = 0;
bool lastButtonState  = HIGH;

// ── BMP280 ────────────────────────────────────────────────────────────────────

void readCalib(uint8_t addr, BMP280_Calib &c) {
  uint8_t d[24];
  Wire.beginTransmission(addr); Wire.write(0x88); Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)24);
  for (int i = 0; i < 24; i++) d[i] = Wire.read();
  c.T1 = (d[1]<<8)|d[0];  c.T2 = (d[3]<<8)|d[2];   c.T3 = (d[5]<<8)|d[4];
  c.P1 = (d[7]<<8)|d[6];  c.P2 = (d[9]<<8)|d[8];   c.P3 = (d[11]<<8)|d[10];
  c.P4 = (d[13]<<8)|d[12]; c.P5 = (d[15]<<8)|d[14]; c.P6 = (d[17]<<8)|d[16];
  c.P7 = (d[19]<<8)|d[18]; c.P8 = (d[21]<<8)|d[20]; c.P9 = (d[23]<<8)|d[22];
}

void initSensor(uint8_t addr, BMP280_Calib &c) {
  Wire.beginTransmission(addr);
  Wire.write(0xF4);
  Wire.write(0x27);
  Wire.endTransmission();
  readCalib(addr, c);
}

float readTemperature(uint8_t addr, BMP280_Calib &c, float offset) {
  Wire.beginTransmission(addr); Wire.write(0xFA); Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)3);
  int32_t adc = ((uint32_t)Wire.read()<<12) | ((uint32_t)Wire.read()<<4) | ((uint32_t)Wire.read()>>4);
  int32_t v1 = ((((adc>>3) - ((int32_t)c.T1<<1))) * ((int32_t)c.T2)) >> 11;
  int32_t v2 = (((((adc>>4) - ((int32_t)c.T1)) * ((adc>>4) - ((int32_t)c.T1))) >> 12) * ((int32_t)c.T3)) >> 14;
  c.t_fine = v1 + v2;
  return (float)((c.t_fine * 5 + 128) >> 8) / 100.0f + offset;
}

float readPressure(uint8_t addr, BMP280_Calib &c) {
  Wire.beginTransmission(addr); Wire.write(0xF7); Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)3);
  int32_t adc = ((uint32_t)Wire.read()<<12) | ((uint32_t)Wire.read()<<4) | ((uint32_t)Wire.read()>>4);
  int64_t v1 = (int64_t)c.t_fine - 128000;
  int64_t v2 = v1 * v1 * (int64_t)c.P6;
  v2 = v2 + ((v1 * (int64_t)c.P5) << 17);
  v2 = v2 + ((int64_t)c.P4 << 35);
  v1 = ((v1 * v1 * (int64_t)c.P3) >> 8) + ((v1 * (int64_t)c.P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)c.P1) >> 33;
  if (v1 == 0) return 0;
  int64_t p = 1048576 - adc;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = ((int64_t)c.P9 * (p >> 13) * (p >> 13)) >> 25;
  v2 = ((int64_t)c.P8 * p) >> 19;
  return (float)(((p + v1 + v2) >> 8) + ((int64_t)c.P7 << 4)) / 256.0f;
}

// ── Kalman filter ─────────────────────────────────────────────────────────────

float updateKalman(KalmanFilter &kf, float z) {
  kf.error_estimate += kf.q;
  float K = kf.error_estimate / (kf.error_estimate + kf.error_measure);
  kf.estimate += K * (z - kf.estimate);
  kf.error_estimate = (1.0f - K) * kf.error_estimate;
  return kf.estimate;
}

// ── Zeroing ───────────────────────────────────────────────────────────────────

void calibrateOffset(int samples, int interval_ms) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    readTemperature(ADDR_1, cal1, 0);
    readTemperature(ADDR_2, cal2, 0);
    sum += readPressure(ADDR_1, cal1) - readPressure(ADDR_2, cal2);
    delay(interval_ms);
  }
  pressure_offset = sum / samples;
  kf.estimate = 0;
}

// ── Bernoulli: v = sqrt(2|dP| / rho) ─────────────────────────────────────────

float calcSpeed(float p1, float p2) {
  float dP = (p1 - p2) - pressure_offset;
  if (abs(dP) < DEADBAND_PA) return 0.0f;
  return sqrt(2.0f * abs(dP) / RHO_AIR);
}

// ── Cek chip ID BMP280 (register 0xD0 harus return 0x58) ─────────────────────

bool checkSensor(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  uint8_t id = Wire.read();
  return (id == 0x58 || id == 0x60);  // 0x60 = BME280
}

// ── Setup & loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.print(F("Sensor 0x76 (total):  "));
  if (checkSensor(ADDR_1)) { Serial.println(F("OK")); initSensor(ADDR_1, cal1); }
  else                      { Serial.println(F("TIDAK TERDETEKSI - cek wiring!")); }

  Serial.print(F("Sensor 0x77 (static): "));
  if (checkSensor(ADDR_2)) { Serial.println(F("OK")); initSensor(ADDR_2, cal2); }
  else                      { Serial.println(F("TIDAK TERDETEKSI - cek wiring!")); }

  Serial.println(F("Calibrating..."));
  calibrateOffset(20, 50);
  Serial.print(F("Offset: ")); Serial.println(pressure_offset, 3);
  Serial.println(F("Ready. Tekan tombol untuk zeroing ulang."));
  Serial.println();
  Serial.println(F("dP(Pa)\tRaw_V(m/s)\tFiltered_V(m/s)"));
}

void loop() {
  bool btn = digitalRead(BUTTON_PIN);
  if (btn == LOW && lastButtonState == HIGH) {
    Serial.println(F("Zeroing..."));
    calibrateOffset(10, 20);
    Serial.print(F("New offset: ")); Serial.println(pressure_offset, 3);
    delay(200);
  }
  lastButtonState = btn;

  readTemperature(ADDR_1, cal1, TEMP_OFF_1);
  float p1 = readPressure(ADDR_1, cal1);

  readTemperature(ADDR_2, cal2, TEMP_OFF_2);
  float p2 = readPressure(ADDR_2, cal2);

  float dP         = (p1 - p2) - pressure_offset;
  float raw_v      = calcSpeed(p1, p2);
  float filtered_v = updateKalman(kf, raw_v);

  Serial.print(dP, 2);         Serial.print(F("\t"));
  Serial.print(raw_v, 2);      Serial.print(F("\t\t"));
  Serial.println(filtered_v, 2);

  delay(50);
}
