#include <Wire.h>

struct BMP280_Calib {
  uint16_t T1; int16_t T2, T3;
  uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
  int32_t t_fine;
};

struct KalmanFilter {
  float estimate = 0;       
  float error_estimate = 2; 
  float error_measure = 0.5; 
  float q = 0.01;           
};

BMP280_Calib cal1, cal2;
KalmanFilter kf_speed;

const float RHO_AIR = 1.225;
float pressure_offset = 0; 

// --- VARIABEL TOMBOL ---
const int BUTTON_PIN = 2;     // Tombol di Pin D2
bool systemOn = false;        // Status sistem
bool lastButtonState = HIGH;  // Untuk deteksi pencetan (debounce sederhana)

// --- DEKLARASI FUNGSI ---

void readCalibrationData(uint8_t addr, BMP280_Calib &c) {
  uint8_t d[24];
  Wire.beginTransmission(addr);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)24);
  for (int i = 0; i < 24; i++) d[i] = Wire.read();
  c.T1 = (d[1] << 8) | d[0]; c.T2 = (d[3] << 8) | d[2]; c.T3 = (d[5] << 8) | d[4];
  c.P1 = (d[7] << 8) | d[6]; c.P2 = (d[9] << 8) | d[8]; c.P3 = (d[11] << 8) | d[10];
  c.P4 = (d[13] << 8) | d[12]; c.P5 = (d[15] << 8) | d[14]; c.P6 = (d[17] << 8) | d[16];
  c.P7 = (d[19] << 8) | d[18]; c.P8 = (d[21] << 8) | d[20]; c.P9 = (d[23] << 8) | d[22];
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
  int32_t adc = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | ((uint32_t)Wire.read() >> 4);
  int32_t v1 = ((((adc >> 3) - ((int32_t)c.T1 << 1))) * ((int32_t)c.T2)) >> 11;
  int32_t v2 = (((((adc >> 4) - ((int32_t)c.T1)) * ((adc >> 4) - ((int32_t)c.T1))) >> 12) * ((int32_t)c.T3)) >> 14;
  c.t_fine = v1 + v2;
  return (float)((c.t_fine * 5 + 128) >> 8) / 100.0 + offset; 
}

float readPressure(uint8_t addr, BMP280_Calib &c) {
  Wire.beginTransmission(addr);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)3);
  int32_t adc = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | ((uint32_t)Wire.read() >> 4);
  int64_t v1 = (int64_t)c.t_fine - 128000;
  int64_t v2 = v1 * v1 * (int64_t)c.P6;
  v2 = v2 + ((v1 * (int64_t)c.P5) << 17);
  v2 = v2 + ((int64_t)c.P4 << 35);
  v1 = ((v1 * v1 * (int64_t)c.P3) >> 8) + ((v1 * (int64_t)c.P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)c.P1) >> 33;
  if (v1 == 0) return 0;
  int64_t p = 1048576 - adc;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)c.P9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (((int64_t)c.P8) * p) >> 19;
  return (float)(((p + v1 + v2) >> 8) + ((int64_t)c.P7 << 4)) / 256.0;
}

float updateKalman(KalmanFilter &kf, float measurement) {
  kf.error_estimate = kf.error_estimate + kf.q;
  float kalman_gain = kf.error_estimate / (kf.error_estimate + kf.error_measure);
  kf.estimate = kf.estimate + kalman_gain * (measurement - kf.estimate);
  kf.error_estimate = (1.0 - kalman_gain) * kf.error_estimate;
  return kf.estimate;
}

float calculateSpeed(float p1, float p2) {
  float deltaP = (p1 - p2) - pressure_offset;
  if (abs(deltaP) < 0.5) return 0; 
  return sqrt((2.0 * abs(deltaP)) / RHO_AIR) * 3.6; 
}

// --- MAIN FUNCTIONS ---

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); 

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Gunakan internal pullup

  initSensor(0x76, cal1);
  initSensor(0x77, cal2);
  
  Serial.println("Calibrating... Please wait.");
  float sum_diff = 0;
  for(int i=0; i<20; i++) {
    readTemperature(0x76, cal1, 0);
    readTemperature(0x77, cal2, 0);
    sum_diff += (readPressure(0x76, cal1) - readPressure(0x77, cal2));
    delay(50);
  }
  pressure_offset = sum_diff / 20.0;
  
  Serial.println("System Ready. Press Button to Toggle ON/OFF.");
}

void loop() {
  // --- LOGIKA TOMBOL (RESET / ZEROING) ---
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Jika tombol ditekan (LOW karena pakai PULLUP)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    Serial.println("Zeroing / Recalibrating...");
    
    float sum_diff = 0;
    for(int i = 0; i < 10; i++) {
      readTemperature(0x76, cal1, 0);
      readTemperature(0x77, cal2, 0);
      sum_diff += (readPressure(0x76, cal1) - readPressure(0x77, cal2));
      delay(20);
    }
    pressure_offset = sum_diff / 10.0; // Update offset baru
    
    // Reset juga estimasi Kalman agar tidak ada lonjakan dari data lama
    kf_speed.estimate = 0;
    
    Serial.print("New Offset: "); Serial.println(pressure_offset);
    delay(200); // Debounce
  }
  lastButtonState = currentButtonState;

  // --- LOGIKA PEMBACAAN (Jalan Terus) ---

  float t1 = readTemperature(0x76, cal1, -1.5);
  float p1 = readPressure(0x76, cal1);
  
  readTemperature(0x77, cal2, -8.89); 
  float p2 = readPressure(0x77, cal2);

  float raw_v = calculateSpeed(p1, p2);
  float filtered_v = updateKalman(kf_speed, raw_v);

  // Print data ke Serial
  Serial.print("Temp:"); Serial.print(t1); Serial.print(" ");
  Serial.print("Raw_V:"); Serial.print(raw_v); Serial.print(" ");
  Serial.print("Filtered_V:"); Serial.println(filtered_v);

  delay(50);
}
