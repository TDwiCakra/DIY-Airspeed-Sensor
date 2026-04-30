# Airspeed Sensor - Sistem Pengukur Kecepatan Udara

![Version](https://img.shields.io/badge/version-1.0-blue)
![Arduino](https://img.shields.io/badge/Arduino-Compatible-green)
![Platform](https://img.shields.io/badge/platform-Arduino-orange)

Sistem pengukur kecepatan udara berbasis prinsip Pitot Tube menggunakan dua sensor barometer BMP280, dilengkapi dengan Kalman Filter untuk pembacaan yang stabil dan akurat.

Note:
- Pastikan kedua sensor BMP280 terpasang dengan benar sebelum upload program
- Lakukan kalibrasi awal (zeroing) dengan menekan tombol saat kondisi angin tenang / tidak ada aliran udara


## 💡 Cara Kerja

Sistem ini mengukur kecepatan udara menggunakan prinsip **Pitot Tube** yang memanfaatkan perbedaan tekanan statis dan tekanan dinamis:

```
                    ┌──────────────┐
   Aliran Udara →   │   BMP280 #1  │  (Stagnation / Dynamic Pressure)
                    └──────────────┘
                          vs
                    ┌──────────────┐
   Aliran Udara →   │   BMP280 #2  │  (Static Pressure)
                    └──────────────┘
                           ↓
              Hitung ΔP = P1 - P2 - Offset
                           ↓
              v = √(2 × |ΔP| / ρ) × 3.6  [km/h]
                           ↓
              Output difilter dengan Kalman Filter
```

Kecepatan dihitung menggunakan persamaan Bernoulli, lalu diperhalus dengan **Kalman Filter** untuk meredam noise sensor.

---

## 🌟 Fitur Utama

- ✅ Pengukuran kecepatan udara real-time (km/h)
- ✅ Dual BMP280 sensor (I2C address 0x76 & 0x77)
- ✅ Kalman Filter untuk output yang stabil dan minim noise
- ✅ Kalibrasi otomatis offset tekanan saat startup (20 sampel)
- ✅ Fungsi Zeroing / Re-kalibrasi via tombol tekan
- ✅ Deadzone threshold (< 0.5 Pa = 0 km/h) untuk menghindari noise saat diam
- ✅ Offset koreksi suhu per sensor
- ✅ Output Serial Monitor lengkap (Suhu, Raw Speed, Filtered Speed)

---

## 🛠 Hardware Requirements

| Komponen | Spesifikasi | Jumlah |
|----------|-------------|--------|
| **Microcontroller** | Arduino (Uno/Nano/Mega) | 1 |
| **Pressure Sensor** | BMP280 (I2C, address 0x76) | 1 |
| **Pressure Sensor** | BMP280 (I2C, address 0x77) | 1 |
| **Push Button** | Tactile Switch / Push Button | 1 |
| **Resistor** | 10kΩ (opsional, sudah ada internal pullup) | 1 |
| **Pitot Tube** | Tabung pitot untuk mengarahkan angin | 1 |
| **Breadboard/PCB** | Untuk koneksi | 1 |
| **Kabel Jumper** | Male-Female, Male-Male | Secukupnya |

---

## 📥 Instalasi Arduino IDE

### Windows

1. **Download Arduino IDE**
   - Kunjungi [Arduino Official Website](https://www.arduino.cc/en/software)
   - Download versi terbaru (2.x.x atau 1.8.x)
   - Pilih "Windows Installer" (untuk instalasi mudah)

2. **Install Arduino IDE**
   - Jalankan file installer (.exe)
   - Ikuti wizard instalasi
   - Centang "Install USB Driver" saat instalasi
   - Klik "Finish" setelah selesai

3. **Buka Arduino IDE**
   - Jalankan Arduino IDE dari Start Menu
   - IDE siap digunakan

### macOS

1. **Download Arduino IDE**
   - Kunjungi [Arduino Official Website](https://www.arduino.cc/en/software)
   - Download versi untuk macOS

2. **Install Arduino IDE**
   - Buka file .dmg yang didownload
   - Drag aplikasi Arduino ke folder Applications
   - Buka Arduino IDE dari Applications

### Linux (Ubuntu/Debian)

```bash
# Download dan install Arduino IDE
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_latest_Linux_64bit.AppImage

# Berikan permission untuk execute
chmod +x arduino-ide_latest_Linux_64bit.AppImage

# Jalankan Arduino IDE
./arduino-ide_latest_Linux_64bit.AppImage
```

Atau install via snap:

```bash
sudo snap install arduino
```

---

## 📚 Instalasi Library

Program ini hanya menggunakan library bawaan Arduino sehingga **tidak memerlukan instalasi library tambahan**!

Library yang digunakan:

| Library | Keterangan | Status |
|---------|------------|--------|
| `Wire.h` | Komunikasi I2C | ✅ Built-in Arduino |

### Verifikasi Library Terinstall

```cpp
// Buka Arduino IDE dan coba compile sketch ini:
#include <Wire.h>

void setup() {}
void loop() {}
```

Jika tidak ada error, library sudah siap digunakan!

---

## 🔧 Konfigurasi Hardware

### Wiring Diagram

#### I2C Connection (BMP280 #1 & BMP280 #2)

```
Arduino UNO      BMP280 #1 (0x76)
--------------------------
5V / 3.3V  -----> VCC
GND        -----> GND
A4 (SDA)   -----> SDA
A5 (SCL)   -----> SCL
SDO        -----> GND   ← untuk set alamat 0x76

Arduino UNO      BMP280 #2 (0x77)
--------------------------
5V / 3.3V  -----> VCC
GND        -----> GND
A4 (SDA)   -----> SDA  (parallel dengan BMP280 #1)
A5 (SCL)   -----> SCL  (parallel dengan BMP280 #1)
SDO        -----> VCC  ← untuk set alamat 0x77
```

**⚠️ Penting:** Pin `SDO` pada BMP280 digunakan untuk mengatur alamat I2C:
- `SDO → GND` = alamat **0x76**
- `SDO → VCC` = alamat **0x77**

#### Tombol Zeroing

```
Arduino UNO      Button
--------------------------
D2         -----> PIN 1
GND        -----> PIN 2
```

> Internal Pull-Up diaktifkan via `INPUT_PULLUP`, sehingga tidak diperlukan resistor eksternal.

### Tips Pemasangan

1. **Sensor BMP280 #1 (Dynamic / Stagnation)**
   - Pasang menghadap aliran udara (sisi lubang pitot tube)
   - Pastikan lubang inlet tidak tertutup

2. **Sensor BMP280 #2 (Static Pressure)**
   - Pasang pada sisi yang tidak terkena aliran langsung
   - Jauhkan dari turbulensi yang tidak diinginkan

3. **Tombol Zeroing**
   - Tempatkan di posisi yang mudah dijangkau
   - Tekan saat sistem tidak ada aliran udara untuk kalibrasi ulang

---

## 📤 Upload Program ke Arduino

### 1. Persiapan

1. **Buka File Program**
   - Download file `Airspeed_Sensor.ino`
   - Buka dengan Arduino IDE: `File → Open`

2. **Sesuaikan Offset Suhu (Opsional)**

   Edit bagian ini di dalam `void loop()` jika sensor menunjukkan suhu yang tidak akurat:
   ```cpp
   float t1 = readTemperature(0x76, cal1, -1.5);   // Offset sensor #1 (-1.5°C)
   readTemperature(0x77, cal2, -8.89);              // Offset sensor #2 (-8.89°C)
   ```
   Sesuaikan nilai offset dengan mengukur selisih suhu aktual terhadap referensi termometer.

### 2. Konfigurasi Board

1. **Pilih Board**
   - `Tools → Board → Arduino AVR Boards → Arduino Uno` (sesuaikan dengan board Anda)

2. **Pilih Port COM**
   - Hubungkan Arduino ke komputer via USB
   - `Tools → Port → COMx` (Windows) atau `/dev/ttyUSBx` (Linux) atau `/dev/cu.usbmodem-xxx` (macOS)

3. **Konfigurasi Upload Settings (Arduino Uno)**
   ```
   Tools → Board: Arduino Uno
   Tools → Processor: ATmega328P
   Tools → Upload Speed: 115200
   ```

### 3. Compile & Upload

1. **Verify/Compile**
   - Klik tombol ✓ (Verify) atau `Sketch → Verify/Compile`
   - Tunggu hingga proses compile selesai
   - Pastikan tidak ada error

2. **Upload ke Arduino**
   - Klik tombol → (Upload) atau `Sketch → Upload`
   - Tunggu proses upload selesai

3. **Buka Serial Monitor**
   - `Tools → Serial Monitor`
   - Set baud rate ke **9600**
   - Tunggu pesan `"System Ready. Press Button to Toggle ON/OFF."`

---

## 🚀 Penggunaan

### Startup & Kalibrasi Awal

Saat pertama dinyalakan, sistem akan otomatis melakukan kalibrasi:

```
Calibrating... Please wait.
System Ready. Press Button to Toggle ON/OFF.
```

Kalibrasi dilakukan dengan mengambil **20 sampel** selisih tekanan antara kedua sensor dan menghitung rata-rata sebagai `pressure_offset`. Proses ini memakan waktu sekitar **1 detik**.

> ⚠️ **Pastikan tidak ada aliran udara saat proses kalibrasi startup berlangsung!**

### Membaca Output Serial

Setelah sistem ready, output akan tampil setiap 50ms:

```
Temp:25.30 Raw_V:12.45 Filtered_V:11.87
Temp:25.31 Raw_V:12.60 Filtered_V:12.10
Temp:25.30 Raw_V:0.00  Filtered_V:8.32
```

| Field | Keterangan | Satuan |
|-------|------------|--------|
| `Temp` | Suhu dari BMP280 #1 (sudah terkoreksi offset) | °C |
| `Raw_V` | Kecepatan mentah sebelum filtering | km/h |
| `Filtered_V` | Kecepatan setelah Kalman Filter | km/h |

### Zeroing / Re-kalibrasi

Tekan tombol di **Pin D2** kapan saja untuk melakukan re-kalibrasi:

```
Zeroing / Recalibrating...
New Offset: 0.35
```

Sistem akan mengambil **10 sampel** baru dan memperbarui `pressure_offset`. Estimasi Kalman Filter juga di-reset ke 0 untuk menghindari lonjakan nilai.

---

## 🔬 Penjelasan Algoritma

### 1. Pembacaan BMP280 via I2C (Raw Driver)

Program ini **tidak menggunakan library BMP280 eksternal**, melainkan berkomunikasi langsung dengan register sensor via I2C. Hal ini membuat program lebih ringan dan tidak bergantung library tambahan.

Urutan pembacaan:
1. Baca 24 byte data kalibrasi dari register `0x88`
2. Inisialisasi sensor ke mode normal (`0xF4 = 0x27`)
3. Baca raw ADC suhu dari register `0xFA` → konversi ke °C menggunakan formula kompensasi BMP280
4. Baca raw ADC tekanan dari register `0xF7` → konversi ke Pa menggunakan `t_fine` dari kalkulasi suhu

### 2. Kalman Filter

Kalman Filter digunakan untuk meredam noise pada pembacaan kecepatan:

```
Parameters:
  error_estimate  = 2     (uncertainty awal estimasi)
  error_measure   = 0.5   (uncertainty pengukuran sensor)
  q               = 0.01  (process noise)
```

Semakin kecil nilai `error_measure`, semakin percaya sistem terhadap sensor. Semakin kecil `q`, semakin halus output namun semakin lambat respons terhadap perubahan cepat.

### 3. Perhitungan Kecepatan (Bernoulli)

```
ΔP = P1 - P2 - pressure_offset
v  = √(2 × |ΔP| / ρ) × 3.6    [km/h]

dimana:
  ρ = 1.225 kg/m³  (densitas udara standar pada 15°C, sea level)
```

Jika `|ΔP| < 0.5 Pa`, kecepatan dianggap **0** untuk menghindari noise saat tidak ada aliran udara.

---

## 📌 Pin Configuration

| Pin Arduino | Fungsi | Keterangan |
|-------------|--------|------------|
| **A4 (SDA)** | I2C Data | Terhubung ke SDA kedua BMP280 |
| **A5 (SCL)** | I2C Clock | Terhubung ke SCL kedua BMP280 |
| **D2** | Tombol Zeroing | Input dengan `INPUT_PULLUP` |

---

## 🔴 Troubleshooting

### Problem: Serial Monitor tidak menampilkan output

**Solution:**
- Pastikan baud rate di Serial Monitor diset ke **9600**
- Cek koneksi USB ke Arduino
- Coba tekan tombol Reset pada board Arduino

### Problem: Kecepatan selalu 0 padahal ada aliran udara

**Solution:**
```
1. Cek sambungan I2C (SDA/SCL) ke kedua sensor BMP280
2. Pastikan alamat I2C sensor benar (cek pin SDO)
3. Lakukan scan I2C untuk verifikasi sensor terdeteksi
4. Pastikan pressure_offset tidak terlalu besar akibat kalibrasi yang salah
5. Tekan tombol zeroing untuk kalibrasi ulang
```

Kode scan I2C untuk diagnosa:
```cpp
#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(9600);
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("Sensor ditemukan di alamat 0x");
      Serial.println(a, HEX);
    }
  }
}
void loop() {}
```

### Problem: Nilai kecepatan sangat noise / tidak stabil

**Solution:**
- Pastikan supply daya stabil (gunakan USB yang bagus atau power supply terpisah)
- Periksa sambungan kabel I2C (gunakan kabel pendek, max 30cm)
- Tambahkan kapasitor 100nF dekat pin VCC sensor BMP280
- Turunkan nilai `error_measure` pada Kalman Filter untuk filtering lebih halus:
  ```cpp
  float error_measure = 0.2; // default 0.5, lebih kecil = lebih halus
  ```

### Problem: Kecepatan bernilai negatif atau sangat besar

**Solution:**
- Lakukan **zeroing** ulang dengan menekan tombol saat tidak ada aliran udara
- Cek apakah pemasangan fisik sensor sudah benar (sensor mana yang menghadap aliran)
- Pastikan kedua sensor tidak saling bertukar posisi (0x76 = dynamic, 0x77 = static)

### Problem: "while (!Serial)" — Arduino tidak mulai-mulai

**Catatan:** Baris `while (!Serial);` menyebabkan program menunggu Serial Monitor dibuka. Jika digunakan **tanpa komputer** (standalone), hapus baris tersebut:

```cpp
void setup() {
  Wire.begin();
  Serial.begin(9600);
  // while (!Serial);  // ← hapus / comment baris ini untuk standalone
  ...
}
```

---


## 🙏 Acknowledgments

- **Bosch Sensortec** - untuk sensor BMP280 yang andal dan terdokumentasi dengan baik
- **Arduino Community** - untuk support dan resources
- **Kalman Filter Reference** - implementasi berdasarkan prinsip Kalman Filter 1D


## 📝 Changelog

### [1.0.0] - 2026-03-04
#### Initial Release
- Dual BMP280 airspeed measurement via Pitot Tube principle
- Kalman Filter untuk output yang stabil
- Auto-kalibrasi offset tekanan saat startup
- Fungsi zeroing via push button
- Offset koreksi suhu per sensor
- Output Serial: Suhu, Raw Speed, Filtered Speed

---

## ⚠️ Disclaimer

**Sistem ini adalah alat pengukur kecepatan udara untuk keperluan eksperimen dan edukasi.**

- Akurasi pengukuran bergantung pada kualitas pemasangan fisik pitot tube
- Densitas udara yang digunakan adalah konstanta standar (1.225 kg/m³) dan **tidak** memperhitungkan variasi ketinggian atau kelembaban
- Jangan gunakan sebagai satu-satunya referensi untuk aplikasi kritis (penerbangan, dll)
- Developer tidak bertanggung jawab atas kerusakan atau kecelakaan akibat penggunaan sistem ini

---

**Last Updated:** 2026-03-04

---
