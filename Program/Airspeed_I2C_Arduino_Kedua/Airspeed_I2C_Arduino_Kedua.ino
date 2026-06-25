#include <Wire.h>

#define SLAVE_ADDR 0x08

float dP = 0;
float raw_v = 0;
float filtered_v = 0;

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveData);

  Serial.println("Raw_V Filtered_V");
}

void loop() {
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.print("Raw_V:");
    Serial.print(raw_v, 2);

    Serial.print("\tFiltered_V:");
    Serial.println(filtered_v, 2);
  }
}

void receiveData(int byteCount) {
  if (byteCount == 12) {
    Wire.readBytes((byte*)&dP, sizeof(dP));
    Wire.readBytes((byte*)&raw_v, sizeof(raw_v));
    Wire.readBytes((byte*)&filtered_v, sizeof(filtered_v));
  }
}