#include <Arduino.h>

#include <stdint.h>

int16_t i = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication
}

struct SensorData { // Define a struct to hold sensor data. Change resolution as needed.
  int16_t gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
  // Add more variables for other data values
};

void loop() {
  SensorData data;
  data.gyroX = 30000;
  data.gyroY = 20000;
  data.gyroZ = 10000;
  data.accelX = -10000;
  data.accelY = -20000;
  data.accelZ = -30000;
  // Assign values to other data variables

  Serial.write((byte*)&data, sizeof(data));

  delay(1);
}