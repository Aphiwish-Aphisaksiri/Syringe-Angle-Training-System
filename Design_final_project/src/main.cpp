#include <Arduino.h>
#include <Wire.h>

#include <stdint.h>
#include <math.h>

int16_t i = 0;

struct SensorData { // Define a struct to hold sensor data. Change resolution as needed.
  int16_t roll, pitch, yaw;
  // Add more variables for other data values
};

void initializeGyro() {
  Wire.beginTransmission(0x68); // Start communication with the gyro sensor
  Wire.write(0x6B); // Power management register
  Wire.write(0); // Wake up the gyro sensor
  Wire.endTransmission(true);
}

void readAccelerometer(SensorData& data) {
  Wire.beginTransmission(0x68); // Start communication with the accelerometer
  Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true); // Request 6 bytes of data from the accelerometer

  // Read accelerometer data
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();

  // Convert accelerometer data to g-force
  float accelScale = 2.0; // Set the accelerometer scale (±2g, ±4g, ±8g, ±16g)
  float accelXg = accelX / 16384;
  float accelYg = accelY / 16384;
  float accelZg = accelZ / 16384;

  // Calculate the roll and pitch angles using accelerometer data
  float roll = atan2(accelYg, sqrt(accelXg * accelXg + accelZg * accelZg)) * 180.0 / M_PI;
  float pitch = atan2(-accelXg, sqrt(accelYg * accelYg + accelZg * accelZg)) * 180.0 / M_PI;
  float yaw = 0.0; // Set yaw to 0 since we are only reading accelerometer data

  // Assign roll, pitch, and yaw values to data variables
  data.roll = roll;
  data.pitch = pitch;
  data.yaw = yaw;
}

void setup() {
  Serial.begin(115200); // Initialize serial communication
  Wire.begin(); // Initialize I2C communication
  initializeGyro(); // Initialize the gyro sensor
}

void loop() {
  SensorData data; // Create a struct to hold sensor data
  readAccelerometer(data); // Read accelerometer data
  // Assign values to other data variables
  //Serial.println(data.roll);
  //Serial.write((byte*)&data, sizeof(data));

  //delay(1);
}