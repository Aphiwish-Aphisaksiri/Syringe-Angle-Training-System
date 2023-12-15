#include <Arduino.h>
#include <Wire.h>

#include <stdint.h>
#include <math.h>

int16_t i = 0;

struct SensorData { // Define a struct to hold sensor data. Change resolution as needed.
  int16_t accX, accY, accZ;
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

  data.accX = accelX - 16384 * 0.089; // off set
  data.accY = accelY - 16384 * 0.64;
  data.accZ = accelZ + 16384 * 0.0;
}

void setup() {
  Serial.begin(115200); // Initialize serial communication
  Wire.begin(); // Initialize I2C communication
  initializeGyro(); // Initialize the gyro sensor
  pinMode(3, OUTPUT); // Set pin 3 as output for the buzzer
}

void loop() {
  SensorData data; // Create a struct to hold sensor data
  readAccelerometer(data); // Read accelerometer data
  Serial.write((byte*)&data, sizeof(data));

  // Convert accelerometer data to g-force
  // float accelXg = data.accX / 16384;
  // float accelYg = data.accY / 16384;
  // float accelZg = data.accZ / 16384;

  // Calculate the roll and pitch angles using accelerometer data
  // float roll = atan2(accelYg, sqrt(accelXg * accelXg + accelZg * accelZg)) * 180.0 / M_PI;
  // float pitch = -atan2(accelXg, sqrt(accelYg * accelYg + accelZg * accelZg)) * 180.0 / M_PI;
  
  static unsigned long buzzerStartTime = 0;
  static bool buzzerOn = false;

  // read command from serial port if command = 1, turn on buzzer
  if (Serial.available() > 0) {
    byte command[1];
    Serial.readBytes(command, 1);
    if (command[0] == 1) {
      analogWrite(3, 128); // Turn on the buzzer at 50% duty cycle
      buzzerStartTime = millis(); // Record the start time of the buzzer
      buzzerOn = true;
    }
  }
  else{
    if (buzzerOn && (millis() - buzzerStartTime >= 250)) {
      analogWrite(3, 0); // Turn off the buzzer
      buzzerOn = false;
    }
  }
  delay(1);
}