/* 
 * Quick Note: I had the idea of integrating a button into this project but later decided not to implement.
 * So you may see some button code in here, just ignore it, doesnt effect the project whatsoever. 
 *
 * IMU Reader - MPU9250 Serial Streamer
 * 
 * Author: Cameron Nix
 * 
 * Reads accelerometer, gyroscope, and magnetometer data from an MPU9250
 * 9-axis IMU over I2C, and streams it as comma-separated values over
 * serial for real-time processing (Kalman filtering, orientation
 * estimation) in MATLAB.
 * 
 * Output format (one line per sample):
 * timestamp_ms,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ
 * 
 * Hardware: ESP32-WROOM-32, MPU9250 breakout (I2C)
 * Library: MPU9250_asukiaaa
 */

#include <MPU9250_asukiaaa.h> // MPU9250 Library

#define BUTTON 4
#define SDA 21
#define SCL 22

MPU9250_asukiaaa mySensor; // create object for sensor
float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ; // define data variables for sensor

void setup() {
Serial.begin(115200);
pinMode(BUTTON, INPUT_PULLUP); // Button setup
Wire.begin(SDA, SCL); // I2C communication setup
mySensor.setWire(&Wire);
mySensor.beginAccel(); // begin sensor
mySensor.beginGyro();
mySensor.beginMag();
}

void loop() {
int result; // create variable

result = mySensor.accelUpdate();
if (result == 0) {
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
}

result = mySensor.gyroUpdate();
if (result == 0) {
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
}

result = mySensor.magUpdate();
if (result != 0) {
  mySensor.beginMag();
  result = mySensor.magUpdate();
} 
if (result == 0) {
  mX = mySensor.magX();
  mY = mySensor.magY();
  mZ = mySensor.magZ();
}
Serial.println(String(millis()) + "," + String(aX) + "," + String(aY) + "," + String(aZ) + "," + String(gX) + "," + String(gY) + "," + String(gZ) + "," + String(mX) + "," + String(mY) + "," + String(mZ));
delay(16.666666667); // 60 Hz
}
