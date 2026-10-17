#include <Arduino.h>
#include <Wire.h>


//Accelerometer code, currently unfinished.


const int MPU = 0x68; // I2C address of MPU-6050

float accelX, accelY, accelZ; // Acceleration for X, Y, Z axis (g)
float accelErrorX, accelErrorY, accelErrorZ; // Error variables for accelerometer data (used to calibrate the sensor).
float elapsedTime; // Time between samples (seconds)
unsigned long currentTime, previousTime; // Timestamps in microseconds
unsigned long lastPrintTime; // Timestamp of last Serial print in milliseconds
int c = 0; // Sample counter for calculate_IMU_error()