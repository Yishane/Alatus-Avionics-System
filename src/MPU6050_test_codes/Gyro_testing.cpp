#include <Arduino.h>
#include <Wire.h>

const int MPU = 0x68; // I2C address of MPU-6050
const float GYRO_SENSITIVITY = 16.4; // LSB per degree/second at +/- 2000 degrees/second (datasheet register 27)
const unsigned long SAMPLE_PERIOD_US = 1000; // Read the gyro every 1 ms
const unsigned long PRINT_PERIOD_MS = 20; // Print angles every 20 ms (50 Hz)
const int CALIBRATION_SAMPLES = 200; // 200 samples, 1 ms apart = ~200 ms calibration

float gyroX, gyroY, gyroZ; // Angular rates for X, Y, Z axis (degrees/second)
float prevGyroX, prevGyroY, prevGyroZ; // Angular rates from the previous sample, used by the trapezoidal rule
float angleX, angleY, angleZ; // Roll, pitch, yaw (degrees), calculated through integrating the gyro data
float gyroErrorX, gyroErrorY, gyroErrorZ; // Error variables for gyro data (used to calibrate the sensor).
float elapsedTime; // Time between samples (seconds)
unsigned long currentTime, previousTime; // Timestamps in microseconds
unsigned long lastPrintTime; // Timestamp of last Serial print in milliseconds
int c = 0; // Sample counter for calculate_IMU_error()

// Function prototypes (definitions are below loop)
void readGyro();
void calculate_IMU_error();

void setup(){

    Serial.begin(115200);
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); // Writing to the power management register
    Wire.write(0x00); // Writing 0 to bits 0 through 7 to wake up the MPU-6050, using 8MHz clock
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU);
    Wire.write(0x1B); // Write to the Gyro configuration register
    Wire.write(0x18); // Set the gryo to full scale range of +/- 2000 degrees/second.
    Wire.endTransmission(true);

    // Keep the sensor still during calibration (~200 ms)
    calculate_IMU_error();

    // Seed the trapezoidal rule with a first bias-corrected sample so the first step is not a jump from 0
    readGyro();
    prevGyroX = gyroX - gyroErrorX;
    prevGyroY = gyroY - gyroErrorY;
    prevGyroZ = gyroZ - gyroErrorZ;

    angleX = 0;
    angleY = 0;
    angleZ = 0;

    previousTime = micros();
    lastPrintTime = millis();
}

void loop(){

    currentTime = micros();
    if (currentTime - previousTime < SAMPLE_PERIOD_US){
        return; // Wait until 1 ms has passed since the last sample
    }

    elapsedTime = (currentTime - previousTime) / 1000000.0; // Measured dt in seconds
    previousTime = currentTime;

    readGyro();

    // Remove the calibrated bias
    gyroX -= gyroErrorX;
    gyroY -= gyroErrorY;
    gyroZ -= gyroErrorZ;

    // Trapezoidal rule: angle += (previous rate + current rate) / 2 * dt
    angleX += (prevGyroX + gyroX) * 0.5 * elapsedTime; // Roll
    angleY += (prevGyroY + gyroY) * 0.5 * elapsedTime; // Pitch
    angleZ += (prevGyroZ + gyroZ) * 0.5 * elapsedTime; // Yaw

    prevGyroX = gyroX;
    prevGyroY = gyroY;
    prevGyroZ = gyroZ;

    if (millis() - lastPrintTime >= PRINT_PERIOD_MS){
        lastPrintTime = millis();
        Serial.print("Roll: ");
        Serial.print(angleX);
        Serial.print("  Pitch: ");
        Serial.print(angleY);
        Serial.print("  Yaw: ");
        Serial.println(angleZ);
    }

}

// Reads registers 0x43-0x48 and stores the raw angular rates (degrees/second) in gyroX, gyroY, gyroZ.
void readGyro(){
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // GYRO_XOUT_H, the first gyro data register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 bytes: X high/low, Y high/low, Z high/low

    // Each axis is a 16-bit two's complement value, high byte first
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();

    gyroX = rawX / GYRO_SENSITIVITY;
    gyroY = rawY / GYRO_SENSITIVITY;
    gyroZ = rawZ / GYRO_SENSITIVITY;
}

// Averages the gyro output while the sensor sits still. That average is the bias (error) to subtract later.
void calculate_IMU_error(){
    gyroErrorX = 0;
    gyroErrorY = 0;
    gyroErrorZ = 0;

    for (c = 0; c < CALIBRATION_SAMPLES; c++){
        readGyro();
        gyroErrorX += gyroX;
        gyroErrorY += gyroY;
        gyroErrorZ += gyroZ;
        delay(1);
    }

    gyroErrorX /= CALIBRATION_SAMPLES;
    gyroErrorY /= CALIBRATION_SAMPLES;
    gyroErrorZ /= CALIBRATION_SAMPLES;
}
