#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/* For this program, we'll be taking combining the gyroscope code as well as the accelerometer
 * code. The gyroscope is fairly accurate, but in the long run, it will drift. So we will apply
 * a complementary filter. We will trust the gyro 96% and the accelerometer 4%. Basically, the 
 * accelerometer is there to keep the gyro in check.
 */


const int MPU = 0x68; // The address for the MPU
const int CALIBRATION_STEPS = 200;
const float ACCELEROMETER_SENSITIVITY = 4096; //LSB per g (section 4.17 on the datasheet under Accelerometer measurements)
const float GYRO_SENSITIVITY = 16.4; // LSB per degree/second at +/- 2000 degrees/second (datasheet register 27)


const unsigned long SAMPLE_PERIOD_US = 1000;  // the amount of time (in us) between each sample (1ms)
const unsigned long PRINT_PERIOD_MS = 20;     // the amount of time in ms between each print onto the monitor



float roll, pitch, yaw; // The angles about the x, y, and z axis, respectively.
float prevGyroX, prevGyroY, prevGyroZ; // The previous values of the roll, pitch, and yaw, so that we can apply the trapezoidal rule

float gyroX, gyroY, gyroZ; // Angular rates for X, Y, Z axis (degrees/second)
float gyroErrorX, gyroErrorY, gyroErrorZ; // Error variables for gyro data (used to calibrate the sensor).

float accelX, accelY, accelZ; // Acceleration for X, Y, Z axis (g)
float accelAngleX, accelAngleY; //Accelerometer roll and pitch angles, respectively
float accelErrorX, accelErrorY, accelErrorZ; // Error variable s for accelerometer data (used to calibrate the sensor).

float dt; // Time between samples (seconds)
unsigned long currentTime, previousTime; // Timestamps in microseconds
unsigned long lastPrintTime; // Timestamp of last Serial print in milliseconds
int c = 0; // Sample counter for calculate_IMU_error()


void readAccel();
void readGyro();
void calculate_IMU_ERROR();
void calculateAccel();

void setup(){
    // Set up the serial monitor
    Serial.begin (115200);

    // Start the I2C with the Arduino as the master
    Wire.begin();

    // Set the MPU to wake mode, 8MHz oscillator (the clock)
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0X00);
    Wire.endTransmission(true);


    Wire.beginTransmission(MPU);
    Wire.write(0x1B); // Configure the gyro first
    Wire.write(0x18); // Set the gryo to full scale range of +/- 2000 degrees/second.
    Wire.endTransmission(true);
    
    Wire.beginTransmission(MPU);
    Wire.write(0x1C); // Configure the Accelerometer
    Wire.write(0x10); //Setting the full scale range of the accelerometer to +/- 8g.
    Wire.endTransmission(true);
    
    // Run the calibration function
    calculate_IMU_ERROR();

    // readGyro(): reads the latest X, Y, Z angular rates from the MPU-6050 and stores them in
    // gyroX, gyroY, gyroZ in degrees/second. Defined below loop().
    // Seed the trapezoidal rule with a first bias-corrected sample so the first step is not a jump from 0
    readGyro();
    prevGyroX = gyroX - gyroErrorX;  // To not have to make another set of variables, temporarily store them into these variables.
    prevGyroY = gyroY - gyroErrorY;
    prevGyroZ = gyroZ - gyroErrorZ;

    // Initiallize roll, pitch, and yaw. Set roll and pitch to the calculated accelerometer angles so we
    // can account for the error.
    calculateAccel();
    roll = accelAngleX;
    pitch = accelAngleY;
    yaw = 0;

    // micros(): returns the number of microseconds since the Arduino started (resolution of 4 us on the Nano).
    // Used for the 1 ms sample timing because millis() is too coarse to measure dt accurately.
    previousTime = micros();

    // millis(): returns the number of milliseconds since the Arduino started. Used for the 20 ms print timing.
    lastPrintTime = millis();

}

void loop(){

    currentTime = micros();
    if(currentTime - previousTime < SAMPLE_PERIOD_US){
        return; // Restart void loop repeatedly until we reach 1ms (1000 microseconds)
    }

    dt = (currentTime - previousTime) / 1000000.0; //time between samples, dt.
    previousTime = currentTime;


    // Use the trapezoidal rule + the complementary filter to calculate the Angles
    // the complementary uses 96% gyro, 4% accelerometer, since the accelerometer can't measure yaw,
    // we can only apply the complementary filter for pitch and roll.
    readGyro();
    gyroX -= gyroErrorX;
    gyroY -= gyroErrorY;
    gyroZ -= gyroErrorZ;

    // Calculate the angles that the accelerometer gives us
    calculateAccel();

    // We need to account for the fact that the roll is the gyroscope corrected by the accelerometer,
    // so we need to use the corrected roll rather than just the previous gyroscope angle.
    roll = ((0.96) * (roll + (gyroX + prevGyroX) * (dt / float(2))))
         + (0.04) * accelAngleX;
    
    pitch = ((0.96) * (pitch + (gyroY + prevGyroY) * (dt / float(2))))
          + (0.04) * accelAngleY;

    yaw += (dt / float(2)) * (prevGyroZ + gyroZ);

    prevGyroX = gyroX;
    prevGyroY = gyroY;
    prevGyroZ = gyroZ;



   


    if(millis() - lastPrintTime >= PRINT_PERIOD_MS){

        lastPrintTime = millis();

        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print("   |Pitch: ");
        Serial.print(pitch);
        Serial.print("   |Yaw: ");
        Serial.println(yaw);
    }

}


void readAccel(){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU, 6, true);

    int16_t accel_rawX = Wire.read() << 8 | Wire.read();
    int16_t accel_rawY = Wire.read() << 8 | Wire.read();
    int16_t accel_rawZ = Wire.read() << 8 | Wire.read();

    accelX = accel_rawX / ACCELEROMETER_SENSITIVITY;
    accelY = accel_rawY / ACCELEROMETER_SENSITIVITY;
    accelZ = accel_rawZ / ACCELEROMETER_SENSITIVITY;

}

void readGyro(){
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU, 6, true);

    int16_t gyro_rawX = Wire.read() << 8 | Wire.read();
    int16_t gyro_rawY = Wire.read() << 8 | Wire.read();
    int16_t gyro_rawZ = Wire.read() << 8 | Wire.read();

    gyroX = gyro_rawX / GYRO_SENSITIVITY;
    gyroY = gyro_rawY / GYRO_SENSITIVITY;
    gyroZ = gyro_rawZ / GYRO_SENSITIVITY;
}

void calculate_IMU_ERROR(){
    accelErrorX = 0;
    accelErrorY = 0;
    accelErrorZ = 0;

    gyroErrorX = 0;
    gyroErrorY = 0;
    gyroErrorZ = 0;

    // Calibrate the accelerometer
    for(c = 0; c < CALIBRATION_STEPS; c++){
        readAccel();

        accelErrorX += accelX;
        accelErrorY += accelY;
        accelErrorZ += accelZ;

    }

    // Callibrate the gyroscope
        for(c = 0; c < CALIBRATION_STEPS; c++){
        readGyro();

        gyroErrorX += gyroX;
        gyroErrorY += gyroY;
        gyroErrorZ += gyroZ;
        
    }

    accelErrorX /= CALIBRATION_STEPS;
    accelErrorY /= CALIBRATION_STEPS;
    accelErrorZ = accelErrorZ/CALIBRATION_STEPS - 1.0;  // Need to subtract one since the gyro sitting flat on a desk
                                                        // reads +1.0g, so we subtract 1.0 to get the proper error.

    gyroErrorX /= CALIBRATION_STEPS;
    gyroErrorY /= CALIBRATION_STEPS;
    gyroErrorZ /= CALIBRATION_STEPS;

}

void calculateAccel(){

    readAccel();

    accelX -= accelErrorX;
    accelY -= accelErrorY;
    accelZ -= accelErrorZ;


    /* Why pitch has a minus sign but roll doesn't:
     *
     * The gyro decides which direction is "positive" using the right-hand rule: point your right
     * thumb along the axis arrow on the board, and the way your fingers curl is a positive rotation.
     * The accelerometer just measures which way is "up" (+1 g on whichever axis points at the ceiling).
     *
     * Roll (rotation about X):
     *   Thumb along the X arrow, curl your fingers -> the Y end of the board goes UP.
     *   Y pointing up makes accelY positive, so atan2(accelY, accelZ) is positive.
     *   Gyro says +, accelerometer says +  -> they agree, no minus needed.
     *
     * Pitch (rotation about Y):
     *   Thumb along the Y arrow, curl your fingers -> the X end of the board goes DOWN.
     *   X pointing toward the floor makes accelX negative, so atan2(accelX, ...) is negative.
     *   Gyro says +, accelerometer says -  -> they disagree, so we flip the accelerometer's sign.
     *
     * Neither sensor is wrong. For X the finger curl lifts Y, for Y the finger curl drops X, and the
     * complementary filter needs both sensors to agree on which way is positive.
     */
    accelAngleX = atan2(accelY,accelZ) * 180 / PI;
    accelAngleY = -atan2(accelX, sqrt(square(accelZ) + square(accelY))) * 180 / PI;

}
