#include <Arduino.h>
#include <Wire.h>
#include <math.h>


//Accelerometer code, currently unfinished.


const int MPU = 0x68; // I2C address of MPU-6050
const float ACCELEROMETER_SENSITIVITY = 4096; //LSB per g (section 4.17 on the datasheet under Accelerometer measurements)

const unsigned long SAMPLE_PERIOD_US = 1000; //Read the accelerometer every 1ms (1000 microseconds)
const unsigned long PRINT_PERIOD_MS = 20; //Print angles every 20 ms
const int calibration_steps = 200; // 200 samples, 1ms apart for a total of 200 ms.

float accelX, accelY, accelZ; // Acceleration for X, Y, Z axis (g)
float accelAngleX, accelAngleY; //Accelerometer roll and pitch angles, respectively
float accelErrorX, accelErrorY, accelErrorZ; // Error variables for accelerometer data (used to calibrate the sensor).
float elapsedTime; // Time between samples (seconds)
unsigned long currentTime, previousTime; // Timestamps in microseconds
unsigned long lastPrintTime; // Timestamp of last Serial print in milliseconds
int c = 0; // Sample counter for calculate_IMU_error()


//Reads the accelerometer and returns values for accelX, accelY, and accelZ
void readAccelerometer();
// Calculates the error in the IMU (all of the IMU's will have a different error, so this part is essential).
// when the calibration is happening, make sure to keep the mpu flat (calibration should take less than a second).
void calculate_IMU_error();

void setup(){

    Serial.begin(115200); // Start the Serial Monitor
    

    // Wire.begin(): starts the I2C bus with the Arduino as the master. On the Nano this uses A4 (SDA) and A5 (SCL).
    Wire.begin();

    // Wire.beginTransmission(address): starts a message to the I2C device at this address.
    // Bytes passed to Wire.write() are queued up and only sent when Wire.endTransmission() is called.
    Wire.beginTransmission(MPU); //Start communication with the MPU


    //Turning the MPU on and initializing it
    Wire.write(0x6B); //Write to register 6B, the power management register
    Wire.write(0x00); //Set the MPU to wake. using the 8MHz internal oscillator (the clock)
    // Wire.endTransmission(stop): sends the queued bytes. true = send a STOP and release the bus,
    // false = keep the bus open so a read can follow straight away (a "repeated start").
    Wire.endTransmission(true); 

    //Initialize the Accelerometer
    Wire.beginTransmission(MPU); //Start communication with the MPU
    Wire.write(0x1C); //This is the register to talk to for the accelerometer. 1C = 16 + 12 (C = 12 for hexadecimal)
    Wire.write(0x10); //Setting the full scale range of the accelerometer to +/- 8g.
    Wire.endTransmission(true);

     /* Call the calibration function, this will give us error values for X, Y, and Z, given to us
      * as the variables accelErrorX, accelErrorY, and accelErrorZ. We will subtract these values from our
      * accelX, accelY, accelZ values to account for the error in the IMU.
      */
    calculate_IMU_error();

     // micros(): returns the number of microseconds since the Arduino started (resolution of 4 us on the Nano).
     // used so we can accurately read every ms
    previousTime = micros();

    // millis(): returns the number of milliseconds since the Arduino started. Used for the 20 ms print timing.
    lastPrintTime = millis();

}

void loop(){


    // Wait until 1ms has passed since the previous read, "return;" simply exits void loop and starts a new void loop.
    currentTime = micros();
    if(currentTime - previousTime < SAMPLE_PERIOD_US){
        return;
    }

    previousTime = currentTime;

    readAccelerometer();

    accelX -= accelErrorX;
    accelY -= accelErrorY;
    accelZ -= accelErrorZ;

    // Use simple trigonometry to calculate the pitch and rolling angles, atan2 just makes it so that we don't
    // have anything that would be undefined. You can find the full explanation of this calculation in the 
    // documentation.
    accelAngleX = atan2(accelY,accelZ) * 180 / PI; 
    accelAngleY = atan2(accelX, sqrt(square(accelZ) + square(accelY))) * 180 / PI;

    if(millis() - lastPrintTime >= PRINT_PERIOD_MS){

        lastPrintTime = millis();

        Serial.print("Roll:  ");
        Serial.print(accelAngleX);
        Serial.print("   |  Pitch:  ");
        Serial.println(accelAngleY);

    }
}

void readAccelerometer(){
Wire.beginTransmission(MPU);
Wire.write(0x3B); // First Accelerometer Register
Wire.endTransmission(false); //Basically says that we want to continue doing stuff with this register


Wire.requestFrom(MPU, 6, true); // Read the next 6 bytes from the MPU starting from 0x3B

//Read the raw values of X, Y, and Z from the accelerometer, given to us in units of LSB 
int16_t Accel_raw_X = Wire.read() << 8 | Wire.read();
int16_t Accel_raw_Y = Wire.read() << 8 | Wire.read();
int16_t Accel_raw_Z = Wire.read() << 8 | Wire.read();

//Divide by the LSB for our configuration (4096 for +/- 9g)
accelX = Accel_raw_X / ACCELEROMETER_SENSITIVITY; 
accelY = Accel_raw_Y / ACCELEROMETER_SENSITIVITY;
accelZ = Accel_raw_Z / ACCELEROMETER_SENSITIVITY;

}

// Take the average of 200 samples to get the error
void calculate_IMU_error(){

//Set the errors to 0
accelErrorX = 0;
accelErrorY = 0;
accelErrorZ = 0;


//Read 200 samples, sum the samples.
for(c = 0; c < calibration_steps; c++){
    readAccelerometer();

    accelErrorX += accelX;
    accelErrorY += accelY;
    accelErrorZ += accelZ;

    delay(1);
}

// Divide by the number of steps to get the average
accelErrorX /= calibration_steps;
accelErrorY /= calibration_steps;
accelErrorZ = (accelErrorZ/calibration_steps - 1.0);

}