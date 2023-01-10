// author:

// Object: fix all functions to behave as independantable
// commit msg "fix: remove all dependancy"

#include <Wire.h>
#include <Arduino.h>
#include <kalman.h>

#include "../include/calibrateIMU.h"


#define MPU 0x68
#define sample 4000
#define POWER_MANAGEMENT_REGISTER 0x6B
#define ACC_CONFIG_REGISTER 0x1C
#define GYRO_CONFIG_REGISTER 0x1B

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

long sampling_timer;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float kalAngleX, kalAngleY;
float elapsedTime, currentTime, previousTime;
t_errVal MPU6050_OFFSET;

// API
// Funciton name: initializeIMU()
// Parameters: None
// return valuse:
//    0: success
//    1: PM error
//    2: ACCEL_CONFIG error
//    3: GYRO_CONFIG error

// initializing IMU wire
char initializeIMU(){
  // if => 0 = false. not 0 = true
  // not(=!): !0 true(=not 0) <=> !not0 : false(=0)
  // endTranmission => 0:success 1-4: error

  // Start communication with MPU6050
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(POWER_MANAGEMENT_REGISTER);                  // Talk to the register 6B => Power Management
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  if(Wire.endTransmission(false)){        //end the transmission
    return 1;
  }

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  //Wire.beginTransmission(MPU);      // TODO : IF NOT FUNCTIONING WELL UNCOMMENT
  Wire.write(ACC_CONFIG_REGISTER);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  if(Wire.endTransmission(false)){
    return 2;
  }

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  //Wire.beginTransmission(MPU);      // TODO : IF NOT FUNCTIONING WELL UNCOMMENT
  Wire.write(GYRO_CONFIG_REGISTER);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  if(Wire.endTransmission(true)){
    return 3;
  }
  delay(20);

  return 0;
}

// get acceleroneer raw and error data
void talkToAccData(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
}

void talkToGyroData(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
}

// calculate angle
t_angleVal calcAngleValue(t_accVal accVal, t_gyroVal gyroVal, t_errVal errVal){
  t_angleVal ret;

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(accVal.accY / sqrt(pow(accVal.accX, 2) + pow(accVal.accZ, 2))) * 180 / PI) - errVal.AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * accVal.accX / sqrt(pow(accVal.accY, 2) + pow(accVal.accZ, 2))) * 180 / PI) - errVal.AccErrorY; // AccErrorY ~(-1.58)

  // === read angle with filter ===
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

  // Correct the outputs with the calculated error values
  gyroVal.gyroX = gyroVal.gyroX - errVal.GyroErrorX; // GyroErrorX ~(-0.56)
  gyroVal.gyroY = gyroVal.gyroY - errVal.GyroErrorY; // GyroErrorY ~(2)
  gyroVal.gyroZ = gyroVal.gyroZ - errVal.GyroErrorZ; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + gyroVal.gyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyroVal.gyroY * elapsedTime; // gyroAngleX = GyroX * elapsedTime; then the movement than the stabilization

  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.996 * gyroAngleX + 0.004 * accAngleX;
  gyroAngleY = 0.996 * gyroAngleY + 0.004 * accAngleY;

  kalmanX.setAngle(ret.roll); // Set starting angle
  kalmanY.setAngle(ret.pitch);
  gyroAngleX = ret.roll;
  gyroAngleY = ret.pitch;
  ret.yaw = ret.yaw + gyroVal.gyroZ * elapsedTime;
  return ret;
}

// Print the values on the serial monitor
void printAngleVal(t_angleVal angleVal){
  Serial.print("ROLL: ");
  Serial.print(angleVal.roll);
  Serial.print("/");
  Serial.print("PITCH:  ");
  Serial.print(angleVal.pitch);
  Serial.print("/");
  Serial.print("YAW: ");
  Serial.println(angleVal.yaw);
}

// sampling clock
void samplingRate(){
  while(micros() - sampling_timer < sample); //
  sampling_timer = micros(); //Reset the sampling timer
}

t_accVal getAccRaw(){
  talkToAccData();
  t_accVal ret;
  ret.accX = (Wire.read() << 8 | Wire.read()) / LSB_Sensitivity_R4_2;
  ret.accY = (Wire.read() << 8 | Wire.read()) / LSB_Sensitivity_R4_2;
  ret.accZ = (Wire.read() << 8 | Wire.read()) / LSB_Sensitivity_R4_2;

  return ret;
}

t_gyroVal getGyroRaw(){
  talkToGyroData();
  t_gyroVal ret;
  ret.gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  ret.gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  ret.gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  return ret;
}

// main function imu
// imu initializing
void setup(){
  // System Initializing
  Serial.begin(9600);
  Wire.begin();  // Initialize comunication

  // Module Initializing
  if (char DBG_INIT_IMU = initializeIMU()){  // wire setup => in detail
    Serial.println("Error with Initializing IMU CODE: ");
    Serial.println(DBG_INIT_IMU);
  }
  MPU6050_OFFSET = calibrateIMU();
  delay(100);   // TODO: mode change
}

void loop(){
  // get values
  t_accVal accVal = getAccRaw();  // accVal에 함수 거친 것이 저장됨
  t_gyroVal gyroVal = getGyroRaw();
  //t_errVal errVal = calcIMUErr();  // TODO: loop에서 여기까지 계속 가져오니까 문제가 생기는 듯

  // calculate roll, pitch, yaw
  t_angleVal angleVal = calcAngleValue(accVal, gyroVal, MPU6050_OFFSET);
  printAngleVal(angleVal);
  samplingRate();
}

//TODO: low speed and inaccurate value
//TODO: error value가 갱신됨
