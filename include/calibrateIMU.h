#ifndef IMU_CALIBRATEIMU_H
#define IMU_CALIBRATEIMU_H

#include <Wire.h>
#include <Arduino.h>

#include "imu-datatype.h"

#define MPU 0x68
#define LSB_Sensitivity_R4_2 16384.0
#define cali_sampling_rate 200

typedef struct s_errVal{  // error val
  float AccErrorX;
  float AccErrorY;
  float GyroErrorX;
  float GyroErrorY;
  float GyroErrorZ;
} t_errVal;

t_accVal getAccRawContinue();
t_gyroVal getGyroRawContinue();
t_errVal calcIMUerr();
void printIMUErr(t_errVal errVal);
t_errVal calibrateIMU();

#endif
