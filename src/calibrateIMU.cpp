#include "../include/calibrateIMU.h"

// getting raw data function
// get accelerator raw data
t_accVal getAccRawContinue(){
  t_accVal ret;
  ret.accX = (Wire.read() << 8 | Wire.read()) / LSB_Sensitivity_R4_2;
  ret.accY = (Wire.read() << 8 | Wire.read()) / LSB_Sensitivity_R4_2;
  ret.accZ = (Wire.read() << 8 | Wire.read()) / LSB_Sensitivity_R4_2;

  return ret;
} // ret에 채워 넣은 걸 반환하겠다

// get raw gyroscope data
t_gyroVal getGyroRawContinue(){
  t_gyroVal ret;
  ret.gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  ret.gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  ret.gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  return ret;
} // t_gyroVal는 반환 형태임

// API
// Function Name: calcuIMUErr
// Parameter: None
// Return Value: t_errVal

//caculate IMU error
t_errVal calcIMUErr(){
  t_errVal ret;
  ret.AccErrorX = 0;
  ret.AccErrorY = 0;
  ret.GyroErrorX = 0;
  ret.GyroErrorY = 0;
  ret.GyroErrorZ = 0;

  int cnt = 0;

  // Initialize connection with MPU
  // Wire.beginTransmission(MPU);         // TODO: Find why this code has I2C timing error (line 41->line 44)

  while (cnt++ < cali_sampling_rate){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Start address of ACCEL data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    t_accVal accVal = getAccRawContinue();
    // Serial.print(accVal.accX);
    // Serial.print("/");
    // Serial.print(accVal.accY);
    // Serial.print("/");
    // Serial.println(accVal.accZ);
    // Sum all readings
    ret.AccErrorX = ret.AccErrorX + ((atan((accVal.accY) / sqrt(pow((accVal.accX), 2) + pow((accVal.accZ), 2))) * 180 / PI));
    ret.AccErrorY = ret.AccErrorY + ((atan(-1 * (accVal.accX) / sqrt(pow((accVal.accY), 2) + pow((accVal.accZ), 2))) * 180 / PI));
    // Serial.print("AccErrorX: ");
    // Serial.print(ret.AccErrorX);
    // Serial.print("/AccErrorY: ");
    // Serial.println(ret.AccErrorY);
  }
  // Serial.print("final AccErrorX: ");
  // Serial.println(ret.AccErrorX);
  // Serial.print("final AccErrorY: ");
  // Serial.println(ret.AccErrorY);
  //Divide the sum by 200 to get ghithe error value
  ret.AccErrorX = ret.AccErrorX / cali_sampling_rate;
  ret.AccErrorY = ret.AccErrorY / cali_sampling_rate;

  cnt = 0;
  // Read gyro values 200 times
  while (cnt++ < cali_sampling_rate) {
    Wire.write(0x43);   // Start address of GYRO data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    t_gyroVal gyroVal = getGyroRawContinue();

    // Sum all readings
    ret.GyroErrorX = ret.GyroErrorX + gyroVal.gyroX;
    ret.GyroErrorY = ret.GyroErrorY + gyroVal.gyroY;
    ret.GyroErrorZ = ret.GyroErrorZ + gyroVal.gyroZ;
  }

  //Divide the sum by 200 to get the error value
  ret.GyroErrorX = ret.GyroErrorX / cali_sampling_rate;
  ret.GyroErrorY = ret.GyroErrorY / cali_sampling_rate;
  ret.GyroErrorZ = ret.GyroErrorZ / cali_sampling_rate;

  return ret;
}

void printIMUErr(t_errVal errVal) {
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(errVal.AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(errVal.AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(errVal.GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(errVal.GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(errVal.GyroErrorZ);
}

t_errVal calibrateIMU(){
  Serial.println("Calibration Processing...");
  t_errVal errVal = calcIMUErr();
  printIMUErr(errVal);
  delay(20);
  Serial.println("Calibration Successed");

  return errVal;
}
