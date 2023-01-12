#ifndef IMU-DATATYPE_H
#define IMU-DATATYPE_H

// get raw data
typedef struct s_accVal{ // raw data from accel
  float accX;
  float accY;
  float accZ;
} t_accVal;

typedef struct s_gyroVal{ // raw data from gyro
  float gyroX;
  float gyroY;
  float gyroZ;
} t_gyroVal;

typedef struct s_angleVal{
  float roll;
  float pitch;
  float yaw;
} t_angleVal;

#endif



// *{.cpp} => *.o => binary


// imu-datatype.h - calibrateIMU.cpp - mpu6050_loop_func.cpp

// calibrateIMU.h - calibrateIMU.cpp

// calibrateIMU.cpp - mpu6050_loop_func.cpp