#ifndef DRIVING_H
#define DRIVING_H

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include "math.h"
//#include <MPU6050.h>

/*sensor config*/
#define IR_LEFT A1
#define IR_FRONT A2
#define IR_RIGHT A3
#define LIMIT4IR 100
#define INTERUPT 2

/*Motor config*/
#define LMOTOR_DIR1 4
#define LMOTOR_DIR2 5
#define LMOTOR_PWM 6
#define RMOTOR_DIR1 7
#define RMOTOR_DIR2 8
#define RMOTOR_PWM 9

/*offset of IMU*/
#define X_ACC_OFFSET 40
#define Y_ACC_OFFSET 1593
#define Z_ACC_OFFSET 963
#define X_GYRO_OFFSET 51
#define Y_GYRO_OFFSET 8
#define Z_GYRO_OFFSET 21

//extern MPU6050 mpu;
//extern uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//extern uint16_t fifoCount;     // count of all bytes currently in FIFO
//extern uint8_t fifoBuffer[64]; // FIFO storage buffer


struct Imu{
  Quaternion* quater;
  VectorInt16* acc;
  VectorInt16* gyro;
  VectorFloat* gravity;
  float* euler;
  float* ypr;
  bool dmpState;
};

//extern Imu imu;

struct myPID{
    float curVal;
    float preVal;
    float lastError;
    float error;
    float sumError;
    int Kp;
    int Ki;
    int Kd;
};

//setup function of driving.h
void setup_driving();

/*wait to be realized*/
void update(int* sensors,int size);
int check(int sensor);
int checks(int* sensors,int size);

//IMU operation
void releaseImu(Imu* imu);
void updateImu();
void printImu();

//Motor operation
void set_motor(int left,int right);
void set_lmotor_dir(int one,int two);
void set_rmotor_dir(int one,int two);

//PID calculation
void setup_pid(myPID* pid);
int pid(myPID* pid,Imu* imu,float setVal);
void printPID(myPID* pid);

#endif
