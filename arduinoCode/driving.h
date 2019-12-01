#ifndef DRIVING_H
#define DRIVING_H

#include "Wire.h"
#include "mpu6050/I2Cdev.h"
#include "math.h"
#include "mpu6050/MPU6050_6Axis_MotionApps20.h"

/*sensor config*/
#define IR_LEFT A1
#define IR_FRONT A2
#define IR_RIGHT A3
#define LIMIT4IR 100

/*Motor config*/
#define LMOTOR_DIR1 4
#define LMOTOR_DIR2 5
#define LMOTOR_PWM 6
#define RMOTOR_DIR1 7
#define RMOTOR_DIR2 8
#define RMOTOR_PWM 9

/*offset of IMU*/
#define X_ACC_OFFSET 40;
#define Y_ACC_OFFSET 1593;
#define Z_ACC_OFFSET 963;

typedef IMU{
    MPU6050* mpu;
    int yAcc;
    int zAcc;
    int xGyro;
    float curAngle;
    float angleAcc;
    float angleGyro;
    float preAngleGyro;
    float dt;
}Imu;

typedef PID{
    float curVal;
    float preVal;
    float lastError;
    float error;
    float sumError;
    int Kp;
    int Ki;
    int Kd;
}Pid;

//setup function of driving.h
void setup_driving(Imu* imu,MPU6050* mpu6050);

/*wait to be realized*/
void update(int* sensors,int size);
int check(int sensor);
int checks(int* sensors,int size);

//IMU operation
void initImu(Imu* imu,MPU6050* mpu6050);
void updateImu(Imu* imu);
void printImu(Imu* imu);

//Motor operation
void set_motor(int left,int right);
void set_lmotor_dir(int one,int two);
void set_rmotor_dir(int one,int two);

//PID calculation
void setup_pid(Pid* pid);
int pid(Pid* pid,Imu* imu,float setVal);
void printPID(Pid* pid);

#endif