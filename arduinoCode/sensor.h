#ifndef SENSOR_H
#define SENSOR_H

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050/MPU6050_6Axis_MotionApps20.h"

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
/*sensor config*/
//#define IR_LEFT 
//#define IR_FRONT
//#define IR_RIGHT
//define LIMIT4IR

/*wait to be realized*/
void update(int* sensors,int size);
int check(int sensor);
int checks(int* sensors,int size);
void setupMPU(); 
void getYPR(float* ypr, float* euler, VectorFloat* gravity, Quaternion* q);
void getAccXYZ(VectorInt16* acc, VectorInt16* acc_real, VectorFloat* gravity, Quaternion* q);

#endif