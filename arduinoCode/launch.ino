#include "driving.h"
#include "time.h"

MPU6050 mpu6050;
Imu imu;
Pid pid;
Timer timer;


void setup(){
    Serial.beign(115200);
    while(!Serial);
    Serial.println("Initializing devices....")
    setup_driving(&imu,&mpu6050);
    setup_pid(&pid,40,40,0.05);
}

void loop(){
    /*test demo for motor*/
    // set_motor(255,255);
    
    /*test demo for imu*/
    // start_time(&timer,1000);
    // updateImu(&imu);
    // if(timer.isEnd){
    //     printImu(&imu);
    //     reset(&timer);
    // }

    delay(BASE_TIME);
}