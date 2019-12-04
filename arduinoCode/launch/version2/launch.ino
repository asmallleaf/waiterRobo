#include "driving.h"
#include "timer.h"
#include "bluetooth.h"

//Imu imu;
myPID mypid={};
Timer timer;

void setup(){
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Initializing devices....");
    setup_driving();
    mypid.Kp=40;
    mypid.Ki=40;
    mypid.Kd=0.05;
    setup_pid(&mypid);
}

void loop(){
    /*test demo for motor*/
    // set_motor(255,255);
    
    /*test demo for imu*/
     start(&timer,1000);
     updateImu();
     update(&timer);
     if(timer.isEnd==1){
         printImu();
         reset(&timer);
     }

    delay(BASE_TIME);
}
