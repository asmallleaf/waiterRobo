#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
/***********************************************/
/*             head files                      */
/***********************************************/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "math.h"
//#include <MPU6050.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/***********************************************/
/*             pin config number               */
/***********************************************/
/*sensor config*/
#define IR_LEFT A1
#define IR_FRONT A2
#define IR_RIGHT A3
#define LIMIT4IR 100
#define INTERRUPT 19
/*Motor config*/
#define LMOTOR_DIR1 4
#define LMOTOR_DIR2 5
#define LMOTOR_PWM 6
#define RMOTOR_DIR1 7
#define RMOTOR_DIR2 8
#define RMOTOR_PWM 9
/*offset of IMU*/
#define X_ACC_OFFSET 1408
#define Y_ACC_OFFSET -5454
#define Z_ACC_OFFSET 890
#define X_GYRO_OFFSET 43
#define Y_GYRO_OFFSET 5
#define Z_GYRO_OFFSET -19//0
/**print config**/
//#define __STOP__
#define __TEST__
//#define __PRINT_OFFSETS__
//#define __AUTOCALIBRATION__
//#define __PRINT_MOTOR__
#define __PRINT_IMU__
/*basic define constant*/
#define BASE_TIME 0.05
#define KP 14 //15
#define KI 1.5 //1.5
#define KD 35 //30
const float KR = 1;
const float ANGLE2PWM_L=0.6;
const float ANGLE2PWM_R=0.6;
#define MIN_PWM_L 25
#define MIN_PWM_R 27
#define MAX_DEG 25
#define MAX_I_LIMIT 325
/***********************************************/
/*          self defined structures            */
/***********************************************/
struct Imu{
  Quaternion quater;
  VectorInt16 acc;
  VectorInt16 gyro;
  VectorFloat gravity;
  float* euler;
  float* ypr;
  bool dmpState;
  uint8_t imuInterruptState;
  uint8_t devState;
  int16_t gyroX;
  float gyroX_f;
};

struct myPID{
    float curVal;
    float preVal;
    float lastError;
    float error;
    float sumError;
    float composVal;
    int Kp;
    int Ki;
    int Kd;
};

typedef struct TIMER{
    int set;
    int now;
    unsigned long time_stamp1;
    unsigned long time_stamp2;
    int isEnd;
    int isStart;
}Timer;

struct Msg{
    int length;
    int pos;
    char* pbuffer;
};

typedef struct MSGS{
    int size;
    Msg** pbuffer;
    int pos;
}Msgs;
/***********************************************/
/*             gloabal variables               */
/***********************************************/
MPU6050 mpu;
Imu imu;
myPID mypid={};
Timer timer={};

// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float pwm_val=0;
int pid2_val=0;
int MAX_LIMIT = 300;
float set_val = 0;
float lastDeg = 0;
float newDeg = 0;
int isStartTurn = 0;
int counter = 0;
volatile bool imuInterrupt = false; // variable in interrupt
/***********************************************/
/*             functions                       */
/***********************************************/

//setup function of driving.h
void setup_driving();

/*wait to be realized*/
void update(int* sensors,int size);
int check(int sensor);
int checks(int* sensors,int size);

//IMU operation
void dmpReady(){imuInterrupt = true;}
bool check_Int();
bool collect_Int();
void releaseImu();
void updateImu();
void printImu();
void calibration();

//Motor operation
void set_motor(int left,int right);
void set_lmotor_dir(int one,int two);
void set_rmotor_dir(int one,int two);
void stepmotor(int turnSpeed);
void forward();
int turn();

//PID calculation
void setup_pid();
float pid(float setVal);
int pid2(float setVal);
void printPID();

//timer operation
void start(Timer* timer, int time);
void update(Timer* timer);
void stop(Timer* timer);
void reset(Timer* timer);

//bluetooth operation
// initial bluetooth
void setBt();
// print initial msg
void printMsg();
// create message stack
Msgs* create_msgs(int size);
// create a message
Msg* create_msg(char* str,int size);
// clear the whole stack
void release_msgs(Msgs* msgs);
// delet the message
void release_msg(Msg* msg);
// send a message on the top of stack
void send(Msgs* msgs);
// put in a message into stack
int pull(Msgs* msgs, Msg* msg);
// delet top message 
int pop(Msgs* msgs, Msg* msg);
// add string into a message
Msg* join(Msg* msg, char* str);
// change int into string
char* int2str(int num);
/***********************************************/
/*             main function                   */
/***********************************************/
void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Initializing devices....");
    setup_driving();
    setup_pid();
    //Calibration();
    //delay(100000);
}

void loop() {
    /*test demo for motor*/
    //set_motor(255,255);
    
    /*test demo for imu*/
     while(!imuInterrupt && fifoCount < packetSize){
      if(imuInterrupt && fifoCount < packetSize){
        fifoCount = mpu.getFIFOCount();
      }
      //Serial.println("run motor!");
      start(&timer,10);
      update(&timer);
      if(!timer.isEnd)continue;
//      pwm_val = pid2(0.2);
//      Serial.print("pwm: ");
//      Serial.println(pwm_val);

      reset(&timer);
      //Serial.println(pwm_val);
      //hprintImu();
      //printPID();
     }

     imuInterrupt = false;
     imu.imuInterruptState = mpu.getIntStatus();
     fifoCount = mpu.getFIFOCount();

     if((imu.imuInterruptState & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024){
      mpu.resetFIFO();
     }
     else if(imu.imuInterruptState & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
      while (fifoCount >= packetSize){
        mpu.getFIFOBytes(fifoBuffer,packetSize);
        fifoCount -= packetSize;
      }
      
      updateImu();
      /**turning method example**/
//      if(!isStartTurn&&counter<1){
//        lastDeg = imu.ypr[0]*180/M_PI+180;
//        isStartTurn = 1;
//        Serial.println("turnning");
//      }
//      if(counter>1){
//        stand();
//        Serial.println("standing");
//      }
//      int tempResult = turn(90);
//      if(tempResult&&isStartTurn){
//        isStartTurn = 0;
//        counter=2;
//        Serial.println("ending");
//      }
      /**move forward**/
//      start(&timer,5000);
//      if(timer.isEnd){
//        stand();
//        stop(&timer);
//      }
//      else{
//        forward();
//      }
//      update(&timer);
      /**stop**/
      stand();
      #ifdef __PRINT_IMU__
      printImu();
      #endif
     }
}

/***********************************************/
/*           realize of functions              */
/***********************************************/
void setup_driving(){
    Serial.println("initializing sensors....");
    pinMode(IR_FRONT,INPUT);
    pinMode(IR_LEFT,INPUT);
    pinMode(IR_RIGHT,INPUT);
    pinMode(LMOTOR_DIR1,OUTPUT);
    pinMode(LMOTOR_DIR2,OUTPUT);
    pinMode(LMOTOR_PWM,OUTPUT);
    pinMode(RMOTOR_DIR1,OUTPUT);
    pinMode(RMOTOR_DIR2,OUTPUT);
    pinMode(RMOTOR_PWM,OUTPUT);

    //max_limit = MAX_DEG*KP+MAX_I_LIMIT*KI*0.01+KD*MAX_DEG/0.01;
    
    Serial.println("initializing mpu....");
    mpu.initialize();
    pinMode(INTERRUPT,INPUT);
    if(mpu.testConnection()){
        Serial.println("connected!");
    }
    else{
        Serial.println("connect failed!");
    }
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    delay(500);
    imu.devState = mpu.dmpInitialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setXGyroOffset(X_GYRO_OFFSET);
    mpu.setYGyroOffset(Y_GYRO_OFFSET);
    mpu.setZGyroOffset(Z_GYRO_OFFSET);
    mpu.setXAccelOffset(X_ACC_OFFSET);
    mpu.setYAccelOffset(Y_ACC_OFFSET);
    mpu.setZAccelOffset(Z_ACC_OFFSET);
    if(!imu.devState){
      #ifdef __AUTOCALIBRATION__
      calibration();
      #endif
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT),dmpReady,RISING);
      imu.imuInterruptState = mpu.getIntStatus();
      imu.dmpState = true;
      
      packetSize = mpu.dmpGetFIFOPacketSize();
      imu.euler = (float*)malloc(sizeof(float)*3);
      imu.ypr = (float*)malloc(sizeof(float)*3);
    }
    else{
      Serial.println("DMP initaled failed");
    }
}

void calibration()
{
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      #ifdef __PRINT_OFFSETS__
      mpu.PrintActiveOffsets();
      #endif
}

void releaseImu(){
    free(imu.euler);
    free(imu.ypr);
}

void updateImu(){
      mpu.dmpGetQuaternion(&imu.quater,fifoBuffer);
      mpu.dmpGetGravity(&imu.gravity,&imu.quater);
      mpu.dmpGetYawPitchRoll(imu.ypr,&imu.quater,&imu.gravity);
      //imu.gyroX = mpu.getRotationX();
      //imu.gyroX_f = (float)imu.gyroX/32.8;
}

int now  = millis();
int last = 0;

void printImu(){
//    last = now;
//    now = millis();
//    Serial.print(now-last);
//    Serial.print(", ");
//    Serial.print(F("pitch: "));
    Serial.print(pwm_val);
    Serial.print(", ");
    Serial.print(set_val+mypid.composVal);
    Serial.print(", ");
    Serial.print(imu.ypr[1]*180/M_PI);
    Serial.print(", ");
    Serial.print(imu.ypr[0]*180/M_PI);
    Serial.println();
    //Serial.print(F("yaw: "));
    //Serial.println(imu.ypr[0]*180/M_PI);
    //Serial.print(F("roll: "));
    //Serial.println(imu.ypr[2]*180/M_PI);
}

float error2pwm(float val){
  
  if(val>=0){
    val = val+60;
    return val;
  }
  else{
    val = val-60;
    return val;
  }
}

void set_motor(int left,int right){
//  if(left>=0){
//    left = constrain(left,0,255);
//  }
//  else{
//    left = constrain(left,-255,0);
//  }
//  if(right>=0){
//    right = constrain(right,0,255);
//  }
//  else{
//    right = constrain(right,-255,0);
//  }
    left = constrain(left,-255,255);
    right = constrain(right,-255,255);
    if(left>=0){
        set_lmotor_dir(LOW,HIGH);
        analogWrite(LMOTOR_PWM,left);
    }
    else{
      set_lmotor_dir(HIGH,LOW);
      analogWrite(LMOTOR_PWM,abs(left));
    }
    if(right>=0){
      set_rmotor_dir(LOW,HIGH);
      analogWrite(RMOTOR_PWM,abs(right));

    }
    else{
      set_rmotor_dir(HIGH,LOW);
      if(right>-40&&right<-10)
        //analogWrite(RMOTOR_PWM,abs(right+5));
        analogWrite(RMOTOR_PWM,abs(right));
      else
        analogWrite(RMOTOR_PWM,abs(right));
    }
    #ifdef __PRINT_MOTOR__
    Serial.print("l: ");
    Serial.print(left);
    Serial.print(", ");
    Serial.print("r: ");
    Serial.print(right);
    Serial.println();
    #endif
}

void set_lmotor_dir(int one,int two){
    digitalWrite(LMOTOR_DIR1,one);
    digitalWrite(LMOTOR_DIR2,two);
}
void set_rmotor_dir(int one,int two){
    digitalWrite(RMOTOR_DIR1,one);
    digitalWrite(RMOTOR_DIR2,two);
}

void stepmotor(int turnSpeed){
    pwm_val = KR*pid(set_val); //1.5
    if(set_val == 0){
      if(pwm_val > 0)
        mypid.composVal += 0.0015;
      else{
        mypid.composVal -= 0.0015;
      }
    }
    #ifdef __TEST__
    set_motor(-turnSpeed+pwm_val,turnSpeed+pwm_val);
    #endif
    #ifdef __STOP__
    set_motor(0,0);
    #endif
}

void stand(){
  if(set_val > 0.5){
    set_val -= 0.005;
  }
  else if(set_val < -0.5){
    set_val += 0.005; 
  }
  else
    set_val = 0;
  //pwm_val = error2pwm(pwm_val);
  stepmotor(0);
}

void forward(){
    if (set_val <2.5 && set_val > -2.5){
      if(set_val >=0){
        set_val+=0.05;
      }
      else{
        set_val -= 0.05;
      }
    }
    stepmotor(0);
}

int turn(float deg){
  newDeg = imu.ypr[0]*180/M_PI+180;
  if(abs(newDeg-lastDeg)<=deg){
    stepmotor(30);
    return 0;
  }
  else{
    //stand();
    return 1;
  }
}

void setup_pid(){
    mypid.curVal = 0;
    mypid.preVal = 0.0;
    mypid.lastError = 0.0;
    mypid.error = 0.0;
    mypid.sumError = 0.0;
//    pid->Kp = KP;
//    pid->Ki = KI;
//    pid->Kd = KD;
}


float pid(float setVal){
    //mypid.preVal = mypid.curVal;
    mypid.curVal = imu.ypr[1]*180/M_PI;
    //mypid.curVal = imu.ypr[1];
    if(mypid.curVal>=30 || mypid.curVal <=-30){
      mypid.error = 0;
      mypid.sumError = 0;
      mypid.lastError = 0;
      mypid.composVal = 0;
      return 0;
    }
    constrain(mypid.composVal,-2,2);
    mypid.error = setVal-mypid.curVal+mypid.composVal;
    mypid.preVal = mypid.error-mypid.lastError;
    mypid.sumError += mypid.error;
    mypid.sumError = constrain(mypid.sumError,-400,400);
    float temp = (KP*mypid.error)+(KI*mypid.sumError)+(KD*mypid.preVal);
    mypid.lastError = mypid.error;
    
    return temp;
}

void printPID(){
    Serial.print("error is: ");
    Serial.println(mypid.error);
//    Serial.print("sum of error is: ");
//    Serial.println(mypid.sumError);
}

void start(Timer* timer, int times){
    if(timer->isStart==0){
        timer->set = times;
        timer->now = 0;
        timer->isStart = 1;
        timer->isEnd = 0;
        timer->time_stamp1= millis();
    }
}

void update(Timer* timer){
    if(timer->isStart==1&&timer->isEnd!=1){
        timer->time_stamp2 = millis();
        timer->now = timer->time_stamp2-timer->time_stamp1;
    }
    if((timer->now)>=(timer->set)){
        timer->isEnd=1;
    }
}

void stop(Timer* timer){
    timer->isEnd=1;
}

void reset(Timer* timer){
    timer->now=0;
    timer->isStart=1;
    timer->isEnd=0;
    timer->time_stamp1 = millis();
}

void setBt(){
    Serial3.begin(9600);
    Serial1.begin(9600);
    Serial3.print("AT");
    delay(1000);
    printMsg();
    Serial3.print("AT+NameGROUP3");
    delay(1000);
    printMsg();
    Serial3.print("AT+PIN1234");
    delay(1000);
    printMsg();
    Serial1.println("You are connected");
    delay(1000);
}

void printMsg(){
    char c = ' ';
    if(Serial3.available()>0){
        while(Serial3.available()){
            c = Serial3.read();
            Serial1.write(c);
        }
    }
    delay(1000);
}

Msgs* create_msgs(int size){
    Msgs* msgs = (Msgs*)malloc(sizeof(Msgs));
    Msg** ptemp = (Msg**)malloc(sizeof(Msg*)*size);
    msgs->size = size;
    msgs->pbuffer = ptemp;
    msgs->pos = -1;
    return msgs;
}

Msg* create_msg(char* str, int size){
    Msg* msg = (Msg*)malloc(sizeof(Msg));
    char* ptemp = (char*)malloc(sizeof(char)*size);
    msg->pbuffer = ptemp;
    strcpy(msg->pbuffer,str);
    msg->length = size;
    msg->pos = -1;
    return msg;
}

void release_msgs(Msgs* msgs){
    if (msgs->pos>-1){
        for(int i=0;i<=msgs->pos;++i)
            release_msg(msgs->pbuffer[i]);
    }
    msgs->size=0;
    msgs->pos=-1;
    free(msgs->pbuffer);
}

void release_msg(Msg* msg){
    free(msg->pbuffer);
}

void send(Msgs* msgs,int num){
    if(msgs->pos>-1){
        if (num == 0)
            Serial.write(msgs->pbuffer[msgs->pos]->pbuffer);
        else if(num == 1)
        {
            Serial1.write(msgs->pbuffer[msgs->pos]->pbuffer);
        }
        else if(num == 2){
            Serial2.write(msgs->pbuffer[msgs->pos]->pbuffer);
        }
        else if(num == 3){
            Serial3.write(msgs->pbuffer[msgs->pos]->pbuffer);
        }
        release_msg(msgs->pbuffer[msgs->pos]);
        --(msgs->pos);
    }
}

int pull(Msgs* msgs, Msg* msg){
    if(msgs->pos<(msgs->size-1)){
        msgs->pbuffer[++(msgs->pos)]=msg;
        return msgs->pos;
    }
    else{
        return -1;
    }
}

int pop(Msgs* msgs, Msg* msg){
    if(msgs->pos>-1){
        release_msg(msgs->pbuffer[msgs->pos]);
        msgs->pbuffer[msgs->pos]=NULL;
        --(msgs->pos);
        return msgs->pos;
    }
    else
    {
        return -1;
    }
    
}

Msg* join(Msg* msg, char* str){
    int temp_num = strlen(str);
    if (msg->pos+temp_num>msg->length)
        return msg;
    else{
        strcat(msg->pbuffer,str);
    }
    return msg;
}
/*
char* int2str(int num){
    char temp[10];
    itoa(num, temp, 10);
    return temp;
}*/
