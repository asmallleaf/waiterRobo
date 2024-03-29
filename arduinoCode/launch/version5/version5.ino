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
#define INTERRUPT 2
/*Motor config*/
#define LMOTOR_DIR1 4
#define LMOTOR_DIR2 5
#define LMOTOR_PWM 6
#define RMOTOR_DIR1 7
#define RMOTOR_DIR2 8
#define RMOTOR_PWM 9
/*offset of IMU*/
#define X_ACC_OFFSET 60
#define Y_ACC_OFFSET 19
#define Z_ACC_OFFSET 16375
#define X_GYRO_OFFSET -3314
#define Y_GYRO_OFFSET 0
#define Z_GYRO_OFFSET 1688//0
/*basic define constant*/
#define BASE_TIME 0.05
#define KP 25
#define KI 0
#define KD 0.02
const float ANGLE2PWM_L=0.6;
const float ANGLE2PWM_R=0.6;
#define MIN_PWM_L 25
#define MIN_PWM_R 27
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
};

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
Timer timer;

// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int pwm_val=0;
volatile bool imuInterrupt = false; // variable in interrupt

//version 5
int16_t gyro[3]={};
float gyro_f[3]={};
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
void releaseImu();
void updateImu();
void printImu();
void Calibration();

//Motor operation
void set_motor(int left,int right);
void set_lmotor_dir(int one,int two);
void set_rmotor_dir(int one,int two);

//PID calculation
void setup_pid();
int pid(float setVal);
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
    updateImu();
    //Serial.println("run motor!");
//    start(&timer,10);
//    update(&timer);
//    if(!timer.isEnd)continue;
    pwm_val = pid(0.2);
    Serial.print("pwm: ");
    Serial.println(pwm_val);
    set_motor(pwm_val,pwm_val);
//    reset(&timer);
    //Serial.println(pwm_val);
    //set_motor(0,0);
    //printImu();
    //printPID();      
    //printImu();
     
     delay(100);
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
    
    Serial.println("initializing mpu....");
    mpu.initialize();
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
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setXGyroOffset(X_GYRO_OFFSET);
    mpu.setYGyroOffset(Y_GYRO_OFFSET);
    mpu.setZGyroOffset(Z_GYRO_OFFSET);
    mpu.setXAccelOffset(X_ACC_OFFSET);
    mpu.setYAccelOffset(Y_ACC_OFFSET);
    mpu.setZAccelOffset(Z_ACC_OFFSET);
    if(!imu.devState){
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
    }
    else{
      Serial.println("DMP initaled failed");
    }
}

void Calibration()
{
  #define CALIBTIME 1000
  #define CALIBSIZE 7
  int calibData[CALIBSIZE];
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //sum
  for (int i = 0; i < CALIBTIME; ++i) {
    int mpuVals[CALIBSIZE];
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.requestFrom(0x68, CALIBSIZE * 2, true);
    Wire.endTransmission(true);
    for (long i = 0; i < CALIBSIZE; ++i) {
      mpuVals[i] = Wire.read() << 8 | Wire.read();
    }
    for (int j = 0; j < CALIBSIZE; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //average
  for (int i = 0; i < CALIBSIZE; ++i) {
    calibData[i] = int(valSums[i] / CALIBTIME);
    Serial.println(calibData[i]);
  }
  //calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

//bool check_Int(){
//  if(!imu.dmpState){
//    return true;
//  }
//  if(!imuInterrupt && fifoCount < packetSize){
//    if(imuInterrupt && fifoCount < packetSize){
//      fifoCount = mpu.getFIFOCount();
//      return false;
//    }
//    else{
//      return true;
//    }
//  }
//  else
//    return true;
////  if(fifoCount < packetSize){
////    fifoCount = mpu.getFIFOCount();
////    return false;
////  }
////  else{
////    return true;
////  }
//}
//
//bool collect_Int(){
//  imuInterrupt = false;
//  imu.imuInterruptState = mpu.getIntStatus();
//  fifoCount = mpu.getFIFOCount();
//
//  if((imu.imuInterruptState & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024){
//    Serial.println("here1");
//    mpu.resetFIFO();
//    //fifoCount = mpu.getFIFOCount();
//    return false;
//  }
//  else if(imu.imuInterruptState & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
//    while(fifoCount < packetSize){
//      fifoCount = mpu.getFIFOCount();
//    }
//    mpu.getFIFOBytes(fifoBuffer,packetSize);
//    fifoCount -= packetSize;
//    return true;
//  }
////    while(fifoCount < packetSize){
////      fifoCount = mpu.getFIFOCount();
////    }
////    mpu.getFIFOBytes(fifoBuffer,packetSize);
////    fifoCount -= packetSize;
//    //return true;
//  //Serial.println("here2");
//  return false;
//}

void releaseImu(){
    free(imu.euler);
    free(imu.ypr);
}

void updateImu(){
    gyro[0]=mpu.getRotationX();
    gyro[1]=mpu.getRotationY();
    gyro[2]=mpu.getRotationZ();
    gyro_f[0]=gyro[0]/32.8;
    gyro_f[1]=gyro[1]/32.8;
    gyro_f[2]=gyro[2]/32.8;
}

void printImu(){
    Serial.print(F("x: "));
    Serial.println(gyro[0]/32.8);
    //Serial.print(F("y: "));
    //Serial.println(gyro[1]);
    //Serial.print(F("z: "));
    //Serial.println(gyro[2]);
}

void set_motor(int left,int right){
    left = constrain(left,-255,255);
    right = constrain(right,-255,255);
    if(left>=0){
        //adjust here during test
        set_lmotor_dir(LOW,HIGH);
        analogWrite(LMOTOR_PWM,left);
    }
    else{
      set_lmotor_dir(HIGH,LOW);
      analogWrite(LMOTOR_PWM,-left);
    }
    if(right>=0){
      //adjust here during test
      set_rmotor_dir(LOW,HIGH);
      analogWrite(RMOTOR_PWM,right);
    }
    else{
      set_rmotor_dir(HIGH,LOW);
      analogWrite(RMOTOR_PWM,-right);
    }
//    left = abs(constrain(ANGLE2PWM_L*left,-255,255));
//    right = abs(constrain(ANGLE2PWM_R*right,-255,255));
//    left = max(left,MIN_PWM_L);
//    right = max(right,MIN_PWM_R); 
//    Serial.print(left);
//    Serial.print("  ");
//    Serial.println(right);
//    analogWrite(LMOTOR_PWM,left);
//    analogWrite(RMOTOR_PWM,right);
}

void set_lmotor_dir(int one,int two){
    digitalWrite(LMOTOR_DIR1,one);
    digitalWrite(LMOTOR_DIR2,two);
}
void set_rmotor_dir(int one,int two){
    digitalWrite(RMOTOR_DIR1,one);
    digitalWrite(RMOTOR_DIR2,two);
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

int pid(float setVal){
    //mypid.preVal = mypid.curVal;
    mypid.curVal = gyro_f[0];
    mypid.error = setVal-mypid.curVal;
    mypid.preVal = mypid.error-mypid.lastError;
    //Serial.println(mypid.preVal);
    //if(mypid.preVal>0.25) return mypid.error = mypid.lastError;
    mypid.sumError += mypid.error;
    mypid.sumError = constrain((mypid.sumError),-300,300);
    int temp = (KP*mypid.error)+(KI*(mypid.sumError))+(KD*mypid.preVal);
    mypid.lastError = mypid.error;
    return temp;
}

void printPID(){
    Serial.print("error is: ");
    Serial.println(mypid.error);
    Serial.print("sum of error is: ");
    Serial.println(mypid.sumError);
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
