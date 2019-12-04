#include "driving.h"

MPU6050 mpu;
Imu imu;

// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

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
    //pinMode(INTERUPT,INPUT);
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
    
    if(!mpu.dmpInitialize()){
      Serial.println("mpu initial failed!!");
    }
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(X_ACC_OFFSET);
    mpu.setYAccelOffset(Y_ACC_OFFSET);
    mpu.setZAccelOffset(Z_ACC_OFFSET);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT),dmpReady,RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    imu.euler = (float*)malloc(sizeof(float)*3);
    imu.ypr = (float*)malloc(sizeof(float)*3);
//    imu->quater = &Quaternion();
//    imu->acc = &VectorInt16();
//    imu->gyro = &VectorInt16();
//    imu->gravity = &VectorFloat();
    imu.dmpState = true;
}


void releaseImu(Imu* imu){
    free(imu->euler);
    free(imu->ypr);
}

void updateImu(){
    if(imu.dmpState){
      while(fifoCount<packetSize)
        fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer,packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(imu.quater,fifoBuffer);
      mpu.dmpGetGravity(imu.gravity,imu.quater);
      mpu.dmpGetYawPitchRoll(imu.ypr,imu.quater,imu.gravity);
    }
}

void printImu(){
    Serial.print(F("pitch: "));
    Serial.println(imu.ypr[1]*180/M_PI);
    Serial.print(F("yaw: "));
    Serial.println(imu.ypr[0]*180/M_PI);
    Serial.print(F("roll: "));
    Serial.println(imu.ypr[2]*180/M_PI);
}

void set_motor(int left,int right){
    left = constrain(left,-255,255);
    right = constrain(right,-255,255);
    if(left<0){
        //adjust here during test
        set_lmotor_dir(1,0);
    }
    if(right<0){
        set_rmotor_dir(1,0);
    }
    analogWrite(LMOTOR_PWM,left);
    analogWrite(RMOTOR_PWM,right);
}

void set_lmotor_dir(int one,int two){
    digitalWrite(LMOTOR_DIR1,one);
    digitalWrite(LMOTOR_DIR2,two);
}
void set_rmotor_dir(int one,int two){
    digitalWrite(RMOTOR_DIR1,one);
    digitalWrite(RMOTOR_DIR2,two);
}

void setup_pid(myPID* pid){
    pid->curVal = 0.0;
    pid->preVal = 0.0;
    pid->lastError = 0.0;
    pid->error = 0.0;
    pid->sumError = 0.0;
//    pid->Kp = KP;
//    pid->Ki = KI;
//    pid->Kd = KD;
}

int pid(myPID* pid, Imu* imu,float setVal){
    pid->preVal = pid->curVal;
    pid->curVal = imu->ypr[1];
    pid->lastError = pid->error;
    pid->error = pid->curVal-setVal;
    pid->sumError += pid->error;
    pid->sumError = constrain((pid->sumError),-500,500);
    int temp = (pid->Kp*pid->error)+(pid->Ki*pid->sumError)+(pid->Kd*(pid->error-pid->lastError));
    return temp;
}

void printPID(myPID* pid){
    Serial.print("error is: ");
    Serial.println(pid->error);
    Serial.print("sum of error is: ");
    Serial.println(pid->sumError);
}
