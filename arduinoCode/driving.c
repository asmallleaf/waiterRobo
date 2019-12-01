#include "driving.h"

void setup_driving(Imu* imu, MPU6050* mpu6050){
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
    mpu6050->initialize();
    if(mpu6050->testConnection()){
        Serial.println("connected!");
    }
    else{
        Serial.println("connect failed!");
    }
    initImu(imu,mpu6050);
}

void initImu(Imu* imu,MPU6050* mpu6050){
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    imu->mpu = mpu6050;
    imu->mpu->setXAccelOffset(X_ACC_OFFSET);
    imu->mpu->setYAccelOffset(Y_ACC_OFFSET);
    imu->mpu->setZAccelOffset(Z_ACC_OFFSET);
    imu->yAcc = 0;
    imu->zAcc = 0;
    imu->xGyro = 0;
    imu->preAngleGyro = 0.0;
    imu->curAngle = 0.0;
    imu->angleAcc = 0.0;
    imu->angleGyro = 0.0;
    imu->dt = 0.005;
}

void updateImu(Imu* imu){
    imu->yAcc = imu->mpu->getAccelerationY();
    imu->zAcc = imu->mpu->getAccelerationZ();
    imu->yGyro = imu->mpu->getRotationX();
    imu->angleAcc = atan2(imu->yAcc,imu->zAcc)*57.297;
    imu->preAngleGyro = imu->angleGyro;
    imu->angleGyro = (float)map(imu->xGyro,-32768,32768,-250,250);
    imu->curAngle = 0.9934*(imu->preAngleGyro+imu->angleGyro*imu->dt)+0.0066*(imu->angleAcc);
}

void printImu(Imu* imu){
    Serial.println("current angle is:%f",imu->curAngle);
    Serial.println("yAcc is: %d",imu->yAcc);
    Serial.println("zAcc is: %d",imu->zAcc);
    Serial.println("yGyro is: %d",imu->yGyro);
    Serial.println("angleAcc is: %f",imu->angleAcc);
    Serial.println("angleGyro is: %f",imu->angleGyro);
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

void setup_pid(Pid* pid,int KP,int KI,int KD){
    pid->curVal = 0.0;
    pid->preVal = 0.0;
    pid->lastError = 0.0;
    pid->error = 0.0;
    pid->sumError = 0.0;
    pid->Kp = KP;
    pid->Ki = KI;
    pid->Kd = KD;
}

int pid(Pid* pid, Imu* imu,float setVal){
    pid->preVal = pid->curVal;
    pid->curVal = imu->curAngle;
    pid->lastError = pid->error;
    pid->error = pid->curVal-setVal;
    pid->sumError += pid->error;
    pid->sumError = constrain((pid->sumError),-500,500);
    int temp = (pid->Kp*pid->error)+(pid->Ki*pid->sumError*imu->dt)+(pid->Kd*(pid->error-pid->lastError)/imu->dt);
    return temp;
}

void printPID(Pid* pid){
    Serial.println("error is: %f",pid->error);
    Serial.println("sum of error is: %f",pid->sumError);
}