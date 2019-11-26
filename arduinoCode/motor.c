#include "motor.h"

void setmotor(int left,int right){
    left = constrain(left,0,255);
    right = constrain(right,0,255);
    analogWrite(LMOTOR,left);
    analogWrite(RMOTOR,right);
}

int pid(PidOut* outs){
    float diff = outs->error-outs->last_error;
    outs->last_error = outs->error;
    outs->error = outs->measured-outs->setted;
    int output = (int)(KI*temp+KD*(diff));
    return output;
}