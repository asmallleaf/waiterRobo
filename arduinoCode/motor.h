#ifndef MOTOR_H
#define MOTOR_H

/*motor pin config*/
//#define LMOTOR
//#define RMOTOR
/*define P,D parameters*/
//#define KP
//#define KD

typedef struct PIDOUT{
    float measured;
    float setted;
    float error;
    float last_error;
}PidOut;

void setmotor(int left,int right);
int pid(PidOut* outs);

#endif
