#ifndef TIMER_H
#define TIMER_H

#define BASE_TIME 5

#include <arduino.h>

typedef struct TIMER{
    int set;
    int now;
    unsigned long time_stamp1;
    unsigned long time_stamp2;
    int isEnd;
    int isStart;
}Timer;

void start(Timer* timer, int time);
void update(Timer* timer);
void stop(Timer* timer);
void reset(Timer* timer);

#endif
