#include "timer.h"

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
