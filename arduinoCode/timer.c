#include "timer.h"

void start(Timer* timer, int time){
    timer->set = time%BASE_TIME;
    timer->now = 0;
    timer->isStart = 1;
    timer->isEnd = 0;
}

void update(Timer* timer){
    if(timer->isStart==1&&timer->isEnd!=1){
        ++timer->now;
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
}