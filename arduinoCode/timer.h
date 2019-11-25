#ifndef TIMER_H
#define TIMER_H

#define BASE_TIME 20

typedef structure TIMER{
    int set;
    int now;
    int isEnd;
    int isStart;
}Timer;

void start(Timer* timer, int time);
void update(Timer* timer);
void stop(Timer* timer);
void reset(Timer* timer);

#endif