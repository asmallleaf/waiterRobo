#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arduino.h>

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
#endif
