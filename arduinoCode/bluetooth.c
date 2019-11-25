#include "bluetooth.h"

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

void create_msgs(Msgs* msgs, int size){
    Msg* ptemp = (Msg*)malloc(sizeof(Msg)*size);
    msgs->size = size;
    msgs->pbuffer = ptemp;
    msgs->pos = -1;
}

void create_msg(Msg* msg,int size){
    char* ptemp = (char*)malloc(sizeof(char)*size);
    msg->length = size;
    msg->pos = -1;
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

void send(Msgs* msgs){
    if(msgs->pos>-1){
        Serail3.write(msgs->pbuffer[msgs->pos].pbuffer);
        release_msg(msgs->pbuffer[msgs->pos]);
        --(msgss->pos);
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

char* int2str(int num){
    char temp[10];
    itoa(num, str, 10);
    return temp;
}