### Log Report
------
__This is log of programming of ACS6502 Project__
__Any changing of the codes should be logged here in formal style__
__Enjoy codes!!__

----
#### File structure
```
- launch.ino
- bluetooth.h
- bluetooth.c
``` 
---
#### Notes
__actually a chatting board__
1.Rule of version number(__A.XY+LTS__)
+ A is the structure version of codes
+ X can be 0 or 1, 0 means has not been tested, 1 means has been tested
+ Y is the updating version
+ LTS represents that it is the final version
+ e.g. 2.04 means the structure of codes has been changed for one version, it is the forth update of this structure. It has not been tested.


---
#### Update log
[11.19] Weng:
```c
+ API of bluetooth has been updated. The current version is 1.01
  new methods are:
    void setBt(); // initial bluetooth
    void printMsg(); // print initial msg
    void create_msgs(Msgs* msgs,int size); // create message stack
    void create_msg(Msg* msg,int size); // create a message
    void release_msgs(Msgs* msgs); // clear the whole stack
    void release_msg(Msg* msg); // delet the message
    void send(Msgs* msgs); // send a message on the top of stack
    int pull(Msgs* msgs, Msg* msg); // put in a message into stack
    int pop(Msgs* msgs, Msg* msg); // delet top message 
    Msg* join(Msg* msg, char* str); // add string into a message
    char* int2str(int num); // change int into string
+ bluetooth.h and bluetooth.c has been uploaded. 
```