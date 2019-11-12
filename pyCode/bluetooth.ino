
void printMsg();

void setup(){
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
}

void loop(){
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