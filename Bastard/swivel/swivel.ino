#include <Servo.h>
#define swivelPin 10

//ultrasonic
#define trigger 8
#define echo 7
long distanceInCM;

Servo swivel;

bool left,front,right = false;

void setup(){

    Serial.begin(9600);
    pinMode(echo,INPUT);
    pinMode(trigger, OUTPUT);

}

void loop(){

    measureDistance();

}

void resetSwivel(){

    swivel.attach(10);
    swivel.write(90);
}
void lookRight(){
    int pos = swivel.read(); // SHOULD BE 90
    for(int i = pos; i > 0; i--){
        swivel.write(i);
    }
    delay(1000);
    if(measureDistance() <= 10){
        right = true;
    }
    else{
        right = false;
    }
}
void lookLeft(){

    int pos = swivel.read(); // SHOULD BE 90
    for(int i = pos; i <= 180; i++){
        swivel.write(i);
    }
    delay(1000);
    if(measureDistance() <= 10){
        left = true;
    }
    else{
        left = false;
    }

}
void lookAround(){
    lookLeft();
    delay(1000);
    resetSwivel();
    delay(1000);
    lookRight();
    delay(1000);
    resetSwivel();
    delay(1000);
}

String evaluate(){
    if(front == true && left == true && right == true){
        return "Walls everywhere!";
    }
    else if (front == false)
    {
        return "I can go forward!";
    }
    else if (front == true && left == false && right == true)
    {
        return "I can go left!";
    }
    else if (front == true && left == true && right == false)
    {
        return "I can go right";
    }
    else if (front == true && left == false && right == false)
    {
        return "I can go right or left!";
    }
    else{
        return "No walls detected!";
    }
    
}
//ultrasonic
long measureDistance(){
    digitalWrite(trigger,LOW);
    delayMicroseconds(5);
    digitalWrite(trigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger,LOW);

    distanceInCM = (pulseIn(echo,HIGH)/2) / 29.1;
    Serial.println(distanceInCM);
    return distanceInCM;
}

void printDistance(long distanceInCM){
    delay(2000);
    Serial.print(distanceInCM);
    Serial.println("cm");
}