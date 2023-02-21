//libaries
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//gripper servo
Servo gripper;
int gripperPosition;
// swivel servo 
Servo swivel;
#define swivelPin 10
bool left,front,right = false;

//ultrasonic
#define trigger 8
#define echo 7
long distanceInCM;

//wheels
#define motorLeftForward 6
#define motorLeftBackwards 5
#define motorRightForward 3
#define motorRightBackwards 11
#define sensorRight 1
#define sensorLeft 2

//neo pixel
#define pixelPin 4
#define pixelCount 4
Adafruit_NeoPixel neoPixel(pixelCount, pixelPin, NEO_GRB + NEO_KHZ800);


void setup(){
    Serial.begin(9600);
    //Neo Pixel
        neoPixel.begin();
    //gripper
        pinMode(9,INPUT);
    //swivel
        pinMode(swivelPin,INPUT);
    //wheels
        pinMode(motorLeftForward,INPUT); 
        pinMode(motorLeftBackwards,INPUT);
        pinMode(motorRightForward,INPUT);
        pinMode(motorRightBackwards,INPUT);
        pinMode(sensorLeft,OUTPUT);
        pinMode(sensorRight,OUTPUT);
    //ultrasonic
        pinMode(trigger,OUTPUT);
        pinMode(echo,INPUT);
    //reflecance sensor

    //methods
    swivel.attach(swivelPin);
    swivel.write(90);
}

void loop(){
    
    forward();
    lightGreen();
    if(measureDistance() <= 15){
        front = true;
        stop();
        lightRed();
        lookAround();
        if(evaluate() == "I can go right!" || evaluate() == "I can go right or left!"){
            turnRight();
        }
        else if (evaluate() == "I can go left!")
        {
            turnLeft();
        }
        else if (evaluate() == "Walls everywhere!"){
            stop();
            
        }
        
    }
    else{
        front = false;
    }
    Serial.println(evaluate());
    delay(500);
}


// gripper methods
void openGrip(){
    gripper.attach(9);
    for(int pos = 40; pos >= 120; pos++){
        gripper.write(pos);
        delay(15);
    }
    gripper.detach();
}

void closeGrip(){
    gripper.attach(9);
    for(int pos = 120; pos >= 40; pos--){
        gripper.write(pos);
        delay(15);
    }
}

// swivel methods
void resetSwivel(){

    swivel.attach(10);
    swivel.write(90);
}
void lookRight(){
    int pos = swivel.read(); // SHOULD BE 90
    for(int i = pos; i >= 0; i--){
        swivel.write(i);
    }
    delay(1000);
    if(measureDistance() <= 15){
        right = true;
    }
    else{
        right = false;
    }
}
void lookLeft(){

    int pos = swivel.read(); // SHOULD BE 90
    for(int i = pos; i <= 190; i++){
        swivel.write(i);
    }
    delay(1000);
    if(measureDistance() <= 15){
        left = true;
    }
    else{
        left = false;
    }

}
void lookAround(){
    lookLeft();
    delay(500);
    resetSwivel();
    delay(500);
    lookRight();
    delay(500);
    resetSwivel();
    delay(500);
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
    delayMicroseconds(swivelPin);
    digitalWrite(trigger,LOW);

    distanceInCM = (pulseIn(echo,HIGH)/2) / 29.1;
    return distanceInCM;
}

void printDistance(long distanceInCM){
    delay(2000);
    Serial.print(distanceInCM);
    Serial.println("cm");
}

//wheels
void forward(){
    analogWrite(motorLeftForward,200);
    analogWrite(motorRightForward,200);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackwards,0);
    analogWrite(motorRightBackwards,0);
}

void turnLeft(){
    stop();
    analogWrite(motorLeftBackwards,200);
    analogWrite(motorRightForward,200);
    delay(600);
    stop();
}

void turnRight(){
    stop();
    analogWrite(motorRightBackwards,200);
    analogWrite(motorLeftForward,200);
    delay(600);
    stop();
}

void measureWheelSpeed(){
    int sensorLeftRead = analogRead(sensorLeft)/4;
    int sensorRightRead = analogRead(sensorRight)/4;
    while(sensorLeftRead > 200 && sensorRightRead > 200){
        Serial.print("Left Wheel: ");
        Serial.println(sensorLeftRead);
        Serial.print("Right Wheel: ");
        Serial.println(sensorRightRead);
    }
}

//pixel lights
void lightRed(){
    for(int i = 0; i<pixelCount; i++){
        neoPixel.setPixelColor(i,neoPixel.Color(0,255,0));
        neoPixel.show();
    }
}

void lightGreen(){
    for(int i = 0; i<pixelCount; i++){
        neoPixel.setPixelColor(i,neoPixel.Color(200,0,50));
        neoPixel.show();
    }
}

//reflectance sensor