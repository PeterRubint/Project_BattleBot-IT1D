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
bool swivelReset = true;

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
        pinMode(10,INPUT);
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
    swivel.attach(10);
     if(swivel.read() != 90){
            swivel.write(90);
            swivel.detach();
            
    }
    stop();
}

void loop(){
    
    while (measureDistance()>10 && measureDistance() != 0){
        lightGreen();
        forward();
        if(distanceInCM<=10){
            lightRed();
            stop();
            lookAround();
        }
    }
}

//logic 


// gripper methods
void openGrip(){
    gripper.attach(2);
    for(int pos = 40; pos >= 120; pos++){
        gripper.write(pos);
        delay(15);
    }
    gripper.detach();
}

void closeGrip(){
    gripper.attach(2);
    for(int pos = 120; pos >= 40; pos--){
        gripper.write(pos);
        delay(15);
    }
}

// swivel methods
void lookAround(){
    swivel.attach(10);
    int currentPos = swivel.read();
    if(currentPos != 180){
        for(int pos = currentPos; pos <= 180; pos++){
            swivel.write(pos);
        }
    }
    if(currentPos == 180){
        for(int pos = currentPos; pos > 0; pos--){
            swivel.write(pos);
        }
    }
    swivel.detach();
}

void resetSwivel(){

    swivel.attach(10);
    swivel.write(90);
    swivel.detach();
}
//ultrasonic
long measureDistance(){
    digitalWrite(trigger,LOW);
    delayMicroseconds(5);
    digitalWrite(trigger,HIGH);
    delayMicroseconds(10);
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
    analogWrite(motorLeftForward,255);
    analogWrite(motorRightForward,255);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackwards,0);
    analogWrite(motorRightBackwards,0);
}

void turnLeft(){
    stop();
    analogWrite(motorLeftBackwards,255);
    analogWrite(motorRightForward,255);
    delay(400);
    stop();
}

void turnRight(){
    stop();
    analogWrite(motorRightBackwards,255);
    analogWrite(motorLeftForward,255);
    delay(400);
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