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
#define trigger 10
#define echo 9
long distanceInCM;

//wheels
#define motorLeftForward 5
#define motorLeftBackwards 6
#define motorRightForward 12
#define motorRightBackwards 13
#define sensorRight 8
#define sensorLeft 7

//neo pixel
#define pixelPin 4
#define pixelCount 4
Adafruit_NeoPixel neoPixel(pixelCount, pixelPin, NEO_GRB + NEO_KHZ800);


void setup(){
    Serial.begin(9600);
    //Neo Pixel
        neoPixel.begin();
    //gripper
        pinMode(2,INPUT);
        swivel.attach(11);
    //swivel
        pinMode(11,INPUT);
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
}

void loop(){
    //swivelReset
    if(swivelReset){
        if(!(swivel.read() == 90)){
            swivel.write(90);
            
        }
        swivelReset = false;
    }
    
    while (measureDistance()>10 && measureDistance() != 0){
        lightGreen();
        forward();
        if(distanceInCM<=10){
            lightRed();
            stop();
            turn();
        }
    }
}

//logic 
void turn(){
    if(lookAround() > 100){
        turnLeft();
    }
    else if( lookAround() < 80){
        turnRight();
    }
    else{
        //turn();
        Serial.println("Repeat scan");
    }
}

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
    gripper.detach();
}

// swivel methods
int lookAround(){
    //swivel.attach(11);
    for(int pos = 0; pos <= 180; pos++){
        swivel.write(pos);
        if(measureDistance() > 50){
            Serial.println(swivel.read());
            return swivel.read();
        }
        else {
           // lookAround();
        }
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