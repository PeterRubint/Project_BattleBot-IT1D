//Testing file for reflectance sensor
//inlcude
#include <QTRSensors.h> // sensor line following
#include <Adafruit_NeoPixel.h> // Neo pixel
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket

//Gripper
#define gripper 9
#define gripperOpenPulse 1600
#define gripperClosePulse 971
#define gripperPulseRepeat 10

//wheels
#define motorLeftForward 6
#define motorLeftBackward 5
#define motorRightForward 10
#define motorRightBackward 11

//rotation sensor
#define leftWheelSensor 3
#define rightWheelSensor 2
int leftRotationCount =  0;
int rightRotationCount = 0;

// sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int sensorLala[8];
int sensorLimit = 600;

//ultrasonic
#define frontTrigger 8
#define frontEcho 7
#define rightTrigger 12
#define rightEcho 13
long frontDistanceInCM, rightDistanceInCM;
long front, right;

//neo pixel
#define pixelPin 4
#define pixelCount 4
Adafruit_NeoPixel neoPixel(pixelCount, pixelPin, NEO_GRB + NEO_KHZ800);
#endif

#define detectionLimit 15

//setup
void setup(){
    Serial.begin(9600);
    //Pins
        pinMode(motorLeftForward,INPUT); 
        pinMode(motorLeftBackward,INPUT);
        pinMode(motorRightForward,INPUT);
        pinMode(motorRightBackward,INPUT);
        pinMode(gripper,INPUT);
    //line sensors
        qtr.setTypeAnalog();
        qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    //sensor
        pinMode(leftWheelSensor,INPUT);
        pinMode(rightWheelSensor,INPUT);
        attachInterrupt(digitalPinToInterrupt(3),countLeftSensor,CHANGE);
        attachInterrupt(digitalPinToInterrupt(2),countRightSensor,CHANGE);
    //ultrasonic
        pinMode(frontTrigger,OUTPUT);
        pinMode(frontEcho,INPUT);
        pinMode(rightTrigger,OUTPUT);
        pinMode(rightEcho,INPUT);
    start(); // start of maze
}

void loop(){
    qtr.read(sensorLala);
    for (uint8_t i = 0; i < SensorCount; i++){
        Serial.print(sensorLala[i]);
        Serial.print('\t');
    }
    Serial.println();
    Serial.print("Distance = ");
    Serial.println(" cm");

    //Line following logic
    if(sensorLala[0] > 900 && sensorLala[1] > 900 && sensorLala[2] > 900 && sensorLala[3] > 900 && sensorLala[4] > 900 && sensorLala[5] > 900 && sensorLala[6] > 900 && sensorLala[7] > 900){
        analogWrite(motorLeftForward,0);
        analogWrite(motorRightForward,0);
        openGrip();
        backward(20);
    }
    else if( sensorLala[3] > sensorLimit  || sensorLala[4] > sensorLimit ){ 
        if(sensorLala[0] > sensorLimit &&  sensorLala[1] > sensorLimit  && sensorLala[2] > sensorLimit){
           doRight();
        }
        else if((sensorLala[2]>sensorLimit || sensorLala[3]>sensorLimit || sensorLala[4]>sensorLimit) && (sensorLala[1]< sensorLimit || sensorLala[2] < sensorLimit || sensorLala[3]<sensorLimit)){
            analogWrite(motorLeftForward,200);
            analogWrite(motorRightForward,193);
        }
        else { 
            analogWrite(motorLeftForward,200);
            analogWrite(motorRightForward,193);
        }
    }
    else if(sensorLala[0] > sensorLimit || sensorLala[1] > sensorLimit || sensorLala[2] > sensorLimit){
        doRight();
    }
    else if(sensorLala[5] > sensorLimit || sensorLala[6] > sensorLimit || sensorLala[7] > sensorLimit){
        doLeft();
    }
    else if(sensorLala > 900){ // Finish maze 
        openGrip();
        backward(15);
        delay(500);
        stop();
        exit(0);
    }
    else if(sensorLala < sensorLimit){ // Maze logic
        lightRed();
        long right = measureRight();
        long front = measureFront();
        
        if(right > 30){
            delay(10);
            turnRight();
            Serial.print("Right");
            lightRed();
        }
        else if(front < 15){
            delay(10);
            turnLeft();
            lightRed();
            Serial.println("Left: " + String(front) + ";");
        }
        else if(front < 15 && right < 15){
            stop();
            turnLeft();
            if(front < 20){
                turnLeft();
            }
        }
        else{
            paulWay();
            Serial.print("Paul");
        }
    }
}

void countLeftSensor(){
    leftRotationCount++;
}

void countRightSensor(){
    rightRotationCount++;
}

void start(){
    openGrip();
    forwardByCM(34);
    closeGrip();
    turnLeftStart();
    forwardByCM(5);
}

void forward(){
    analogWrite(motorLeftForward,200);
    analogWrite(motorRightForward,193);
}

void forwardByCM(int distance){
    while(rightRotationCount < distance*2){ // 40 = 20cm 
        analogWrite(motorLeftForward,200);
        analogWrite(motorRightForward,193);
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
}

void backward(int distance){
    while(rightRotationCount < distance*2){ // 1cm = 2
        analogWrite(motorLeftBackward,200);
        analogWrite(motorRightBackward,193);
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void paulWay(){
    // used to keep the robot straight in the maze
    if(rightDistanceInCM > 6.5){
        analogWrite(motorRightForward,190);
        analogWrite(motorLeftForward,250);
    }
    else if(rightDistanceInCM < 6){
        analogWrite(motorLeftForward,250);
        analogWrite(motorRightForward,250);
    }
    else{
        forward();
    }
}

void oneRotationForward(){
    while(rightRotationCount < 40){
        forward();
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void turnLeftStart(){
     while(rightRotationCount < 49){
        analogWrite(motorRightBackward, 190);
        analogWrite(motorLeftForward,200);
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void turnLeft(){
    stop();
    while(rightRotationCount < 8){
        analogWrite(motorRightForward,255);
        analogWrite(motorLeftBackward,190);
        // Serial.print(rightRotationCount);
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void turnRight(){
    stop();
    while(rightRotationCount < 8){
        analogWrite(motorLeftForward,255);
        analogWrite(motorRightForward,80);
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void doRight(){
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
    analogWrite(motorLeftForward,200);
    analogWrite(motorRightForward,80);
}

void doLeft(){
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
    analogWrite(motorRightForward,200);
    analogWrite(motorLeftForward,80);
}

void turnAround(){
    stop();
    analogWrite(motorLeftBackward,207);
    analogWrite(motorRightForward,200);
    delay(1000);
}

//Gripper
void gripperServo(int pulse){
    for(int i = 0; i < gripperPulseRepeat;i++){
        digitalWrite(gripper,HIGH);
        delayMicroseconds(pulse);
        digitalWrite(gripper,LOW);
        delay(20);
    }
}

void openGrip(){
    gripperServo(gripperOpenPulse);
}

void closeGrip(){
    gripperServo(gripperClosePulse);
}

//Ultrasonic
long measureFront(){
    //Front sensor
    digitalWrite(frontTrigger,LOW);
    delayMicroseconds(10);
    digitalWrite(frontTrigger,HIGH);
    digitalWrite(frontTrigger,LOW);
    frontDistanceInCM = (pulseIn(frontEcho,HIGH)/2) / 29.1;
    
    return frontDistanceInCM;
}

long measureRight(){
    //Right sensor
    digitalWrite(rightTrigger,LOW);
    delayMicroseconds(10);
    digitalWrite(rightTrigger,HIGH);
    digitalWrite(rightTrigger,LOW);
    rightDistanceInCM = (pulseIn(rightEcho,HIGH)/2) / 29.1;
    
    return  rightDistanceInCM;

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
