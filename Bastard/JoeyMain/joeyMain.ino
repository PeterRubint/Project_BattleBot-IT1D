//Testing file for reflectance sensor
//inlcude
#include <SoftwareSerial.h> //Bluetooth connection
#include <QTRSensors.h> // sensor line following


//Pins

//Gripper
#define gripper 9
#define gripperOpenPulse 1600
#define gripperClosePulse 971
#define gripperPulseRepeat 10


//wheels
#define motorLeftForward 6
#define motorLeftBackward 5
#define motorRightForward 3
#define motorRightBackward 11

// sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
unsigned int sensorLala[8];


//setup
void setup(){
    Serial.begin(9600);
    //Pins
        pinMode(motorLeftForward,INPUT); 
        pinMode(motorLeftBackward,INPUT);
        pinMode(motorRightForward,INPUT);
        pinMode(motorRightBackward,INPUT);
        pinMode(gripper,INPUT);
    //sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    setCalibration();
}
//loop
void loop(){
    qtr.readCalibrated(sensorLala);
    for (uint8_t i = 0; i < SensorCount; i++){
        Serial.print(sensorLala[i]);
        Serial.print('\t');
    }
    if( sensorLala[3] > 600  || sensorLala[4] > 600 ){ 
        if(sensorLala[0] > 600 &&  sensorLala[1] > 600  && sensorLala[2] > 600){
            turnRight();
        }
        else if((sensorLala[2]>600 || sensorLala[3]>600 || sensorLala[4]>600) && (sensorLala[1]< 600 || sensorLala[2] < 600 || sensorLala[3]<600)){
            forward();
        }
        else { 
            forward();
        }
    }
    else if(sensorLala[0] < 600 && sensorLala[1] < 600 && sensorLala[2] < 600 && sensorLala[3] < 600 && sensorLala[4] < 600 && sensorLala[5] < 600 && sensorLala[6] < 600 && sensorLala[7] < 600){
        turnRight();
    }
    else if(sensorLala[0] > 600 || sensorLala[1] > 600 || sensorLala[2] > 600){ // line is right from the robot
        turnRight();
    }
    else if(sensorLala[5] > 600 && sensorLala[6] > 600 && sensorLala[7] > 600){ // line is left from the robot
        
    }
    else if(sensorLala < 600){
        turnAround();
    }
}

//Methods
/*void followline(){
    int positon = qtr.readCalibrated(sensorValues);
    if( sensorValues[3] > 750  && sensorValues[4] > 750 && sensorValues[5] > 750  && sensorValues[6] > 750 ){
        
    }
    int error = positon - 1000;
    analogWrite(motorLeftForward,250);
    analogWrite(motorRightForward,250);
    if(error < -500){ //left turn
        analogWrite(motorLeftForward,0);
    }
    if(error < 500){
        analogWrite(motorRightForward,0);
    }
}*/

void setCalibration(){
    openGrip();
    analogWrite(motorLeftForward,177);
    analogWrite(motorRightForward,170);
    for(uint8_t i = 0; i < 35 ; i++){
        qtr.calibrate();
        //delay(20);
        Serial.print("Calibration: ");
        Serial.println(i);
    }
    stop();
        /*delay(200);
        analogWrite(motorLeftBackward,170);
        analogWrite(motorRightBackward,170);
        for(uint8_t i = 0; i < 25 ; i++){
            qtr.calibrate();
            //delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        Serial.println("Calibration done");
        stop();
        */
    
    delay(100);
    closeGrip();
}

void forward(){
    analogWrite(motorLeftForward,240);
    analogWrite(motorRightForward,220);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
}

void backward(){
    analogWrite(motorLeftBackward,240);
    analogWrite(motorRightBackward,220);
}

void turnRight(){
    stop();
    analogWrite(motorRightBackward,200);
    analogWrite(motorLeftForward,200);
    delay(500);
    stop();
}

void turnAround(){
    stop();
    analogWrite(motorLeftBackward,200);
    analogWrite(motorRightForward,200);
    delay(1000);
}

void openGrip(){
    gripperServo(gripperOpenPulse);
}

void closeGrip(){
    gripperServo(gripperClosePulse);
}

void gripperServo(int pulse){
    for(int i = 0; i < gripperPulseRepeat;i++){
        digitalWrite(gripper,HIGH);
        delayMicroseconds(pulse);
        digitalWrite(gripper,LOW);
        delay(20);
    }
}
