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
#define motorRightForward 10
#define motorRightBackward 11

// sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
unsigned int sensorLala[8];
int sensorLimit = 600;

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
    //start
    //setCalibration();
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
    if( sensorLala[3] > sensorLimit  || sensorLala[4] > sensorLimit ){ 
        if(sensorLala[0] > sensorLimit &&  sensorLala[1] > sensorLimit  && sensorLala[2] > sensorLimit){
           doRight();
        }
        else if(sensorLala[0] > 900 && sensorLala[1] > 900 && sensorLala[2] > 900 && sensorLala[3] > 900 && sensorLala[4] > 900 && sensorLala[5] > 900 && sensorLala[6] > 900 && sensorLala[7] > 900){
        openGrip();
        backward();
        delay(500);
        stop();
        }
        else if((sensorLala[2]>sensorLimit || sensorLala[3]>sensorLimit || sensorLala[4]>sensorLimit) && (sensorLala[1]< sensorLimit || sensorLala[2] < sensorLimit || sensorLala[3]<sensorLimit)){
            forward();
        }
        else { 
            forward();
        }
    }
    else if(sensorLala[0] > sensorLimit || sensorLala[1] > sensorLimit || sensorLala[2] > sensorLimit){
        doRight();
    }
    else if(sensorLala[5] > sensorLimit || sensorLala[6] > sensorLimit || sensorLala[7] > sensorLimit){
        doLeft();
    }
    else if(sensorLala > 900){
        openGrip();
        backward();
        delay(500);
        stop();
    }
    else if(sensorLala < sensorLimit){
      //Maze part....
    }
}

void setCalibration(){
    openGrip();
    analogWrite(motorLeftForward,177);
    analogWrite(motorRightForward,170);
    for(uint8_t i = 0; i < 35 ; i++){
        qtr.calibrate();
        Serial.print("Calibration: ");
        Serial.println(i);
    }
    delay(500);
    stop();
    delay(100);
    closeGrip();
    turnLeft();
}

void forward(){
    analogWrite(motorLeftForward,237);
    analogWrite(motorRightForward,230);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
}

void backward(){
    analogWrite(motorLeftBackward,237);
    analogWrite(motorRightBackward,230);
}

void turnRight(){
    stop();
    analogWrite(motorRightBackward,200);
    analogWrite(motorLeftForward,207);
    delay(500);
    stop();
}

void turnLeft(){
    stop();
    analogWrite(motorRightForward,200);
    analogWrite(motorLeftBackward,207);
    delay(300);
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
