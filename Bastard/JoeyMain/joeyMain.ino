//Testing file for reflectance sensor
//inlcude
#include <SoftwareSerial.h> //Bluetooth connection
#include <QTRSensors.h> // sensor line following


//Pins

//wheels
#define motorLeftForward 6
#define motorLeftBackwards 5
#define motorRightForward 3
#define motorRightBackwards 11

// sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


//setup
void setup(){
    Serial.begin(9600);

    //sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    setCalibration();
}
bool c = true;
//loop
void loop(){
    qtr.readLine(sensorValues);

    // print the sensor values as numbers from 0 to 1023, where 0 means maximum
    // reflectance and 1023 means minimum reflectance
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();

    delay(250);
}


//Methods
void followline(){
    int positon = qtr.readLine(sensorValues);
    //Finish me
}

void setCalibration(){
   for(int i = 0; i < 2;i++){
        forward(170);
        for(uint8_t i = 0; i < 25 ; i++){
            qtr.calibrate();
            //delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        stop();
        delay(200);
        backward(170);
        for(uint8_t i = 0; i < 25 ; i++){
            qtr.calibrate();
            //delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        Serial.println("Calibration done");
        stop();
    }
}

void forward(int speed){
    analogWrite(motorLeftForward,speed);
    analogWrite(motorRightForward,speed);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackwards,0);
    analogWrite(motorRightBackwards,0);
}

void backward(int speed){
    analogWrite(motorLeftBackwards,speed);
    analogWrite(motorRightBackwards,speed);
}
