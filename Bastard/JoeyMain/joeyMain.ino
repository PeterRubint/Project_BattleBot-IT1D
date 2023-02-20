//Testing file for reflectance sensor
//inlcude
#include <SoftwareSerial.h> //Bluetooth connection
#include <QTRSensors.h> // sensor line following

//Pins

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
    //qtr.setEmitterPin(2);
    
}
//loop
void loop(){
    // read raw sensor values
    qtr.read(sensorValues);

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
