//inlcude
#include <SoftwareSerial.h> //Bluetooth connection
#include <QTRSensors.h> // sensor line following

//Pins

//Wheels 
const int forwardLeft = 5;
const int forwardRight = 9;

const int backwardsLeft = 6;
const int backwardsRight = 10;

int sensorLeft = 7;
int sensorRight = 8;

/*bluetooth

const int RX = 3;
const int TX = 1;

SoftwareSerial myBluetooth(3,1); // RX / TX

*/
// sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


//setup
void setup(){
    Serial.begin(9600);
   // myBluetooth.begin(38400); Bluetooth connection
    pinConfiguration();

    //sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    //qtr.setEmitterPin(2);
    
}
//loop
void loop(){
    //btRecieve(); Bluetooth recieve function

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

void pinConfiguration(){
    pinMode(forwardLeft,INPUT);
    pinMode(forwardRight,INPUT);
    pinMode(backwardsLeft,INPUT);
    pinMode(backwardsRight,INPUT);
    pinMode(sensorLeft,OUTPUT);
    pinMode(sensorRight,OUTPUT);
    //pinMode(RX,OUTPUT); Bluetooth
    //pinMode(TX,INPUT); Bluetooth

}

void start(){
    spinLeft();
    delay(3000);
    spinRight();
    delay(3000);
    turnAround();
    delay(3000);
}

void forward(){
    digitalWrite(forwardLeft,HIGH);
    digitalWrite(forwardRight,HIGH);
}

void spinLeft(){
    digitalWrite(backwardsLeft,HIGH);
    digitalWrite(forwardRight,HIGH);
    delay(310);
    stop();
}

void spinRight(){
    digitalWrite(forwardLeft,HIGH);
    digitalWrite(backwardsRight,HIGH);
    delay(315);
    stop();
}

void turnAround(){
    digitalWrite(backwardsLeft,HIGH);
    digitalWrite(forwardRight,HIGH);
    delay(650);
    stop();
}

void readWheelSpeed(){
    sensorLeft = analogRead(7);
    Serial.print("LEFT");
    Serial.println(sensorLeft);
    sensorRight = analogRead(8);
    Serial.print("RIGHT");
    Serial.println(sensorRight);
}

void stop(){
    digitalWrite(forwardLeft,LOW);
    digitalWrite(forwardRight,LOW);
    digitalWrite(backwardsLeft,LOW);
    digitalWrite(backwardsRight,LOW);
}




/*
void btRecieve(){
  // myBluetooth.write(analogRead(7));
   myBluetooth.println(analogRead(7));
   //Serial.println(myBluetooth.read());
   delay(1000);
    /*if(myBluetooth.available())
        
    else{
        Serial.println("NOT WORKING");  
    }
}  


String btTransmit(String input){
    
}

*/