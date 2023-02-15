//inlcude

//Pins

//Wheels 
const int forwardLeft = 5;
const int forwardRight = 9;

const int backwardsLeft = 6;
const int backwardsRight = 10;

int sensorLeft = 7;
int sensorRight = 8;

//setup
void setup(){
    pinConfiguration();
    spinLeft();
}
//loop
void loop(){

}

//Methods

void pinConfiguration(){
    pinMode(forwardLeft,INPUT);
    pinMode(forwardRight,INPUT);
    pinMode(backwardsLeft,INPUT);
    pinMode(backwardsRight,INPUT);
    pinMode(sensorLeft,OUTPUT);
    pinMode(sensorRight,OUTPUT);
}

void start(){

}

void forward(){
    digitalWrite(forwardLeft,HIGH);
    digitalWrite(forwardRight,HIGH);
}

void spinLeft(){
    digitalWrite(backwardsLeft,HIGH);
    digitalWrite(forwardRight,HIGH);
    delay(3000);
    stop();
}

void readWheelSpeed(){
    int sensorLeft = analogRead(7);
    Serial.print("LEFT");
    Serial.println(sensorLeft);
    int sensorRight = analogRead(8);
    Serial.print("RIGHT");
    Serial.println(sensorRight);
}


void stop(){
    digitalWrite(forwardLeft,LOW);
    digitalWrite(forwardRight,LOW);
}
void sensorLeft(){
    
}

void sensorRight(){
    int sensorRight = analogRead(8);
    Serial.print("RIGHT");
    Serial.println(sensorRight);
}