void setup(){
    pinMode(5,OUTPUT);
    pinMode(9,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    Serial.begin(9600);
    
   
}

void loop(){
    digitalWrite(5,HIGH);
    digitalWrite(9,HIGH);
    sensor();   
}

void sensor(){
    sensorLeft();
    sensorRight();
    delay(1000);
}

void sensorLeft(){
    int sensorLeft = analogRead(7);
    Serial.print("LEFT");
    Serial.println(sensorLeft);
}

void sensorRight(){
    int sensorRight = analogRead(8);
    Serial.print("RIGHT");
    Serial.println(sensorRight);
}