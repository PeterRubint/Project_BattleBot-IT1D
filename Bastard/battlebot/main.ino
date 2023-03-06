//libaries
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define swivelPin 10
int left,front,right = 0;

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
        pinMode(swivelPin,OUTPUT);
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
    resetSwivel();
    //methods
   
}

void loop(){
    
    String eval = evaluate();
    Serial.println(eval);
    while(eval == "f"){
        lightGreen();
        forward();
        if(measureDistance() <= 12 && measureDistance() != 0){
            front = measureDistance();
            stop();
            lightRed();
            lookAround();
            
            if(evaluate() == "r"){
                turnRight();
            }
            else if (evaluate() == "l")
            {
                turnLeft();
            }
            else if (evaluate() == "s")
            {
                stop();
            }
        }
    }

    
}


// gripper methods
void openGrip(){
    
}

void closeGrip(){
   
    }


// swivel methods
void resetSwivel(){
    digitalWrite(swivelPin,HIGH);
    delayMicroseconds(1500);
    digitalWrite(swivelPin,LOW);
}
int lookRight(){

    int rightWall = 0;
    digitalWrite(swivelPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(swivelPin,LOW);
    delay(1000);
    rightWall = measureDistance();

    return rightWall;
   
    
}
int lookLeft(){

    int leftWall = 0;
    digitalWrite(swivelPin,HIGH);
    delayMicroseconds(2500);
    digitalWrite(swivelPin,LOW);
    delay(1000);
    leftWall = measureDistance();

    return leftWall;

    

}
int lookAround(){

    left = lookLeft();
    delay(500);
    resetSwivel();
    delay(500);
    right = lookRight();
    resetSwivel();
    delay(500);

    return left,right;
}

String evaluate(){

    if(front > 12){
        return "f";
    }
    else if (right > 17)
    {
        return "r";
    }
    else if (left > 17)
    {
        return "l";
    }
    else if (right > 17 && left > 17)
    {
        return "r";
    }
    else
    {
        return "f";
    }
    
    
    
    
}


//ultrasonic
long measureDistance(){
    digitalWrite(trigger,LOW);
    delayMicroseconds(5);
    digitalWrite(trigger,HIGH);
    delayMicroseconds(swivelPin);
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
    analogWrite(motorLeftForward,200);
    analogWrite(motorRightForward,193);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackwards,0);
    analogWrite(motorRightBackwards,0);
}

void turnLeft(){
    stop();
    analogWrite(motorLeftBackwards,200);
    analogWrite(motorRightForward,200);
    delay(500);
    stop();
}

void turnRight(){
    stop();
    analogWrite(motorRightBackwards,200);
    analogWrite(motorLeftForward,200);
    delay(500);
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