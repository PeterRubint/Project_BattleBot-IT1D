#define motorLeftForward 6
#define motorLeftBackwards 5
#define motorRightForward 10
#define motorRightBackwards 11
#define frontTrigger 8
#define frontEcho 7
#define rightTrigger 12
#define rightEcho 13
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
long frontDistanceInCM, rightDistanceInCM;
long front, right;
//neo pixel
#define pixelPin 4
#define pixelCount 4
Adafruit_NeoPixel neoPixel(pixelCount, pixelPin, NEO_GRB + NEO_KHZ800);
#endif

#define detectionLimit 15

//rotation sensor
#define leftWheelSensor 3
#define rightWheelSensor 2
int leftRotationCount =  0;
int rightRotationCount = 0;

bool done = false;


void setup(){
    Serial.begin(9600);
    pinMode(motorLeftForward,INPUT); 
    pinMode(motorLeftBackwards,INPUT);
    pinMode(motorRightForward,INPUT);
    pinMode(motorRightBackwards,INPUT);
    //ultrasonic
    pinMode(frontTrigger,OUTPUT);
    pinMode(frontEcho,INPUT);
    pinMode(rightTrigger,OUTPUT);
    pinMode(rightEcho,INPUT);

    neoPixel.begin();
    //sensor
    pinMode(leftWheelSensor,INPUT);
    pinMode(rightWheelSensor,INPUT);
    attachInterrupt(digitalPinToInterrupt(3),countLeftSensor,CHANGE);
    attachInterrupt(digitalPinToInterrupt(2),countRightSensor,CHANGE);

}

void loop(){

    turnRight();
    delay(1000);

    // front = measureFront();
    // right = measureRight();
    // forward();
    // while(right <= detectionLimit){
    //     forward();
    //     lightGreen();
    //     front = measureFront();
    //     right = measureRight();

    //     if(right > detectionLimit){
    //         delay(100);
    //         turnRight();
    //         Serial.println("RIGHT");
    //     }
    //     else if(front <= detectionLimit){
    //         if(right > detectionLimit){
    //             turnRight();
    //             Serial.println("RIGHT");
    //         }
    //         else if(right <= detectionLimit){
    //             turnLeft();
    //         }
    //     }
    // }
    


}

void oneRotationForward(){
    while(rightRotationCount < 40){
        forward();
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    stop();
}

void countLeftSensor(){
    leftRotationCount++;
    // Serial.print("LeftRotation:");
    // Serial.println(leftRotationCount);
}

void countRightSensor(){
    rightRotationCount++;
    // Serial.print("RightSensor:");
    // Serial.println(rightRotationCount);
}

long measureFront(){
    //Front sensor
    digitalWrite(frontTrigger,LOW);
    delayMicroseconds(5);
    digitalWrite(frontTrigger,HIGH);
    digitalWrite(frontTrigger,LOW);

    frontDistanceInCM = (pulseIn(frontEcho,HIGH)/2) / 29.1;
    Serial.println("Front distance: ");
    Serial.print(frontDistanceInCM);
    Serial.println(" ");
    
    return frontDistanceInCM;
}

long measureRight(){
    //Right sensor
    digitalWrite(rightTrigger,LOW);
    delayMicroseconds(5);
    digitalWrite(rightTrigger,HIGH);
    digitalWrite(rightTrigger,LOW);

    rightDistanceInCM = (pulseIn(rightEcho,HIGH)/2) / 29.1;
    Serial.println("Right distance: ");
    Serial.print(rightDistanceInCM);
    Serial.println(" ");

    return  rightDistanceInCM;

}

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
    analogWrite(motorLeftBackwards,190);
    analogWrite(motorRightForward,200);
    delay(500);
    stop();
}

void turnRight(){
    stop();
    while(rightRotationCount < 1 && leftRotationCount < 1){
        analogWrite(motorLeftForward,200);
        analogWrite(motorRightBackwards,190);
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
}

void goBack(){
    stop();
    analogWrite(motorRightBackwards, 193);
    analogWrite(motorLeftBackwards, 200);
    delay(500);
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