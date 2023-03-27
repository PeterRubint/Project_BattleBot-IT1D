#define motorLeftForward 6
#define motorLeftBackward 5
#define motorRightForward 10
#define motorRightBackward 11
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
    pinMode(motorLeftBackward,INPUT);
    pinMode(motorRightForward,INPUT);
    pinMode(motorRightBackward,INPUT);
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
    long right = measureRight();
    long front = measureFront();

    if(right > 30){
        turnRight();
        Serial.print("Right");
    }
    else if(front < 20){
        turnLeft();
        Serial.println("Left: " + String(front) + ";");
    }
    /*else if(front < 20 && right < 20){
        stop();
        while(rightRotationCount < 8){
            analogWrite(motorLeftForward,255);
            analogWrite(motorRightForward,80);
        }
        rightRotationCount = 0;
        leftRotationCount = 0;
        stop();
    }*/
    
    else{
        paulWay();
        Serial.print("Paul");
    }

}

void paulWay(){
    // used to keep the robot straight in the maze
    if(rightDistanceInCM > 6.5){
        analogWrite(motorRightForward,200);
        analogWrite(motorLeftForward,255);
    }
    else if(rightDistanceInCM < 4.5){
        analogWrite(motorLeftForward,255);
        analogWrite(motorRightForward,255);
    }
    else{
        analogWrite(motorLeftForward,255);
        analogWrite(motorRightForward,240);
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
    delayMicroseconds(10);
    digitalWrite(frontTrigger,HIGH);
    digitalWrite(frontTrigger,LOW);

    frontDistanceInCM = (pulseIn(frontEcho,HIGH)/2) / 29.1;
    /*
    Serial.println("Front distance: ");
    Serial.print(frontDistanceInCM);
    Serial.println(" ");
    */
    return frontDistanceInCM;
}

long measureRight(){
    //Right sensor
    digitalWrite(rightTrigger,LOW);
    delayMicroseconds(10);
    digitalWrite(rightTrigger,HIGH);
    digitalWrite(rightTrigger,LOW);

    rightDistanceInCM = (pulseIn(rightEcho,HIGH)/2) / 29.1;
    /*
    Serial.println("Right distance: ");
    Serial.print(rightDistanceInCM);
    Serial.println(" ");
    */

    return  rightDistanceInCM;

}

void forward(){
    analogWrite(motorLeftForward,200);
    analogWrite(motorRightForward,193);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
}

void turnLeft(){
    stop();
    while(rightRotationCount < 8){
        analogWrite(motorRightForward,200);
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

void goBack(){
    stop();
    analogWrite(motorRightBackward, 193);
    analogWrite(motorLeftBackward, 200);
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
