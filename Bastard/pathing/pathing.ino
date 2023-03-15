#define motorLeftForward 6
#define motorLeftBackwards 5
#define motorRightForward 3
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

#define detectionLimit 12


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
    

}

void loop(){
    
    
    front = measureFront();
    right = measureRight();
    forward();
    while(right <= detectionLimit){
        forward();
        lightGreen();
        front = measureFront();
        right = measureRight();

        if(right > detectionLimit){
            delay(100);
            turnRight();
        }
        else if (front <= detectionLimit && right > detectionLimit){
            delay(100);
            turnRight();
        }
        else if(front <= detectionLimit && right <= detectionLimit){
            delay(100);
            turnLeft();
        }
        else if(front <= detectionLimit && right <= detectionLimit){
            delay(100);
            turnLeft();
        }
    }

    


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
    analogWrite(motorRightForward,190);
}

void stop(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackwards,0);
    analogWrite(motorRightBackwards,0);
}

void turnLeft(){
    stop();
    analogWrite(motorLeftBackwards,180);
    analogWrite(motorRightForward,200);
    delay(500);
    stop();
}

void turnRight(){
    stop();
    analogWrite(motorRightBackwards,180);
    analogWrite(motorLeftForward,200);
    delay(500);
    stop();
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