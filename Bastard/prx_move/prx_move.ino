#define SERVO_A1 5 //LEFT WHEEL
#define SERVO_A2 6 //PINS
#define SERVO_B1 9 //RIGHT WHEEL
#define SERVO_B2 10 //PINS

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     4  // The number of LEDs (pixels) on NeoPixel
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

int trigPin = 13;    // Trigger
int echoPin = 12;    // Echo
long duration, cm;


void setup() {
  // put your setup code here, to run once:
  pinMode(SERVO_A1,INPUT); //LEFT
  pinMode(SERVO_A2,INPUT);

  pinMode(SERVO_B1,INPUT); //RIGHT
  pinMode(SERVO_B2,INPUT);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  for(int i=0; i<NUM_PIXELS; i++) { //Turn LEDs off
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();
  }
//Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

 while (measureDistance()>20 && measureDistance() != 0){
    green();
    moveForward();

    if (cm<=20){
      red();
      turnRight();
    }

  }
  
 

}
double measureDistance(){
   // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  return cm;
}
void moveForward() {
  analogWrite(SERVO_A1, 240); // LEFT WHEEL SPIN FORWARD
  analogWrite(SERVO_A2, 0);

  analogWrite(SERVO_B1, 250); // RIGHT WHEEL SPIN FORWARD
  analogWrite(SERVO_B2, 0);
}
void moveBack() {
  analogWrite(SERVO_A1, 0); // LEFT WHEEL SPIN BACKWARD
  analogWrite(SERVO_A2, 240);

  analogWrite(SERVO_B1, 0); // RIGHT WHEEL SPIN BACKWARD
  analogWrite(SERVO_B2, 250);

  
}

void stop() {
  analogWrite(SERVO_A1, 0);
  analogWrite(SERVO_B1, 0);
}



void turnRight() { // Near 90 degree turn to the right
  digitalWrite(SERVO_A1,HIGH);
  digitalWrite(SERVO_B2,HIGH);
  delay(800);
  stop();
}


void red() { //Turns all pixels red

  for(int i=0; i<NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
      pixels.show();
  }

}
void green() { //Turns all pixels green

  for(int i=0; i<NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(200, 0, 50));
      pixels.show();
  }

}
