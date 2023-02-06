/* +++++++++[LIBRARIES]+++++++++*/

// NEOPIXEL
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

/* +++++++++[PINS]+++++++++*/

// WHEEL SERVOS A = LEFT B = RIGHT 
const int SERVO_A1 = 5;
const int SERVO_A2 = 6;
const int SERVO_B1 = 9;
const int SERVO_B2 = 10;

// WHEEL SENSORS
const int R1 = 7;
const int R2 = 8;

// NEOPIXEL
const int  NPX_PIN = 4;   // Arduino pin that connects to NeoPixel
const int NUM_PIXELS = 4;  // The number of LEDs (pixels) on NeoPixel
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800); //Initialize Neopixel object

// ULTRASONIC DISTANCE SENSOR
const int trigPin = 13;    // Trigger
const int echoPin = 12;    // Echo
long duration, cm;

/* +++++++++[CODE]+++++++++*/

void setup() {
  // put your setup code here, to run once:

  pinMode(SERVO_A1,INPUT); //LEFT WHEEL
  pinMode(SERVO_A2,INPUT);

  pinMode(SERVO_B1,INPUT); //RIGHT WHEEL
  pinMode(SERVO_B2,INPUT);

  pinMode(trigPin, OUTPUT); // UDS Trigger
  pinMode(echoPin, INPUT);  // UDS Echo

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  for(int i=0; i<NUM_PIXELS; i++) { //Turn LEDs off
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();
  }
  //Serial Port begin
  Serial.begin (9600);

  
  


}

void loop() {

}

/* +++++++++[FUNCTIONS]+++++++++*/

float measureDistance() {
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

void moveForward() { // Moves the robot forward

  analogWrite(SERVO_A1, 250); // LEFT WHEEL SPIN FORWARD
  analogWrite(SERVO_A2, 0);

  analogWrite(SERVO_B1, 250); // RIGHT WHEEL SPIN FORWARD
  analogWrite(SERVO_B2, 0);
}

void stop() { // Stops both wheels

  analogWrite(SERVO_A1, 0);
  analogWrite(SERVO_B1, 0);
  
}

void turnRight() { // Turns the robot right

  analogWrite(SERVO_A1,250);
  analogWrite(SERVO_B1,0);

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