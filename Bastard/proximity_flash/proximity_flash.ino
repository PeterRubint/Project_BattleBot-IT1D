#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     4  // The number of LEDs (pixels) on NeoPixel
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

int trigPin = 12;    // Trigger
int echoPin = 13;    // Echo
long duration, cm;

void setup() {
  // put your setup code here, to run once:

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
  
  delay(250);

  if(cm<10){ //If something is closer than 10cm, the Neopixels turn red

    red();

  }
  else{

    green();

  }

}

void red() { //Turns all pixels red

  for(int i=0; i<NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
      pixels.show();
  }

}
void green() { //Turns all pixels green

  for(int i=0; i<NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 0, 0));
      pixels.show();
  }

}
