#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     4  // The number of LEDs (pixels) on NeoPixel
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
     pixels.clear();
     for(int i=0; i<NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();
  }
    
  } else {
    // turn LED off:
    for(int i=0; i<NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 100, 0));
      pixels.show();
  }
   
  }
}
