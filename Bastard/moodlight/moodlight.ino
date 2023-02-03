#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     4  // The number of LEDs (pixels) on NeoPixel
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500
#define pIndex 0

void setup() {
  // put your setup code here, to run once:
  pixels.begin();
}
int changeColor(pIndex)
{
      pixels.setPixelColor(pIndex, pixels.Color(0, 255, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(pIndex, pixels.Color(0, 200, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(pIndex, pixels.Color(0, 155, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(pIndex, pixels.Color(0, 100, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(pIndex, pixels.Color(0, 55, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(pIndex, pixels.Color(0, 0, 0));
      pixels.show();
      delay(500);
      pixels.show();
  
}
void loop() {
  // put your main code here, to run repeatedly:
  pixels.clear();
  changeColor(0);
  changeColor(1);
  changeColor(2);
  changeColor(3);
}
