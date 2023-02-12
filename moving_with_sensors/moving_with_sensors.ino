#include <Adafruit_NeoPixel.h>
//motor pins
const int MOTOR_A1=11;
const int MOTOR_A2=5;
const int MOTOR_B1=6;
const int MOTOR_B2=9;
//rotary sensors
const int MOTOR_R1=3;
const int MOTOR_R2=2;
int countA=0;
int countB=0;
//leds variables
const int LED_PIN=4;
Adafruit_NeoPixel leds(4,LED_PIN,NEO_RGB+ NEO_KHZ800);
//buttons
const int BUTTON_1=7;
const int BUTTON_2=8;
//FUNCTIONS DECLARATIONS
//setup functions
void setupMotors();
void setupButtons();
//Motor functions
void idle();
void moveForward(int movingSpeed);
void moveBack(int movingSpeed);

//Sensor interrupts
void CountA();
void CountB();
void setup()
{
  Serial.begin(9600);
  setupMotors();
  setupButtons();
}
int countsLeft=0,previousCount;
void loop() 
{
  //moveForward(255);
  if (digitalRead(BUTTON_1)==LOW)
  {
     countsLeft=20;
     previousCount=countA;
     analogWrite( MOTOR_A2,255);
  }
  if (countsLeft>0)
  {
    countsLeft-=(countA-previousCount);
    previousCount=countA;
  }
  else 
   {
    idle();
   }
  leds.show();
  
}





//FUNCTIONS DEFINITIONS
void setupMotors()
{
 //setup the pins for the motors
 pinMode(MOTOR_A1,OUTPUT);
 pinMode(MOTOR_A2,OUTPUT);
 pinMode(MOTOR_B1,OUTPUT);
 pinMode(MOTOR_B2,OUTPUT);
 pinMode(MOTOR_R1,INPUT);
 pinMode(MOTOR_R2,INPUT); 
 //setup the rotary sensors
 attachInterrupt(digitalPinToInterrupt(MOTOR_R1),CountA,CHANGE);
 attachInterrupt(digitalPinToInterrupt(MOTOR_R2),CountB,CHANGE);
}
void setupButtons()
{
  pinMode(BUTTON_1,INPUT);
  pinMode(BUTTON_2,INPUT);
}

void idle()
{
  leds.clear();
  digitalWrite(MOTOR_A1,0);
  digitalWrite(MOTOR_A2,0);
  digitalWrite(MOTOR_B1,0);
  digitalWrite(MOTOR_B2,0);
}
void CountA()
{
  noInterrupts();
  countA++;
  Serial.println(countA);
  interrupts();
}
void CountB()
{
  noInterrupts();
  countB++;
  interrupts();
}
void moveForward(int movingSpeed)
{
  analogWrite(MOTOR_A2,movingSpeed);
  analogWrite(MOTOR_B1,movingSpeed);
}
