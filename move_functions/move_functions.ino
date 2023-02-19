 #include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
const int PIXEL_PIN=4;
const int PIXEL_NUMBER=4;
Adafruit_NeoPixel leds(PIXEL_NUMBER,PIXEL_PIN,NEO_RGB +NEO_KHZ800);
// bluetooth pins
const int BT_R=12;
const int BT_T=13;
SoftwareSerial BT_module(BT_R,BT_T);
//motor pins
const int  MOTOR_A1=11;
const int MOTOR_A2=5;
const int MOTOR_B1=6;
const int MOTOR_B2=9;
const int MOTOR_R1=3;
const int MOTOR_R2=2;
//colors 
const uint32_t RED=leds.Color(255,0,0);
const uint32_t YELLOW=leds.Color(234,255,0);
const uint32_t BLUE=leds.Color(0,0,255);

//button pins
const int BUTTON1=7;
const int BUTTON2=8;
const int BUTTON3=12;
//button states;
int buttonState1=0;
int buttonState2=0;
int buttonState3=0;
//function declaration
//moving functions
void idle();//keeps the motors still
void setup_motor_pins();//setup the pin mode for the pins moving the DC motors
void moveForward(int power=255);//move the robot forward
void moveBackward(int power=255);//move the robot backwards
/*
void turnLeft(int moveTime);//moves the robot in the left direction for moveTime miliseconds
void turnRight(int moveTime);//moves the robot in the right direction for moveTime miliseconds
*/
// control functions
void BT_control();
void button_control();

void setupButtons();
void setup() {
  Serial.begin(9600);
  BT_module.begin(9600);
  setup_motor_pins();
  setupButtons();
  leds.begin();
  
 
}
int dir=0;
int cntPulses=0;
void loop() {
  
  button_control();
  leds.show();
  Serial.print(analogRead(MOTOR_R1));
  Serial.print(" ");
  Serial.print(digitalRead(MOTOR_R1));
  Serial.print("\n");
  
}





void setup_motor_pins()
{
  pinMode(MOTOR_A1,OUTPUT);
  pinMode(MOTOR_A2,OUTPUT);
  pinMode(MOTOR_B1,OUTPUT);
  pinMode(MOTOR_B2,OUTPUT);
  pinMode(MOTOR_R1,INPUT);
  pinMode(MOTOR_R2,INPUT);

  
}
void moveForward(int power=255)
{
  leds.clear();
  leds.fill(BLUE,2,2);
  leds.show(); 
  analogWrite(MOTOR_A2,power);
  analogWrite(MOTOR_B1,power);
}
void moveBackward(int power=255)
{
  leds.clear();
  leds.fill(RED,0,2);
  leds.show();
  analogWrite(MOTOR_A1,power);
  analogWrite(MOTOR_B2,power);
}
void idle()
{
  int motor_pins[4]={MOTOR_A1,MOTOR_A2,MOTOR_B1,MOTOR_B2};
  leds.clear();
  for (int i=0;i<4;i++)
    {
      
      analogWrite(motor_pins[i],0);
    }
}
void setupButtons()
{
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  pinMode(BUTTON3,INPUT);
}
void BT_control()
{
  if (BT_module.available())
  {
    int input=BT_module.parseInt();
    if (input!=0)
      dir=input;
    Serial.println(input);
  }
  if (dir!=0)  
   {
    Serial.println(dir);
   }
  
  switch (dir)
  {
    case 1:
    {
      moveForward(255);
      
      break;
    }
    case 2:
    {
      moveBackward(255);
      leds.fill(RED,2,2);
      break;
    }
    default:
    {
      idle();
      break;
    }
  }
}
void button_control()
{
  buttonState1=digitalRead(BUTTON1);
  buttonState2=digitalRead(BUTTON2);
  if (buttonState1==LOW)
  {
    moveForward(255);
  }
  else if (buttonState2==LOW)
  {
    moveBackward(255);
  }
  else 
  {
    idle();
  }
  
}
