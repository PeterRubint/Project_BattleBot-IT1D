//<====================================================================Libraries Declarations==============================================================>

#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <QTRSensors.h>
#include <Servo.h>

//<====================================================================Variables and constants declarations================================================>

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
//Line sensors
const uint8_t sensorCount=8;
uint16_t sensorValues[sensorCount];
QTRSensors lineSensors;

//button pins
const int BUTTON1=7;
const int BUTTON2=8;
const int BUTTON3=12;
//button states;
int buttonState1=0;
int buttonState2=0;
int buttonState3=0;
//Gripper
Servo gripper;
const int GRIPPER_PIN=10;
int gripperPosition=0;// initial position of the gripper

//<====================================================Function declarations===================================================================================>

//LineSensorFunctions
void setupLineSensors();
void calibrateSensors();
//moving functions
void idle();//keeps the motors still
void setup_motor_pins();//setup the pin mode for the pins moving the DC motors
void moveForward(int power=255);//move the robot forward
void moveBackward(int power=255);//move the robot backwards
//Gripper Functions
void setupGripper();
// control functions
void BT_control();
void button_control();

void setupButtons();
void setup() {
  delay(3000);
  Serial.begin(9600);
  BT_module.begin(9600);
  setupGripper();
  gripper.write(0);
  setup_motor_pins();
  setupButtons();
  calibrateSensors();
  delay(1000);
  gripper.write(180);
  delay(500);
  gripper.write(0);
  idle();
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(lineSensors.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(lineSensors.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  leds.begin();
  
 
}
int dir=0;
void loop() {
  
 // button_control();
 BT_control();
  leds.show();
  
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
void setupLineSensors()
{
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6,A7}, sensorCount);
  lineSensors.setEmitterPin(2);
}

void calibrateSensors()
{
  moveForward(230);
  for (uint16_t i=0;i<800;i++)
  {
    lineSensors.calibrate();
  }
  //idle();
}
void setupGripper()
{
  gripper.attach(GRIPPER_PIN);
  
}
