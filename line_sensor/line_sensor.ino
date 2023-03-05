 #include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <QTRSensors.h>
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
//line sensors
QTRSensors lineSensors;
const int SENSOR_COUNT=8;
int sensorValues[SENSOR_COUNT];
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
void moveForward(int powerA=255,int powerB=255);//move the robot forward
void moveBackward(int powerA=255,int powerB=255);//move the robot backwards

// control functions
void BT_control();
void button_control();

void setupButtons();

//line sensor
void setupLineSensors();
void calibration();
bool isBlack(int value);
bool canMoveForward();
void setup() {
  Serial.begin(9600);
  BT_module.begin(9600);
  setup_motor_pins();
  setupButtons();
  setupLineSensors();
  calibration();
  for (int i=0;i<SENSOR_COUNT;i++)
  {
    Serial.print(lineSensors.calibrationOn.minimum[i]);
    Serial.print(" ");
  }
  Serial.println();
  for (int i=0;i<SENSOR_COUNT;i++)
  {
    Serial.print(lineSensors.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  leds.begin();
  Serial.println("Calibration done");
  moveForward(200,200);
  
}
int dir=0;
int prevVal[SENSOR_COUNT];
bool allBlack()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    if (!isBlack(sensorValues[i]))
      return false;
 return true;
}
const float KD=5;
int lastError=0;
const float KP=0.1;
void loop() {
  
  //button_control();
  leds.show();
  for (int i=0;i<SENSOR_COUNT;i++)
   prevVal[i]=sensorValues[i];
  uint16_t pos=lineSensors.readLineBlack(sensorValues);
  for (int i=0;i<SENSOR_COUNT;i++)
    {
      Serial.print(isBlack(sensorValues[i]));
      Serial.print(" ");
    }
   Serial.println();
  if (allBlack())
  {
    idle();
  }

  int error=pos-1000;
  int motorSpeed= KP * error + KD * (error - lastError);
  lastError=error;
  int m1Speed=150+motorSpeed;
  int m2Speed=150-motorSpeed;
  if (m1Speed<0)
    m1Speed=0;
  if (m2Speed<0)
    m2Speed=0;
  moveForward(m1Speed,m2Speed);
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
void moveForward(int powerA=255,int powerB=255)
{
  leds.clear();
  leds.fill(BLUE,2,2);
  leds.show(); 
  analogWrite(MOTOR_A2,powerA);
  analogWrite(MOTOR_B1,powerB);
}
void moveBackward(int powerA=255,int powerB=255)
{
  leds.clear();
  leds.fill(RED,0,2);
  leds.show();
  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_B2,powerB);
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
      moveForward(255,255);
      
      break;
    }
    case 2:
    {
      moveBackward(255,255);
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
    moveForward(255,255);
  }
  else if (buttonState2==LOW)
  {
    moveBackward(255,255);
  }
  else 
  {
    idle();
  }
}
void setupLineSensors()
{
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7},SENSOR_COUNT); 
}  

void calibration()
{
   for(int i = 0; i < 2;i++){
        moveForward(200,200);
        for(uint8_t i = 0; i < 25 ; i++){
            lineSensors.calibrate();
            //delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        idle();
        delay(100);
        moveBackward(170,170);
        for(uint8_t i = 0; i < 25 ; i++){
            lineSensors.calibrate();
            //delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        Serial.println("Calibration done");
        
    }
    idle();



}
bool isBlack(int value)
{
  return value>700;
}
