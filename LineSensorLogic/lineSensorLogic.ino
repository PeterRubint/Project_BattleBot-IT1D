//Libraries
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h> 

//Variables and pins

//NEOPIXELS
const int PIXELS=9;
const int PIXELS_NUMBER=4;
Adafruit_NeoPixel pixels(PIXELS_NUMBER,PIXELS,NEO_RGB + NEO_KHZ800);

//ULTRASENSOR
const int TRIG_PIN=8;
const int ECHO_PIN=7;
int distance;
long duration;
//GRIPPER
const int GRIPPER=4;
const int GRIPPER_OPEN_PULSE=1600;
const int GRIPPER_CLOSE_PULSE=971;
const int GRIPPER_PULSE_REPEAT=10;
//MOTOR
const int MOTOR_A_FWD=5;
const int MOTOR_A_BWD=12;
const int MOTOR_B_FWD=6;
const int MOTOR_B_BWD=13;
const int MOTOR_A_R=2;
const int MOTOR_B_R=3;
volatile int countA;
volatile int countB;
const int MIN_R=130;
const int MIN_L=140;
//LINE SENSOR VARIABLES
QTRSensors lineSensors;
const int SENSOR_COUNT=8;
int sensorValues[SENSOR_COUNT];
int pos=0;

//LINE MAZE LOGIC VARIABLES
bool wait =true;
bool set =false;
bool solved=false ;
bool victory=false;

//FUNCTION HEADERS

//MOTOR FUNCTIONS
void setupMotorPins();
void rotationA();
void rotationB();
void forward(int left=250, int right=250);
void backward(int left=250, int right=250);
void adjustLeft();
void adjustRight();
void stop();
void rightTurn(int degree);// turns right using one motor
void leftTurn(int degree); //turns left using one motor
void rotateLeft(int degree);//turns left using both motors
void leftSpin();

//ULTRA SONIC SENSORS 
int getDistance();
void detectObject();

//LINE SENSOR
void setupLineSensors();
bool middleLine();
bool lineLeft();
bool lineRight();
bool allWhite();
bool allBlack();
bool isBlack(int value);

//LINE MAZE LOGIC
void solveMaze();
void endRace();
void beginRace();

//GRIPPER FUNCTIONS
void gripperServo(int pulse);
void openGripper();
void closeGripper();




void setup() 
{
 Serial.begin (9600);
 pixels.begin();
 pixels.clear();
 pixels.show();

 setupMotorPins();
 setupLineSensors();
 pinMode(TRIG_PIN,OUTPUT);
 pinMode(ECHO_PIN,INPUT);
}

void loop() 
{
 Serial.println("");
 lineSensors.read(sensorValues);
 lineSensors.calibrate();
 pos=lineSensors.readLineBlack(sensorValues);
 if (wait)
 {
  detectObject(); 
 }
 else if (!set)
 {
  beginRace();
 }
 else if (!solved)
 {
  maze();
 }
 else if (!victory)
 {
  endRace();
 }
}
//FUNCTION DEFINITIONS

//FUNCTION DECLARATIONS

//MOTOR FUNCTIONS
void setupMotorPins()
{
  pinMode(MOTOR_A_FWD,OUTPUT);
  pinMode(MOTOR_A_BWD,OUTPUT);
  pinMode(MOTOR_B_FWD,OUTPUT);
  pinMode(MOTOR_B_BWD,OUTPUT);
  pinMode(MOTOR_A_R,INPUT);
  pinMode(MOTOR_B_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_A_R),rotationA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B_R),rotationB,CHANGE);
}

void rotationA()
{
  noInterrupts();
  countA++;
  interrupts();
}
void rotationB()
{
  noInterrupts();
  countB++;
  interrupts();
}
void forward(int left=250,int right=250 )
{
  stop();
  analogWrite(MOTOR_A_FWD,left);
  analogWrite(MOTOR_B_FWD,right); 
}
void backward(int left=250,int right=250)
{
  stop();
  analogWrite(MOTOR_A_BWD,left);
  analogWrite(MOTOR_B_BWD,right);
}
void stop()
{
  analogWrite(MOTOR_A_FWD,0);
  analogWrite(MOTOR_A_BWD,0);
  analogWrite(MOTOR_B_FWD,0);
  analogWrite(MOTOR_B_BWD,0);
}
void adjustLeft()
{
  stop();
  analogWrite(MOTOR_A_FWD,130);
  analogWrite(MOTOR_B_FWD,250);
}
void adjustRight()
{
  stop();
  analogWrite(MOTOR_A_FWD,250);
  analogWrite(MOTOR_B_FWD,130);
}
void rightTurn(int grades)
{
  bool rotate=true;
  int cycles=grades;
  countA=0;
  while (rotate)
  {
    if (countA<cycles)
    {
      stop();
      forward(200,0);
    }
    else if (countA>cycles)
    {
      rotate=false;
    }
  }
}
void leftTurn(int grades)
{
  bool rotate=true;
  int cycles=grades;
  countB=0;
  while (rotate)
  {
    if (countB<cycles)
    {
      stop();
      forward(0,200);
    }
    else if (countB>cycles)
      rotate=false;
  }
}
void leftTurn2(int grades)
{
  bool rotate=true;
  int cycle=grades;
  countB=0;
  while (rotate==true)
  {
    if (countB < cycle)
    {
      stop();
      forward(0,150);
      backward(150,0);
    }
    else if (countB>cycle)
      rotate=false;
  }
}
void leftSpin()
{
  stop();
  analogWrite(MOTOR_B_FWD,130);
  analogWrite(MOTOR_A_BWD,130);
}

int getDistance() 
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  return distance;
}

void detectObject()
{
  int distance = getDistance();

  if(distance < 25)
  {
    wait = false;
  } 
}

//LINE SENSORS

void setupLineSensors()
{
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SENSOR_COUNT);
}
bool isBlack(int value)
{
  return value>700;
}
bool middleLine()
{
  return (isBlack(sensorValues[3]) && isBlack(sensorValues[4]));
}
bool lineLeft()
{
  return (isBlack(sensorValues[0]) && isBlack(sensorValues[1]) && isBlack(sensorValues[2]));
}
bool lineRight()
{
  return (isBlack(sensorValues[5]) && isBlack(sensorValues[6]) && isBlack(sensorValues[7]));
}
bool allWhite()
{
  return (!middleLine && !lineLeft() && !lineRight());
}
bool allBlack()
{
  return (middleLine && lineLeft() && lineRight());
}

//GRIPPER
void gripperServo(int pulse)
{
    for(int i = 0; i < GRIPPER_PULSE_REPEAT;i++)
    {
        digitalWrite(GRIPPER,HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER,LOW);
        delay(20);
    }
}
void openGripper()
{
    gripperServo(GRIPPER_OPEN_PULSE);
}
void closeGripper()
{
  gripperServo(GRIPPER_CLOSE_PULSE);
}

//LINE MAZE
void beginRace()
{
  delay(2000);
  int lines=0;
  forward(MIN_L,MIN_R);
  while (true)
  {
    lineSensors.read(sensorValues);
    if (lines>4)
    {
      stop();
      delay(200);
      openGripper();
      delay(50);
      closeGripper();
      delay(1000);
      leftTurn(28);
      break;
    }
    else if (allBlack())
    {
      lines++;
      delay(200);
    }
  }
  set=true;
}
void maze()
{
  if (lineRight() && isBlack(sensorValues[4]))
  {
    forward();
    delay(100);
    stop();
    while (true)
    {
      lineSensors.read(sensorValues);
      if (allBlack())
      {
        stop();
        delay(250);
        openGripper();
        solved=true;
        break;
      }
      else
      {
        rightTurn(42);
        break;
      }
    }
  }
  else if (middleLine())
  {
    forward();
  }
  else if (lineRight())
  {
    adjustRight();
  }
  else if (lineLeft())
  {
    adjustLeft();
  }
  else if (allBlack())
  {
    forward();
    delay(100);
    leftTurn2(26);
    while(true)
    {
      lineSensors.read(sensorValues);
      if (allBlack())
      {
        break;
      }
      if (allWhite())
      {
        leftSpin();
      }
    }
  }
  
}
void endRace()
{
  leftTurn(200);
  stop();
  rightTurn(200);
  stop();
}
