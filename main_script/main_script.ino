//-----------------------------------------------------------------------------------LIBRARIES----------------------------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
//-----------------------------------------------------------------------------------VARIABLES AND CONSTANTS--------------------------------------------------------------------------------
bool isRaceFinished;
//NEOPIXEL VARIABLES
const int PIXEL_PIN=4;
const int PIXEL_NUMBER=4;
Adafruit_NeoPixel leds(PIXEL_NUMBER,PIXEL_PIN,NEO_RGB +NEO_KHZ800);

//NEOPIXELS COLORS 
const uint32_t RED=leds.Color(255,0,0);
const uint32_t YELLOW=leds.Color(234,255,0);
const uint32_t BLUE=leds.Color(0,0,255);
const uint32_t ORANGE=leds.Color(255,165,0);
//MOTOR PINS
const int  MOTOR_A1=11;
const int MOTOR_A2=5;
const int MOTOR_B1=6;
const int MOTOR_B2=9;

//MOTOR ROTATION SENSORS
const int MOTOR_R1=3;
const int MOTOR_R2=2;

//LINE SENSOR VARIABLES
QTRSensors lineSensors;
const int SENSOR_COUNT=8;
int sensorValues[SENSOR_COUNT];
int pos=0;

//BUTTON PINS
const int BUTTON1=7;
const int BUTTON2=8;
const int BUTTON3=12;

//BUTTON STATES
int buttonState1=0;
int buttonState2=0;
int buttonState3=0;

//GRIPPER
const int GRIPPER_PIN=10;
const int GRIPPER_OPEN_PULSE=1600;
const int GRIPPER_CLOSE_PULSE=971;
const int GRIPPER_PULSE_REPEAT=10;


//DISTANCE SENSOR VARIABLES
const int ECHO_PIN=7;
const int TRIG_PIN=8;
int distance;
long duration;


//---------------------------------------------------------------------------------------FUNCTION HEADERS-----------------------------------------------------------------------------------
//MOVING FUNTIONS
void setup_motor_pins();//setup the pin mode for the pins moving the DC motors
void idle();//keeps the motors still
void moveForward(int powerA=255,int powerB=255);//move the robot forward
void moveBackward(int powerA=255,int powerB=255);//move the robot backwards

// ROBOT CONTROL FUNCTIONS
void setupButtons();
void button_control();

//LINE SENSORS
void setupLineSensors();
void calibration();
bool isBlack(int value);
void showSensorValues();
//LINE FOLLOWER FUNCTIONS
bool isBlack(int value); //checks if a sensor value represents black or white
bool lineAhead(); // checks if the line is situated straight ahead of the robot
bool lineRight(); // checks if the line begins to curve right
bool lineLeft(); // checks if the line begins to curve 
bool mostlyBlack(); // checks if there is a crossroad ahead
bool allBlack(); //checks if there is a perpendicular line on the track simbolizing a crossroads or a beginning line 
void lineFollow(); //contains the line follower algorithm
void beginRace();
//DISTANCE SENSOR FUNCTIONS
void readDistance();
void movePastObstcle();
bool isObjectAhead();
//GRIPPER
void gripperServo(int pulse);
void openGripper();
void closeGripper();
//---------------------------------------------------------------------------------FUNCTION DECLARATIONS------------------------------------------------------------------------------------
// SETUP AND LOOP FUNTIONS
void setup() {
  isRaceFinished=false;
  openGripper();
  leds.begin();
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  setup_motor_pins();
  //setupButtons();
  setupLineSensors();
  calibration();
  idle();
  moveForward();
  delay(700);
  idle();
  Serial.println("Beginning Race");
  while(!allBlack())
  {
    pos=lineSensors.readLineBlack(sensorValues);
    showSensorValues();
    beginRace();
  }
  Serial.println("Picking up object");
  idle();
  delay(1000);
  closeGripper();
  Serial.println("Picked up the object");
  moveForward(200,200);
  delay(300);
  idle();
  moveForward(0,170);
  while (allBlack() || mostlyBlack())
    pos=lineSensors.readLineBlack(sensorValues);
  idle();
  Serial.println("Beginning race");
  

}

void loop() {
  if (isRaceFinished==false)
  {
   // button_control();
    leds.show();
    pos=lineSensors.readLineBlack(sensorValues);
    //  showSensorValues();
    readDistance();
    if (isObjectAhead())
    {
      Serial.println("avoid object");
      avoidObject();  
      Serial.println("avoided object");
    }
    else
    {
      lineFollow();
    }
  }
  else if(isRaceFinished==true)
  {
    idle();
    moveBackward(200,200);
    delay(1000);
    idle();
    moveForward(180,0);
    leds.clear();
    leds.fill(RED,0,1);
    leds.fill(BLUE,2,2);
    leds.fill(ORANGE,1,1);
    delay(1000);
    leds.clear();
    delay(1000);
  }
}

//MOTOR FUNCTIONS
void idle()
{
  int motor_pins[4]={MOTOR_A1,MOTOR_A2,MOTOR_B1,MOTOR_B2};
  leds.clear();
  for (int i=0;i<4;i++)
    {      
      analogWrite(motor_pins[i],0);
    }
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
  if (powerA==powerB)
  leds.fill(BLUE,2,2);
  else if (powerA>powerB)
    leds.fill(BLUE,2,1);
  else
    leds.fill(BLUE,3,1);
  leds.show(); 
  analogWrite(MOTOR_A2,powerA);
  analogWrite(MOTOR_B1,powerB);
}

void moveBackward(int powerA=255,int powerB=255)
{
  leds.clear();
  leds.fill(RED,0,2);
  if (powerA==powerB)
  leds.fill(RED,0,2);
  else if (powerA>powerB)
    leds.fill(RED,0,1);
  else
    leds.fill(RED,1,1);
  leds.show();
  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_B2,powerB);
}

//CONTROL FUNCTIONS
void setupButtons()
{
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  pinMode(BUTTON3,INPUT);
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

//LINE SENSOR FUNCTIONS
void setupLineSensors()
{
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7},SENSOR_COUNT); 
}  

void calibration()
{
  //begin the calibration process
   for(int i = 0; i < 2;i++)
   {
        moveForward(170,180);
        for(uint8_t i = 0; i < 25 ; i++)
        {
            lineSensors.calibrate();
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        idle();
        
        moveBackward(160,170);
        delay(1000);
        idle();
        moveForward(170,180);
        for(uint8_t i = 0; i < 25 ; i++)
        {
            lineSensors.calibrate();
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        idle();
        moveBackward(180,170);
        delay(1000);
        idle();
          
    }
    Serial.println("Calibration done");
    //end of the calibration
    idle();
    moveForward(140,150);
    delay(500);
    idle();
    //note the maximum and minimum values recorded during the calibration process
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
}
void showSensorValues()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    {
      Serial.print(isBlack(sensorValues[i]));
      Serial.print(" ");
    }
   Serial.println();
}
//LINE FOLLOWER LOGIC FUNCTIONS
bool isBlack(int value)
{
  return value>700;
}

bool lineAhead()
{
  if (!isBlack(sensorValues[3]) || !isBlack(sensorValues[4]))
    return false;

  for (int i=0;i<SENSOR_COUNT;i++)
  {
    if (i==3 || i==4)
      continue;
    if (isBlack(sensorValues[i]))
      return false;
  }
  return true;
}

bool lineRight()
{
  for (int i=SENSOR_COUNT/2+1;i<SENSOR_COUNT;i++)
  {
    if (isBlack(sensorValues[i]))
      return false;
  }
  return true;
}

bool lineLeft()
{
  for (int i=0;i<SENSOR_COUNT/2-1;i++)
  {
    if (isBlack(sensorValues[i]))
      return false;
  }
  return true;
}

bool mostlyBlack()
{
  int k=0;
  for (int i=0;i<SENSOR_COUNT/2-1;i++)
    if (isBlack(sensorValues[i]))
      k=1;
  for (int i=SENSOR_COUNT/2+1;i<SENSOR_COUNT;i++)
    if (isBlack(sensorValues[i]))
      k=2;
  if (k==2)
    return true;
 return false;
}

bool allBlack()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    if (!isBlack(sensorValues[i]))
        return false;
  return true;  
}

bool allWhite()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    if (isBlack(sensorValues[i]))
        return false;
  return true;
}
int lastMove=0;//last move before the robot got out of the truck - useful for reentering the track - can have value 1 for right or -1 for left
void lineFollow()
{
  if (allBlack() )
  {
    idle();
    moveForward(180,180);
    delay(500);
    idle();
    //if the line is still black then this is the end of the race and the object will be dropped
    pos=lineSensors.readLineBlack(sensorValues);
    if (allBlack())
    {
      openGripper();
      isRaceFinished=true;
      
    }
  }
  else if (allWhite())
    {
      idle();
      if (lastMove==-1)
        moveForward(0,200);
      else
        moveForward(200,0);
      while(allWhite())
        pos=lineSensors.readLineBlack(sensorValues);
      idle();
      
    }
  else if (lineAhead())
  {
    moveForward(240,240);
  }
  else  if (lineRight())
  {
    idle();
    lastMove=1;
    moveForward(220,0);
  }
 else if (lineLeft())
  {
    idle();
    lastMove=-1;
    moveForward(0,220);
  }
 else if (mostlyBlack())
 {
  idle();
  moveForward(220,220);
  while (mostlyBlack())
   pos=lineSensors.readLineBlack(sensorValues);
  idle();
 }
  else
  idle();
}

void beginRace()
{
  if (lineAhead())
  {
    moveForward(200,200);
  }
  else  if (lineRight())
  {
    idle();
    moveForward(200,0);
  }
 else if (lineLeft())
  {
    idle();
    moveForward(0,200);
  }
 else if (allWhite())
 {
  idle();
 }
}




void gripperServo(int pulse)
{
    for(int i = 0; i < GRIPPER_PULSE_REPEAT;i++)
    {
        digitalWrite(GRIPPER_PIN,HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER_PIN,LOW);
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
bool isObjectAhead()
{
  return (distance<=15);
}
void readDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  //Sets trigPin on High state for 10 microSec
  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  //Reads echoPin, returns the wave travel time in miceoSec
  duration=pulseIn(ECHO_PIN,HIGH);
  //Calculate distance
  distance=duration*0.034/2;
  //Prints the distance on monitor
  Serial.print("Distance:");
  Serial.println(distance);
}
void avoidObject1()
{
  int turningTime=420;
  idle();
  leds.clear();
  leds.fill(ORANGE,2,2);
  leds.show();
  delay(3000);
  //get off the track
  moveForward(200,0);
  delay(turningTime);
  idle();
  moveForward(200,200);
  delay(1100);
  idle();
  moveForward(0,200);
  delay(2*turningTime);
  idle();
  moveForward(200,200);
  while (allWhite() || mostlyBlack())
    pos=lineSensors.readLineBlack(sensorValues);
  idle();
  
}
void avoidObject(){
  bool itMoved=false;
  bool isTurningLeft=true;
  bool isTurningLeftAgain=false;
  bool isTurningRight=false;
 
   if(isTurningLeft){
    moveForward(0,200);
    delay(900);
    isTurningLeft=false;
    itMoved=true;
   }
    if(itMoved){
    moveForward(200,200);
    delay(700);
    itMoved=false;
    isTurningRight=true;
    }
   if(isTurningRight){
    moveForward(200,0);
    delay(900);
    isTurningRight=false;
    itMoved=true;
   }
    if(itMoved){
    moveForward(200,200);
    delay(700);
    itMoved=false;
    isTurningRight=true;
    }
    if(isTurningRight){
    moveForward(200,0);
    delay(900);
    isTurningRight=false;
    isTurningLeftAgain=true;
   }
    if(itMoved){
    moveForward(200,200);
    delay(800);
    itMoved=false;
    isTurningLeftAgain=true;
    }
  if(isTurningLeftAgain){
    moveForward(0,200);
    delay(900);
    isTurningLeftAgain=false;
    idle();
    }
}
