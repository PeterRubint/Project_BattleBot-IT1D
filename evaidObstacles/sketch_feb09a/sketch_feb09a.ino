const int echoPin = 7;   //pin of the first button
const int trigPin = 8;  //pin of the second button
long duration;
int distance;

const int  MOTOR_A1=11; //left back
const int MOTOR_A2=5; //left fw
const int MOTOR_B1=6;//right
const int MOTOR_B2=9;

//MOTOR ROTATION SENSORS
const int MOTOR_R1=3;
const int MOTOR_R2=2;


void setup() {
  // put your setup code here, to run once:
  
 pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  setup_motor_pins();
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
void loop() {
  // put your main code here, to run repeatedly:
  
   readDistance();
 
    if(distance<=8){
    movePastObstcle();
    
  }

  
}

void readDistance(){
    digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //Sets trigPin on High state for 10 microSec
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //Reads echoPin, returns the wave travel time in miceoSec
  duration=pulseIn(echoPin,HIGH);
  //Calculate distance
  distance=duration*0.034/2;
  //Prints the distance on monitor
  Serial.print("Distance:");
  Serial.println(distance);
  }

  //MOTOR FUNCTIONS
void idle()
{
  int motor_pins[4]={MOTOR_A1,MOTOR_A2,MOTOR_B1,MOTOR_B2};

  for (int i=0;i<4;i++)
    {      
      analogWrite(motor_pins[i],0);
    }
}



void moveForward(int powerA=255,int powerB=255)
{
  
  analogWrite(MOTOR_A2,powerA);
  analogWrite(MOTOR_B1,powerB);
}

void moveBackward(int powerA=255,int powerB=255)
{
  
  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_B2,powerB);
}
void moveLeft(int powerA=0,int powerB=255)
{
  
  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_B2,powerB);
}
void moveRight(int powerA=-255,int powerB=0)
{
  
  analogWrite(MOTOR_A2,powerA);
  analogWrite(MOTOR_B2,powerB);
}

void movePastObstcle(){
  bool itMoved=false;
  bool isTurningLeft=true;
  bool isTurningLeftAgain=false;
  bool isTurningRight=false;
 
   if(isTurningLeft){
    moveLeft(255,0);
    delay(700);
    isTurningLeft=false;
    idle();
    delay(900);
    itMoved=true;
   }
    if(itMoved){
    moveForward(255,255);
    delay(700);
    idle();
    itMoved=false;
    delay(500);
    isTurningRight=true;
    }
   if(isTurningRight){
    moveRight(0,255);
    delay(700);
    isTurningRight=false;
    idle();
    itMoved=true;
   }
    if(itMoved){
    idle();
    moveForward(255,255);
    delay(1700);
    idle();
    itMoved=false;
    delay(500);
    isTurningRight=true;
    }
    if(isTurningRight){
    moveRight(0,255);
    delay(700);
    isTurningRight=false;
    idle();
    itMoved=true;
   }
    if(itMoved){
    idle();
    moveForward(255,255);
    delay(800);
    idle();
    itMoved=false;
    delay(500);
    isTurningLeftAgain=true;
    }
  if(isTurningLeftAgain){
    moveLeft(255,0);
    delay(700);
    isTurningLeftAgain=false;
    idle();
    }
  
    
    
  

}
