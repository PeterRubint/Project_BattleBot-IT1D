#define SERVO_A1 10 //LEFT WHEEL
#define SERVO_A2 11 //PINS

#define SERVO_B1 8 //RIGHT WHEEL
#define SERVO_B2 9 //PINS

void setup() {
  // put your setup code here, to run once:
  pinMode(SERVO_A1,INPUT); //LEFT
  pinMode(SERVO_A2,INPUT);

  pinMode(SERVO_B1,INPUT); //RIGHT
  pinMode(SERVO_B2,INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(SERVO_A1, LOW); // LEFT WHEEL SPIN FORWARD
  digitalWrite(SERVO_A2, HIGH);

  digitalWrite(SERVO_B1, LOW); // RIGHT WHEEL SPIN FORWARD
  digitalWrite(SERVO_B2, HIGH);
 

}
