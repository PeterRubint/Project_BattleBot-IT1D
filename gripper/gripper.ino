#include <Servo.h>
const int gripper_pin=10;
const int button1_pin=7;
Servo gripper;
void setup() {
  // put your setup code here, to run once:
  gripper.attach(gripper_pin);
  Serial.begin(9600);
}
int value=0;
void loop() {
  if (digitalRead(button1_pin)==LOW)
    {
      value++;
      gripper.write(180);
      delay(5000);    
    }
   
    if (value>0)
         value--;
  
  gripper.write(0);
}
