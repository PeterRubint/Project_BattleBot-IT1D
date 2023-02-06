#define IR_PIN 12
#define D1_PIN 13

void setup() {
  // put your setup code here, to run once:
  pinMode(IR_PIN,INPUT);
  pinMode(D1_PIN,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int dataIR = analogRead(IR_PIN);
  int dataD1 = analogRead(D1_PIN);
  Serial.print("IR Reading: ");
  Serial.println(dataIR);
  delay(1000);

  Serial.print("D1 Reading: ");
  Serial.println(dataD1);
  delay(1000);

  

}
