int trigPin = 13;    // Ultrasonic Trigger
int echoPin = 12;    // Ultrasonic Echo
long duration, cm; // variables for distance measurement

Serial.begin (9600);


void setup() {  //Define inputs and outputs

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {

    if(measureDistance() <= 20){

        Serial.println("Wall detected!");

        /*
            TURN THE SENSOR AND SCAN FOR WALLS

            MOVE ACCORDINGLY
        
        
        */
    }

}

float measureDistance() {  // Returns the distance from an object detected by the ultrasonic distance sensor

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  return cm;
}