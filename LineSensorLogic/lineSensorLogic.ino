#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>

const int PIXEL_PIN = 7;
const int PIXEL_NUMBER = 4;
Adafruit_NeoPixel leds(PIXEL_NUMBER, PIXEL_PIN, NEO_RGB + NEO_KHZ800);

// motor pins
const int MOTOR_A1 = 11;
const int MOTOR_A2 = 8;
const int MOTOR_B1 = 9;
const int MOTOR_B2 = 5;
const int MOTOR_R1 = 3;
const int MOTOR_R2 = 2;

// motor constants
const int MINIMUM_SPEED = 200;
const int MAXIMUM_SPEED = 255;
const int KD = 5;
const int KP = 2;

// colors
const uint32_t RED = leds.Color(255, 0, 0);
const uint32_t YELLOW = leds.Color(234, 255, 0);
const uint32_t BLUE = leds.Color(0, 0, 255);

// line sensors
QTRSensors lineSensors;
const int SENSOR_COUNT = 8;
int sensorValues[SENSOR_COUNT];
int pos = 0;
int error = 0;
int lastError = 0;

// button pins
const int BUTTON1 = 12;
const int BUTTON2 = 13;
const int BUTTON3 = 2;

// button states;
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;
// function declaration

// moving functions
void idle();                                           // keeps the motors still
void setup_motor_pins();                               // setup the pin mode for the pins moving the DC motors
void moveForward(int powerA = 255, int powerB = 255);  // move the robot forward
void moveBackward(int powerA = 255, int powerB = 255); // move the robot backwards

// control functions
void BT_control();
void button_control();

void setupButtons();

// line sensor
void setupLineSensors();
void calibration();
bool isBlack(int value);
void lineFollow();

void setup()
{
    Serial.begin(9600);

    setup_motor_pins();
    setupButtons();
    setupLineSensors();
    calibration();
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(lineSensors.calibrationOn.minimum[i]);
        Serial.print(" ");
    }
    Serial.println();
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(lineSensors.calibrationOn.maximum[i]);
        Serial.print(" ");
    }
    leds.begin();
    Serial.println("Calibration done");
    moveForward(200, 200);
}

void loop()
{

    // button_control();
    leds.show();
    pos = lineSensors.readLineBlack(sensorValues);
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(isBlack(sensorValues[i]));
        Serial.print(" ");
    }
    Serial.println();
    lineFollow1();
}

void setup_motor_pins()
{
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);
    pinMode(MOTOR_R1, INPUT);
    pinMode(MOTOR_R2, INPUT);
}

void moveForward(int powerA = 255, int powerB = 255)
{
    leds.clear();
    leds.fill(BLUE, 2, 2);
    leds.show();
    analogWrite(MOTOR_A2, powerA);
    analogWrite(MOTOR_B1, powerB);
}

void moveBackward(int powerA = 255, int powerB = 255)
{
    leds.clear();
    leds.fill(RED, 0, 2);
    leds.show();
    analogWrite(MOTOR_A1, powerA);
    analogWrite(MOTOR_B2, powerB);
}

void idle()
{
    int motor_pins[4] = {MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2};
    leds.clear();
    for (int i = 0; i < 4; i++)
    {

        analogWrite(motor_pins[i], 0);
    }
}

void setupButtons()
{
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    pinMode(BUTTON3, INPUT);
}

void button_control()
{
    buttonState1 = digitalRead(BUTTON1);
    buttonState2 = digitalRead(BUTTON2);
    if (buttonState1 == LOW)
    {
        moveForward(255, 255);
    }
    else if (buttonState2 == LOW)
    {
        moveBackward(255, 255);
    }
    else
    {
        idle();
    }
}

void setupLineSensors()
{
    lineSensors.setTypeAnalog();
    // lineSensors.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7},SENSOR_COUNT);
    lineSensors.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SENSOR_COUNT);
}

void calibration()
{
    for (int i = 0; i < 2; i++)
    {
        moveForward(200, 200);
        for (uint8_t i = 0; i < 25; i++)
        {
            lineSensors.calibrate();
            // delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        idle();
        delay(100);
        moveBackward(170, 170);
        for (uint8_t i = 0; i < 25; i++)
        {
            lineSensors.calibrate();
            // delay(20);
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        Serial.println("Calibration done");
    }
    
    idle();
}

bool isBlack(int value)
{
    return value > 700;
}

bool lineAhead()
{
    if (!isBlack(sensorValues[3]) || !isBlack(sensorValues[4]))
        return false;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (i == 3 || i == 4)
            continue;
        if (isBlack(sensorValues[i]))
            return false;
    }
    return true;
}

bool lineRight()
{
    for (int i = SENSOR_COUNT / 2 + 1; i < SENSOR_COUNT; i++)
    {
        if (isBlack(sensorValues[i]))
            return false;
    }
    return true;
}

bool lineLeft()
{
    for (int i = 0; i < SENSOR_COUNT / 2 - 1; i++)
    {
        if (isBlack(sensorValues[i]))
            return false;
    }
    return true;
}

bool mostlyBlack()
{
    int k = 0;
    for (int i = 0; i < SENSOR_COUNT / 2 - 1; i++)
        if (isBlack(sensorValues[i]))
            k = 1;
    for (int i = SENSOR_COUNT / 2 + 1; i < SENSOR_COUNT; i++)
        if (isBlack(sensorValues[i]))
            k = 2;
    if (k == 2)
        return true;
    return false;
}

bool allBlack()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
        if (!isBlack(sensorValues[i]))
            return false;
    return true;
}

bool allWhite()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
        if (isBlack(sensorValues[i]))
            return false;
    return true;
}

void lineFollow1()
{
    if (allBlack())
    {
        idle();
        moveForward(200, 200);
        delay(200);
        idle();
    }
    else if (allWhite())
    {
        moveForward(200, 0);
    }
    else if (lineAhead())
    {
        moveForward(200, 200);
    }
    else if (lineRight())
    {
        idle();
        moveForward(200, 0);
    }
    else if (lineLeft())
    {
        idle();
        moveForward(0, 200);
    }
    else if (mostlyBlack())
    {
        idle();
        moveForward(200, 200);
        delay(200);
        idle();
    }
    else
        idle();
}
