#include <math.h>

const int wheelDiameter = 65;             // millimeters
const int robotDiameter = 200;            // millimeters
const int unitLength = 200;               // millimeters

const float stepSize = 1;
const int stepsPerRotation = 200;

const int sleepPin = 2;
const int resetPin = 3;
const int m2Pin = 4;
const int m1Pin = 5;
const int m0Pin = 6;
const int enablePin = 7;

const int rightDirPin = 8;
const int rightStepPin = 9;

const int leftDirPin = 10;
const int leftStepPin = 11;

const int rightTrigPin = 31;
const int rightEchoPin = 19;
const int frontTrigPin = 35;
const int frontEchoPin = 20;
const int leftTrigPin = 39;
const int leftEchoPin = 21;

unsigned long prevRightMicros = 0;
unsigned long currRightMicros = 0;
unsigned long prevLeftMicros = 0;
unsigned long currLeftMicros = 0;

unsigned long prevPulseMicros = 0;
unsigned long currPulseMicros = 0;
unsigned long prevLoopMicros = 0;
unsigned long currLoopMicros = 0;
unsigned long prevEchoMicros = 0;
unsigned long currEchoMicros = 0;

unsigned long  rightMotorStepInterval = 0;
unsigned long  leftMotorStepInterval = 0;

int defaultMotorSpeed = 100;              //  RPM
int rightMotorSpeed = defaultMotorSpeed;  //  RPM
int leftMotorSpeed = defaultMotorSpeed;   //  RPM

int distancePerStep;
int stepsPerUnitLength;
int stepsPerUnitTurn;

void stepMotor(const int stepPin, unsigned long stepInterval);
unsigned long getStepInterval(int motorSpeed);
  
int distanceToWall = 10;
volatile int distance;
volatile int duration;


void setup(){

  Serial.begin(9600);

  pinMode(sleepPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m1Pin, OUTPUT);
  pinMode(m0Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(rightStepPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);

  pinMode(leftStepPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);

  distancePerStep = stepSize * (wheelDiameter * M_PI) / stepsPerRotation;
  stepsPerUnitLength = round(unitLength / distancePerStep);
  stepsPerUnitTurn = round((robotDiameter * M_PI / 4) / distancePerStep);

  rightMotorStepInterval = getStepInterval(rightMotorSpeed);
  leftMotorStepInterval = getStepInterval(leftMotorSpeed);

  digitalWrite(rightDirPin, HIGH);
  digitalWrite(leftDirPin, LOW);

}

void loop(){

  currRightMicros = micros();
  currLeftMicros = micros();

  stepMotor(0, rightStepPin, rightMotorStepInterval, currRightMicros, prevRightMicros);
  stepMotor(1, leftStepPin, leftMotorStepInterval, currLeftMicros, prevLeftMicros);

}

void stepMotor(int motor, const int stepPin, unsigned long stepInterval, unsigned long currMicros, unsigned long prevMicros){

  if (currMicros - prevMicros >= stepInterval){
    if(motor == 0)
      prevRightMicros += stepInterval;
    else if (motor == 1)
      prevLeftMicros += stepInterval;
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
  }

}

unsigned long getStepInterval(int motorSpeed){
  return (60.0000 / (motorSpeed * stepsPerRotation)) * 1E6;
}