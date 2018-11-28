#include <math.h>
#include <NewPing.h>

const int wheelDiameter = 65;
const int robotDiameter = 200;
const int unitLength = 200;

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

unsigned long  rightMotorStepInterval = 0;
unsigned long  leftMotorStepInterval = 0;

int defaultMotorSpeed = 120;              //  RPM
int rightMotorSpeed = defaultMotorSpeed;  //  RPM
int leftMotorSpeed = defaultMotorSpeed;   //  RPM

int distancePerStep;
int stepsPerUnitLength;
int stepsPerUnitTurn;

float distance = 0;
float prevDistance = 0;

const int maxDistance = 50;

NewPing sonar(rightTrigPin, rightEchoPin, maxDistance);

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

  leftMotorSpeed = 120;
  rightMotorSpeed = 120;

  rightMotorStepInterval = getStepInterval(rightMotorSpeed);
  leftMotorStepInterval = getStepInterval(leftMotorSpeed);

  digitalWrite(rightDirPin, HIGH);
  digitalWrite(leftDirPin, LOW);

}

void loop(){

  int speedAdjust = 2;
  int distanceToWall = 4;
  float scale = 1;
  int maxSpeed = 140;
  int minSpeed = 100;

  currPulseMicros = micros();

  if(currPulseMicros - prevPulseMicros >= 100000){

    prevPulseMicros = currPulseMicros;

    distance = sonar.ping();
    distance = distance * 0.034 / 2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    //rightMotorSpeed = 120 - (scale * (distance - distanceToWall));
    //leftMotorSpeed = 120 + (scale * (distance - distanceToWall));

    if(distance > 5){
      rightMotorSpeed += scale * (distance - distanceToWall);
      leftMotorSpeed -= scale * (distance - distanceToWall);
    } else if(distance < 3){
      rightMotorSpeed -= scale * (distance - distanceToWall);
      leftMotorSpeed += scale * (distance - distanceToWall);
    } else{
      rightMotorSpeed = 120;
      leftMotorSpeed = 120;
    }

    if(rightMotorSpeed >= maxSpeed)
      rightMotorSpeed = maxSpeed;
        
    if(rightMotorSpeed <= minSpeed)
      rightMotorSpeed = minSpeed;

    if(leftMotorSpeed >= maxSpeed)
      leftMotorSpeed = maxSpeed;

    if(leftMotorSpeed <= minSpeed)
      leftMotorSpeed = minSpeed;

    rightMotorStepInterval = getStepInterval(rightMotorSpeed);
    leftMotorStepInterval = getStepInterval(leftMotorSpeed);

    Serial.print("RightMotorSpeed: ");
    Serial.println(rightMotorSpeed);
    Serial.print("LeftMotorSpeed: ");
    Serial.println(leftMotorSpeed);

  }

  currRightMicros = micros();
  currLeftMicros = micros();

  if(distance != 0){
  stepMotor(0, rightStepPin, rightMotorStepInterval, currRightMicros, prevRightMicros);
  stepMotor(1, leftStepPin, leftMotorStepInterval, currLeftMicros, prevLeftMicros);
  }

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