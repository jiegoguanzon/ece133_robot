#include <math.h>
#include <NewPing.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "pinDefs.h"

const int MAX_SENSING_DISTANCE = 50;

const int WHEEL_DIAMETER = 65;
const int ROBOT_DIAMETER = 200;
const int UNIT_LENGTH = 200;

const int DEFAULT_MOTOR_SPEED = 120;
const int MIN_MOTOR_SPEED = 80;
const int MAX_MOTOR_SPEED = 160;

const int STEPS_PER_ROTATION = 200;

const long SENSOR_PULSE_INTERVAL = 100000;

const float P_GAIN = 3;
const float I_GAIN = 1;
const float D_GAIN = 2;

unsigned long prevRightMicros;
unsigned long currRightMicros;
unsigned long prevLeftMicros;
unsigned long currLeftMicros;

unsigned long prevPulseMicros;
unsigned long currPulseMicros;

unsigned long rightMotorStepInterval;
unsigned long leftMotorStepInterval;

int rightMotorSpeed;
int leftMotorSpeed;

int distanceToWall;

int distancePerStep;
int stepsPerUnitLength;
int stepsPerUnitTurn;

float distance;
float prevDistance;

float error;
float prevError;
float errorSum;

float stepSize = 1;

long rightCurrPos = 0;
long leftCurrPos = 0;
int state = 0;
int frontSteps = 285;
int turnSteps = 125;
long positions[2];

AccelStepper rightStepper(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
AccelStepper leftStepper(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);

MultiStepper steppers;

NewPing sonar(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_SENSING_DISTANCE);

void setup(){

  Serial.begin(9600);

  pinMode(RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_M0_PIN, OUTPUT);
  pinMode(RIGHT_M1_PIN, OUTPUT);
  pinMode(RIGHT_M2_PIN, OUTPUT);
  pinMode(RIGHT_RESET_PIN, OUTPUT);
  pinMode(RIGHT_SLEEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  pinMode(LEFT_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_M0_PIN, OUTPUT);
  pinMode(LEFT_M1_PIN, OUTPUT);
  pinMode(LEFT_M2_PIN, OUTPUT);
  pinMode(LEFT_RESET_PIN, OUTPUT);
  pinMode(LEFT_SLEEP_PIN, OUTPUT);
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);

  digitalWrite(RIGHT_ENABLE_PIN, HIGH);   // disable right motor
  digitalWrite(LEFT_ENABLE_PIN, HIGH);    // disable left motor

  digitalWrite(RIGHT_RESET_PIN, LOW);     // reset right motor
  digitalWrite(RIGHT_SLEEP_PIN, LOW);     // sleep right motor

  digitalWrite(LEFT_RESET_PIN, LOW);      // reset left motor
  digitalWrite(LEFT_SLEEP_PIN, LOW);      // sleep left motor

  rightStepper.setMaxSpeed(300);
  rightStepper.setAcceleration(10.0);

  leftStepper.setMaxSpeed(300);
  leftStepper.setAcceleration(10.0);

  digitalWrite(RIGHT_M2_PIN, LOW);
  digitalWrite(RIGHT_M1_PIN, LOW);
  digitalWrite(RIGHT_M0_PIN, LOW);

  digitalWrite(LEFT_M2_PIN, LOW);
  digitalWrite(LEFT_M1_PIN, LOW);
  digitalWrite(LEFT_M0_PIN, LOW);

  digitalWrite(RIGHT_DIR_PIN, HIGH);
  digitalWrite(LEFT_DIR_PIN, LOW);

  digitalWrite(RIGHT_SLEEP_PIN, HIGH);    // wake right motor
  digitalWrite(RIGHT_RESET_PIN, HIGH);    // un-reset right motor

  digitalWrite(LEFT_SLEEP_PIN, HIGH);     // wake left motor
  digitalWrite(LEFT_RESET_PIN, HIGH);     // un-reset left motor

  digitalWrite(RIGHT_ENABLE_PIN, LOW);    // enable right motor
  digitalWrite(LEFT_ENABLE_PIN, LOW);     // enable left motor

  steppers.addStepper(rightStepper);
  steppers.addStepper(leftStepper);

  moveForward();
  moveForward();
  moveForward();
  turnLeft();
  turnLeft();
  moveForward();
  moveForward();
  turnRight();
  turnRight();
  turnLeft();
  turnRight();
  turnRight();
  turnLeft();
  turnLeft();

}

void loop(){
}

void moveForward(){
  positions[0] = rightStepper.currentPosition() + frontSteps;
  positions[1] = leftStepper.currentPosition() - frontSteps;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  delay(1000);
}

void turnLeft(){
  positions[0] = rightStepper.currentPosition() + turnSteps;
  positions[1] = leftStepper.currentPosition() + turnSteps;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  delay(1000);
  moveForward();
}

void turnRight(){
  positions[0] = rightStepper.currentPosition() - turnSteps;
  positions[1] = leftStepper.currentPosition() - turnSteps;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  delay(1000);
  moveForward();
}