#include <math.h>
#include <NewPing.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "pinDefs.h"

const int SONAR_NUM = 3;
const int MAX_SENSING_DISTANCE = 20;
const int PING_INTERVAL = 33;

const int WHEEL_DIAMETER = 65;
const int ROBOT_DIAMETER = 200;
const int UNIT_LENGTH = 200;

const int DEFAULT_MOTOR_SPEED = 500;
const int MIN_MOTOR_SPEED = 300;
const int MAX_MOTOR_SPEED = 700;

const int STEPS_PER_ROTATION = 200;

const long SENSOR_PULSE_INTERVAL = 100000;

const float P_GAIN = 3;
const float I_GAIN = 1;
const float D_GAIN = 2;

int m2 = 0;
int m1 = 0;
int m0 = 0;

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

unsigned long pingTimer[SONAR_NUM];

float distance[SONAR_NUM];
float prevDistance[SONAR_NUM];

float error;
float prevError;
float errorSum;

float stepSize;

const int BASE_FORWARD_STEPS = 285;
const int BASE_TURN_STEPS = 125;

long positions[2];

AccelStepper rightStepper(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
AccelStepper leftStepper(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);

NewPing sonar[SONAR_NUM] = {
  NewPing(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_SENSING_DISTANCE),
  NewPing(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_SENSING_DISTANCE),
  NewPing(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_SENSING_DISTANCE)
};

void setup(){

  Serial.begin(115200);

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

  rightStepper.setMaxSpeed(DEFAULT_MOTOR_SPEED);
  rightStepper.setAcceleration(1000);

  leftStepper.setMaxSpeed(DEFAULT_MOTOR_SPEED);
  leftStepper.setAcceleration(1000);

  digitalWrite(RIGHT_M2_PIN, m2);
  digitalWrite(RIGHT_M1_PIN, m1);
  digitalWrite(RIGHT_M0_PIN, m0);

  digitalWrite(LEFT_M2_PIN, m2);
  digitalWrite(LEFT_M1_PIN, m1);
  digitalWrite(LEFT_M0_PIN, m0);

  stepSize = getStepSize(m2, m1, m0);

  stepsPerUnitLength = getUnitLengthSteps(stepSize);
  stepsPerUnitTurn = getUnitTurnSteps(stepSize);

  digitalWrite(RIGHT_DIR_PIN, HIGH);
  digitalWrite(LEFT_DIR_PIN, LOW);

  pingTimer[0] = millis();

  for (int i = 1; i < SONAR_NUM; i++){
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  digitalWrite(RIGHT_SLEEP_PIN, HIGH);    // wake right motor
  digitalWrite(RIGHT_RESET_PIN, HIGH);    // un-reset right motor

  digitalWrite(LEFT_SLEEP_PIN, HIGH);     // wake left motor
  digitalWrite(LEFT_RESET_PIN, HIGH);     // un-reset left motor

  digitalWrite(RIGHT_ENABLE_PIN, LOW);    // enable right motor
  digitalWrite(LEFT_ENABLE_PIN, LOW);     // enable left motor

}

void loop(){

  for (int i = 0; i < SONAR_NUM; i++){

    if(millis() >= pingTimer[i]){

      pingTimer[i] = PING_INTERVAL * SONAR_NUM;

      distance[i] = sonar[i].ping() * 0.034 / 2;
      prevDistance[i] = distance[i];

      if(i == (SONAR_NUM - 1)){

        error = distance[0] - distance[1];
        errorSum = error + prevError;

        rightMotorSpeed = DEFAULT_MOTOR_SPEED - ((P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError)));
        leftMotorSpeed = DEFAULT_MOTOR_SPEED + ((P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError)));

        rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
        leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

        rightStepper.setMaxSpeed(rightMotorSpeed);
        leftStepper.setMaxSpeed(leftMotorSpeed);

        prevError = error;

      }

    }

  }
  
  leftStepper.run();
  rightStepper.run();

}

void moveForward(){
  positions[0] = rightStepper.currentPosition() + stepsPerUnitLength;
  positions[1] = leftStepper.currentPosition() - stepsPerUnitLength;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
}

void turnLeft(){
  positions[0] = rightStepper.currentPosition() + stepsPerUnitTurn;
  positions[1] = leftStepper.currentPosition() + stepsPerUnitTurn;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
}

void turnRight(){
  positions[0] = rightStepper.currentPosition() - stepsPerUnitTurn;
  positions[1] = leftStepper.currentPosition() - stepsPerUnitTurn;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
}

float getStepSize(int m2, int m1, int m0){

  int stepSize = (m2 * 4) + (m1 * 2) + m0;

  switch (stepSize) {
    case 0:
      stepSize = 1;
      break;
    case 1:
      stepSize = 1/2;
      break;
    case 2:
      stepSize = 1/4;
      break;
    case 3:
      stepSize = 1/8;
      break;
    case 4:
      stepSize = 1/16;
      break;
    default:
      stepSize = 1/32;
      break;
  }

  return stepSize;

}

int getUnitLengthSteps(float stepSize){
  return floor(BASE_FORWARD_STEPS / stepSize);
}

int getUnitTurnSteps(float stepSize){
  return floor(BASE_TURN_STEPS / stepSize);
}