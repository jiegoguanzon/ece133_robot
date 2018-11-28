#include <math.h>
#include <NewPing.h>
#include "pinDefs.h"

const int SENSOR_COUNT = 3;
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

NewPing sonar[SONAR_COUNT] = {
  NewPing(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_SENSING_DISTANCE);
  NewPing(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_SENSING_DISTANCE);
  NewPing(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_SENSING_DISTANCE);
}


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

  rightMotorSpeed = DEFAULT_MOTOR_SPEED;
  leftMotorSpeed = DEFAULT_MOTOR_SPEED;

  prevRightMicros = 0;
  prevLeftMicros = 0;
  prevPulseMicros = 0;
  prevDistance = 0;
  prevError = 0;
  errorSum = 0;

  distanceToWall = 4;

  distancePerStep = stepSize * (WHEEL_DIAMETER * M_PI) / STEPS_PER_ROTATION;
  stepsPerUnitLength = round(UNIT_LENGTH / distancePerStep);
  stepsPerUnitTurn = round((ROBOT_DIAMETER * M_PI / 4) / distancePerStep);

  rightMotorStepInterval = getStepInterval(rightMotorSpeed);
  leftMotorStepInterval = getStepInterval(leftMotorSpeed);

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

}

void loop(){
  
  currPulseMicros = micros();

  if((currPulseMicros - prevPulseMicros) >= 250000){

    prevPulseMicros = currPulseMicros;

    distance = sonar.ping() * 0.034 / 2;

    error = distanceToWall - distance;
    errorSum = errorSum + error;

    rightMotorSpeed = DEFAULT_MOTOR_SPEED + (P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError));
    leftMotorSpeed = DEFAULT_MOTOR_SPEED - (P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError));

    prevError = error;

    rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

    rightMotorStepInterval = getStepInterval(rightMotorSpeed);
    leftMotorStepInterval = getStepInterval(leftMotorSpeed);

  }

  currRightMicros = micros();
  currLeftMicros = micros();

  stepMotor(0, RIGHT_STEP_PIN, rightMotorStepInterval, currRightMicros, prevRightMicros);
  stepMotor(1, LEFT_STEP_PIN, leftMotorStepInterval, currLeftMicros, prevLeftMicros);

}

void stepMotor(int motor, const int stepPin, unsigned long stepInterval, unsigned long currMicros, unsigned long prevMicros){
  if ((currMicros - prevMicros) >= stepInterval){
    if(motor == 0)
      prevRightMicros = currRightMicros;
    else if (motor == 1)
      prevLeftMicros = currLeftMicros;
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
  }
}

unsigned long getStepInterval(int motorSpeed){
  return (60.0000 / (motorSpeed * STEPS_PER_ROTATION)) * 1E6;
}