#include <math.h>
#include <NewPing.h>
#include <AccelStepperJigs.h>
#include "pinDefs.h"
#include "robotParameters.h"
#include "mazeParameters.h"

unsigned long pingTimer[SONAR_NUM];
float distance[SONAR_NUM];

typedef struct {
  int dest;
  int cost;
} edgelist_t;

edgelist_t ** adjlist;
edgelist_t * shortestPath;

int pathCost[GRID_XY];
int pathPred[GRID_XY];
int visited[GRID_XY];
int mapped[GRID_XY];

int arrayXOffset;
int arrayYOffset;

int wallErrorInSteps;

int command;
int weight;

bool pidIsOn = false;

float error;
float errorSum;
float prevError;

float phi;
float centerDistance;
float lostSteps;

AccelStepper rightStepper(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
AccelStepper leftStepper(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);

NewPing sensor[SONAR_NUM] = {
  NewPing(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_SENSING_DISTANCE),
  NewPing(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_SENSING_DISTANCE),
  NewPing(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_SENSING_DISTANCE)
};

void setup() {

  Serial.begin(115200);

  initializePins();
  initializeTimerInterrupt();
  disableSteppers();
  setStepSize();

  stepSize = getStepSize(M2, M1, M0);
  stepsPerUnitLength = getUnitLengthSteps(stepSize);
  stepsPerUnitTurn = getUnitTurnSteps(stepSize);

  startXPos = startYPos = startNode = 0;
  currXPos = currYPos = currNode = 0;
  prevXPos = prevYPos = prevNode = 0;
  startHeading = currHeading = prevHeading = 1;

  arrayXOffset = arrayYOffset = 0;

  adjlist = (edgelist_t **) calloc(GRID_XY + 1, sizeof(edgelist_t *));
  checkNullPointer(adjlist);

  for (int i = 0; i < GRID_XY; i++) {

    adjlist[i] = (edgelist_t *) calloc(1, sizeof(edgelist_t));
    checkNullPointer(adjlist[i]);

    adjlist[i][0].dest = i;
    adjlist[i][0].cost = 0;

    mapped[i] = 0;

  }

  pingTimer[0] = millis();

  for (int i = 1; i < SONAR_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  resetErrors();
  pingAllSensors();
  enableSteppers();

}

void loop() {

  if (!mapped[0]) {

    for (int i = 0; i < SONAR_NUM; i++){
      if(millis() >= pingTimer[i]){
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;
        distance[i] = sensor[i].ping() * 0.034 / 2;

        if (pidIsOn)
          getError();
        else
          resetErrors();
        
        motorSpeedController();
        prevError = error;

        if (pidIsOn)
          getLostSteps();
        else
          lostSteps = 0;
      }
    }

    if (!rightStepper.isRunning() || !leftStepper.isRunning()) {
      resetErrors();
      resetMotorSpeeds();

      if (pidIsOn)
        lostSteps = ceil(lostSteps);
      else
        lostSteps = 0;

      pingAllSensors();

      if (distance[2])
        getWallError();
      else
        wallErrorInSteps = 0;
      
      rightStepper.move(lostSteps - wallErrorInSteps);
      leftStepper.move(-lostSteps + wallErrorInSteps);
      waitForMotors();

      moveDecision();
      
      localize();
      if (!mapped[currNode])
        updateAdjList();

    }

  } else {
  }

}

void localize() {
  prevNode = currNode;

  switch (command) {
    case 0:
      switch(currHeading){
        case 0:
          currXPos++;
          break;
        case 1:
          currYPos--;
          break;
        case 2:
          currXPos--
          break;
        case 3:
          currYPos++
          break;
      }
      break;
    case 1:
      currHeading++;
      break;
    case 2:
      currHeading--;
      break;
    case 3:
      currHeading += 2;
      break;
  }

  if(currHeading == -1)
    currHeading = 3;
  else if(currHeading == 4)
    currHeading = 0;
  else if(currHeading == 5)
    currHeading = 1;

  if (currXPos == -1)
    arrayXOffset++;
  if (currYPos == -1)
    arrayYOffset++;

  adjustArray();

  currXPos += arrayXOffset;
  currYPos += arrayYOffset;

  startXPos += arrayXOffset;
  startYPos += arrayYOffset;

  currNode = (GRID_Y * currXPos) + currYPos;
}

void adjustArray() {

}

void updateAdjList() {

}

void moveDecision() {
  if (pidIsOn)
    moveForward(1);
  else if (!distance[1])
    faceLeft(1);
  else if (!distance[2])
    moveForward(1);
  else if (!distance[0])
    faceRight(1);
  else if (!(distance[0] && distance[1] && distance[2]))
    moveForward(1);
  else
    faceLeft(2);
}

void waitForMotors() {
  while(rightStepper.isRunning() || leftStepper.isRunning()){
  }
}

void getWallError() {
  error = distance[2] - DISTANCE_TO_WALL;
  if (error < 0) {
    wallErrorInSteps = -floor(abs(error * 9.794));
  } else {
    wallErrorInSteps = ceil(abs(error * 9.794));
  }
}

void getLostSteps() {
  currentRightMotorSpeed = rightStepper.getCurrentSpeed();
  currentLeftMotorSpeed = leftStepper.getCurrentSpeed();

  phi = phi + (0.033 * 0.1625 * (abs(currentRightMotorSpeed) - abs(currentLeftMotorSpeed)));
  centerDistance = ((abs(currentRightMotorSpeed) + abs(currentLeftMotorSpeed)) / 2) * 0.033;
  lostSteps = lostSteps + (0.033 * centerDistance * (1 - cos(phi)));
}

void getError() {
    
  if (distance[0] && distance[1])
    error = distance[0] - distance[1];
  else if (distance[0] && !distance[1])
    error = distance[0] - DISTANCE_TO_WALL;
  else if (!distance[0] && distance[1])
    error = DISTANCE_TO_WALL - distance[1];

  errorSum = errorSum + error;

}

void resetErrors(){
  error = 0;
  errorSum = 0;
  prevError = 0;
}

void motorSpeedController() {
  rightMotorSpeed = DEFAULT_MOTOR_SPEED - ((P_GAIN * error) + (I_GAIN  * errorSum) + (D_GAIN * (error - prevError)));
  leftMotorSpeed = DEFAULT_MOTOR_SPEED + ((P_GAIN * error) + (I_GAIN  * errorSum) + (D_GAIN * (error - prevError)));

  rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  rightStepper.setMaxSpeed(rightMotorSpeed);
  leftStepper.setMaxSpeed(leftMotorSpeed);
}

void pingAllSensors() {
  distance[0] = sensor[0].ping() * 0.034 / 2;
  delay(33);
  distance[1] = sensor[1].ping() * 0.034 / 2;
  delay(33);
  distance[2] = sensor[2].ping() * 0.034 / 2;
  delay(33);
}

void resetMotorSpeeds() {
  rightMotorSpeed = DEFAULT_MOTOR_SPEED;
  leftMotorSpeed = DEFAULT_MOTOR_SPEED;

  rightStepper.setMaxSpeed(DEFAULT_MOTOR_SPEED);
  leftStepper.setMaxSpeed(DEFAULT_MOTOR_SPEED);

  rightStepper.setAcceleration(DEFAULT_ACCELERATION);
  leftStepper.setAcceleration(DEFAULT_ACCELERATION);
}

void moveForward(int counts) {
  resetMotorSpeeds();
  pidIsOn = true;
  rightStepper.move(counts * stepsPerUnitLength);
  leftStepper.move(-counts * stepsPerUnitLength);
  command = 0;
  weight = 1;
}

void faceLeft(int counts) {
  resetMotorSpeeds();
  pidIsOn = false;
  rightStepper.move(counts * stepsPerUnitTurn);
  leftStepper.move(counts * stepsPerUnitTurn);
  command = (2 * counts) - 1;
  weight = counts + 1;
}

void faceRight(int counts) {
  resetMotorSpeeds();
  pidIsOn = false;
  rightStepper.move(-counts * stepsPerUnitTurn);
  leftStepper.move(-counts * stepsPerUnitTurn);
  command = 2;
  weight = 2;
}

void initializePins() {
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
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
}

void disableSteppers() {
  digitalWrite(RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_ENABLE_PIN, HIGH);

  digitalWrite(RIGHT_RESET_PIN, LOW);
  digitalWrite(RIGHT_SLEEP_PIN, LOW);

  digitalWrite(LEFT_RESET_PIN, LOW);
  digitalWrite(LEFT_SLEEP_PIN, LOW);
}

void enableSteppers() {
  digitalWrite(RIGHT_SLEEP_PIN, HIGH);
  digitalWrite(RIGHT_RESET_PIN, HIGH);

  digitalWrite(LEFT_SLEEP_PIN, HIGH);
  digitalWrite(LEFT_RESET_PIN, HIGH);

  digitalWrite(RIGHT_ENABLE_PIN, LOW);
  digitalWrite(LEFT_ENABLE_PIN, LOW);
}

void setStepSize(){
  digitalWrite(RIGHT_M2_PIN, M2);
  digitalWrite(RIGHT_M1_PIN, M1);
  digitalWrite(RIGHT_M0_PIN, M0);

  digitalWrite(LEFT_M2_PIN, M2);
  digitalWrite(LEFT_M1_PIN, M1);
  digitalWrite(LEFT_M0_PIN, M0);
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

void initializeTimerInterrupt() {
  TIMSK2 = (TIMSK2 & B11111110) | 0x01;
  TCCR2B = (TCCR2B & B11111000) | 0x01;
}

void safeFree(void *ptr) {
  if (ptr != NULL)
    free(ptr);
}

void checkNullPointer(void *ptr) {
  if (ptr == NULL)
    exit(1);
}

ISR(TIMER2_OVF_vect){
  leftStepper.run();
  rightStepper.run();
}