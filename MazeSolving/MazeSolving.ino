#include <math.h>
#include <NewPing.h>
#include <AccelStepperJigs.h>
#include "pinDefs.h"
#include "robotConstants.h"
#include "mazeConstants.h"

int m2 = 0;
int m1 = 0;
int m0 = 0;

int rightMotorSpeed;
int leftMotorSpeed;

int currentRightMotorSpeed;
int currentLeftMotorSpeed;

int stepsPerUnitLength;
int stepsPerUnitTurn;

unsigned long pingTimer[SONAR_NUM];

float distance[SONAR_NUM];

float error;
float prevError;
float errorSum;

float stepSize;

long positions[2];

int command = 4;
float phi = 0;
float dy = 0;
float dc = 0;

typedef struct {
  int dest;
  int cost;
} edgelist_t;

edgelist_t** adjlist;

void safeFree(void *ptr){
  if(ptr != NULL)
    free(ptr);
}

void checkNullPointer(void *ptr){
  if(ptr == NULL)
    exit(1);
}

int currXPos = X_START_POS;
int currYPos = Y_START_POS;
int currHeading = 1;  // currHeading * 90 deg to get degrees from horizontal

int prevXPos = 0;
int prevYPos = 0;

int visited[GRID_XY];

int currNode = 0;
int prevNode = 0;

int weight = 0;

int userInput = 0;

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

  TIMSK2 = (TIMSK2 & B11111110) | 0x01;
  TCCR2B = (TCCR2B & B11111000) | 0x01;

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

  pingSensors();

  adjlist = (edgelist_t **) calloc((GRID_XY) + 1, sizeof(edgelist_t *));
  checkNullPointer(adjlist);

  for(int i = 0; i < GRID_XY; i++){
    adjlist[i] = (edgelist_t *) calloc(1, sizeof(edgelist_t));
    checkNullPointer(adjlist[i]);
    adjlist[i][0].cost = 0;
    visited[i] = 0;
  }


  digitalWrite(RIGHT_SLEEP_PIN, HIGH);    // wake right motor
  digitalWrite(RIGHT_RESET_PIN, HIGH);    // un-reset right motor

  digitalWrite(LEFT_SLEEP_PIN, HIGH);     // wake left motor
  digitalWrite(LEFT_RESET_PIN, HIGH);     // un-reset left motor

  digitalWrite(RIGHT_ENABLE_PIN, LOW);    // enable right motor
  digitalWrite(LEFT_ENABLE_PIN, LOW);     // enable left motor

}

void loop(){

  scapegoat:

  if(!visited[0]){

    for (int i = 0; i < SONAR_NUM; i++){

      if(millis() >= pingTimer[i]){

        pingTimer[i] = PING_INTERVAL * SONAR_NUM;

        distance[i] = sonar[i].ping() * 0.034 / 2;

        if(i == (SONAR_NUM - 1) && command == 0 && distance[0] && distance[1]){

          error = distance[0] - distance[1];
          errorSum = errorSum + error;

          rightMotorSpeed = DEFAULT_MOTOR_SPEED - ((P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError)));
          leftMotorSpeed = DEFAULT_MOTOR_SPEED + ((P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError)));

          rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
          leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

          rightStepper.setMaxSpeed(rightMotorSpeed);
          leftStepper.setMaxSpeed(leftMotorSpeed);

          prevError = error;

        }

        currentRightMotorSpeed = rightStepper.getCurrentSpeed();
        currentLeftMotorSpeed = leftStepper.getCurrentSpeed();

        phi = phi + (0.033 * 0.1625 * (abs(currentRightMotorSpeed) - abs(currentLeftMotorSpeed)));
        dc = ((abs(currentRightMotorSpeed) + abs(currentLeftMotorSpeed)) / 2) * 0.023;
        dy = dy + (0.033 * dc * (1 - cos(phi)));

      }
    }

    if (leftStepper.distanceToGo() == 0 && rightStepper.distanceToGo() == 0){

      prevNode = currNode;

      localize();

      currNode = (4 * currXPos) + currYPos;

      if(!visited[currNode])
        updateAdjList();

      error = 0;
      prevError = 0;
      errorSum = 0;

      rightMotorSpeed = DEFAULT_MOTOR_SPEED;
      leftMotorSpeed = DEFAULT_MOTOR_SPEED;

      dy = ceil(dy);

      positions[0] = rightStepper.currentPosition() + dy;
      positions[1] = leftStepper.currentPosition() - dy;
      rightStepper.moveTo(positions[0]);
      leftStepper.moveTo(positions[1]);

      while(rightStepper.distanceToGo() != 0 || leftStepper.distanceToGo() != 0)
        Serial.println("Compensating...");

      dy = 0;

      pingSensors();
      //printDistances();

      if (command)
        moveForward();
      else if (distance[1] == 0)
        turnLeft();
      else if (distance[2] == 0)
        moveForward();
      else if (distance[0] == 0 && distance[1])
        turnRight();
      else if (!(distance[0] && distance[1] && distance[2]))
        moveForward();
      else
        aboutFace();

    }

  } else {

    if(Serial.available() > 0){

      userInput = Serial.read();

      if(userInput == 189){
        printAdjList();
        goto scapegoat;
      }

      if((userInput != currNode) &&  (userInput >= 0 && userInput <= GRID_XY)){

      }

    }
  

  }


}

void printAdjList(){
  for(int i = 0; i < GRID_XY; i++){
    Serial.println("Node " + i);
    for(int j = 0; j <= adjlist[i][0].cost; j++){
      Serial.print("(" + adjlist[i][j].dest);
      Serial.print(", " + adjlist[i][j].cost);
      Serial.print(")\t");
    }
  }
}

void updateAdjList(){
  adjlist[currNode] = (edgelist_t *) realloc(adjlist[currNode], (adjlist[currNode][0].cost + 2) * sizeof(edgelist_t));
  adjlist[prevNode] = (edgelist_t *) realloc(adjlist[prevNode], (adjlist[prevNode][0].cost + 2) * sizeof(edgelist_t));

  adjlist[currNode][adjlist[currNode][0].cost + 1].dest = prevNode;
  adjlist[currNode][adjlist[currNode][0].cost + 1].cost = weight;
  adjlist[prevNode][adjlist[prevNode][0].cost + 1].dest = currNode;
  adjlist[prevNode][adjlist[prevNode][0].cost + 1].cost = weight;

  visited[currNode] = 1;
}

ISR(TIMER2_OVF_vect){
  leftStepper.run();
  rightStepper.run();
}

void pingSensors(){
  distance[0] = sonar[0].ping() * 0.034 / 2;
  delay(33);
  distance[1] = sonar[1].ping() * 0.034 / 2;
  delay(33);
  distance[2] = sonar[2].ping() * 0.034 / 2;
  delay(33);
}

void printDistances(){
  Serial.print(distance[0]);
  Serial.print("\t");
  Serial.print(distance[1]);
  Serial.print("\t");
  Serial.print(distance[2]);
  Serial.println("");
}

void moveForward(){
  positions[0] = rightStepper.currentPosition() + stepsPerUnitLength;
  positions[1] = leftStepper.currentPosition() - stepsPerUnitLength;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 0;
  weight = 1;
}

void turnLeft(){
  positions[0] = rightStepper.currentPosition() + stepsPerUnitTurn;
  positions[1] = leftStepper.currentPosition() + stepsPerUnitTurn;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 1;
  weight = 2;
}

void turnRight(){
  positions[0] = rightStepper.currentPosition() - stepsPerUnitTurn;
  positions[1] = leftStepper.currentPosition() - stepsPerUnitTurn;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 2;
  weight = 2;
}

void aboutFace(){
  positions[0] = rightStepper.currentPosition() + 2 * stepsPerUnitTurn + 4;
  positions[1] = leftStepper.currentPosition() + 2 * stepsPerUnitTurn + 4;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 3;
  weight = 2;
}

void localize(){

  prevXPos = currXPos;
  prevYPos = currYPos;

  switch (command) {
    case 0: // moveForward
      switch(currHeading){
        case 0:
          currXPos--;
          break;
        case 1:
          currYPos++;
          break;
        case 2:
          currXPos++;
          break;
        case 3:
          currYPos--;
          break;
      }
      break;
    case 1: // turnLeft
      currHeading++;
      break;
    case 2: // turnRight
      currHeading--;
      break;
    case 3: // aboutFace
      currHeading += 2;
      break;
  }

  if(currHeading == -1)
    currHeading = 3;
  else if(currHeading == 4)
    currHeading = 0;
  else if(currHeading == 5)
    currHeading = 1;

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