#include <math.h>
#include <NewPing.h>
#include <AccelStepperJigs.h>
#include "pinDefs.h"
#include "robotParameters.h"
#include "mazeParameters.h"
#include "hardCodedMaze.h"

const int INF = 9999;

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
int phiErrorInSteps;

int command;
int weight;

int startNodePathCount;
int shortestPathNodeCount;

bool pidIsOn = false;
bool isMapping = false;
bool mazeIsMapped = false;

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
  enableSteppers();
  //startSequence();
  resetMotorSpeeds();

}
int count = 0;
void loop() {

  for (int i = 0; i < SONAR_NUM; i++){
    if(millis() >= pingTimer[i]){
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      distance[i] = sensor[i].ping() * 0.034 / 2;

      if (pidIsOn)
        getError();
      else
        resetErrors();

      if (pidIsOn) 
        motorSpeedController();
      else
        resetMotorSpeeds();

      prevError = error;

      if (pidIsOn)
        getLostSteps();
      else
        lostSteps = 0;

      Serial.print("\nRight: ");
      Serial.print(distance[0]);
      Serial.print("\tLeft: ");
      Serial.print(distance[1]);
      Serial.print("\tFront: ");
      Serial.print(distance[2]);
    }

  }

  //if (!(mapped[startNode] && (currNode == startNode) && (currHeading == startHeading)) && !(currNode == startNode && count >= 13)) {
  if (!(currNode == startNode && count >= 13)) {

    if (!rightStepper.distanceToGo() || !leftStepper.distanceToGo()) {
      rightStepper.stop();
      leftStepper.stop();
      resetErrors();
      resetMotorSpeeds();

      if (pidIsOn){
        lostSteps = ceil(lostSteps);
        phiErrorInSteps = ceil(phi) * 3.14 / 100;
      }
      else {
        lostSteps = 0;
        phiErrorInSteps = 0;
      }

      pingAllSensors();

      if (distance[2])
        getWallError();
      else
        wallErrorInSteps = 0;

      rightStepper.move(lostSteps + wallErrorInSteps);
      leftStepper.move(-lostSteps - wallErrorInSteps);
      waitForMotors();

      phi = 0;
      lostSteps = 0;
      wallErrorInSteps = 0;
      phiErrorInSteps = 0;

      if(prevNode == startNode && !command){
        startNodePathCount--;
        //mapped[startNode] = 1;
      }

      resetErrors();
      resetMotorSpeeds();
      rightStepper.stop();
      leftStepper.stop();
      moveDecision();
      
      localize();

      Serial.println(startNodePathCount);

      if(!command)
        isMapping = true;

      if (!mapped[currNode] && isMapping && !command) {
        updateAdjList();
        count++;
      }
      
      arrayXOffset = 0;
      arrayYOffset = 0;

    }

  } else {
    if (!mazeIsMapped) {
      waitForMotors();

      Serial.println("Maze mapped.");

      mapped[startNode] = 1;
      mazeIsMapped = true;

      matchHeading(startHeading);

      printMappedList();
      printAdjList();

      dijkstras(currNode, TARGET_NODE);
      shortestPathTraversal();
    }
  }

}

void dijkstras(int src, int dest) {
  int currentNode = src;
  int prevNode = src;
  int leastCost = INF;

  Serial.println("Initializing arrays...");
  for (int i = 0; i < GRID_XY; i++) {
    visited[i] = 0;
    pathCost[i] = INF;
    pathPred[i] = INF;
  }
  Serial.println("Done.");

  Serial.println("Setting starting parameters...");
  visited[src] = 1;
  pathCost[src] = 0;
  pathPred[src] = src;
  Serial.println("Done.");

  Serial.println("Traversing adjacency list...");
  while (currentNode != dest) {

    Serial.println("Getting node with least cost...");
    for (int i = 0; i < GRID_XY; i++) {
      if (!visited[i])
        continue;
      
      for (int j = 1; j <= adjlist[i][0].cost; j++) {
        if (visited[adjlist[i][j].dest])
          continue;
        
        if (adjlist[i][j].cost < leastCost) {
          currentNode = adjlist[i][j].dest;
          leastCost = adjlist[i][j].cost;
          pathCost[currentNode] = leastCost;
          pathPred[adjlist[i][j].dest] = i;

          Serial.println(currentNode);
          Serial.println(leastCost);
        }
      }
    }
    Serial.println("Done.");

    Serial.println("Setting current node as visited...");
    visited[currentNode] = 1;
    leastCost = INF;
    Serial.println("Done.");

  }
  Serial.println("Done.");

  for (int i = dest;;) {
    shortestPathNodeCount++;
    //Serial.println(i);
    if (i == src)
      break;
    i = pathPred[i];
  }
}

void shortestPathTraversal(){

  int deltaNode = 0;
  int deltaHeading = 0;
  int absHeading = 0;
  int forwardCounter = 0;

  int shortestPath[shortestPathNodeCount];
  int nodePath = TARGET_NODE;

  for (int i = shortestPathNodeCount - 1; i >= 0; i--){
    shortestPath[i] = nodePath;
    nodePath = pathPred[nodePath];
  } 

  for (int i = 1; i < shortestPathNodeCount; i++) {
    deltaNode = shortestPath[i] - currNode;
    switch (deltaNode) {
      case 4:
        absHeading = 0;
        break;
      case -1:
        absHeading = 1;
        break;
      case -4:
        absHeading = 2;
        break;
      case 1:
        absHeading = 3;
        break;
    }
    matchHeading(absHeading);
    moveForward(1);
    currNode = shortestPath[i];
    waitForMotors();
      pingAllSensors();

      if (distance[2])
        getWallError();
      else
        wallErrorInSteps = 0;

      rightStepper.move(wallErrorInSteps);
      leftStepper.move(-wallErrorInSteps);
      waitForMotors();
      wallErrorInSteps = 0;
  }

}

void localize() {

  if(isMapping && !command){
    mapped[currNode] = 1;
  }

  prevXPos = currXPos;
  prevYPos = currYPos;
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
          currXPos--;
          break;
        case 3:
          currYPos++;
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

  adjustMappedList();

  currXPos += arrayXOffset;
  currYPos += arrayYOffset;

  prevXPos += arrayXOffset;
  prevYPos += arrayYOffset;

  startXPos += arrayXOffset;
  startYPos += arrayYOffset;

  currNode = (GRID_Y * currXPos) + currYPos;
  prevNode = (GRID_Y * prevXPos) + prevYPos;
  startNode = (GRID_Y * startXPos) + startYPos;

  /*
  Serial.print("\ncurrNode: ");
  Serial.print(currNode);
  Serial.print("\tprevNode: ");
  Serial.print(prevNode);
  Serial.print("\tstartNode: ");
  Serial.print(startNode);
  Serial.print("\tcurrHeading: ");
  Serial.print(currHeading);
  Serial.print("\tstartHeading: ");
  Serial.print(startHeading);
  Serial.print("\tmapped[startNode]: ");
  Serial.print(mapped[startNode]);
  Serial.print("\tarrayXOffset: ");
  Serial.print(arrayXOffset);
  Serial.print("\tarrayYOffset: ");
  Serial.print(arrayYOffset);
  Serial.println("");
  */

  adjustAdjList();

}

void adjustMappedList() {
  if (arrayYOffset) {
    for (int i = (GRID_XY - 1); i > 0; i--) {
      mapped[i] = mapped[i - 1];
    }
    mapped[0] = 0;
  } else if (arrayXOffset) {
    for (int i = (GRID_XY - 1); i > (GRID_Y - 1); i--) {
      mapped[i] = mapped[i - GRID_Y];
    }
    mapped[0] = mapped[1] = mapped[2] = mapped[3] = 0;
  }
}

void adjustAdjList() {
  for (int i = 0; i < GRID_XY; i++)
    for(int j = 1; j <= adjlist[i][0].cost; j++)
      adjlist[i][j].dest += (GRID_Y * arrayXOffset) + arrayYOffset;

  if (arrayYOffset) {
    for (int i = (GRID_XY - 1); i > 0; i--) {
      adjlist[i] = (edgelist_t *) realloc(adjlist[i], (adjlist[i - 1][0].cost + 2) * sizeof(edgelist_t));
      adjlist[i] = adjlist[i - 1];
      adjlist[i - 1] = NULL;
    }
    adjlist[0] = (edgelist_t *) realloc(adjlist[0], sizeof(edgelist_t));
    adjlist[0][0].dest = 0;
    adjlist[0][0].cost = 0;
  } else if (arrayXOffset) {
    for (int i = (GRID_XY - 1); i > (GRID_Y - 1); i--) {
      adjlist[i] = (edgelist_t *) realloc(adjlist[i], (adjlist[i - GRID_Y][0].cost + 2) * sizeof(edgelist_t));
      adjlist[i] = adjlist[i - GRID_Y];
      adjlist[i - GRID_Y] = NULL;
    }
    adjlist[0] = (edgelist_t *) realloc(adjlist[0], sizeof(edgelist_t));
    adjlist[1] = (edgelist_t *) realloc(adjlist[1], sizeof(edgelist_t));
    adjlist[2] = (edgelist_t *) realloc(adjlist[2], sizeof(edgelist_t));
    adjlist[3] = (edgelist_t *) realloc(adjlist[3], sizeof(edgelist_t));
    adjlist[0][0].dest = adjlist[1][0].dest = adjlist[2][0].dest = adjlist[3][0].dest = 0;
    adjlist[0][0].cost = adjlist[1][0].cost = adjlist[2][0].cost = adjlist[3][0].cost = 0;
  }
}

void updateAdjList() {
  int willUpdate = 1;

  for (int i = 1; i <= adjlist[currNode][0].cost; i++){
    if (adjlist[currNode][i].dest == prevNode)
      willUpdate = 0;
  }

  for (int i = 1; i <= adjlist[prevNode][0].cost; i++){
    if (adjlist[prevNode][i].dest == currNode)
      willUpdate = 0;
  }

  if (willUpdate) {
  adjlist[currNode] = (edgelist_t *) realloc(adjlist[currNode], (adjlist[currNode][0].cost + 2) * sizeof(edgelist_t));
  adjlist[prevNode] = (edgelist_t *) realloc(adjlist[prevNode], (adjlist[prevNode][0].cost + 2) * sizeof(edgelist_t));

  adjlist[currNode][adjlist[currNode][0].cost + 1].dest = prevNode;
  adjlist[currNode][adjlist[currNode][0].cost + 1].cost = weight;
  adjlist[prevNode][adjlist[prevNode][0].cost + 1].dest = currNode;
  adjlist[prevNode][adjlist[prevNode][0].cost + 1].cost = weight;

  adjlist[currNode][0].cost++;
  adjlist[prevNode][0].cost++;
  }

  //printAdjList();

}

void printAdjList() {
  for (int i = 0; i < GRID_XY; i++){
    Serial.print("\nNode ");
    Serial.print(i);
    Serial.print(": ");
    for(int j = 1; j <= adjlist[i][0].cost; j++){
      Serial.print("(");
      Serial.print(adjlist[i][j].dest);
      Serial.print(", ");
      Serial.print(adjlist[i][j].cost);
      Serial.print(")\t");
    }
  }
}

void printMappedList() {
  for (int i = 0; i < GRID_XY; i++)
    Serial.print(mapped[i]);
}

void moveDecision() {
  resetMotorSpeeds();
  if (!pidIsOn && command) 
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

void matchHeading(int targetHeading) {
  int deltaHeading = currHeading - targetHeading;
  switch(deltaHeading){
      case -1:
      case 3:
        faceLeft(1);
        break;
      case 1:
      case -3:
        faceRight(1);
        break;
      case 2:
      case -2:
        faceLeft(2);
        break;
    }
  currHeading = targetHeading;
  waitForMotors();
}

void waitForMotors() {
  int imblue = 1;
  while(rightStepper.distanceToGo() || leftStepper.distanceToGo()){
    if (imblue){
      imblue = 0;
      Serial.println("I'm Blue - da ba dee da ba dye, da ba dee da ba dye");
    }
    Serial.println("Da ba dee da ba dye, da ba dee da ba dye");
  }
  delay(100);
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

  phi = phi + (PING_INTERVAL / 1000 * 0.1625 * (abs(currentRightMotorSpeed) - abs(currentLeftMotorSpeed)));
  centerDistance = ((abs(currentRightMotorSpeed) + abs(currentLeftMotorSpeed)) / 2) * PING_INTERVAL / 1000;
  lostSteps = lostSteps + (PING_INTERVAL / 1000 * centerDistance * (1 - cos(phi)));
}

void getError() {
    
  if (distance[0] && distance[1])
    error = distance[0] - distance[1];
  /*
  else if (distance[0] && !distance[1])
    error = distance[0] - DISTANCE_TO_WALL;
  else if (!distance[0] && distance[1])
    error = DISTANCE_TO_WALL - distance[1];
  */
  else {
    error = 0;
    errorSum = 0;
  }

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
  waitForMotors();
}

void faceRight(int counts) {
  resetMotorSpeeds();
  pidIsOn = false;
  rightStepper.move(-counts * stepsPerUnitTurn);
  leftStepper.move(-counts * stepsPerUnitTurn);
  command = 2;
  weight = 2;
  waitForMotors();
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

void startSequence() {
  pingAllSensors();
  if(!distance[0])
    startNodePathCount++;
  if(!distance[1])
    startNodePathCount++;
  if(!distance[2])
    startNodePathCount++;
  faceLeft(2);
  waitForMotors();
  pingAllSensors();
  if(!distance[2])
    startNodePathCount++;
  if(startNodePathCount == 0);
    startNodePathCount = 4;
  faceLeft(2);
  waitForMotors();
  //if (startNodePathCount != 1)
    //startNodePathCount++;
  command = 0;
  pidIsOn = false;
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