#include <math.h>
#include <NewPing.h>
#include <AccelStepperJigs.h>
#include "pinDefs.h"
#include "robotConstants.h"
#include "mazeConstants.h"

const int INF = 9999;

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
edgelist_t* shortestPath;

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
int mapped[GRID_XY];
int pathCost[GRID_XY];
int pathPred[GRID_XY];

int currNode = 0;
int prevNode = 0;

int weight = 0;

String userInput = "";

int input = 0;

int shortestPathNodeCount;

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
    mapped[i] = 0;
  }

  digitalWrite(RIGHT_SLEEP_PIN, HIGH);    // wake right motor
  digitalWrite(RIGHT_RESET_PIN, HIGH);    // un-reset right motor

  digitalWrite(LEFT_SLEEP_PIN, HIGH);     // wake left motor
  digitalWrite(LEFT_RESET_PIN, HIGH);     // un-reset left motor

  digitalWrite(RIGHT_ENABLE_PIN, LOW);    // enable right motor
  digitalWrite(LEFT_ENABLE_PIN, LOW);     // enable left motor

  //moveDecision();

}

void loop(){

  if(!mapped[0]){

    for (int i = 0; i < SONAR_NUM; i++){

      if(millis() >= pingTimer[i]){

        pingTimer[i] = PING_INTERVAL * SONAR_NUM;

        distance[i] = sonar[i].ping() * 0.034 / 2;

        if(i == (SONAR_NUM - 1) && command == 0 && distance[0] && distance[1]){
        //if(command == 0 && distance[0] && distance[1]){

          error = distance[0] - distance[1];
          errorSum = errorSum + error;

          rightMotorSpeed = DEFAULT_MOTOR_SPEED - ((P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError)));
          leftMotorSpeed = DEFAULT_MOTOR_SPEED + ((P_GAIN * error) + (I_GAIN * errorSum) + (D_GAIN * (error - prevError)));

          rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
          leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

          rightStepper.setMaxSpeed(rightMotorSpeed);
          leftStepper.setMaxSpeed(leftMotorSpeed);

          prevError = error;

          currentRightMotorSpeed = rightStepper.getCurrentSpeed();
          currentLeftMotorSpeed = leftStepper.getCurrentSpeed();

          phi = phi + (0.033 * 0.1625 * (abs(currentRightMotorSpeed) - abs(currentLeftMotorSpeed)));
          dc = ((abs(currentRightMotorSpeed) + abs(currentLeftMotorSpeed)) / 2) * 0.033;
          dy = dy + (0.035 * dc * (1 - cos(phi)));

        }

      }
    }

    if (leftStepper.distanceToGo() == 0 && rightStepper.distanceToGo() == 0){

      error = 0;
      prevError = 0;
      errorSum = 0;

      rightMotorSpeed = DEFAULT_MOTOR_SPEED;
      leftMotorSpeed = DEFAULT_MOTOR_SPEED;

      rightStepper.setMaxSpeed(DEFAULT_MOTOR_SPEED);
      leftStepper.setMaxSpeed(DEFAULT_MOTOR_SPEED);

      if(command == 0){
        dy = ceil(dy);

        positions[0] = rightStepper.currentPosition() + (int) dy;
        positions[1] = leftStepper.currentPosition() - (int) dy;
        rightStepper.moveTo(positions[0]);
        leftStepper.moveTo(positions[1]);

        while(rightStepper.distanceToGo() != 0 || leftStepper.distanceToGo() != 0){
          Serial.println("Compensating...");
        }
      }

      dy = 0;

      pingSensors();

      if(distance[2])
        wallCompensate();

      //printDistances();

      moveDecision();

      prevNode = currNode;

      localize();

      if(!mapped[currNode])
        updateAdjList();

      Serial.println("Done updating adjacency list.");

    }

  } else {

    Serial.println("Maze mapped.");

    while(!Serial.available()){

    }

    userInput = Serial.readString();

    input = userInput.toInt();

    Serial.println(input);

    if(input == 189){
      printAdjList();
    } else if (input == 188){
      printMappedList();
    } else if ((input != currNode) &&  (input >= 0 && input <= GRID_XY)){
      dijsktra(currNode, input);
      printShortestPath();
      shortestPathTraversal();
    }

  }

}

void wallCompensate(){
  float error = distance[2] - 4;
  int errorInSteps;
  if(error < 0){
    errorInSteps = floor(abs(error * 9.79));
    positions[0] = rightStepper.currentPosition() - errorInSteps;
    positions[1] = leftStepper.currentPosition() + errorInSteps;
    rightStepper.moveTo(positions[0]);
    leftStepper.moveTo(positions[1]);
  }
  else{
    errorInSteps = ceil(abs(error * 9.79));
    positions[0] = rightStepper.currentPosition() + errorInSteps;
    positions[1] = leftStepper.currentPosition() - errorInSteps;
    rightStepper.moveTo(positions[0]);
    leftStepper.moveTo(positions[1]);
  }
  while(rightStepper.distanceToGo() != 0 || leftStepper.distanceToGo() != 0){
    Serial.println("Compensating...");
  }
}

void shortestPathTraversal(){

  int deltaNode = 0;
  int deltaHeading = 0;
  int absHeading = 0;

  for(int i = 0; i < shortestPathNodeCount; i++){
    deltaNode = currNode - shortestPath[i].dest;
    switch(deltaNode){
      case -4:
        absHeading = 2;
        break;
      case -1:
        absHeading = 1;
        break;
      case 1:
        absHeading = -1;
        break;
      case 4:
        absHeading = 0;
        break;
    }
    deltaHeading = currHeading - absHeading;
    switch(deltaHeading){
      case -1:
      case 3:
        turnLeft();
        break;
      case 1:
      case -3:
        turnRight();
        break;
      case 2:
      case -2:
        aboutFace();
        break;
    }
    currHeading = absHeading;
    while(rightStepper.distanceToGo() || leftStepper.distanceToGo()){

    }
    moveForward();
    localize();
    while(rightStepper.distanceToGo() || leftStepper.distanceToGo()){

    }
  }

}

void printMappedList(){
  for(int i = 0; i < GRID_XY; i++)
    Serial.print(mapped[i]);
  Serial.println("");
}

void moveDecision(){
  if (command != 0)
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

void dijsktra(int src, int dest){

  Serial.println("About to do Dijsktra's Algorithm...");

  int currentNode = src;
  shortestPathNodeCount = 1;

  Serial.println("Initializing arrays...");

  for(int i = 0; i < GRID_XY; i++){   // initialize visited, path cost, and path predecessor lists
    visited[i] = 0;
    pathCost[i] = INF;                  //  path weight
    pathPred[i] = INF;                    //  predecessor
  }

  Serial.println("Setting source node parameters...");

  visited[src] = 1;                   // set starting node as visited
  pathCost[src] = 0;                    // set starting node cost to zero
  pathPred[src] = src;                    // set starting node predecessor to itself

  Serial.println("Traversing adjacency list...");

  while(currentNode != dest){

    Serial.println("Getting node with least cost...");

    for(int i = 0; i < GRID_XY; i++){
      if((pathCost[i] < pathCost[currentNode]) && (!visited[i]) && mapped[i])
        currentNode = i;
    }

    Serial.println("Setting current node as visited...");

    visited[currentNode] = 1;

    Serial.println("Calculating new costs...");
  
    for(int j = 1; j <= adjlist[currentNode][0].cost; j++){
      if(!visited[adjlist[currentNode][j].dest] && mapped[adjlist[currentNode][j].dest]){
        if(pathCost[adjlist[currentNode][j].dest] > (pathCost[currentNode] + adjlist[currentNode][j].cost)){
          pathCost[adjlist[currentNode][j].dest] = pathCost[currentNode] + adjlist[currentNode][j].cost;
          pathPred[adjlist[currentNode][j].dest] = currentNode;
        }
      }
    }

  }

  Serial.println("Getting shortest path node count...");

  for(; 1; shortestPathNodeCount++){
    if(currentNode == src)
      break;
    currentNode = pathPred[currentNode];
  }

  shortestPath = (edgelist_t *) calloc(1, shortestPathNodeCount * sizeof(edgelist_t));

  Serial.println("Getting shortest path list...");

  for(int i = shortestPathNodeCount, j = dest; i >= 0; i--){
    shortestPath[i].dest = j;
    j = pathPred[j];
  }

}

void printShortestPath(){
  Serial.println("Printing shortest path...");
  for(int i = 1; i <= shortestPathNodeCount; i++){
    Serial.print(shortestPath[i].dest + "\t");
  }
}

void printAdjList(){
  Serial.println("Printing adjacency list...");
  for(int i = 0; i < GRID_XY; i++){
    Serial.print("\nNode ");
    Serial.print(i);
    Serial.print(":\n");
    for(int j = 1; j <= adjlist[i][0].cost; j++){
      Serial.print("(");
      Serial.print(adjlist[i][j].dest);
      Serial.print(", ");
      Serial.print(adjlist[i][j].cost);
      Serial.print(")\t");
    }
  }
}

void updateAdjList(){

  Serial.println("Updating adjacency list...");

  adjlist[currNode] = (edgelist_t *) realloc(adjlist[currNode], (adjlist[currNode][0].cost + 2) * sizeof(edgelist_t));
  adjlist[prevNode] = (edgelist_t *) realloc(adjlist[prevNode], (adjlist[prevNode][0].cost + 2) * sizeof(edgelist_t));

  Serial.println("Done reallocating.");

  adjlist[currNode][adjlist[currNode][0].cost + 1].dest = prevNode;
  adjlist[currNode][adjlist[currNode][0].cost + 1].cost = weight;
  adjlist[prevNode][adjlist[prevNode][0].cost + 1].dest = currNode;
  adjlist[prevNode][adjlist[prevNode][0].cost + 1].cost = weight;

  adjlist[currNode][0].cost++;
  adjlist[prevNode][0].cost++;

  Serial.println(currNode);

  mapped[currNode] = 1;
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
  Serial.println("Moving forward...");
  positions[0] = rightStepper.currentPosition() + stepsPerUnitLength;
  positions[1] = leftStepper.currentPosition() - stepsPerUnitLength;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 0;
  weight = 1;
}

void turnLeft(){
  Serial.println("Turning left...");
  positions[0] = rightStepper.currentPosition() + stepsPerUnitTurn;
  positions[1] = leftStepper.currentPosition() + stepsPerUnitTurn;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 1;
  weight = 2;
}

void turnRight(){
  Serial.println("Turning right...");
  positions[0] = rightStepper.currentPosition() - stepsPerUnitTurn + 2;
  positions[1] = leftStepper.currentPosition() - stepsPerUnitTurn - 2;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 2;
  weight = 2;
}

void aboutFace(){
  Serial.println("About facing...");
  positions[0] = rightStepper.currentPosition() + 2 * stepsPerUnitTurn + 4;
  positions[1] = leftStepper.currentPosition() + 2 * stepsPerUnitTurn + 4;
  rightStepper.moveTo(positions[0]);
  leftStepper.moveTo(positions[1]);
  command = 3;
  weight = 2;
}

void localize(){

  Serial.println("Localizing...");

  prevXPos = currXPos;
  prevYPos = currYPos;

    Serial.print("Command: ");
    Serial.print(command);
    Serial.print("\t");
    Serial.print("CurrX: ");
    Serial.print(currXPos);
    Serial.print("\t");
    Serial.print("CurrY: ");
    Serial.print(currYPos);
    Serial.print("\t");
    Serial.print("CurrHeading: ");
    Serial.print(currHeading);
    Serial.print("\t");
    Serial.print("CurrNode: ");
    Serial.print(currNode);
    Serial.println("");

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


  currNode = (GRID_Y * currXPos) + currYPos;

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