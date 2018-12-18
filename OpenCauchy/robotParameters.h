const int SONAR_NUM = 3;
const int MAX_SENSING_DISTANCE = 10;
const int PING_INTERVAL = 25;

const int WHEEL_DIAMETER = 65;
const int ROBOT_DIAMETER = 200;
const int UNIT_LENGTH = 200;

const int DEFAULT_MOTOR_SPEED = 500;
const int DEFAULT_ACCELERATION = DEFAULT_MOTOR_SPEED * 2;
/*
const int MIN_MOTOR_SPEED = 0;
const int MAX_MOTOR_SPEED = 99999;
*/
const int MIN_MOTOR_SPEED = DEFAULT_MOTOR_SPEED * 0.6;
const int MAX_MOTOR_SPEED = DEFAULT_MOTOR_SPEED * 1.4;

const int STEPS_PER_ROTATION = 200;

const float DISTANCE_TO_WALL = 4.5;

const int M2 = 0;
const int M1 = 0;
const int M0 = 0;

const float P_GAIN = 0.3;
const float I_GAIN = 0.05;
const float D_GAIN = 0.1;

int rightMotorSpeed;
int leftMotorSpeed;

int currentRightMotorSpeed;
int currentLeftMotorSpeed;

int stepsPerUnitLength;
int stepsPerUnitTurn;

int currXPos;
int currYPos;
int currHeading;
int currNode;

int prevXPos;
int prevYPos;
int prevNode;
int prevHeading;

int startXPos;
int startYPos;
int startHeading;
int startNode;

float stepSize;