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
const int rightEchoPin = 33;
const int frontTrigPin = 35;
const int frontEchoPin = 37;
const int leftTrigPin = 39;
const int leftEchoPin = 41;

void stepperRotate(float rotation, float rpm);

void setup(){

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

  Serial.begin(9600);

}

void loop(){

  moveForward(1, 100);
  delay(1000);
  turnLeft(100);
  delay(1000);
  turnRight(100);
  delay(1000);

}

void turnLeft(float rpm){

  digitalWrite(rightDirPin, HIGH);
  digitalWrite(leftDirPin, HIGH);

  float distancePerStep = stepSize * (wheelDiameter * 3.14) / stepsPerRotation;
  float totalSteps = round((robotDiameter * 3.14 / 4) / distancePerStep);

  unsigned long stepDelay = getStepDelay(rpm);

  stepMotor(stepDelay, totalSteps);

  delay(100);
  moveForward(1, rpm);

}

void turnRight(float rpm){

  digitalWrite(rightDirPin, LOW);
  digitalWrite(leftDirPin, LOW);

  float distancePerStep = stepSize * (wheelDiameter * 3.14) / stepsPerRotation;
  float totalSteps = round((robotDiameter * 3.14 / 4) / distancePerStep);

  unsigned long stepDelay = getStepDelay(rpm);

  stepMotor(stepDelay, totalSteps);

  delay(100);
  moveForward(1, rpm);

}

void moveForward(float units, float rpm){

  digitalWrite(rightDirPin, HIGH);
  digitalWrite(leftDirPin, LOW);

  float distancePerStep = stepSize * (wheelDiameter * 3.14) / stepsPerRotation;
  int totalSteps = round((units * unitLength) / distancePerStep);

  unsigned long stepDelay = getStepDelay(rpm);

  stepMotor(stepDelay, totalSteps);

}

unsigned long getStepDelay(float rpm){
  return ((60.0000 / (rpm * stepsPerRotation)) * 1E6 / 2.0000) - 5;
}

void stepMotor(unsigned long stepDelay, int totalSteps){

  for (unsigned long i = 0; i < totalSteps; i++) {
    digitalWrite(rightStepPin, HIGH);
    digitalWrite(leftStepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(rightStepPin, LOW);
    digitalWrite(leftStepPin, LOW);
    delayMicroseconds(stepDelay);
  }

}
