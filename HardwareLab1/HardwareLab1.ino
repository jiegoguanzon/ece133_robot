const int rightDirPin = 2;
const int rightStepPin = 4;
const int rightEnablePin = 6;
const int leftDirPin = 8;
const int leftStepPin = 10;
const int leftEnablePin = 12;

const int rightTrigPin = 33;
const int rightEchoPin = 35;
const int frontTrigPin = 37;
const int frontEchoPin = 39;
const int leftTrigPin = 41;
const int leftEchoPin = 43;

void setup() {

  Serial.begin(9600);

  pinMode(rightDirPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  pinMode(rightEnablePin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(leftStepPin, OUTPUT);
  pinMode(leftEnablePin, OUTPUT);
  
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  
  digitalWrite(rightDirPin, HIGH);
  
}

void loop() {

  stepMotor();
  
}

void stepMotor() {
  digitalWrite(rightStepPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightStepPin, LOW);
  delayMicroseconds(10);
}
