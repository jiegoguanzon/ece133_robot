const float motorAngle = 1.8;
const float stepSize = 1;
const int stepsPerRotation = 200;

const int SleepPin = 2;
const int ResetPin = 3;
const int M2Pin = 4;
const int M1Pin = 5;
const int M0Pin = 6;
const int EnablePin = 7;

const int rightDirPin = 8;
const int rightStepPin = 9;

const int leftDirPin = 10;
const int leftStepPin = 11;

void stepperRotate(float rotation, float rpm);
float getStepSize(bool M0, bool M1, bool M2);

void setup() {

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

  Serial.begin(9600);

}

void loop() {

  stepperRotate(1, 200);
  delay(100);
  stepperRotate(-1, 200);
  delay(100);

}

void stepperRotate(float rotation, float rpm) {
  
  if (rotation > 0) {
    digitalWrite(dirPin, HIGH);
  }
  else {
    digitalWrite(dirPin, LOW);
    rotation = rotation * -1;
  }

  float totalSteps = rotation * stepsPerRotation;
  
  unsigned long stepDelay = ((60.0000 / (rpm * stepsPerRotation)) * 1E6 / 2.0000) - 5;

  for (unsigned long i = 0; i < totalSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

}
