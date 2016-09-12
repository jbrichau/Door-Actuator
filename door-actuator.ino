#include "AccelStepperSpark.h"

#define MOTOR_DIR_PIN D0
#define MOTOR_STEP_PIN D1
#define MOTOR_ENABLE_PIN D2
#define MOTOR_SLEEP_PIN D3
#define END_OF_TRAVEL_LEFT_PIN D4
#define END_OF_TRAVEL_RIGHT_PIN D5

AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);

void setup() {
  pinMode(MOTOR_DIR_PIN,OUTPUT);
  pinMode(MOTOR_STEP_PIN,OUTPUT);
  pinMode(MOTOR_SLEEP_PIN,OUTPUT);
  pinMode(MOTOR_ENABLE_PIN,OUTPUT);
  //digitalWrite(MOTOR_STEP_PIN,LOW);
  digitalWrite(MOTOR_ENABLE_PIN,LOW);

  pinMode(END_OF_TRAVEL_LEFT_PIN,INPUT_PULLDOWN);
  pinMode(END_OF_TRAVEL_RIGHT_PIN,INPUT_PULLDOWN);
}

void loop() {
  openDoor();
  delay(3000);
  closeDoor();
  delay(3000);
}

void closeDoor() {
  //digitalWrite(MOTOR_DIR_PIN, HIGH);
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(20);
  stepper.moveTo(-200);
  moveToEnd(END_OF_TRAVEL_LEFT_PIN);
}

void openDoor() {
  //digitalWrite(MOTOR_DIR_PIN, LOW);
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(20);
  stepper.moveTo(200);
  moveToEnd(END_OF_TRAVEL_RIGHT_PIN);
}

void moveToEnd(int end_of_travel_pin) {
  int end_of_travel = digitalRead(end_of_travel_pin);
  while(!end_of_travel) {
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());
    stepper.run();
    end_of_travel = digitalRead(end_of_travel_pin);
  }
}

void motorSleep() {
  digitalWrite(MOTOR_SLEEP_PIN,LOW);
}

void motorWake() {
  digitalWrite(MOTOR_SLEEP_PIN,HIGH);
}

// This was code to do the timings manually
void moveToEnd_DIRECT(int end_of_travel_pin) {
  motorWake();
  int end_of_travel = digitalRead(end_of_travel_pin);
  int i = 50000;
  int steps = 0;
  while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    //delayMicroseconds(100);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(i);
    steps++;
    if(i>1600) {
      if(steps%50 == 0) i = i/2;
    }
    else
      i = 1000;
    end_of_travel = digitalRead(end_of_travel_pin);
  };
  motorSleep();
}

// This was an experiment to see how fast we can make it run
int slow = 400;
int fast = 1200;
int accelSteps = 100;
int freqStep = (fast - slow) / accelSteps;

void moveToEnd_PWM(int end_of_travel_pin) {
  int end_of_travel = digitalRead(end_of_travel_pin);
  motorWake();
  for(int i=0;i<=accelSteps;i++) {
    end_of_travel = digitalRead(end_of_travel_pin);
    if(end_of_travel)
      break;
    analogWrite(MOTOR_STEP_PIN,125,slow + (i * freqStep));
    delay(10);
  };
  while(!end_of_travel) {
    delay(1);
    //digitalWrite(MOTOR_STEP_PIN,LOW);
    //delay(1);
    end_of_travel = digitalRead(end_of_travel_pin);
  };
  analogWrite(MOTOR_STEP_PIN,0,1000);
  motorSleep();
}
