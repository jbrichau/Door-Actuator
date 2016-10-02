#include "AccelStepperSpark.h"

#define MOTOR_DIR_PIN D0
#define MOTOR_STEP_PIN D1
#define MOTOR_ENABLE_PIN D2
#define MOTOR_SLEEP_PIN D3

#define END_OF_TRAVEL_LEFT_PIN D4
#define END_OF_TRAVEL_RIGHT_PIN D5

#define HALL_SENSOR_POWER A5
#define HALL_SENSOR_SENSE D6

#define STEP_FACTOR 4

#define STATE_CLOSED 0
#define STATE_OPEN 1
#define STATE_MOTORCLOSING 2
#define STATE_MOTOROPENING 3
#define STATE_MOTORSTALLED 4
#define STATE_STATICOPEN 5

SYSTEM_MODE(MANUAL);

AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
Timer closer(3000, auto_close_door, true);
int rotation_step = 0;
int previous_rotation_step = 0;
int doorState;
int previous_stepper_distance;

void setup() {
  // Motor
  pinMode(MOTOR_DIR_PIN,OUTPUT);
  pinMode(MOTOR_STEP_PIN,OUTPUT);
  pinMode(MOTOR_SLEEP_PIN,OUTPUT);
  pinMode(MOTOR_ENABLE_PIN,OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN,LOW);
  stepper.setMaxSpeed(1000*STEP_FACTOR);
  stepper.setAcceleration(300*STEP_FACTOR);

  // Limit switches
  pinMode(END_OF_TRAVEL_LEFT_PIN,INPUT_PULLDOWN);
  pinMode(END_OF_TRAVEL_RIGHT_PIN,INPUT_PULLDOWN);

  // Movement detector
  pinMode(HALL_SENSOR_POWER,OUTPUT);
  pinMode(HALL_SENSOR_SENSE,INPUT_PULLUP);
  digitalWrite(HALL_SENSOR_POWER,HIGH);
  attachInterrupt(HALL_SENSOR_SENSE, rotation_step_detected, FALLING, 5);

  Particle.connect();
  Particle.variable("position",stepper.currentPosition());
  Particle.variable("doorstate",doorState);
  Particle.variable("rotation",rotation_step);

  calibrate();
  doorState = STATE_CLOSED;
  Serial.begin(9600);
  Serial.println("Door actuator ready for operation");
}

void loop() {
  switch(doorState) {

    case STATE_CLOSED:
      if(movement_detected()) {
        schedule_close();
        doorState = STATE_OPEN;
      }
      Particle.process();
      break;

    case STATE_OPEN:
      if(movement_detected())
        schedule_close();
      Particle.process();
      break;

    case STATE_MOTORCLOSING:
      doMotorStep(END_OF_TRAVEL_LEFT_PIN,STATE_CLOSED);
      break;

    case STATE_MOTOROPENING:
      doMotorStep(END_OF_TRAVEL_RIGHT_PIN,STATE_OPEN);
      break;

    case STATE_MOTORSTALLED:
      if(movement_detected())
        schedule_close();
      Particle.process();
      break;

    case STATE_STATICOPEN:
      Particle.process();
      break;
  }
}

void setStalled(int nextState) {
  doorState = STATE_MOTORSTALLED;
  motorSleep();
  Serial.println("Motor stalled");
  stepper.setCurrentPosition(stepper.currentPosition());
  if(nextState == STATE_CLOSED)
    schedule_close();
  //rotation_step = 0;
  //previous_rotation_step = 0;
}

void setMotorClosing() {
  doorState = STATE_MOTORCLOSING;
  Serial.printlnf("Closing door from rotation step %d, position %d",rotation_step,-(rotation_step/3)*STEP_FACTOR*200);
  if(stepper.currentPosition() == 0)
    stepper.setCurrentPosition(-(rotation_step/3)*STEP_FACTOR*200);
  stepper.moveTo(0);
  previous_stepper_distance = stepper.distanceToGo();
  rotation_step = 0;
  previous_rotation_step = 0;
  motorWake();
}

void setClosed() {
  doorState = STATE_CLOSED;
  Serial.println("Closed!");
  motorSleep();
  rotation_step = 0;
  previous_rotation_step = 0;
}

bool movement_detected() {
  bool movement = rotation_step > previous_rotation_step;
  if(movement)
    Serial.printlnf("Movement detected of %d steps",rotation_step - previous_rotation_step);
  previous_rotation_step = rotation_step;
  return movement;
}

void schedule_close() {
  if(closer.isActive()) {
    Serial.println("Reseting autoclose");
    closer.reset();
  } else {
    Serial.println("Scheduling autoclose");
    closer.start();
  }
}

void auto_close_door() {
  Serial.println("Auto closing");
  setMotorClosing();
}

void rotation_step_detected() {
  rotation_step++;
}

void doMotorStep(int end_of_travel_pin, int nextState) {
  int end_of_travel = digitalRead(end_of_travel_pin);
  if(end_of_travel || (stepper.distanceToGo() == 0)) {
    doorState = nextState;
    setClosed();
    return;
  }
  if(previous_stepper_distance == NULL)
    previous_stepper_distance = stepper.distanceToGo();
  stepper.run();
  if(abs((stepper.distanceToGo() - previous_stepper_distance)) > 200*STEP_FACTOR) {
    Serial.printlnf("Checking movement at distance to go:%d",stepper.distanceToGo());
    if(!movement_detected()) {
      setStalled(nextState);
      Serial.println("Stall!");
    }
    previous_stepper_distance = stepper.distanceToGo();
  }
}

void motorSleep() {
  digitalWrite(MOTOR_SLEEP_PIN,LOW);
}

void motorWake() {
  digitalWrite(MOTOR_SLEEP_PIN,HIGH);
}

void calibrate() {
  motorWake();
  digitalWrite(MOTOR_DIR_PIN, LOW);
  int end_of_travel = digitalRead(END_OF_TRAVEL_RIGHT_PIN);
  /*while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(500);
    end_of_travel = digitalRead(END_OF_TRAVEL_RIGHT_PIN);
  };*/
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  end_of_travel = digitalRead(END_OF_TRAVEL_LEFT_PIN);
  int steps = 0;
  rotation_step = 0;
  while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    steps++;
    delayMicroseconds(2);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(500);
    if (steps == (200*STEP_FACTOR)) {
      Serial.printlnf("Motor rotation corresponds to %d pulley rotation steps",rotation_step);
      rotation_step = 0;
      steps = 0;
    }
    end_of_travel = digitalRead(END_OF_TRAVEL_LEFT_PIN);
  };
  motorSleep();
  stepper.setCurrentPosition(0);
}
