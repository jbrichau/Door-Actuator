#include "AccelStepperSpark.h"

// Need to keep D0 and D1 free for the battery shield
#define MOTOR_DIR_PIN A0
#define MOTOR_STEP_PIN A1
#define MOTOR_ENABLE_PIN D2
#define MOTOR_SLEEP_PIN D3
#define MOTOR_FAULT_PIN A3

#define END_OF_TRAVEL_CLOSED_PIN D4
#define END_OF_TRAVEL_OPEN_PIN D5

#define HALL_SENSOR_POWER A5
#define HALL_SENSOR_SENSE A4

#define STEP_FACTOR 2

#define STATE_CLOSED 0
#define STATE_OPEN 1
#define STATE_MOTORCLOSING 2
#define STATE_MOTOROPENING 3
#define STATE_MOTORSTALLED 4
#define STATE_STATICOPEN 5

// Set the mode to manual to avoid Particle network busyness during motor runs
SYSTEM_MODE(MANUAL);

AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
Timer closer(3000, auto_close_door, true);

int rotation_step = 0;
int previous_rotation_step = 0;
int previous_stepper_distance;
int doorState;


void setup() {
  // Motor
  pinMode(MOTOR_DIR_PIN,OUTPUT);
  pinMode(MOTOR_STEP_PIN,OUTPUT);
  pinMode(MOTOR_SLEEP_PIN,OUTPUT);
  pinMode(MOTOR_ENABLE_PIN,OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN,LOW);
  stepper.setMaxSpeed(1000*STEP_FACTOR);
  stepper.setAcceleration(300*STEP_FACTOR);
  stepper.setPinsInverted(true,false,false);

  // Limit switches
  pinMode(END_OF_TRAVEL_CLOSED_PIN,INPUT_PULLDOWN);
  pinMode(END_OF_TRAVEL_OPEN_PIN,INPUT_PULLDOWN);

  // Movement detector
  pinMode(HALL_SENSOR_POWER,OUTPUT);
  pinMode(HALL_SENSOR_SENSE,INPUT_PULLUP);
  digitalWrite(HALL_SENSOR_POWER,HIGH);
  attachInterrupt(HALL_SENSOR_SENSE, rotation_step_detected, FALLING);

  Particle.connect();
  Particle.variable("doorstate",doorState);
  Particle.variable("rotation",rotation_step);
  Particle.function("opendoor",manual_open_door);
  Particle.function("closedoor",manual_close_door);

  calibrate();
  doorState = STATE_CLOSED;
  Serial.begin(9600);
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
      doMotorStep(END_OF_TRAVEL_CLOSED_PIN,STATE_CLOSED,true);
      break;

    case STATE_MOTOROPENING:
      doMotorStep(END_OF_TRAVEL_OPEN_PIN,STATE_OPEN,false);
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

void setMotorOpening() {
  doorState = STATE_MOTOROPENING;
  stepper.moveTo(-2300*STEP_FACTOR);
  previous_stepper_distance = stepper.distanceToGo();
  rotation_step = 0;
  previous_rotation_step = 0;
  motorWake();
}

void setClosed() {
  doorState = STATE_CLOSED;
  motorSleep();
  rotation_step = 0;
  previous_rotation_step = 0;
  stepper.setCurrentPosition(0);
}

void setOpened() {
  doorState = STATE_OPEN;
  motorSleep();
  schedule_close();
}

bool movement_detected() {
  bool movement = rotation_step > previous_rotation_step;
  //if(movement)
  //  Serial.printlnf("Movement detected of %d steps",rotation_step - previous_rotation_step);
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

int manual_close_door(String arg) {
  setMotorClosing();
  return 0;
}

int manual_open_door(String arg) {
  setMotorOpening();
  return 0;
}

void rotation_step_detected() {
  rotation_step++;
}

void doMotorStep(int end_of_travel_pin, int nextState, bool should_travel_to_end) {
  int end_of_travel = digitalRead(end_of_travel_pin);
  if(end_of_travel || (stepper.distanceToGo() == 0)) {
    doorState = nextState;
    if(nextState == STATE_CLOSED)
      setClosed();
    if(nextState == STATE_OPEN)
      setOpened();
    return;
  }
  if(previous_stepper_distance == NULL)
    previous_stepper_distance = stepper.distanceToGo();

    if(should_travel_to_end && stepper.distanceToGo() < 50*STEP_FACTOR) {
      int current_speed = stepper.speed();
      Serial.printlnf("Distance to go is %d at %f",stepper.distanceToGo(),stepper.speed());
      stepper.moveTo(stepper.targetPosition()+(50*STEP_FACTOR));
      stepper.setSpeed(current_speed);
      previous_stepper_distance = stepper.distanceToGo();
      stepper.runSpeed();
    } else {
      stepper.run();
    }
  if(abs((stepper.distanceToGo() - previous_stepper_distance)) > 100*STEP_FACTOR) {
    if(!movement_detected()) {
      setStalled(nextState);
      Serial.printlnf("Stall detected at position %d",stepper.currentPosition());
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
  int end_of_travel = digitalRead(END_OF_TRAVEL_OPEN_PIN);
  /*while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(500);
    end_of_travel = digitalRead(END_OF_TRAVEL_OPEN_PIN);
  };*/
  digitalWrite(MOTOR_DIR_PIN, LOW);
  end_of_travel = digitalRead(END_OF_TRAVEL_CLOSED_PIN);
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
    end_of_travel = digitalRead(END_OF_TRAVEL_CLOSED_PIN);
  };
  motorSleep();
  stepper.setCurrentPosition(0);
}
