#include "AccelStepperSpark.h"

#define MOTOR_DIR_PIN D0
#define MOTOR_STEP_PIN D1
#define MOTOR_ENABLE_PIN D2
#define MOTOR_SLEEP_PIN D3

#define END_OF_TRAVEL_LEFT_PIN D4
#define END_OF_TRAVEL_RIGHT_PIN D5

#define HALL_SENSOR_POWER A5
#define HALL_SENSOR_SENSE D6

#define STEP_FACTOR 8

//SYSTEM_MODE(MANUAL);

AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);

int magnet = 0;

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

  // Stall detector
  pinMode(HALL_SENSOR_POWER,OUTPUT);
  pinMode(HALL_SENSOR_SENSE,INPUT_PULLUP);
  digitalWrite(HALL_SENSOR_POWER,HIGH);
  attachInterrupt(HALL_SENSOR_SENSE, hall_sense, FALLING);

  Serial.begin(9600);
  Serial.println("Hello Magnetic World!");

  Particle.variable("position",stepper.currentPosition());
  Particle.function("opendoor",remote_open);
  Particle.function("closedoor",remote_close);

  moveToStartPosition();
  stepper.setCurrentPosition(0);
}

void loop() {
  //openDoor();
  //delay(3000);
  //closeDoor();
  //delay(3000);
  //Particle.process();
  Serial.println(magnet);
}

int remote_open(String arg) {
  openDoor();
  return 0;
}

int remote_close(String arg) {
  closeDoor();
  return 0;
}

void hall_sense() {
  magnet++;
}

void closeDoor() {
  //digitalWrite(MOTOR_DIR_PIN, HIGH);
  Serial.println("Closing door");
  stepper.moveTo(0);
  runMotor(END_OF_TRAVEL_LEFT_PIN);
  stepper.setCurrentPosition(0);
}

void openDoor() {
  //digitalWrite(MOTOR_DIR_PIN, LOW);
  Serial.println("Opening door");
  stepper.moveTo(-2300*STEP_FACTOR);
  runMotor(END_OF_TRAVEL_RIGHT_PIN);
  stepper.setCurrentPosition(stepper.currentPosition());
  Serial.printlnf("Door max at %d",stepper.currentPosition());
}


void runMotor(int end_of_travel_pin) {
  int end_of_travel = digitalRead(end_of_travel_pin);
  int previous_magnet_count = magnet;
  int previous_stepper_position = stepper.currentPosition();
  motorWake();
  while(!end_of_travel && (stepper.distanceToGo() != 0)) {
    stepper.run();
    //Serial.println(stepper.currentPosition());
    end_of_travel = digitalRead(end_of_travel_pin);
    if(abs(stepper.currentPosition() - previous_stepper_position) > 600) {
      if(magnet - previous_magnet_count == 0) {
        end_of_travel = true;
        Serial.println("Stall!");
      }
      previous_stepper_position = stepper.currentPosition();
      previous_magnet_count = magnet;
    }
  }
  motorSleep();
}

void motorSleep() {
  digitalWrite(MOTOR_SLEEP_PIN,LOW);
}

void motorWake() {
  digitalWrite(MOTOR_SLEEP_PIN,HIGH);
}

int moveToStartPosition() {
  digitalWrite(MOTOR_DIR_PIN, LOW);
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  motorWake();
  int end_of_travel = digitalRead(END_OF_TRAVEL_LEFT_PIN);
  int steps = 0;
  while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    steps++;
    delayMicroseconds(2);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(500);
    end_of_travel = digitalRead(END_OF_TRAVEL_LEFT_PIN);
  };
  motorSleep();
  return steps;
}
