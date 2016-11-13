#include "AccelStepperSpark.h"
#include "SparkFunMAX17043.h"

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

#define ENCODER_A_PIN A4
#define ENCODER_B_PIN A5

#define ENCODER_MAGDEC_PIN RX
#define ENCODER_MAGINC_PIN TX

#define SONARPIN A2
#define AVR_RANGE 60

#define STEP_FACTOR 2
#define STEPPER_STEPS 200

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
int rotation_steps_per_motorrotation = 7;
int previous_stepper_distance;
int doorState;

//double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
bool alert; // Variable to keep track of whether alert has been triggered

void setup() {
  // Motor
  pinMode(MOTOR_DIR_PIN,OUTPUT);
  pinMode(MOTOR_STEP_PIN,OUTPUT);
  pinMode(MOTOR_SLEEP_PIN,OUTPUT);
  pinMode(MOTOR_ENABLE_PIN,OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN,LOW);
  stepper.setMaxSpeed(1000*STEP_FACTOR);
  stepper.setAcceleration(500*STEP_FACTOR);
  stepper.setPinsInverted(true,false,false);

  // Limit switches
  pinMode(END_OF_TRAVEL_CLOSED_PIN,INPUT_PULLDOWN);
  pinMode(END_OF_TRAVEL_OPEN_PIN,INPUT_PULLDOWN);

  // Movement detector
  pinMode(ENCODER_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCODER_B_PIN, INPUT_PULLDOWN);
  pinMode(ENCODER_MAGDEC_PIN, INPUT_PULLUP);
  pinMode(ENCODER_MAGINC_PIN, INPUT_PULLUP);
  attachInterrupt(ENCODER_A_PIN, read_quadrature, CHANGE);

  lipo.begin(); // Initialize the MAX17043 LiPo fuel gauge
  // Quick start restarts the MAX17043 in hopes of getting a more accurate
  // guess for the SOC.
  lipo.quickStart();
  // We can set an interrupt to alert when the battery SoC gets too low.
  // We can alert at anywhere between 1% - 32%:
  //lipo.setThreshold(10); // Set alert threshold to 20%.

  Particle.connect();
  Particle.variable("doorstate",doorState);
  Particle.variable("rotation",rotation_step);
  Particle.variable("soc",soc);
  Particle.function("opendoor",manual_open_door);
  Particle.function("closedoor",manual_close_door);

  if (Particle.connected() == false) {
    Particle.connect();
  }
  calibrate();
  //doorState = STATE_OPEN;
  //schedule_close();
  //stepper.setCurrentPosition(0);
  Serial.begin(9600);
}

void loop() {
  switch(doorState) {

    case STATE_CLOSED:
      if(movement_detected(10)) {
        schedule_close();
        doorState = STATE_OPEN;
      }
      idle_tasks();
      break;

    case STATE_OPEN:
      if(movement_detected(10))
        schedule_close();
      idle_tasks();
      break;

    case STATE_MOTORCLOSING:
      doMotorStep(END_OF_TRAVEL_CLOSED_PIN,STATE_CLOSED,true);
      /*if(analogRead(SONARPIN) < 188) {
        //Serial.printlnf("Detected presence at %dcm. stop closing door.",prox);
        setMotorOpening();
      }*/
      break;

    case STATE_MOTOROPENING:
      doMotorStep(END_OF_TRAVEL_OPEN_PIN,STATE_OPEN,false);
      break;

    case STATE_MOTORSTALLED:
      if(movement_detected(10))
        schedule_close();
      idle_tasks();
      break;

    case STATE_STATICOPEN:
      idle_tasks();
      break;
  }
}

void idle_tasks() {
  int prox;

  if(Particle.connected())
    Particle.process();
  else
    Particle.connect();

  soc = lipo.getSOC();
  prox = proximity(AVR_RANGE,10);
  if(prox < 60 && doorState == STATE_CLOSED) {
    //Serial.printlnf("Detected presence at %dcm. Opening door.",prox);
    setMotorOpening();
  }

  Serial.printlnf("position: %d, rotation_step: %d",stepper.currentPosition(), rotation_step);
}

int proximity(int average_range,int delay_ms) {
  int sum=0, inches=0, cm=0;

  for (int i = 0; i < average_range ; i++) {
     //Used to read in the analog voltage output that is being sent by the MaxSonar device.
     //Scale factor is (Vcc/512) per inch. A 3.3V supply yields ~6.4mV/in
     //Photon analog pin goes from 0 to 4095, so divide by 8
     int anVolt;

     anVolt = analogRead(SONARPIN) / 8;
     sum += anVolt;
     if(delay_ms > 0) delay(delay_ms);
   }

   inches = sum / average_range;
   cm = inches * 2.54;
   return cm;
}

void setStalled(int nextState) {
  doorState = STATE_MOTORSTALLED;
  Serial.println("Motor stalled");
  /*if(nextState == STATE_CLOSED)
    stepper.setCurrentPosition(stepper.currentPosition()+100);
  else
    stepper.setCurrentPosition(stepper.currentPosition()-100);*/
  motorSleep();
  //stepper.setCurrentPosition(stepper.currentPosition());
  if(nextState == STATE_CLOSED)
    schedule_close();
}

void setMotorClosing() {
  doorState = STATE_MOTORCLOSING;
  float position = ((float)rotation_step / rotation_steps_per_motorrotation)*STEP_FACTOR*STEPPER_STEPS;
  Serial.printlnf("Closing from position: %f",position);
  if(stepper.currentPosition() == 0)
    stepper.setCurrentPosition(-(int)position);
  stepper.moveTo(0);
  previous_stepper_distance = stepper.distanceToGo();
  motorWake();
}

void setMotorOpening() {
  doorState = STATE_MOTOROPENING;
  stepper.moveTo(-3000*STEP_FACTOR);
  previous_stepper_distance = stepper.distanceToGo();
  motorWake();
}

void setClosed() {
  doorState = STATE_CLOSED;
  motorSleep();
  stepper.setCurrentPosition(0);
  rotation_step = 0;
  previous_rotation_step = 0;
}

void setOpened() {
  doorState = STATE_OPEN;
  motorSleep();
  schedule_close();
}

void read_quadrature()
{
  // found a low-to-high on channel A
  if (digitalRead(ENCODER_A_PIN) == HIGH)
  {
    // check channel B to see which way
    if (digitalRead(ENCODER_B_PIN) == LOW)
        rotation_step++;
    else
        rotation_step--;
  }
  // found a high-to-low on channel A
  else
  {
    // check channel B to see which way
    if (digitalRead(ENCODER_B_PIN) == LOW)
        rotation_step--;
    else
        rotation_step++;
  }
}

bool movement_detected(int steps) {
  bool movement = abs(rotation_step - previous_rotation_step) > steps;
  if(movement) {
    //Serial.printlnf("Movement detected of %d > %d",rotation_step - previous_rotation_step,steps);
    previous_rotation_step = rotation_step;
  }
  //else
    //Serial.printlnf("No movement detected: %d < %d", rotation_step - previous_rotation_step,steps);
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
  Particle.publish("manual-opendoor",NULL,60,PRIVATE);
  return 0;
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
    // TODO: does not work on motor opening
    int current_speed = stepper.speed();
    Serial.printlnf("Distance to go is %d at %f",stepper.distanceToGo(),stepper.speed());
    stepper.moveTo(stepper.targetPosition()+(50*STEP_FACTOR));
    stepper.setSpeed(current_speed);
    previous_stepper_distance = stepper.distanceToGo();
    stepper.runSpeed();
  } else {
    stepper.run();
  }

  if(abs((stepper.distanceToGo() - previous_stepper_distance)) > 50*STEP_FACTOR) {
    if(!movement_detected(10)) {
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
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  int end_of_travel = digitalRead(END_OF_TRAVEL_OPEN_PIN);
  while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(1000);
    end_of_travel = digitalRead(END_OF_TRAVEL_OPEN_PIN);
  };
  digitalWrite(MOTOR_DIR_PIN, LOW);
  end_of_travel = digitalRead(END_OF_TRAVEL_CLOSED_PIN);
  int steps = 0;
  while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    steps++;
    delayMicroseconds(2);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    delayMicroseconds(1000);
    if (steps == (STEPPER_STEPS*STEP_FACTOR)) {
      Serial.printlnf("Motor rotation corresponds to %d rotation steps",abs(rotation_step - abs(previous_rotation_step)));
      rotation_steps_per_motorrotation = abs(rotation_step - abs(previous_rotation_step));
      previous_rotation_step = rotation_step;
      steps = 0;
    }
    end_of_travel = digitalRead(END_OF_TRAVEL_CLOSED_PIN);
  };
  motorSleep();
  stepper.setCurrentPosition(0);
}
