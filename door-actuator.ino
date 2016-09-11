#define MOTOR_DIR_PIN D0
#define MOTOR_STEP_PIN D1
#define MOTOR_ENABLE_PIN D2
#define MOTOR_SLEEP_PIN D3
#define END_OF_TRAVEL_LEFT_PIN D4
#define END_OF_TRAVEL_RIGHT_PIN D5

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
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  moveToEnd(END_OF_TRAVEL_LEFT_PIN);
}

void openDoor() {
  digitalWrite(MOTOR_DIR_PIN, LOW);
  moveToEnd(END_OF_TRAVEL_RIGHT_PIN);
}

int slow = 400;
int fast = 1400;
int accelSteps = 100;
int freqStep = (fast - slow) / accelSteps;

void moveToEnd(int end_of_travel_pin) {
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

void moveToEnd2(int end_of_travel_pin) {
  motorWake();
  int end_of_travel = digitalRead(end_of_travel_pin);
  while(!end_of_travel) {
    digitalWrite(MOTOR_STEP_PIN,HIGH);
    delay(1);
    digitalWrite(MOTOR_STEP_PIN,LOW);
    //delay(1);
    end_of_travel = digitalRead(end_of_travel_pin);
  };
  motorSleep();
}

void motorSleep() {
  digitalWrite(MOTOR_SLEEP_PIN,LOW);
}

void motorWake() {
  digitalWrite(MOTOR_SLEEP_PIN,HIGH);
}
