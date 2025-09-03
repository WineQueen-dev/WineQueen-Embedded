#include <AccelStepper.h>

bool isSealed = false;
int retryCount = 0;
const int MAX_RETRIES = 2;

const int INTERRUPT_Z = 2;
const int INTERRUPT_X = 3;
const int X_DIR_PIN = 7;
const int X_STEP_PIN = 8;
const int X_ENA_PIN = 9;
const int Z_DIR_PIN = 10;
const int Z_STEP_PIN = 11;
const int Z_ENA_PIN = 12;
const int LINEAR_IN2 = 13;
const int LINEAR_ENA = A0;
const int LINEAR_IN1 = A1;
const int VACUUM_PUMP_PIN = A2;
const int ELECTROMAGNET_PIN = A3;
const int OPEN_BUTTON_PIN = A4;
const int SEAL_BUTTON_PIN = A5;

const int MICROSTEP_MULTIPLIER = 16;
const long Z_HOME_POSITION = 0;
const long Z_REST_POSITION = 450 * 16;
const long Z_LID_ROOM_HEIGHT = 450 * 16;
const long Z_LID_INSERT_HEIGHT = 460 * 16;
const long Z_LID_OPEN_HEIGHT = 455 * 16;
const long X_HOME_POSITION = 0;
const long X_LID_ROOM_POSITION = -165 * 16; 
const long X_WINE_POSITION_APPROX = 680 * 16;
const long CAMERA_MAGNET_OFFSET = 165 * 16; 
const long CAMERA_VACUUM_OFFSET = 175 * 16;

const float NORMAL_SPEED = 10000 * 16;
const float NORMAL_ACCEL = 7500 * 16;

const float OPEN_LID_SPEED = 25 * 16;
const float OPEN_LID_ACCEL = 15 * 16;

const int LINEAR_MOTOR_SPEED = 250;
const unsigned long LINEAR_DURATION = 1200;

long Sealed_Wine_PositionX = 0;

int sealButtonState = HIGH;
int lastSealButtonState = HIGH;
unsigned long lastSealDebounceTime = 0;
int openButtonState = HIGH;
int lastOpenButtonState = HIGH;
unsigned long lastOpenDebounceTime = 0;
const unsigned long debounceDelay = 50;

enum SystemState { 
  IDLE, 
  HOMING,
  SEALING_ALIGN_Y, 
  SEALING_GET_LID, 
  SEALING_MOVE_TO_WINE, 
  SEALING_ALIGN_CAMERA, 
  SEALING_PLACE_LID, 
  SEALING_APPLY_VACUUM,
  OPENING_MOVE_TO_WINE, 
  OPENING_GET_LID, 
  OPENING_BREAK_VACUUM, 
  OPENING_RETURN_LID,
  FAILED_RETURN_LID 
};
SystemState currentState = HOMING; 

AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

volatile bool zLimitHit = false;
volatile bool xLimitHit = false;

void zLimitSwitchISR() {
  static unsigned long last = 0;
  unsigned long now = micros();
  if (now - last > 50) {
    zLimitHit = true;
    last = now;
  }
}
void xLimitSwitchISR() {
  static unsigned long last = 0;
  unsigned long now = micros();
  if (now - last > 50) {
    xLimitHit = true;
    last = now;
  }
}

void setup() {
  Serial.begin(9600);
  
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  pinMode(VACUUM_PUMP_PIN, OUTPUT);
  pinMode(LINEAR_ENA, OUTPUT);
  pinMode(LINEAR_IN1, OUTPUT);
  pinMode(LINEAR_IN2, OUTPUT);
  pinMode(SEAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(OPEN_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(ELECTROMAGNET_PIN, LOW);
  digitalWrite(VACUUM_PUMP_PIN, LOW);
  digitalWrite(LINEAR_IN1, LOW);
  digitalWrite(LINEAR_IN2, LOW);
  analogWrite(LINEAR_ENA, 0);

  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_ENA_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_ENA_PIN, OUTPUT);
  digitalWrite(Z_ENA_PIN, HIGH);
  digitalWrite(X_ENA_PIN, HIGH);

  stepperZ.setMaxSpeed(NORMAL_SPEED);
  stepperZ.setAcceleration(NORMAL_ACCEL);
  stepperX.setMaxSpeed(NORMAL_SPEED);
  stepperX.setAcceleration(NORMAL_ACCEL);

  pinMode(INTERRUPT_Z, INPUT_PULLUP);
  pinMode(INTERRUPT_X, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_Z), zLimitSwitchISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_X), xLimitSwitchISR, FALLING);
}

void motorsEnable() {
  digitalWrite(Z_ENA_PIN, LOW);
  digitalWrite(X_ENA_PIN, LOW);
  delay(10);
}

void motorsDisable() {
  digitalWrite(Z_ENA_PIN, HIGH);
  digitalWrite(X_ENA_PIN, HIGH);
}

void moveZ_toHome() {
  zLimitHit = false;
  const float HOMING_SPEED = 800*16;
  float originalSpeed = stepperZ.maxSpeed();
  float originalAccel = stepperZ.acceleration();
  stepperZ.setMaxSpeed(HOMING_SPEED);
  stepperZ.setAcceleration(HOMING_SPEED / 2);
  stepperZ.setSpeed(-HOMING_SPEED);
  while (!zLimitHit) { stepperZ.runSpeed(); delay(1); }
  stepperZ.stop();
  stepperZ.setCurrentPosition(Z_HOME_POSITION);

  stepperZ.setMaxSpeed(originalSpeed);
  stepperZ.setAcceleration(originalAccel);
}

void moveX_toHome() {
  xLimitHit = false;
  float originalSpeed = stepperX.maxSpeed();
  float originalAccel = stepperX.acceleration();
  const float HOMING_SPEED = 800*16;
  stepperX.setMaxSpeed(HOMING_SPEED);
  stepperX.setAcceleration(HOMING_SPEED / 2);
  stepperX.setSpeed(-HOMING_SPEED);
  while (!xLimitHit) { stepperX.runSpeed(); delay(1); }
  stepperX.stop();
  stepperX.setCurrentPosition(X_HOME_POSITION);

  stepperX.setMaxSpeed(originalSpeed);
  stepperX.setAcceleration(originalAccel);
}

void initialize() {
  motorsEnable();
  moveZ_toHome();
  stepperX.moveTo(50*16);
  stepperX.runToPosition();
  moveX_toHome();

  stepperZ.moveTo(Z_REST_POSITION);
  stepperZ.runToPosition();
  motorsDisable();
}

void controlElectromagnet(bool power) {
  digitalWrite(ELECTROMAGNET_PIN, power ? HIGH : LOW);
  delay(500);
}

void Vacuum_pump() {
  digitalWrite(VACUUM_PUMP_PIN, HIGH);
  delay(10000);
  digitalWrite(VACUUM_PUMP_PIN, LOW);
}

bool camera_Align() {

  unsigned long startTime = millis();
  while (millis() - startTime < 60000) {
    if (Serial.available() > 0) {
      char command = Serial.read();
      switch (command) {
        case 'R': stepperX.move(3*16); break;
        case 'L': stepperX.move(-3*16); break;
        case 'C':
          stepperX.stop();
          return true;
      }
    }
    stepperX.run();
  }
  stepperX.stop();
  return false;
}

void linearMotor_align() {
  digitalWrite(LINEAR_IN1, HIGH);
  digitalWrite(LINEAR_IN2, LOW);
  analogWrite(LINEAR_ENA, LINEAR_MOTOR_SPEED);
  delay(LINEAR_DURATION);
  digitalWrite(LINEAR_IN1, LOW);
  delay(500);
  digitalWrite(LINEAR_IN2, HIGH);
  delay(LINEAR_DURATION);
  digitalWrite(LINEAR_IN1, LOW);
  digitalWrite(LINEAR_IN2, LOW);
  analogWrite(LINEAR_ENA, 0);
}

bool isSealButtonPressed() {
  int reading = digitalRead(SEAL_BUTTON_PIN);
  if (reading != lastSealButtonState) { lastSealDebounceTime = millis(); }
  if ((millis() - lastSealDebounceTime) > debounceDelay) {
    if (reading != sealButtonState) {
      sealButtonState = reading;
      if (sealButtonState == LOW) {
        lastSealButtonState = reading;
        return true;
      }
    }
  }
  lastSealButtonState = reading;
  return false;
}

bool isOpenButtonPressed() {
  int reading = digitalRead(OPEN_BUTTON_PIN);
  if (reading != lastOpenButtonState) { lastOpenDebounceTime = millis(); }
  if ((millis() - lastOpenDebounceTime) > debounceDelay) {
    if (reading != openButtonState) {
      openButtonState = reading;
      if (openButtonState == LOW) {
        lastOpenButtonState = reading;
        return true;
      }
    }
  }
  lastOpenButtonState = reading;
  return false;
}

void loop() {
  switch (currentState) {
    case IDLE:
      if (isSealButtonPressed()) {
        retryCount = 0;
        Serial.println("1");
        Serial.flush();
        motorsEnable();
        currentState = SEALING_ALIGN_Y;
      }
      if (isOpenButtonPressed()) {
        if (isSealed) {
          Serial.println("2");
          Serial.flush();
          motorsEnable();
          currentState = OPENING_MOVE_TO_WINE;
        }
      }
      break;

    case HOMING:
      delay(3000);
      initialize();
      currentState = IDLE;
      break;

    case SEALING_ALIGN_Y:
      isSealed = false;
      linearMotor_align();
      currentState = SEALING_GET_LID;
      break;

    case SEALING_GET_LID:
      moveZ_toHome();
      stepperX.moveTo(X_LID_ROOM_POSITION);
      stepperX.runToPosition();
      stepperZ.moveTo(Z_LID_ROOM_HEIGHT);
      stepperZ.runToPosition();
      controlElectromagnet(true);
      moveZ_toHome();
      currentState = SEALING_MOVE_TO_WINE;
      break;

    case SEALING_MOVE_TO_WINE:
      stepperX.moveTo(X_WINE_POSITION_APPROX);
      stepperX.runToPosition();
      currentState = SEALING_ALIGN_CAMERA;
      break;

    case SEALING_ALIGN_CAMERA:
      Serial.println("A"); 
      Serial.flush();
      if (camera_Align()) {
        retryCount = 0;
        Sealed_Wine_PositionX = stepperX.currentPosition();
        currentState = SEALING_PLACE_LID;
      } else {
        currentState = FAILED_RETURN_LID;
      }
      break;

    case SEALING_PLACE_LID:
      stepperX.moveTo(Sealed_Wine_PositionX + CAMERA_MAGNET_OFFSET);
      stepperX.runToPosition();
      stepperZ.moveTo(Z_LID_INSERT_HEIGHT);
      stepperZ.runToPosition();
      controlElectromagnet(false);
      delay(500);
      moveZ_toHome();
      currentState = SEALING_APPLY_VACUUM;
      break;

    case SEALING_APPLY_VACUUM:
      stepperX.moveTo(Sealed_Wine_PositionX - CAMERA_VACUUM_OFFSET);
      stepperX.runToPosition();
      stepperZ.moveTo(Z_LID_INSERT_HEIGHT);
      stepperZ.runToPosition();
      Vacuum_pump();
      isSealed = true; 
      moveZ_toHome();
      Serial.println("F"); 
      Serial.flush();
      currentState = HOMING; 
      break;

    case OPENING_MOVE_TO_WINE:
      moveZ_toHome();
      stepperX.moveTo(Sealed_Wine_PositionX + CAMERA_MAGNET_OFFSET);
      stepperX.runToPosition();
      currentState = OPENING_GET_LID;
      break;

    case OPENING_GET_LID:
      stepperZ.moveTo(Z_LID_OPEN_HEIGHT);
      stepperZ.runToPosition();
      controlElectromagnet(true);
      currentState = OPENING_BREAK_VACUUM;
      break;

    case OPENING_BREAK_VACUUM:
      stepperZ.setMaxSpeed(OPEN_LID_SPEED);
      stepperZ.setAcceleration(OPEN_LID_ACCEL);
      stepperZ.moveTo(-15 * 16); stepperZ.runToPosition();
      delay(100);
      stepperZ.moveTo(5 * 16); stepperZ.runToPosition();
      stepperZ.setMaxSpeed(NORMAL_SPEED);
      stepperZ.setAcceleration(NORMAL_ACCEL);
      moveZ_toHome();
      currentState = OPENING_RETURN_LID;
      break;

    case OPENING_RETURN_LID:
      stepperX.moveTo(X_LID_ROOM_POSITION);
      stepperX.runToPosition();
      stepperZ.moveTo(Z_LID_ROOM_HEIGHT);
      stepperZ.runToPosition();
      controlElectromagnet(false);
      isSealed = false; 
      Sealed_Wine_PositionX = 0;
      Serial.println("F");
      Serial.flush();
      currentState = HOMING;
      break;
      
    case FAILED_RETURN_LID:
      isSealed = false;
      moveZ_toHome();
      stepperX.moveTo(X_LID_ROOM_POSITION);
      stepperX.runToPosition();
      stepperZ.moveTo(Z_LID_ROOM_HEIGHT);
      stepperZ.runToPosition();
      controlElectromagnet(false);
      
      if (retryCount < MAX_RETRIES) {
        retryCount++;
        delay(1000);
        currentState = SEALING_ALIGN_Y;
      } else {
        retryCount = 0;
        Sealed_Wine_PositionX = 0;
        Serial.println("F"); 
        Serial.flush();
        currentState = HOMING;
      }
      break;

    default:
      currentState = HOMING;
      break;
  }
}