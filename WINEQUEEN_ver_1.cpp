#include <AccelStepper.h>
// 스텝 모터 핀
const int X_DIR_PIN = 7;
const int X_STEP_PIN = 8;
const int X_ENA_PIN = 9;
const int Z_DIR_PIN = 10;
const int Z_STEP_PIN = 11;
const int Z_ENA_PIN = 12;
// 리니어 모터 드라이버 핀
const int LINEAR_IN2 = 13;
const int LINEAR_ENA = A0;
const int LINEAR_IN1 = A1;
// 릴레이 핀
const int VACUUM_PUMP_PIN = A2;
const int ELECTROMAGNET_PIN = A3;
// 버튼 핀
const int OPEN_BUTTON_PIN = A4;
const int SEAL_BUTTON_PIN = A5;

int sealButtonState = HIGH;
int lastSealButtonState = HIGH;
unsigned long lastSealDebounceTime = 0;
int openButtonState = HIGH;
int lastOpenButtonState = HIGH;
unsigned long lastOpenDebounceTime = 0;
const unsigned long debounceDelay = 50;

const int LINEAR_MOTOR_SPEED = 250;
const unsigned long PUSH_DURATION = 1200;
const unsigned long RETURN_DURATION = 1200;

// 위치 및 거리 상수 정의
const long Z_TOP_POSITION = -453*16; 
const long Z_BOTTOM_POSITION = 0; 
const long X_INITIAL_POSITION = 0;
const long X_LID_ROOM_POSITION = -165 * 16;
const long Z_LID_PICKUP_APPROACH = 0;
const long X_CAMERA_OFFSET = -155 * 16;
const long Z_LID_INSERT_DEPTH = 10 * 16;
const long Z_VACUUM_TUBE_DEPTH = 0; 
const long X_WINE_POSITION = -680*16; // 대략적인 위치
long g_finalWinePositionX = 0; // 밀봉 후 와인 위치 저장용 변수

// --- 시스템 상태 정의 ---
enum SystemState { IDLE, SEALING, OPENING };
SystemState currentState = IDLE;

// 객체 생성
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

void setup() {
  Serial.begin(9600);
  
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  pinMode(VACUUM_PUMP_PIN, OUTPUT);
  pinMode(LINEAR_ENA, OUTPUT);
  pinMode(LINEAR_IN1, OUTPUT);
  pinMode(LINEAR_IN2, OUTPUT);
  // pinMode(Z_LIMIT_SWITCH_PIN, INPUT_PULLUP); // 리미트 스위치 코드 주석 처리
  // pinMode(X_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SEAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(OPEN_BUTTON_PIN, INPUT_PULLUP);
  
  digitalWrite(ELECTROMAGNET_PIN, LOW);
  digitalWrite(VACUUM_PUMP_PIN, LOW);
  digitalWrite(LINEAR_IN1, LOW);
  digitalWrite(LINEAR_IN2, LOW);
  analogWrite(LINEAR_ENA, 0);

  pinMode(Z_ENA_PIN, OUTPUT);
  digitalWrite(Z_ENA_PIN, LOW);
  pinMode(X_ENA_PIN, OUTPUT);
  digitalWrite(X_ENA_PIN, LOW); // X축 활성화
  stepperZ.setMaxSpeed(8000 * 16);
  stepperZ.setAcceleration(5000 * 16);
  stepperX.setMaxSpeed(4000 * 16);
  stepperX.setAcceleration(3000 * 16);
  
  // ★★★ 수정: 전원을 켰을 때의 물리적 위치를 소프트웨어 원점으로 설정 ★★★
  stepperZ.setCurrentPosition(Z_BOTTOM_POSITION);
  stepperX.setCurrentPosition(X_INITIAL_POSITION);

  Serial.println("System Ready. Waiting for button input.");
}
void motorsEnable() {
  //Serial.println("Enabling motor power...");
  digitalWrite(Z_ENA_PIN, LOW);
  digitalWrite(X_ENA_PIN, LOW);
  delay(30);
}
void motorsDisable() {
  //Serial.println("Disabling motor power to save energy.");
  digitalWrite(Z_ENA_PIN, HIGH);
  digitalWrite(X_ENA_PIN, HIGH);
}

// ★★★ 수정: 리미트 스위치 없이 고정된 위치로 이동하는 함수 ★★★
void moveZ_toTop() {
  Serial.println("Moving Z-Axis to Top Position...");
  stepperZ.moveTo(Z_TOP_POSITION);
  stepperZ.runToPosition();
  Serial.println("Z-Axis is at Top.");
}
void moveX_toWine() {
  Serial.println("와인에게 이동중...");
  stepperX.moveTo(X_WINE_POSITION);
  stepperX.runToPosition();
  Serial.println("와인 위에 도착.");
}

// ★★★ 수정: 리미트 스위치 없이 고정된 위치로 이동하는 함수 ★★★
void initialize() {
  motorsEnable(); // ★★★ 추가: 움직이기 전에 모터 전원 ON
  //Serial.println("Returning to initial position...");
  moveZ_toTop();
  // 인터럽트 초기화
  stepperX.moveTo(-100*16);
  stepperX.runToPosition();
  stepperX.moveTo(X_INITIAL_POSITION);
  stepperX.runToPosition();
  stepperZ.moveTo(Z_BOTTOM_POSITION);
  stepperZ.runToPosition();
  //erial.println("Initialization complete.");
}

void linearMotor_align() {
  Serial.println("Aligning wine bottle...");
  digitalWrite(LINEAR_IN1, LOW);
  delay(500);
  digitalWrite(LINEAR_IN2, HIGH);
  delay(PUSH_DURATION);

  digitalWrite(LINEAR_IN1, HIGH);
  digitalWrite(LINEAR_IN2, LOW);
  analogWrite(LINEAR_ENA, LINEAR_MOTOR_SPEED);
  delay(RETURN_DURATION);

  
  digitalWrite(LINEAR_IN1, LOW);
  digitalWrite(LINEAR_IN2, LOW);
  analogWrite(LINEAR_ENA, 0);
  Serial.println("Alignment complete.");
}

void returnLid() {
  Serial.println("!!! Alignment Failed. Returning lid to storage...");
  moveZ_toTop();
  stepperX.moveTo(X_LID_ROOM_POSITION);
  stepperX.runToPosition();
  stepperZ.moveTo(Z_LID_PICKUP_APPROACH);
  stepperZ.runToPosition();
  controlElectromagnet(false);
  moveZ_toTop();
  Serial.println("!!! Lid returned.");
}

bool cameraAlignX() {
  Serial.println("X-Axis alignment ('R':Right, 'L':Left, 'C':Center)...");
  unsigned long startTime = millis();
  while (millis() - startTime < 300000) {
    if (Serial.available() > 0) {
      char command = Serial.read();
      switch (command) {
        case 'R': stepperX.move(-3 * 16); break;
        case 'L': stepperX.move(3 * 16); break;
        case 'C':
          stepperX.stop();
          Serial.println("  -> Camera Alignment Finished.");
          return true;
      }
    }
    stepperX.run();
  }
  Serial.println("  -> Alignment Timed Out!");
  return false;
}

void controlElectromagnet(bool power) {
  digitalWrite(ELECTROMAGNET_PIN, power ? HIGH : LOW);
  //Serial.println(power ? "Electromagnet ON" : "Electromagnet OFF");
  delay(500);
}

void applyVacuum() {
  //Serial.println("Vacuum Pump ON.");
  digitalWrite(VACUUM_PUMP_PIN, HIGH);
  delay(15000);
  digitalWrite(VACUUM_PUMP_PIN, LOW);
  //Serial.println("Vacuum Pump OFF.");
}

void sealProcess() {
  currentState = SEALING;
  Serial.println("\n--- Starting Seal Process ---");
  //Serial.println("Step 1: Aligning Y-Axis...");
  
  //linearMotor_align();
  delay(1500);
  
  //Serial.println("Step 2: Moving to lid area...");
  moveZ_toTop();
  stepperX.moveTo(X_LID_ROOM_POSITION);
  stepperX.runToPosition();
  //Serial.println("Step 3: Acquiring lid...");
  stepperZ.moveTo(Z_LID_PICKUP_APPROACH);
  stepperZ.runToPosition();
  controlElectromagnet(true);
  moveZ_toTop();
  moveX_toWine();

  Serial.println("A");
  Serial.flush();
  
  Serial.println("Step : Aligning X-Axis...");
  bool alignmentSuccess = cameraAlignX();

  if (alignmentSuccess) {
    g_finalWinePositionX = stepperX.currentPosition(); // 밀봉 후 와인 위치 저장
    //Serial.println("Step 5: Placing lid...");
    stepperX.moveTo(g_finalWinePositionX + X_CAMERA_OFFSET - 10*16);
    stepperX.runToPosition();
    stepperZ.moveTo(Z_LID_INSERT_DEPTH);
    stepperZ.runToPosition();
    controlElectromagnet(false);
    
  //  Serial.println("Step 6: Preparing for vacuum...");
    moveZ_toTop();
    stepperX.moveTo(g_finalWinePositionX - X_CAMERA_OFFSET + 20*16);
    stepperX.runToPosition();
    
   // Serial.println("Step 7: Applying vacuum...");
    stepperZ.moveTo(Z_VACUUM_TUBE_DEPTH);
    stepperZ.runToPosition();
    applyVacuum();
    
   // Serial.println("Step 8: Finalizing...");
    initialize();
  } else {
    returnLid();
    initialize();
  }

  Serial.println("--- Seal Process Finished ---");
  Serial.println("F");
  Serial.flush();
  currentState = IDLE;
}

void openProcess() {
  // ★★★ 추가: 저장된 위치가 유효한지 확인하는 안전 코드 ★★★
  if (g_finalWinePositionX == 0) { // 밀봉이 한 번도 실행되지 않았다면
  Serial.println("오류: 저장된 와인 위치가 없습니다. 먼저 와인을 밀봉해주세요.");
  return; // 개봉 프로세스를 즉시 중단
  }
  currentState = OPENING;
 // Serial.println("\n--- Starting Open Process ---");

  // 1. Z축 상승
  moveZ_toTop();

  // 2. 뚜껑 위치로 바로 이동
  stepperX.moveTo(g_finalWinePositionX + X_CAMERA_OFFSET);
  stepperX.runToPosition();

  // 3. Z축 하강
 //Serial.println("Step 2: Picking up lid...");
  stepperZ.moveTo(Z_LID_INSERT_DEPTH);
  stepperZ.runToPosition();
  long original_Z_Speed = stepperZ.maxSpeed();
  long original_Z_Accel = stepperZ.acceleration();
  // 4. 전자석 ON
  controlElectromagnet(true);
  stepperZ.setMaxSpeed(25*16);  // Z축도 같은 속도로 설정
  stepperZ.setAcceleration(25*16);
  // 5. 왕복 운동으로 진공 해제
  //Serial.println("Step 3: Breaking vacuum seal...");
  stepperZ.move(5 * 16);
  stepperZ.runToPosition();
  stepperZ.move(-20 * 16);
  stepperZ.runToPosition();
  stepperZ.move(5 * 16);
  stepperZ.runToPosition();
  stepperZ.move(-20 * 16);
  stepperZ.runToPosition();

  stepperZ.maxSpeed();
  stepperZ.acceleration();

  // 6. Z축 상승 (뚜껑 들기)
  moveZ_toTop();

  // 7. 뚜껑 보관 장소로 이동
  //Serial.println("Step 4: Returning lid to storage...");
  stepperX.moveTo(X_LID_ROOM_POSITION);
  stepperX.runToPosition();
  
  // 8. 전자석 OFF (뚜껑 놓기)
  stepperZ.moveTo(Z_LID_PICKUP_APPROACH); // 뚜껑 놓을 높이로 이동
  stepperZ.runToPosition();
  controlElectromagnet(false);
  
  // 9. 종료,초기화
  //Serial.println("Step 5: Finalizing...");
  initialize();

  Serial.println("--- Open Process Finished ---");
  currentState = IDLE;
  Serial.println("F");
  Serial.flush();
}

bool isSealButtonPressed() {
  int reading = digitalRead(SEAL_BUTTON_PIN);
  if (reading != lastSealButtonState) { lastSealDebounceTime = millis(); }
  if ((millis() - lastSealDebounceTime) > debounceDelay) {
    if (reading != sealButtonState) {
      sealButtonState = reading;
      if (sealButtonState == LOW) {
        lastSealButtonState = reading;
        Serial.println("Seal Button Pressed.");
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
        Serial.println("Open Button Pressed.");
        return true;
      }
    }
  }
  lastOpenButtonState = reading;
  return false;
}

void loop() {
  if (currentState == IDLE) {
    if (isSealButtonPressed()) {
      Serial.println("1");
      Serial.flush(); // ← 이게 필요함
      motorsEnable();  
      sealProcess();
      motorsDisable();
    }
    if (isOpenButtonPressed()) {
      Serial.println("2");
      Serial.flush();
      motorsEnable();
      openProcess();
      motorsDisable();
    }
  }
}