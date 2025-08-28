#include <Wire.h>
// #include <AccelStepper.h> // 스텝 모터 제어를 위한 AccelStepper 라이브러리
#include <XGZP6897D.h>    // 기압 센서 라이브러리

// --- 라즈베리파이 통신 신호 정의 ---
#define CMD_SEAL_START 'S'
#define CMD_OPEN_START 'O'
#define CMD_EMERGENCY_STOP 'E'
#define CMD_RETURN_HOME 'H'
#define POS_MOVE_LEFT '0'
#define POS_MOVE_RIGHT '1'
#define POS_CENTER_OK '2'
#define POS_NOT_FOUND '3'

// --- 하드웨어 핀 번호 설정 ---
// 1. X축 스텝 모터
#define X_STEP_PIN 2
#define X_DIR_PIN 3
// 2. Z축 스텝 모터 
#define Z_STEP_PIN 4
#define Z_DIR_PIN 5
const int stepsPerRevolution = 800;  
// #define Z_LIMIT_SWITCH_PIN 7 // Z축 상단 리미트 스위치

// 3. 전자석 (릴레이)
#define ELECTROMAGNET_PIN 9

// 4. 진공 펌프 (릴레이)
#define VACUUM_PUMP_PIN 10

// 5. 와인병 정렬용 리니어 모터 (DC모터)
#define BOTTLE_MOTOR_IN1 11
#define BOTTLE_MOTOR_IN2 12
#define BOTTLE_MOTOR_ENB 6 // PWM 제어용 핀으로 변경

XGZP6897D pressureSensor(128);

// --- 전역 변수 ---
const float TARGET_VACUUM_KPA = 20.0;
volatile boolean z_axis_homed = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("--- WINEQUEEN 제어 시스템 시작 ---");

  // 모터 초기화
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  pinMode(BOTTLE_MOTOR_IN1, OUTPUT);
  pinMode(BOTTLE_MOTOR_IN2, OUTPUT);
  pinMode(BOTTLE_MOTOR_ENB, OUTPUT);

  // 릴레이 초기화
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  pinMode(VACUUM_PUMP_PIN, OUTPUT);
  digitalWrite(ELECTROMAGNET_PIN, LOW);
  digitalWrite(VACUUM_PUMP_PIN, LOW);

  // 센서 초기화
  if (pressureSensor.begin()) Serial.println("기압 센서 초기화 성공");
  else Serial.println("기압 센서 초기화 실패! 연결을 확인하세요.");
  
  // Z축 인터럽트 설정
  pinMode(Z_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Z_LIMIT_SWITCH_PIN), zHomeISR, FALLING);
  Serial.println("Z축 리미트 스위치 인터럽트가 설정되었습니다.");

  // Z축 원점 복귀 (Homing)
  homeZAxis();
  
  Serial.println("시스템 준비 완료. 라즈베리파이의 명령을 대기합니다.");
}

void zHomeISR() {
  z_stepper.stop(); // 모터 정지
  z_stepper.setCurrentPosition(0); // 현재 위치를 원점(0)으로 설정
  z_axis_homed = true;
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case CMD_SEAL_START: runSealingProcess(); break;
      case CMD_OPEN_START: runOpeningProcess(); break;
      case CMD_RETURN_HOME: x_stepper.moveTo(0); homeZAxis(); break;
      case CMD_EMERGENCY_STOP:
        x_stepper.stop(); z_stepper.stop();
        controlElectromagnet(false); controlVacuumPump(false);
        digitalWrite(BOTTLE_MOTOR_IN1, LOW); digitalWrite(BOTTLE_MOTOR_IN2, LOW);
        Serial.println("!!! 긴급 정지 !!!");
        break;
      case POS_MOVE_LEFT: x_stepper.move(-5 * MICROSTEPS); break;
      case POS_MOVE_RIGHT: x_stepper.move(5 * MICROSTEPS); break;
    }
  }
  x_stepper.run();
  z_stepper.run();
}

// ======================= 세부 기능 함수들 =======================

void homeZAxis(){
    Serial.println("Z축 원점을 찾습니다...");
    z_axis_homed = false;
    z_stepper.setSpeed(1000); // 일정한 속도로 상승
    // ISR이 z_axis_homed를 true로 바꿀 때까지 대기
    while(!z_axis_homed){
        z_stepper.runSpeed();
    }
    Serial.println("Z축 원점 설정 완료.");
}

void alignBottleProcess() {
  Serial.println("1-1. 와인병을 V-block으로 밀어 정렬합니다.");
  digitalWrite(BOTTLE_MOTOR_IN1, HIGH); digitalWrite(BOTTLE_MOTOR_IN2, LOW);
  analogWrite(BOTTLE_MOTOR_ENB, 200);
  delay(2000);
  digitalWrite(BOTTLE_MOTOR_IN1, LOW); digitalWrite(BOTTLE_MOTOR_IN2, LOW);
  delay(500);
  Serial.println("1-2. 정렬 모터를 초기 위치로 복귀합니다.");
  digitalWrite(BOTTLE_MOTOR_IN1, LOW); digitalWrite(BOTTLE_MOTOR_IN2, HIGH);
  analogWrite(BOTTLE_MOTOR_ENB, 200);
  delay(2000);
  digitalWrite(BOTTLE_MOTOR_IN1, LOW); digitalWrite(BOTTLE_MOTOR_IN2, LOW);
}
// <밀봉 프로세스 (Sealing Process)>
// 라즈베리파이로부터 'S' 신호를 받으면 다음 순서로 작동합니다.
// 1. 와인병 정렬: 리니어 모터(DC모터)로 와인병을 V-block에 밀어 정렬시킨 후 복귀합니다.
// 2. 뚜껑 위치로 이동: X축 스텝 모터가 뚜껑 보관 장소로 이동합니다.
// 3. 뚜껑 잡기: Z축이 하강하고 전자석을 켜서 뚜껑을 잡은 뒤, Z축이 원점으로 상승합니다.
// 4. 와인병 위치로 이동: X축이 와인병 위로 이동하며, 라즈베리파이가 카메라로 최종 위치를 보정합니다.
// 5. 뚜껑 놓기: Z축이 하강하고 전자석을 꺼서 뚜껑을 놓은 뒤, Z축이 원점으로 상승합니다.
// 6. 진공관 위치로 이동: X축이 '전자석-진공관' 사이의 거리만큼 이동하여 진공관을 뚜껑 중앙에 위치시킵니다.
// 7. 진공 작업: Z축이 하강하여 뚜껑에 밀착한 뒤, 목표 압력에 도달할 때까지 진공 펌프를 작동시키고 Z축이 원점으로 상승합니다.
// 8. 초기 위치로 복귀: X축 스텝 모터가 초기 위치(원점)로 복귀합니다.
void runSealingProcess() {
  Serial.println("--- 밀봉 프로세스 시작 ---");
  alignBottleProcess(); delay(1000);

  Serial.println("2. 뚜껑 보관 장소로 이동합니다.");
  x_stepper.moveTo(/*<뚜껑 보관 장소의 X축 좌표(steps)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  
  Serial.println("3. 뚜껑을 잡습니다.");
  z_stepper.moveTo(/*<뚜껑 높이까지 하강할 Z축 좌표(steps)>*/);
  while(z_stepper.distanceToGo() != 0) z_stepper.run();
  controlElectromagnet(true); delay(1000);
  homeZAxis(); // 원점으로 상승

  Serial.println("4. 와인병 위치로 이동 후, 정밀 조정을 시작합니다.");
  x_stepper.moveTo(/*<와인병의 대략적인 X축 좌표(steps)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  delay(1000);

  Serial.println("5. 와인병 위에 뚜껑을 놓습니다.");
  z_stepper.moveTo(/*<와인병 높이까지 하강할 Z축 좌표(steps)>*/);
  while(z_stepper.distanceToGo() != 0) z_stepper.run();
  controlElectromagnet(false); delay(1000);
  homeZAxis();

  Serial.println("6. 진공관 위치로 이동합니다.");
  x_stepper.move(/*<'전자석-진공관' 사이의 고정된 거리(steps)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  
  Serial.println("7. 진공 작업을 시작합니다.");
  z_stepper.moveTo(/*<와인병 높이까지 하강할 Z축 좌표(steps)>*/);
  while(z_stepper.distanceToGo() != 0) z_stepper.run();
  controlVacuumPump(true);
  while (readPressureKPa() > TARGET_VACUUM_KPA) {
    Serial.print("현재 압력: "); Serial.println(readPressureKPa()); delay(100);
  }
  controlVacuumPump(false); Serial.println("목표 진공 압력에 도달했습니다.");
  delay(1000);
  homeZAxis();
  
  Serial.println("8. 초기 위치로 복귀합니다.");
  x_stepper.moveTo(0);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  Serial.println("--- 밀봉 프로세스 완료 ---");
}
// <개봉 프로세스 (Opening Process)>
// 라즈베리파이로부터 'O' 신호를 받으면 다음 순서로 작동합니다.
// 1. 와인병 위치로 이동: X축이 와인병 위로 이동하며, 라즈베리파이가 카메라로 최종 위치를 보정합니다.
// 2. 진공 해제: Z축이 하강하여 전자석으로 뚜껑을 잡고, X축을 좌우로 짧게 왕복 운동하여 뚜껑을 비틀어 진공을 해제합니다.
// 3. 뚜껑 제거: 전자석이 켜진 상태로 Z축이 원점으로 상승하여 뚜껑을 들어 올립니다.
// 4. 뚜껑 보관 위치로 이동: X축이 뚜껑 보관 장소로 이동합니다.
// 5. 뚜껑 놓기: Z축이 하강하고 전자석을 꺼서 뚜껑을 놓은 뒤, Z축이 원점으로 상승합니다.
// 6. 초기 위치로 복귀: X축 스텝 모터가 초기 위치(원점)로 복귀합니다.
void runOpeningProcess() {
  Serial.println("--- 개봉 프로세스 시작 ---");
  Serial.println("1. 와인병 위치로 이동 후, 정밀 조정을 시작합니다.");
  x_stepper.moveTo(/*<와인병의 대략적인 X축 좌표(steps)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  delay(1000);

  Serial.println("2. 뚜껑을 잡고 왕복 운동으로 진공을 해제합니다.");
  z_stepper.moveTo(/*<와인병 높이까지 하강할 Z축 좌표(steps)>*/);
  while(z_stepper.distanceToGo() != 0) z_stepper.run();
  controlElectromagnet(true); delay(1000);
  x_stepper.move(/*<진공 해제를 위한 X축 이동 거리(steps)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  x_stepper.move(/*<원위치 복귀를 위한 X축 이동 거리(음수값)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();

  Serial.println("3. 진공 해제된 뚜껑을 들어 올립니다.");
  homeZAxis();
  
  Serial.println("4. 뚜껑 보관 장소로 이동합니다.");
  x_stepper.moveTo(/*<뚜껑 보관 장소의 X축 좌표(steps)>*/);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();

  Serial.println("5. 뚜껑을 보관 장소에 놓습니다.");
  z_stepper.moveTo(/*<뚜껑 높이까지 하강할 Z축 좌표(steps)>*/);
  while(z_stepper.distanceToGo() != 0) z_stepper.run();
  controlElectromagnet(false); delay(1000);
  homeZAxis();

  Serial.println("6. 초기 위치로 복귀합니다.");
  x_stepper.moveTo(0);
  while(x_stepper.distanceToGo() != 0) x_stepper.run();
  Serial.println("--- 개봉 프로세스 완료 ---");
}

// ------------------- 센서 및 액추에이터 제어 파트 -------------------
void controlElectromagnet(bool on) {
  if (on) { digitalWrite(ELECTROMAGNET_PIN, HIGH); Serial.println("전자석 ON"); }
  else { digitalWrite(ELECTROMAGNET_PIN, LOW); Serial.println("전자석 OFF"); }
}

void controlVacuumPump(bool on) {
  if (on) { digitalWrite(VACUUM_PUMP_PIN, HIGH); Serial.println("진공 펌프 ON"); }
  else { digitalWrite(VACUUM_PUMP_PIN, LOW); Serial.println("진공 펌프 OFF"); }
}

float readPressureKPa() {
  float temp, pressurePa;
  if (pressureSensor.readSensor(temp, pressurePa)) return pressurePa / 1000.0;
  else { Serial.println("압력 센서 읽기 실패!"); return -1.0; }
}