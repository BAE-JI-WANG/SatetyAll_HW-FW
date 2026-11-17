#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

SparkFun_APDS9960 apds = SparkFun_APDS9960(); // 근접센서 라이브러리

uint8_t proximity_data = 0; // 근접센서 초기화
const int MPU_ADDR = 0x68;    // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, prev_AcX, prev_AcY, prev_AcZ, prev_Tmp, prev_GyX, prev_GyY, prev_GyZ;   // 가속도(Acceleration)와 자이로(Gyro)
double angleAcX, angleAcY, angleAcZ, prev_angleAcX, prev_angleAcY, prev_angleAcZ; // 가속도 값 저장
double angleGyX, angleGyY, angleGyZ, prev_angleGyX, prev_angleGyY, prev_angleGyZ; // 자이로 값 저장
double angleFiX, angleFiY, angleFiZ, prev_angleFiX, prev_angleFiY, prev_angleFiZ; // 필터링된 x,y,z축 값 저장
boolean equip; // 착용상태 변수
boolean movement; // 움직임 감지 변수
int count = 0;
int angleChange = 0; // 자이로 3축 합성 각속도 크기(°/s) (이름과 달리 각도(°)가 아님)

boolean fall = false; // 낙상이 감지되면 저장
boolean fallAlert = false; // 낙상 확정 후 헬멧을 벗을 때까지 앱에 'F'를 계속 전송하기 위한 상태
int unwornLoops = 0; // 낙상 알림 중 연속으로 미착용이 감지된 루프 횟수
boolean trigger1 = false; // 첫번째 트리거(하한 임계값)가 발생한 경우 저장
boolean trigger2 = false; // 두번째 트리거(상한 임계값)가 발생한 경우 저장
boolean trigger3 = false; // 세번째 트리거(방향 변경)가 발생한 경우 저장
byte trigger1count = 0; // 트리거 1이 true로 설정된 이후 경과된 카운트를 저장
byte trigger2count = 0; // 트리거 2이 true로 설정된 이후 경과된 카운트를 저장
byte trigger3count = 0; // 트리거 3이 true로 설정된 이후 경과된 카운트를 저장

float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; // 3축 진폭 벡터 계산값 저장

const double RADIAN_TO_DEGREE = 180 / 3.14159;  
const double DEG_PER_SEC = 32767 / 250;    // 1초에 회전하는 각도
const double ALPHA = 1 / (1 + 0.04);
// GyX, GyY, GyZ 값의 범위 : -32768 ~ +32767 (16비트 정수범위)


unsigned long now = 0;   // 현재 시간 저장용 변수
unsigned long past = 0;  // 이전 시간 저장용 변수
double dt = 0;           // 한 사이클 동안 걸린 시간 변수 

double averAcX, averAcY, averAcZ; // 가속도의 평균 값
double averGyX, averGyY, averGyZ; // 자이로의 평균 값
double prev_sv; // 알고리즘 계산 값 저장

BluetoothSerial SerialBT; // 블루투스 시리얼 라이브러리

// ===================== 튜닝 파라미터 =====================
// ※ 임계값 재측정/조정 필요
//   아래 값들은 이전 로직 기준으로 실제 낙상 테스트를 하며 맞춘 값이며, 이 버전에서 값 자체는 바꾸지 않았다.
//   다만 판정 경로가 달라졌으므로(착용 중에만 낙상 시작, 낙상 확정 후 'F' 유지 전송 등)
//   배포 전에 실제 낙상/일상동작 테스트로 아래 항목을 다시 확인하고 조정해야 한다.
//   - 낙상: AMP_FREEFALL_MAX, AMP_IMPACT_MIN, ROTATION_MIN_DPS, ROTATION_MAX_DPS, STILL_MAX_DPS, TRIGGER*_LOOPS
//   - 움직임/'W': MOVE_ACC_THRESHOLD, MOVE_GYRO_THRESHOLD, MOVE_MIN_AXES, NO_MOVE_LOOPS
//   - 센서 오프셋(ACC/GYRO_OFFSET_*): 센서(기기)마다 다르므로 기기별로 다시 측정
// 판정에 쓰는 기준값을 한곳에 모음 (값 자체는 기존 코드와 동일)
// 시간 기준은 loop() 1회 = LOOP_DELAY_MS(100ms) + 센서/통신 처리 시간의 '루프 횟수'

// 센서 오프셋 보정값 (기기별로 실측한 원시값)
const int ACC_OFFSET_X = 2050;
const int ACC_OFFSET_Y = 77;
const int ACC_OFFSET_Z = 1947;
const int GYRO_OFFSET_X = -270;
const int GYRO_OFFSET_Y = 351;
const int GYRO_OFFSET_Z = -136;
// 단위 변환 계수 (MPU6050 기본 측정 범위: 가속도 ±2g, 자이로 ±250°/s)
const double ACC_LSB_PER_G = 16384.00;
const double GYRO_LSB_PER_DPS = 131.07;

// 착용 판정: 근접센서 값이 이 값을 초과하면 착용
const int WEAR_PROXIMITY_MIN = 100;

// 움직임 판정: 기준값(prev_*) 대비 변화량(원시값) 임계값
const float MOVE_ACC_THRESHOLD = 200;
const float MOVE_GYRO_THRESHOLD = 50;
// 움직임으로 인정하기 위해 임계값을 넘어야 하는 축 수 (1~6). 6이면 6축 모두 넘어야 하는 기존 동작
// ※ 임계값이 센서 노이즈 수준(가속도 200 ≈ 0.012g, 자이로 50 ≈ 0.4°/s)이라, 값을 낮추면 정지 상태 노이즈도 움직임으로 판정될 수 있음
//   낮추려면 임계값도 함께 실측해서 다시 맞출 것
const int MOVE_MIN_AXES = 6;
// 이 횟수 이상 루프가 지난 뒤에도 움직임이 없으면 'W' 경고 전송 (약 30초 이상)
const int NO_MOVE_LOOPS = 300;

// 낙상 판정 (Amp는 합성 가속도 g * 10, 각속도 단위는 °/s)
const int AMP_FREEFALL_MAX = 2;       // 트리거1: Amp 이하 (자유낙하)
const int AMP_IMPACT_MIN = 12;        // 트리거2: Amp 이상 (충격)
const int ROTATION_MIN_DPS = 30;      // 트리거3: 충격 직후 회전 각속도 하한
const int ROTATION_MAX_DPS = 400;     // 트리거3: 충격 직후 회전 각속도 상한
const int STILL_MAX_DPS = 10;         // 낙상 확정: 이 각속도 이하면 정지로 판단
const int TRIGGER1_TIMEOUT_LOOPS = 6;  // 트리거1 이후 충격을 기다리는 루프 수
const int TRIGGER2_TIMEOUT_LOOPS = 6;  // 트리거2 이후 회전을 기다리는 루프 수
const int TRIGGER3_HOLD_LOOPS = 10;    // 트리거3 이후 정지 여부를 판단하기까지의 루프 수
const int FALL_ALERT_CLEAR_LOOPS = 10; // 낙상 알림 해제: 미착용이 이만큼 연속될 때 (근접센서 순간 오검출로 알림이 꺼지는 것을 방지)

const unsigned long LOOP_DELAY_MS = 100; // loop() 끝에서 대기하는 시간

void setup() 
{
  initSensor(); // 센서 초기화 함수
  Serial.begin(115200); // 시리얼통신 보더레이트
  SerialBT.begin("smartHelmet_v1.0.4"); // 블루투스 디바이스 이름
  caliSensor();   // 초기 센서 캘리브레이션 함수 호출
  past = millis(); // past에 현재 시간 저장
  getData(); // 자이로 센서 데이터 필터링 함수

  prev_AcX = AcX; // 셋업 당시 가속도 X 센서 값
  prev_AcY = AcY; // 셋업 당시 가속도 Y 센서 값                          

  prev_GyX = GyX; // 셋업 당시 자이로 X 센서 값
  prev_GyY = GyY; // 셋업 당시 자이로 Y 센서 값
  prev_GyZ = GyZ; // 셋업 당시 자이로 Z 센서 값

  
  apds.setMode(ALL, true);  

  // 근접센서 (APDS-9960) 초기화 (I2C 통신 구성과 초기값 설정)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // 근접 센서 gain 조정
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // APDS-9960 근접 센서 동작 실행 (인터럽트 x) 
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
}

// 기준값(prev_*) 대비 변화량(원시값)이 임계값(가속도 MOVE_ACC_THRESHOLD, 자이로 MOVE_GYRO_THRESHOLD) 이상인 축이
// MOVE_MIN_AXES개 이상이면 움직임 있음(true). 기본값 6은 "6축 모두 초과해야 움직임"인 기존 동작과 같음
// (실측으로 튜닝된 동작이므로 기본값은 바꾸지 않음. 한 축만 움직이는 동작(예: 고개만 돌림)은 정지로 판정되어 'W'가 나갈 수 있음)
boolean check_movement(float ch_AcX, float ch_AcY, float ch_AcZ, float ch_GyX, float ch_GyY, float ch_GyZ)
{
  int moving_axes = 0;
  if(ch_AcX >= MOVE_ACC_THRESHOLD) moving_axes++;
  if(ch_AcY >= MOVE_ACC_THRESHOLD) moving_axes++;
  if(ch_AcZ >= MOVE_ACC_THRESHOLD) moving_axes++;
  if(ch_GyX >= MOVE_GYRO_THRESHOLD) moving_axes++;
  if(ch_GyY >= MOVE_GYRO_THRESHOLD) moving_axes++;
  if(ch_GyZ >= MOVE_GYRO_THRESHOLD) moving_axes++;
  return moving_axes >= MOVE_MIN_AXES;
}

// 기준값(prev_*)과 현재 센서값이 6개 축 모두 같으면 true, 하나라도 다르면 false
// (기존 else if 구조는 모든 경로에서 값을 반환하지 않아 정의되지 않은 동작이 될 수 있어, 결과는 같게 한 줄로 정리)
boolean compare(float AcX, float AcY, float AcZ, float GyX, float GyY, float GyZ)
{
  return prev_AcX == AcX && prev_AcY == AcY && prev_AcZ == AcZ && prev_GyX == GyX && prev_GyY == GyY && prev_GyZ == GyZ;
}

// 낙상 판정용 트리거 상태 초기화 (센서 읽기 실패 시 이전 트리거가 이어지지 않도록 사용)
void resetFallTriggers() {
  trigger1 = false;
  trigger2 = false;
  trigger3 = false;
  trigger1count = 0;
  trigger2count = 0;
  trigger3count = 0;
}

// 앱에 낙상('F')을 전달
// 앱은 수신한 메시지마다 상태 문구를 덮어쓰고, ':'로 나눈 두 번째 항목(arr[1])에서 'F'를 찾으므로
// 'F'는 같은 사이클의 상태 메시지(E/N/W) 바로 뒤에 매 사이클 반복해서 보내야 화면에 유지됨
void sendFallAlert() {
  SerialBT.write('F');
  SerialBT.print(":");
  SerialBT.print("\n");
}

void loop() {
  
  count++;
  
  // I2C 수신에 실패하면 이전 센서값(또는 -1)으로 낙상/움직임을 판정하지 않도록 이번 사이클은 건너뜀
  // 건너뛰는 동안 count는 계속 증가하고, 이어지던 낙상 트리거는 초기화됨
  if (!getData()) {
    Serial.println("MPU6050 read failed - skip this cycle");
    resetFallTriggers();
    delay(LOOP_DELAY_MS); // 정상 사이클과 같은 주기 유지
    return;
  }
  getDT();

  // 실측으로 구한 센서별 오프셋을 보정한 뒤 단위 변환 (낙상 임계값이 이 보정값 기준으로 튜닝되어 있음)
  // 가속도: 16384 LSB/g (±2g), 자이로: 131.07 LSB/(°/s) (±250°/s) - initSensor()의 기본 측정 범위 기준
  ax = (AcX - ACC_OFFSET_X) / ACC_LSB_PER_G;
  ay = (AcY - ACC_OFFSET_Y) / ACC_LSB_PER_G;
  az = (AcZ - ACC_OFFSET_Z) / ACC_LSB_PER_G;
  gx = (GyX - GYRO_OFFSET_X) / GYRO_LSB_PER_DPS;
  gy = (GyY - GYRO_OFFSET_Y) / GYRO_LSB_PER_DPS;
  gz = (GyZ - GYRO_OFFSET_Z) / GYRO_LSB_PER_DPS;
  // 3축에 대한 진폭 백터 계산
  
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5); // 가속도 3축 합성 크기 (단위: g)
  int Amp = Raw_Amp * 10;  // 0.1g 단위 정수(소수점 절삭). 예: Amp 2 -> 0.2g 이상 0.3g 미만, Amp 12 -> 1.2g 이상 1.3g 미만

  boolean gyro_status = compare(AcX, AcY, AcZ, GyX, GyY, GyZ); // 자이로센서의 현재값과 초기값을 비교하여 저장
  
  float ch_accX = fabs(AcX-prev_AcX); //가속도 x의 변화량
  float ch_accY = fabs(AcY-prev_AcY); //가속도 y의 변화량
  float ch_accZ = fabs(AcZ-prev_AcZ); //가속도 z의 변화량

  float ch_gyX = fabs(GyX-prev_GyX); //자이로 x의 변화량
  float ch_gyY = fabs(GyY-prev_GyY); //자이로 y의 변화량
  float ch_gyZ = fabs(GyZ-prev_GyZ); //자이로 z의 변화량

  movement = check_movement(ch_accX, ch_accY, ch_accZ, ch_gyX, ch_gyY, ch_gyZ); // 움직임 상태 저장

  if ( !apds.readProximity(proximity_data) ) {
    Serial.println("Error reading proximity value");
  } else {
    if(proximity_data>WEAR_PROXIMITY_MIN)
    {
      equip = true;
    }
    else     
    {
      equip = false;
    }
  }
  Serial.print("movement : ");
  Serial.println(movement);

  if(equip == true) // 착용중이고
  {
    
    if(count>=NO_MOVE_LOOPS) // count가 NO_MOVE_LOOPS(300) 이상일 동안 (루프 1회 약 100ms 이상이므로 약 30초 이상)
    {
     
      if(movement == false) // 움직임이 없다면
      {
        // 위험 상태이므로 움직임이 감지될 때까지 매 사이클 반복 전송하는 것이 의도된 동작
        SerialBT.write('W'); // 블루투스로 'W' 경고상태 전달
        SerialBT.print(":"); 
        Serial.println('W');
        
      }
      else if(movement == true) // 움직임이 있다면
      {
        count = 0; // count를 0으로 초기화
        if(gyro_status == false) // 센서값 비교가 false라면
        {
          // 센서 초기값을 현재값으로 초기화
          prev_AcX = AcX; 
          prev_AcY = AcY;
          prev_AcZ = AcZ;
          prev_GyX = GyX;
          prev_GyY = GyY;
          prev_GyZ = GyZ;
        }
      }
    }
    else
    {
      SerialBT.write('E'); // 블루투스를 통하여 'E' 착용상태 전달
      SerialBT.print(":");
      Serial.println('E');
    }
  }
  
  else if(equip == false) // 착용상태가 false라면 
  {
    SerialBT.write('N'); // 블루투스를 통하여 'N' 미착용상태 전달
    SerialBT.print(":");
    Serial.println('N');
    movement = true; // 미착용상태에서 경고를 방지하기 위하여 움직임을 true로 초기화
    count = 0; // count도 동시에 초기화
  }
  
  // 착용 중일 때만 낙상 판정을 시작: 벗어둔 헬멧을 떨어뜨리거나 옮기다 놓친 경우가 낙상으로 오검출되는 것을 방지
  // (이미 시작된 판정은 낙상 중 헬멧이 밀려 근접센서 값이 바뀌어도 중단하지 않고 끝까지 진행)
  if (Amp <= AMP_FREEFALL_MAX && trigger2 == false && equip == true) { // 합성 가속도가 하한 임계값 이하 (Amp <= 2, 즉 0.3g 미만) - 실측 튜닝값(기존 주석의 0.4g는 실제 코드와 달라 정정)     
    trigger1 = true;     
    Serial.println("TRIGGER 1 ACTIVATED");   
  }
  if (trigger1 == true) {     
    trigger1count++;     
    if (Amp >= AMP_IMPACT_MIN) { // 합성 가속도가 상한 임계값 이상 (Amp >= 12, 즉 1.2g 이상) - 실측 튜닝값(기존 주석의 3g는 실제 코드와 달라 정정)
         trigger2 = true;
         Serial.println("TRIGGER 2 ACTIVATED");
         trigger1 = false; 
         trigger1count = 0;
     }
   }
   if (trigger2 == true) {
      trigger2count++;
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); 
      
      if (angleChange >= ROTATION_MIN_DPS && angleChange <= ROTATION_MAX_DPS) { // 합성 각속도가 30~400°/s 범위인 경우 (각도가 아닌 각속도 기준)   
        trigger3 = true;
        trigger2 = false; 
        trigger2count = 0;       
//        Serial.println(angleChange);       
        Serial.println("TRIGGER 3 ACTIVATED");     
    }   
  }   
  if (trigger3 == true) {     
    if (trigger3count < TRIGGER3_HOLD_LOOPS) trigger3count++; // byte 변수라 계속 증가시키면 255에서 0으로 넘어가 10루프 동안 판정/'F' 전송이 끊기므로 기준값에서 멈춤
      
    if (trigger3count >= TRIGGER3_HOLD_LOOPS) {
       angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
       //delay(10);
//       Serial.println(angleChange);
       
      if ((angleChange >= 0) && (angleChange <= STILL_MAX_DPS)) { // 합성 각속도가 0~10°/s 이하(거의 정지)로 유지되는 경우      
        fall = true; 
        trigger3 = false; 
        trigger3count = 0;         
//        Serial.println(angleChange);       
      }       
      else { // 합성 각속도가 10°/s를 초과하는 경우 (기존 주석: 사용자가 정상상태로 복귀)  
         
        // 충격 후 1초 이상 지났는데도 계속 움직이는 경우: 기존 동작대로 이 분기에서도 'F'를 전송
        // (움직임이 멈추면 아래 낙상 확정 분기로 넘어가며, 그때부터는 fallAlert가 'F'를 이어서 전송)
        Serial.println("TRIGGER 3 DEACTIVATED");
//        delay(1000);
        sendFallAlert();

        if(equip == false)
        {
          trigger3 = false; 
          trigger3count = 0;
        }
      }     
    }   
  }   
  if (fall == true) { // 낙상 확정 시 (충격 후 합성 각속도가 정지 수준으로 유지됨)
    Serial.println("FALL DETECTED");       
    fall = false;   
    fallAlert = true; // 기존에는 여기서 블루투스 전송이 없어, 쓰러져 움직이지 않는 상황이 앱에 전달되지 않았음
    unwornLoops = 0;
  }   
  if (fallAlert) {
    if (equip == true) {
      unwornLoops = 0;
    } else {
      unwornLoops++;
    }
    if (unwornLoops >= FALL_ALERT_CLEAR_LOOPS) { // 헬멧을 벗은 것으로 보이면 알림 해제
      fallAlert = false;
      unwornLoops = 0;
    } else {
      sendFallAlert(); // 앱이 메시지마다 상태를 덮어쓰므로 해제될 때까지 매 사이클 반복 전송
    }
  }   
  if (trigger2count >= TRIGGER2_TIMEOUT_LOOPS) { // 방향 변경 허용 시간: 루프 6회 (시간이 아닌 루프 횟수 기준이라 delay/통신 시간이 바뀌면 실제 시간도 달라짐)
     trigger2 = false; 
     trigger2count = 0;
     Serial.println("TRIGGER 2 DECACTIVATED");
   }
   if (trigger1count >= TRIGGER1_TIMEOUT_LOOPS) { // 상한 임계값을 넘기까지 허용 시간: 루프 6회 (약 0.6초 이상)
     trigger1 = false; 
     trigger1count = 0;
     Serial.println("TRIGGER 1 DECACTIVATED");
   }
   delay(LOOP_DELAY_MS);   
}

void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);   // I2C 통신용 어드레스(주소)
  Wire.write(0x6B);    // PWR_MGMT_1 레지스터(0x6B): 기본값이 슬립 모드이므로
  Wire.write(0);       // 0을 써서 슬립을 해제하고 측정을 시작 (측정 범위는 기본값 가속도 ±2g, 자이로 ±250°/s 사용)
  Wire.endTransmission(true);
}

// MPU6050에서 가속도/온도/자이로 14byte를 읽어 전역 변수에 저장
// 반환값: 정상 수신이면 true, I2C 수신 실패면 false (실패 시 전역 센서값은 바꾸지 않고 이전 값을 유지)
boolean getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // AcX 레지스터 위치(주소)를 지칭합니다
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) { // AcX 주소 이후의 14byte의 데이터를 요청
    return false; // 14byte를 받지 못하면 Wire.read()가 -1을 반환해 잘못된 센서값이 만들어지므로 읽지 않음
  }
  AcX = Wire.read() << 8 | Wire.read(); //두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  return true;
}

// loop 한 사이클동안 걸리는 시간을 알기위한 함수 (dt는 현재 낙상/움직임 판정에 사용되지 않음)
void getDT() {
  now = millis();   
  dt = (now - past) / 1000.0;  
  past = now;
}

// 센서의 초기값을 10회 정도 평균값으로 구하여 저장하는 함수
// 평균값(aver*)은 현재 판정에 사용되지 않음 (오프셋 보정은 loop()의 고정값 사용)
void caliSensor() {
  double sumAcX = 0 , sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0 , sumGyY = 0, sumGyZ = 0;
  getData();
  for (int i=0;i<10;i++) {
    getData();
    sumAcX+=AcX;  sumAcY+=AcY;  sumAcZ+=AcZ;
    sumGyX+=GyX;  sumGyY+=GyY;  sumGyZ+=GyZ;
    delay(50);
  }
  averAcX=sumAcX/10;  averAcY=sumAcY/10;  averAcZ=sumAcZ/10;
  averGyX=sumGyX/10;  averGyY=sumGyY/10;  averGyZ=sumGyZ/10;
}
