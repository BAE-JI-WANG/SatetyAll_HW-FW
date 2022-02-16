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
int angleChange = 0; // 각도 변화량 초기화

boolean fall = false; // 낙상이 감지되면 저장
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

boolean check_movement(float ch_AcX, float ch_AcY, float ch_AcZ, float ch_GyX, float ch_GyY, float ch_GyZ)
{
  if(ch_AcX<200 || ch_AcY<200 || ch_AcZ<200 || ch_GyX<50 || ch_GyY<50 || ch_GyZ<50) // 자이로 변화가 없을때
  {
    return false;
  }
  else
  {
    return true;
  }
}

boolean compare(float AcX, float AcY, float AcZ, float GyX, float GyY, float GyZ)
{
  if(prev_AcX != AcX || prev_AcY != AcY || prev_AcZ != AcZ || prev_GyX != GyX || prev_GyY != GyY || prev_GyZ != GyZ) // 셋업당시 센서값과 현재 센서값이 다르다면
  {
    return false;
  }
  else if(prev_AcX == AcX && prev_AcY == AcY && prev_AcZ == AcZ && prev_GyX == GyX && prev_GyY == GyY && prev_GyZ == GyZ)
  {
    return true;
  }
}

void loop() {
  
  count++;
  
  getData(); 
  getDT();

  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
  // 3축에 대한 진폭 백터 계산
  
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5); // 로우데이터의 증폭값
  int Amp = Raw_Amp * 10;  // 10을 곱한 값은 0에서 1사이

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
    if(proximity_data>100)
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
    
    if(count>=300) // count가 300이상일 동안
    {
     
      if(movement == false) // 움직이 없다면
      {
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
  
  if (Amp <= 2 && trigger2 == false) { // 증폭값이 더 낮은 임계값을 깨는 경우 (0.4g)     
    trigger1 = true;     
    Serial.println("TRIGGER 1 ACTIVATED");   
  }
  if (trigger1 == true) {     
    trigger1count++;     
    if (Amp >= 12) { // 증폭값이 더 높은 임계값을 깨는 경우 (3g)
         trigger2 = true;
         Serial.println("TRIGGER 2 ACTIVATED");
         trigger1 = false; 
         trigger1count = 0;
     }
   }
   if (trigger2 == true) {
      trigger2count++;
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); 
      
      if (angleChange >= 30 && angleChange <= 400) { // 각도가 80~100도 사이로 변경되는 경우   
        trigger3 = true;
        trigger2 = false; 
        trigger2count = 0;       
//        Serial.println(angleChange);       
        Serial.println("TRIGGER 3 ACTIVATED");     
    }   
  }   
  if (trigger3 == true) {     
    trigger3count++;   
      
    if (trigger3count >= 10) {
       angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
       //delay(10);
//       Serial.println(angleChange);
       
      if ((angleChange >= 0) && (angleChange <= 10)) { // 각도가 0~10도 사이를 유지하는 경우      
        fall = true; 
        trigger3 = false; 
        trigger3count = 0;         
//        Serial.println(angleChange);       
      }       
      else { // 사용자가 정상상태로 복귀  
         
        Serial.println("TRIGGER 3 DEACTIVATED");
//        delay(1000);
        SerialBT.write('F');
        SerialBT.print(":");
        SerialBT.print("\n"); 

        if(equip == false)
        {
          trigger3 = false; 
          trigger3count = 0;
        }
      }     
    }   
  }   
  if (fall == true) { // 추락감지 시   
    Serial.println("FALL DETECTED");       
    fall = false;   
  }   
  if (trigger2count >= 6) { // 방향 변경에 대한 시간 0.5초 허용
     trigger2 = false; 
     trigger2count = 0;
     Serial.println("TRIGGER 2 DECACTIVATED");
   }
   if (trigger1count >= 6) { // 증폭값이 상한임계값을 깨기까지 0.5초 허용
     trigger1 = false; 
     trigger1count = 0;
     Serial.println("TRIGGER 1 DECACTIVATED");
   }
   delay(100);   
}

void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);   // I2C 통신용 어드레스(주소)
  Wire.write(0x6B);    // MPU6050과 통신을 시작하기 위해서는 0x6B번지에    
  Wire.write(0);
  Wire.endTransmission(true);
}

void getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // AcX 레지스터 위치(주소)를 지칭합니다
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // AcX 주소 이후의 14byte의 데이터를 요청
  AcX = Wire.read() << 8 | Wire.read(); //두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// loop 한 사이클동안 걸리는 시간을 알기위한 함수
void getDT() {
  now = millis();   
  dt = (now - past) / 1000.0;  
  past = now;
}

// 센서의 초기값을 10회 정도 평균값으로 구하여 저장하는 함수
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
  averAcX=sumAcX/10;  averAcY=sumAcY/10;  averAcZ=sumAcY/10;
  averGyX=sumGyX/10;  averGyY=sumGyY/10;  averGyZ=sumGyZ/10;
}
