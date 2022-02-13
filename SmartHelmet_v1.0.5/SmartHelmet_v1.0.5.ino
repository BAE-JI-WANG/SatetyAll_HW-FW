#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

SparkFun_APDS9960 apds = SparkFun_APDS9960();

uint8_t proximity_data = 0;
const int MPU_ADDR = 0x68;    // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, prev_AcX, prev_AcY, prev_AcZ, prev_Tmp, prev_GyX, prev_GyY, prev_GyZ;   // 가속도(Acceleration)와 자이로(Gyro)
double angleAcX, angleAcY, angleAcZ, prev_angleAcX, prev_angleAcY, prev_angleAcZ;
double angleGyX, angleGyY, angleGyZ, prev_angleGyX, prev_angleGyY, prev_angleGyZ;
double angleFiX, angleFiY, angleFiZ, prev_angleFiX, prev_angleFiY, prev_angleFiZ;
boolean equip;
unsigned long m;
boolean movement;
int count = 0;
int angleChange = 0;

boolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred
byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true

float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

const double RADIAN_TO_DEGREE = 180 / 3.14159;  
const double DEG_PER_SEC = 32767 / 250;    // 1초에 회전하는 각도
const double ALPHA = 1 / (1 + 0.04);
// GyX, GyY, GyZ 값의 범위 : -32768 ~ +32767 (16비트 정수범위)


unsigned long now = 0;   // 현재 시간 저장용 변수
unsigned long past = 0;  // 이전 시간 저장용 변수
double dt = 0;           // 한 사이클 동안 걸린 시간 변수 

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;
double prev_sv;

BluetoothSerial SerialBT;

void setup() 
{
  initSensor();
  Serial.begin(115200);
  SerialBT.begin("서영준10"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  caliSensor();   //  초기 센서 캘리브레이션 함수 호출
  past = millis(); // past에 현재 시간 저장
  getData();

  prev_AcX = AcX;
  prev_AcY = AcY;                           

  prev_GyX = GyX;
  prev_GyY = GyY;
  prev_GyZ = GyZ;

  
  apds.setMode(ALL, true);  

//  delay(100);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Start running the APDS-9960 proximity sensor (no interrupts) 
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
}

boolean check_movement(float ch_AcX, float ch_AcY, float ch_AcZ, float ch_GyX, float ch_GyY, float ch_GyZ)
{
  if(ch_AcX<200 || ch_AcY<200 || ch_AcZ<200 || ch_GyX<50 || ch_GyY<50 || ch_GyZ<50) //자이로 변화가 없을때
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
  if(prev_AcX != AcX || prev_AcY != AcY || prev_AcZ != AcZ || prev_GyX != GyX || prev_GyY != GyY || prev_GyZ != GyZ)
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
//  m = count/10;
  
  getData(); 
  getDT();

  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
  // calculating Amplitute vactor for 3 axis
  
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
//  Serial.println(Amp);

  boolean gyro_status = compare(AcX, AcY, AcZ, GyX, GyY, GyZ);
  
  float ch_accX = fabs(AcX-prev_AcX);
  float ch_accY = fabs(AcY-prev_AcY);
  float ch_accZ = fabs(AcZ-prev_AcZ);

  float ch_gyX = fabs(GyX-prev_GyX);
  float ch_gyY = fabs(GyY-prev_GyY);
  float ch_gyZ = fabs(GyZ-prev_GyZ);

  movement = check_movement(ch_accX, ch_accY, ch_accZ, ch_gyX, ch_gyY, ch_gyZ);

//  Serial.print("proxi : ");
//  Serial.println(proximity_data);

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

  if(equip == true)   //9 상태
  {
    
    if(count>=300) //착용중이고, 움직임이 없을 때 태
    {
     
      if(movement == false)
      {
        SerialBT.write('W');
        SerialBT.print(":");
//        SerialBT.print("\n");
        Serial.println('W');
        
      }
      else if(movement == true)
      {
        count = 0;
        if(gyro_status == false)
        {
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
      SerialBT.write('E');
      SerialBT.print(":");
//      SerialBT.print("\n");
      Serial.println('E');
    }
  }
  
  else if(equip == false)
  {
    SerialBT.write('N');
    SerialBT.print(":");
//    SerialBT.print("\n");
    Serial.println('N');
    movement = true;
    count = 0;
  }
  
  if (Amp <= 2 && trigger2 == false) { //if AM breaks lower threshold (0.4g)     
    trigger1 = true;     
    Serial.println("TRIGGER 1 ACTIVATED");   
  }
  if (trigger1 == true) {     
    trigger1count++;     
    if (Amp >= 12) { //if AM breaks upper threshold (3g)
         trigger2 = true;
         Serial.println("TRIGGER 2 ACTIVATED");
         trigger1 = false; 
         trigger1count = 0;
     }
   }
   if (trigger2 == true) {
      trigger2count++;
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); 
//      Serial.println(angleChange);
      
      if (angleChange >= 30 && angleChange <= 400) { //if orientation changes by between 80-100 degrees       
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
       
      if ((angleChange >= 0) && (angleChange <= 10)) { //if orientation changes remains between 0-10 degrees         
        fall = true; 
        trigger3 = false; 
        trigger3count = 0;         
//        Serial.println(angleChange);       
      }       
      else { //user regained normal orientation         
         
        Serial.println("FALL DETECTED");
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
  if (fall == true) { //in event of a fall detection     
    Serial.println("FALL DETECTED");       
    fall = false;   
  }   
  if (trigger2count >= 6) { //allow 0.5s for orientation change
     trigger2 = false; 
     trigger2count = 0;
     Serial.println("TRIGGER 2 DECACTIVATED");
   }
   if (trigger1count >= 6) { //allow 0.5s for AM to break upper threshold
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
