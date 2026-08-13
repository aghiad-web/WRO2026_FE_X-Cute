
#include <Arduino.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include <Adafruit_BNO08x.h>


const int IN1 = PA8;
const int IN2 = PA9;

HardwareSerial SerialDXL(PA3, PA2); // TX-RX
const int8_t DXL_DIR_PIN = -1;
Dynamixel2Arduino dxl(SerialDXL, DXL_DIR_PIN);

const uint8_t DXL_ID = 1;
const uint32_t DXL_BAUDRATE = 57600;

int SERVO_CENTER = 2048;
int SERVO_OFFSET = 400; // أقصى زاوية انحراف

unsigned short _RoundNumber = 0; // متغير لتتبع عدد الجولات التي أكملها الروبوت
bool finshed = false;

const int echo_Ultra4 = PC13;
const int Trig_Ultra4 = PB15;

// Right
const int echo_Ultra5 = PB14;
const int Trig_Ultra5 = PB3;

// Front 
const int echo_UltraFront = PA6;
const int Trig_UltraFront = PA4;

volatile unsigned long startTimeLeft = 0;
volatile unsigned long startTimeRight = 0;
volatile unsigned long startTimeFront = 0; 

volatile float distanceLeft = 0;
volatile float distanceRight = 0;
volatile float distanceFront = 0; 

unsigned long lastTriggerTimeLeft = 0;
unsigned long lastTriggerTimeRight = 0;
unsigned long lastTriggerTimeFront = 0; 

bool flagLeft = true;
bool flagRight = true;
bool flagFront = true; 



float cumulativeYaw = 0.0;    // الزاوية التراكمية المستمرة (تزيد وتنقص بدون حد 180)
float lastRawYaw = 0.0;       // القراءة الخام السابقة
bool isFirstGyroRead = true;  // لضبط أول قراءة
float startTurnYaw = 0.0;     // نقطة مرجعية لمراقبة زاوية الانعطاف الحالي


// ==========================================
// 3. إعدادات الجايروسكوب BNO085 والتصفير البرمجي
// ==========================================
#define BNO08X_RESET PB13
#define BNO08X_INT PB12
// (SDA = PB7, SCL = PB6) يتم إعدادها في الـ Setup

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float currentYaw = 0.0;
float yawOffset = 0.0; // المتغير الذي سيحفظ نقطة الصفر الوهمية

bool flag = true; // متغير لتحديد ما إذا كان يجب إعادة تهيئة الجايروسكوب بعد إعادة التشغيل
// دالة لتحديث قراءة الجايرو وتحويل الـ Quaternion إلى زاوية Yaw
void updateGyro() {

  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  }

if (flag)
{
bno08x.hardwareReset();
delay(500);
flag = false;

} 

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      float sqi = qi * qi;
      float sqj = qj * qj;
      float sqk = qk * qk;
      
      // القراءة الخام بين -180 و +180
      float rawYaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + qr * qr)) * 180.0 / PI;

      if (isFirstGyroRead) {
        lastRawYaw = rawYaw;
        isFirstGyroRead = false;
      }

      // حساب الفرق ومعالجة قفزة الحدود (-180 إلى +180)
      float delta = rawYaw - lastRawYaw;
      while (delta > 180.0) delta -= 360.0;
      while (delta < -180.0) delta += 360.0;

      cumulativeYaw += delta; // إضافة التغير للزاوية التراكمية
      lastRawYaw = rawYaw;
    }
  }
}

// دالة لالتقاط نقطة الصفر الحالية
void zeroGyro() {
  yawOffset = currentYaw;
}

// دالة لمعرفة الزاوية التي قطعها الروبوت منذ آخر تصفير
float getRelativeYaw() {
  float diff = currentYaw - yawOffset;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

// ==========================================
// 4. متغيرات الـ PID وحالة الروبوت (State Machine)
// ==========================================
float Kp = 3;
float Kd = 1.5;
float Ki = 0.05;

float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

enum DriveState
{
  WALL_FOLLOWING,
  WAITING_FOR_LEFT_CORNER,
  WAITING_FOR_RIGHT_CORNER,
  TURNING_LEFT,
  TURNING_RIGHT
};

DriveState currentState = WALL_FOLLOWING;

// ==========================================
// 5. دوال المقاطعة (ISR) للحساسات
// ==========================================
void leftEchoISR()
{
  if (digitalRead(echo_Ultra4) == HIGH) {
    startTimeLeft = micros();
  } else {
    distanceLeft = (micros() - startTimeLeft) * 0.0343 / 2.0;
    flagLeft = true;
  }
}


void rightEchoISR()
{
  if (digitalRead(echo_Ultra5) == HIGH) {
    startTimeRight = micros();
  } else {
    distanceRight = (micros() - startTimeRight) * 0.0343 / 2.0;
    flagRight = true;
  }
}


void frontEchoISR()
{
  if (digitalRead(echo_UltraFront) == HIGH) {
    startTimeFront = micros();
  } else {
    distanceFront = (micros() - startTimeFront) * 0.0343 / 2.0;
    flagFront = true;
  }
}


void Turn(int turn_value)
{
  dxl.setGoalPosition(DXL_ID, turn_value);
}


void setMotor(int speed)
{
  if (speed > 0)
  {
    analogWrite(IN1, speed);
    analogWrite(IN2, 0);
  }
  else if (speed < 0)
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(speed)); // الدوران للخلف للفرملة
  }
  else
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
}


void WallFolwing_PID(float dt, unsigned long currentTime, int speed)
{
  setMotor(speed); 

  float safeLeft = (distanceLeft > 0 && distanceLeft < 150) ? distanceLeft : 150;
  float safeRight = (distanceRight > 0 && distanceRight < 150) ? distanceRight : 150;

  float error = safeRight - safeLeft;
  float P_out = Kp * error;

  integral += (error * dt);
  if (integral > 50.0) integral = 50.0;
  else if (integral < -50.0) integral = -50.0;

  if ((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) {
    integral = 0;
  }

  float I_out = Ki * integral;
  float derivative = (error - previousError) / dt;
  float D_out = Kd * derivative;

  previousError = error;

  float totalOutput = P_out + I_out + D_out;

  if (totalOutput > SERVO_OFFSET) totalOutput = SERVO_OFFSET;
  else if (totalOutput < -SERVO_OFFSET) totalOutput = -SERVO_OFFSET;

  Turn(SERVO_CENTER + totalOutput);
  lastTime = currentTime;
}


void triggerUltrasonicSensors(unsigned long currentTime)
{
  if (flagLeft || currentTime - lastTriggerTimeLeft > 25)
  {
    flagLeft = false;
    digitalWrite(Trig_Ultra4, LOW); delayMicroseconds(2);
    digitalWrite(Trig_Ultra4, HIGH); delayMicroseconds(10);
    digitalWrite(Trig_Ultra4, LOW);
    lastTriggerTimeLeft = currentTime;
  }
  if (flagRight || currentTime - lastTriggerTimeRight > 25)
  {
    flagRight = false;
    digitalWrite(Trig_Ultra5, LOW); delayMicroseconds(2);
    digitalWrite(Trig_Ultra5, HIGH); delayMicroseconds(10);
    digitalWrite(Trig_Ultra5, LOW);
    lastTriggerTimeRight = currentTime;
  }
  if (flagFront || currentTime - lastTriggerTimeFront > 25)
  {
    flagFront = false;
    digitalWrite(Trig_UltraFront, LOW); delayMicroseconds(2);
    digitalWrite(Trig_UltraFront, HIGH); delayMicroseconds(10);
    digitalWrite(Trig_UltraFront, LOW);
    lastTriggerTimeFront = currentTime;
  }
}


bool isRound()
{
  static unsigned short TurnCounter = 0;
  TurnCounter++;

  if(TurnCounter == 4)
  {
    TurnCounter = 0;
    return true;
  }
  else
  {
    return false;
  }
}


int getRoundNumber()
{
  static unsigned short RoundCounter = 0;

  if(isRound())
    {
      RoundCounter++;
    }

      return RoundCounter;
}

void UpdateDriveState(float dt, unsigned long currentTime)
{
  updateGyro();

  // حالة: انتظار اختفاء الجدار الأيسر
  if (currentState == WAITING_FOR_LEFT_CORNER)
  {
    WallFolwing_PID(dt, currentTime, 600);    
    if (distanceLeft >= 100 || distanceFront < 45) 
    {
      setMotor(800);
      startTurnYaw = cumulativeYaw; // التقاط الزاوية الحالية قبل بدء الدوران
      currentState = TURNING_LEFT;
    }
  }

  // حالة: التنفيذ الفعلي للالتفاف لليسار
  else if (currentState == TURNING_LEFT)
  {
    setMotor(800);
    Turn(SERVO_CENTER - SERVO_OFFSET);  

    // قياس الزاوية المقطوعة منذ بدء الدوران
    if (abs(cumulativeYaw - startTurnYaw) >= 89.0) 
    {
      _RoundNumber = getRoundNumber();
      currentState = WALL_FOLLOWING;
      integral = 0;      
      previousError = 0; 
      lastTime = millis(); 
    }
  }

  // حالة: انتظار اختفاء الجدار الأيمن
  else if (currentState == WAITING_FOR_RIGHT_CORNER)
  {
    WallFolwing_PID(dt, currentTime, 600);
    
    if (distanceRight >= 100 || distanceFront < 45)
    {
      setMotor(800);
      startTurnYaw = cumulativeYaw; // التقاط الزاوية الحالية قبل بدء الدوران
      currentState = TURNING_RIGHT;
    }
  }

  // حالة: التنفيذ الفعلي للالتفاف لليمين
  else if (currentState == TURNING_RIGHT)
  {
    setMotor(800);
    Turn(SERVO_CENTER + SERVO_OFFSET); // تم تصحيح الاتجاه إلى (+) لليمين

    // قياس الزاوية المقطوعة منذ بدء الدوران
    if (abs(cumulativeYaw - startTurnYaw) >= 89.0) 
    {
      _RoundNumber = getRoundNumber();
      currentState = WALL_FOLLOWING;
      integral = 0;
      previousError = 0;
      lastTime = millis();
    }
  }

  // حالة: المشي بخط مستقيم وتتبع الجدران
  else if (currentState == WALL_FOLLOWING)
  {
    WallFolwing_PID(dt, currentTime, 900);
  }
}

void setup()
{

  Serial.begin(115200);

  // إعداد دبابيس الألتراسونيك
  pinMode(Trig_Ultra4, OUTPUT);
  pinMode(echo_Ultra4, INPUT_PULLDOWN);
  pinMode(Trig_Ultra5, OUTPUT);
  pinMode(echo_Ultra5, INPUT_PULLDOWN);
  pinMode(Trig_UltraFront, OUTPUT);
  pinMode(echo_UltraFront, INPUT_PULLDOWN);
  
  pinMode(PA0, INPUT_PULLUP);
  
  // ربط المقاطعات
  attachInterrupt(digitalPinToInterrupt(echo_Ultra4), leftEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo_Ultra5), rightEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo_UltraFront), frontEchoISR, CHANGE);

  // إعداد المحرك الدفع
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWriteFrequency(20000);
  analogWriteResolution(10);

  // إعداد السيرفو Dynamixel
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(2.0);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  Turn(SERVO_CENTER);

  // إعداد الجايروسكوب
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  
  if (!bno08x.begin_I2C(0x4A, &Wire, BNO08X_INT)) {
    SerialUSB.println("Failed to find BNO08x chip");
    while(true)
    {

    }

  } else {
    SerialUSB.println("BNO085 Initialized!");
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  }

  lastTime = millis();

  Serial.setTimeout(5);

  // انتظار CRUISE_INTO_ZONE_MSضغطة الزر للبدء
  // delay(5000);
  while (digitalRead(PA0))
  {
  }
}


bool finished = false;
bool stopTimerStarted = false;
unsigned long stopStartTime = 0;

void loop()
{


const unsigned long CRUISE_INTO_ZONE_MS = 1200; 


  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;

  if (dt <= 0.0) return; // حماية من قسمة الصفر

  // إطلاق نبضات الألتراسونيك بشكل دوري وغير معطل
  triggerUltrasonicSensors(currentTime);

  
  if (Serial.available())
  {
    String incomingData = Serial.readStringUntil('\n');
    incomingData.trim();

    if (currentState == WALL_FOLLOWING)
    {
      if (incomingData.indexOf("TURN_LEFT") != -1)
      {
        currentState = WAITING_FOR_LEFT_CORNER; 
        // setMotor(-400);
        // delay(2000);
      }
      else if (incomingData.indexOf("TURN_RIGHT") != -1)
      {
        currentState = WAITING_FOR_RIGHT_CORNER; 
        // setMotor(00);
        // delay(2000);
        
      }
    }
  }


  if(_RoundNumber < 3)
 {
    UpdateDriveState(dt, currentTime);
 }
  else if(!finished)
 {
    if (!stopTimerStarted)
    {
        stopStartTime = currentTime;
        stopTimerStarted = true;
        // اختياري: خلي الروبوت يمشي بسرعة ثابتة وبسيطة لحد ما ينزل بمنطقة التوقف
        WallFolwing_PID(dt, currentTime, 400);  // سرعة منخفضة، لسا PID شغال وآمن
    }
    else if (currentTime - stopStartTime < CRUISE_INTO_ZONE_MS)
    {
        WallFolwing_PID(dt, currentTime, 400);  // استمر بالمشي المحكوم لحد ما تخلص المدة
    }
    else
    {

        setMotor(0);
        Turn(SERVO_CENTER);
        finished = true; 
        while(1)
        {

        }

    }
 }
  
}
