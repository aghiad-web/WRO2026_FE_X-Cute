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

unsigned short _RoundNumber = 0; // تتبع عدد الجولات
bool finished = false;

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

float cumulativeYaw = 0.0;    // الزاوية التراكمية المستمرة
float lastRawYaw = 0.0;       // القراءة الخام السابقة
bool isFirstGyroRead = true;  // لضبط أول قراءة
float startTurnYaw = 0.0;     // مرجع زاوية الانعطاف الحالي

// ==========================================
// إعدادات الجايروسكوب BNO085
// ==========================================
#define BNO08X_RESET PB13
#define BNO08X_INT PB12

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float currentYaw = 0.0;
float yawOffset = 0.0;

void updateGyro() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
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

      float rawYaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + qr * qr)) * 180.0 / PI;

      if (isFirstGyroRead) {
        lastRawYaw = rawYaw;
        isFirstGyroRead = false;
      }

      float delta = rawYaw - lastRawYaw;
      while (delta > 180.0) delta -= 360.0;
      while (delta < -180.0) delta += 360.0;

      cumulativeYaw += delta;
      lastRawYaw = rawYaw;
    }
  }
}

// ==========================================
// متغيرات الـ PID وحالة الروبوت (State Machine)
// ==========================================
float Kp = 3;
float Kd = 1.5;
float Ki = 0.05;

float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

// ---- إعدادات كشف الزاوية التلقائي بـ Ultrasonic الأمامي ----
const float FRONT_TURN_THRESHOLD = 35.0;  // المسافة الأمامية بالـ cm للبدء بالالتفاف يميناً
const int TURN_DETECT_CONFIRM_COUNT = 3;  // عدد القراءات المتتالية للتأكد وتفادي الضجيج
int rightTurnConfirm = 0;

// ---- إعدادات مناورة تفادي المكعبات (جديد) ----
const int AVOID_STEER_OFFSET = 300;         // زاوية ميل الستيرنغ أثناء تفادي المكعب
const int AVOID_SPEED = 600;                 // سرعة الموتور أثناء التفادي
const unsigned long AVOID_DURATION_MS = 600; // مدة المناورة قبل الرجوع للمسار الطبيعي
unsigned long avoidStartTime = 0;

enum DriveState
{
  FORWARD_DRIVE,   // القيادة الطبيعية (Wall Following) + استقبال أوامر الكاميرا
  TURNING_RIGHT,   // التفاف زاوية الحلبة (90 درجة عبر الجايرو)
  AVOID_LEFT,      // تفادي مكعب أخضر - ميل يسار قصير
  AVOID_RIGHT,     // تفادي مكعب أحمر - ميل يمين قصير
  CRUISE_INTO_ZONE,
  STOP_STATE
};

DriveState currentState = FORWARD_DRIVE;

const unsigned long CRUISE_INTO_ZONE_MS = 1200;
unsigned long cruiseStartTime = 0;

// ==========================================
// دوال المقاطعة (ISR) للحساسات
// ==========================================
void leftEchoISR() {
  if (digitalRead(echo_Ultra4) == HIGH) {
    startTimeLeft = micros();
  } else {
    distanceLeft = (micros() - startTimeLeft) * 0.0343 / 2.0;
    flagLeft = true;
  }
}

void rightEchoISR() {
  if (digitalRead(echo_Ultra5) == HIGH) {
    startTimeRight = micros();
  } else {
    distanceRight = (micros() - startTimeRight) * 0.0343 / 2.0;
    flagRight = true;
  }
}

void frontEchoISR() {
  if (digitalRead(echo_UltraFront) == HIGH) {
    startTimeFront = micros();
  } else {
    distanceFront = (micros() - startTimeFront) * 0.0343 / 2.0;
    flagFront = true;
  }
}

void Turn(int turn_value) {
  dxl.setGoalPosition(DXL_ID, turn_value);
}

void setMotor(int speed) {
  if (speed > 0) {
    analogWrite(IN1, speed);
    analogWrite(IN2, 0);
  } else if (speed < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(speed));
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
}

void WallFolwing_PID(float dt, unsigned long currentTime, int speed) {
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

void triggerUltrasonicSensors(unsigned long currentTime) {
  if (flagLeft || currentTime - lastTriggerTimeLeft > 25) {
    flagLeft = false;
    digitalWrite(Trig_Ultra4, LOW); delayMicroseconds(2);
    digitalWrite(Trig_Ultra4, HIGH); delayMicroseconds(10);
    digitalWrite(Trig_Ultra4, LOW);
    lastTriggerTimeLeft = currentTime;
  }
  if (flagRight || currentTime - lastTriggerTimeRight > 25) {
    flagRight = false;
    digitalWrite(Trig_Ultra5, LOW); delayMicroseconds(2);
    digitalWrite(Trig_Ultra5, HIGH); delayMicroseconds(10);
    digitalWrite(Trig_Ultra5, LOW);
    lastTriggerTimeRight = currentTime;
  }
  if (flagFront || currentTime - lastTriggerTimeFront > 25) {
    flagFront = false;
    digitalWrite(Trig_UltraFront, LOW); delayMicroseconds(2);
    digitalWrite(Trig_UltraFront, HIGH); delayMicroseconds(10);
    digitalWrite(Trig_UltraFront, LOW);
    lastTriggerTimeFront = currentTime;
  }
}

bool isRound() {
  static unsigned short TurnCounter = 0;
  TurnCounter++;

  if (TurnCounter == 4) {
    TurnCounter = 0;
    return true;
  } else {
    return false;
  }
}

int getRoundNumber() {
  static unsigned short RoundCounter = 0;
  if (isRound()) {
    RoundCounter++;
  }
  return RoundCounter;
}

// ==========================================
// معالجة أوامر السيريال القادمة من كود الرؤية (بايثون)
// حرف واحد فقط لكل أمر: 'L' / 'R' / 'F' - بدون سطر جديد
// ==========================================
void handleVisionCommands(unsigned long currentTime)
{
  if (!Serial.available()) return;

  char cmd = Serial.read();

  // نقبل أمر جديد بس لو الروبوت بوضع القيادة الطبيعية
  // (يعني مش بمنتصف التفاف زاوية أو مناورة تفادي سابقة)
  if (currentState != FORWARD_DRIVE) return;

  if (cmd == 'L')
  {
    currentState = AVOID_LEFT;
    avoidStartTime = currentTime;
  }
  else if (cmd == 'R')
  {
    currentState = AVOID_RIGHT;
    avoidStartTime = currentTime;
  }
  // cmd == 'F' -> استمر بنفس الحالة، ما في إجراء إضافي
}

void UpdateDriveState(float dt, unsigned long currentTime)
{
  if (currentState == FORWARD_DRIVE)
  {
    WallFolwing_PID(dt, currentTime, 800);

    // التأكد من الاقتراب من الجدار الأمامي لعدة قراءات متتالية لتفادي القراءات الخاطئة
    if (distanceFront > 0 && distanceFront <= FRONT_TURN_THRESHOLD)
    {
      rightTurnConfirm++;
    }
    else
    {
      rightTurnConfirm = 0;
    }

    if (rightTurnConfirm >= TURN_DETECT_CONFIRM_COUNT)
    {
      startTurnYaw = cumulativeYaw;
      currentState = TURNING_RIGHT;
      rightTurnConfirm = 0;
    }
  }

  else if (currentState == TURNING_RIGHT)
  {
    setMotor(400);

    float turnedAngle = abs(cumulativeYaw - startTurnYaw);
    float remainingAngle = 89.0 - turnedAngle;

    if (remainingAngle <= 0.0)
    {
      _RoundNumber = getRoundNumber();
      currentState = FORWARD_DRIVE;

      integral = 0;
      previousError = 0;
      lastTime = millis();
    }
    else
    {
      int minTurnOffset = 150;
      int dynamicOffset = minTurnOffset + ((SERVO_OFFSET - minTurnOffset) * (remainingAngle / 89.0));
      Turn(SERVO_CENTER + dynamicOffset);
    }
  }

  else if (currentState == AVOID_LEFT)
  {
    setMotor(AVOID_SPEED);
    Turn(SERVO_CENTER - AVOID_STEER_OFFSET);

    if (currentTime - avoidStartTime >= AVOID_DURATION_MS)
    {
      currentState = FORWARD_DRIVE;
      integral = 0;
      previousError = 0;
      lastTime = millis();
    }
  }

  else if (currentState == AVOID_RIGHT)
  {
    setMotor(AVOID_SPEED);
    Turn(SERVO_CENTER + AVOID_STEER_OFFSET);

    if (currentTime - avoidStartTime >= AVOID_DURATION_MS)
    {
      currentState = FORWARD_DRIVE;
      integral = 0;
      previousError = 0;
      lastTime = millis();
    }
  }

  else if (currentState == CRUISE_INTO_ZONE)
  {
    WallFolwing_PID(dt, currentTime, 400);

    if (currentTime - cruiseStartTime >= CRUISE_INTO_ZONE_MS)
    {
      currentState = STOP_STATE;
    }
  }

  else if (currentState == STOP_STATE)
  {
    setMotor(0);
    Turn(SERVO_CENTER);
    finished = true;
  }
}

void setup()
{
  Serial.begin(115200); // نفس baud rate المضبوط بكود بايثون (يجب أن يتطابقا)

  pinMode(Trig_Ultra4, OUTPUT);
  pinMode(echo_Ultra4, INPUT_PULLDOWN);
  pinMode(Trig_Ultra5, OUTPUT);
  pinMode(echo_Ultra5, INPUT_PULLDOWN);
  pinMode(Trig_UltraFront, OUTPUT);
  pinMode(echo_UltraFront, INPUT_PULLDOWN);

  pinMode(PA0, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(echo_Ultra4), leftEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo_Ultra5), rightEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo_UltraFront), frontEchoISR, CHANGE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWriteFrequency(20000);
  analogWriteResolution(10);

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(2.0);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  Turn(SERVO_CENTER);

  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  if (!bno08x.begin_I2C(0x4A, &Wire, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (true) {}
  } else {
    Serial.println("BNO085 Initialized!");
    bno08x.hardwareReset();
    delay(500);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  }

  lastTime = millis();
  Serial.setTimeout(5);

  while (digitalRead(PA0)) {}
}

void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;

  if (dt <= 0.0) return;

  updateGyro(); // تحديث قراءة الجايرو بشكل مستمر
  triggerUltrasonicSensors(currentTime);
  handleVisionCommands(currentTime); // استقبال أوامر الكاميرا (L/R/F) من بايثون

  if (_RoundNumber < 3)
  {
    UpdateDriveState(dt, currentTime);
  }
  else if (!finished)
  {
    if (currentState != CRUISE_INTO_ZONE && currentState != STOP_STATE)
    {
      currentState = CRUISE_INTO_ZONE;
      cruiseStartTime = currentTime;
    }
    UpdateDriveState(dt, currentTime);
  }
}
