#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

// --- 分區 PWM 設定 ---
float zone1_limit = 2;       // |input| < 2
float zone2_limit = 3;       // |input| < 3
// float zone3_limit = 4;     // |input| < 4

float zone1_gain = 0.95;     // 小角度修正比較弱
float zone2_gain = 1.0;      // 中角度修正稍強
float zone3_gain = 1.1;      // 大角度修正更強
float fall_angle_limit = 40; // 超過 ±40 度就停車

// MPU6050 與 DMP
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];  // [yaw, pitch, roll]

// 核心控制變數：新增用於 ISR/loop 傳遞的角度和傾倒旗標
volatile float currentDMPAngle = 0.0;
volatile bool is_fallen = false;
int current_zone = 0; // 用於在 loop() 中列印區域

// PID 測試
double input, output, setpoint = -1; // 電源線反方向
double Kp = 63.0, Ki = 220.0, Kd = 1.6;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// 馬達腳位
#define IN1M 7
#define IN2M 6
#define PWMA 9
#define IN3M 13
#define IN4M 12
#define PWMB 10
#define STBY 8

// Flag
volatile bool flag_5ms = false; // 保留原來的旗標定義

void initMotorPins() {
  pinMode(IN1M, OUTPUT); pinMode(IN2M, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(IN3M, OUTPUT); pinMode(IN4M, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void stop() {
  setMotorSpeed(0);
  // 這裡進入無限迴圈，只在 loop() 執行 is_fallen 檢查時呼叫
  while (1) {
    Serial.print("Kp="); Serial.print(Kp, 2);
    Serial.print(" Ki="); Serial.print(Ki, 2);
    Serial.print(" Kd="); Serial.println(Kd, 2);
    Serial.println("小車已傾倒，請重製");
    // 避免一直重複印出
    while (1){}
  }
}

void setMotorSpeed(float speed) {
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);
  if (pwm < 5) pwm = 0;

  if (speed < 0) {
    digitalWrite(IN1M, HIGH); digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, HIGH); digitalWrite(IN4M, LOW);
  } else {
    digitalWrite(IN1M, LOW); digitalWrite(IN2M, HIGH);
    digitalWrite(IN3M, LOW); digitalWrite(IN4M, HIGH);
  }

  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initMotorPins();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // MPU 偏移值（依實際校準）
  mpu.setXAccelOffset(-3353);
  mpu.setYAccelOffset(97);
  mpu.setZAccelOffset(123);
  mpu.setXGyroOffset(81);
  mpu.setYGyroOffset(-30);
  mpu.setZGyroOffset(63);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP 啟動完成");
  } else {
    Serial.print("DMP 啟動失敗，錯誤碼: ");
    Serial.println(devStatus);
    while (1);
  }

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(5);

  // --- Timer2 設定，每 1ms 進一次 ISR ---
  cli(); // 關中斷
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 124;             // 125 次計數 = 1ms (8MHz/64 prescaler)
  TCCR2A |= (1 << WGM21);  // CTC 模式
  TCCR2B |= (1 << CS22);   // 預分頻 64
  TIMSK2 |= (1 << OCIE2A); // 開啟比較中斷
  sei(); // 開中斷
}

// 移動平均濾波（留在 global，在 loop() 中處理）
float angleBuffer[3] = {0};
int idx = 0;

// ISR: 僅負責精準計時和核心控制運算
ISR(TIMER2_COMPA_vect) {
  static int count = 0;
  count++;
  if (count >= 5) {    // 5ms 觸發一次，執行核心控制
    count = 0;
    flag_5ms = true; // 保留旗標，但主要邏輯已移入

    // 檢查是否傾倒，如果是就停止並退出
    if (is_fallen) {
        setMotorSpeed(0);
        return;
    }
    
    // 1. PID 輸入：使用 loop() 中最新讀取到的 DMP 角度
    input = currentDMPAngle; 

    // 2. 傾倒判斷 (設定旗標，讓 loop() 執行 stop())
    if (abs(input) > fall_angle_limit){
        setMotorSpeed(0); // 立即停止馬達
        is_fallen = true; // 設定傾倒旗標
        return; 
    }

    // 3. 執行 PID 運算
    pid.Compute();

    // 4. 分區 PWM 加成 & 區域紀錄
    current_zone = 0;
    if (abs(input) < zone1_limit) {
        output *= zone1_gain;
        current_zone = 1;
    } else if (abs(input) < zone2_limit) {
        output *= zone2_gain;
        current_zone = 2;
    } else if (abs(input) >= zone2_limit) {
        output *= zone3_gain;
        current_zone = 3;
    }

    // 5. 限制輸出並控制馬達
    output = constrain(output, -255, 255);
    setMotorSpeed(output);
  }
}

// loop(): 負責 DMP 數據讀取和序列埠輸出
void loop() {
    // 檢查傾倒，並執行無限迴圈停機
    if (is_fallen) {
        stop();
    }
    
    // 執行耗時的 DMP 數據讀取 (I2C 通訊)
    // 這裡不再依賴 flag_5ms，而是盡快讀取數據
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO 溢出");
      return;
    }

    // 每次只讀取一個最新的數據包，減少延遲
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      float ypr_temp[3]; // 使用區域變數
      mpu.dmpGetYawPitchRoll(ypr_temp, &q, &gravity);

      // 濾波並更新全域角度變數供 ISR 使用 (關鍵數據傳輸)
      angleBuffer[idx] = ypr_temp[2] * 180.0 / M_PI;
      idx = (idx + 1) % 3;
      currentDMPAngle = (angleBuffer[0] + angleBuffer[1] + angleBuffer[2]) / 3.0;
    }

    // 序列埠列印 (非時序敏感)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.print(" Avg: "); Serial.print(currentDMPAngle, 2);
      Serial.print(" | Output: "); Serial.print(output, 2);
      Serial.print(" | Zone: "); Serial.print(current_zone);
      Serial.print(" | Error: "); Serial.println(setpoint - currentDMPAngle, 2);
    }
}