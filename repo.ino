#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

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

// PID 測試
double input, output, setpoint = -1; //電源線反方向
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
  while (1) {
    Serial.print("Kp="); Serial.print(Kp, 2);
    Serial.print(" Ki="); Serial.print(Ki, 2);
    Serial.print(" Kd="); Serial.println(Kd, 2);
    Serial.println("請重製小車");
    
    while(1){}
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
}

// 移動平均濾波
float angleBuffer[3] = {0};
int idx = 0;

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO 溢出");
    return;
  }

  // 清空 FIFO，保留最新一筆資料
  while ((fifoCount = mpu.getFIFOCount()) >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
  }

  // 解析 DMP 資料
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // 平滑處理 roll（ypr[2]）
  angleBuffer[idx] = ypr[2] * 180.0 / M_PI;
  idx = (idx + 1) % 3;
  input = (angleBuffer[0] + angleBuffer[1] + angleBuffer[2]) / 3.0;

  if (input < -50 || input > 50) {
    stop();
  }

  pid.Compute();
  setMotorSpeed(output);

  // 降低印出頻率
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(" Avg: "); Serial.print(input, 2);
    // Serial.print(" |Raw angles: ");
    // Serial.print(angleBuffer[0], 2); Serial.print(", ");
    // Serial.print(angleBuffer[1], 2); Serial.print(", ");
    // Serial.print(angleBuffer[2], 2);
    Serial.print(" | Output: "); Serial.print(output, 2);
    Serial.print(" | Error: "); Serial.println(setpoint - input, 2);
  }
}
