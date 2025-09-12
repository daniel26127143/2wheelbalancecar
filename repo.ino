#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

// --- 分區 PWM 設定 ---
float zone1_limit = 2;     // |input| < 2
float zone2_limit = 3;     // |input| < 3
// float zone3_limit = 4;     // |input| < 4
float zone1_gain = 0.95;    // 小角度修正比較弱
float zone2_gain = 1.0;    // 中角度修正稍強
float zone3_gain = 1.1;    // 大角度修正更強
float fall_angle_limit = 40;  // 超過 ±40 度就停車

// MPU6050 與 DMP
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3]; 

// PID測試
volatile double input, output; 
double setpoint = -1; // 電源線反方向
double Kp = 63.0, Ki = 220.0, Kd = 1.6;

PID pid((double*)&input, (double*)&output, &setpoint, Kp, Ki, Kd, DIRECT);

// 馬達腳位
#define IN1M 7
#define IN2M 6
#define PWMA 9
#define IN3M 13
#define IN4M 12
#define PWMB 10
#define STBY 8

volatile bool pid_compute_flag = false;

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
  
  cli(); 
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

  // --- 修改3：直接設定 Timer2 每 5ms 觸發一次中斷 ---
  // 使用 16MHz 時脈, 預分頻 1024 -> 15625 Hz
  // 5ms (200Hz) -> 15625 / 200 = 78.125 -> OCR2A = 77
  cli(); 
  TCCR2A = 0; TCCR2B = 0; TCNT2  = 0;
  OCR2A = 77;
  TCCR2A |= (1 << WGM21); 
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
  TIMSK2 |= (1 << OCIE2A); 
  sei(); 
}

// 移動平均濾波
float angleBuffer[3] = {0};
int idx = 0;

// ISR中斷副程式
ISR(TIMER2_COMPA_vect) {
  pid.Compute();
  pid_compute_flag = true;
}

void loop() {
  if (pid_compute_flag) {
    pid_compute_flag = false; 

    if (!dmpReady) return;
    
    // 讀取感測器、處理數據、控制馬達
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      angleBuffer[idx] = ypr[2] * 180.0 / M_PI;
      idx = (idx + 1) % 3;
      double current_angle = (angleBuffer[0] + angleBuffer[1] + angleBuffer[2]) / 3.0;

      // 更新 input 值給下一次 ISR 計算使用
      // 這一步不是絕對的原子安全，但對於平衡車來說，微小的誤差可接受
      input = current_angle;

      if (abs(input) > fall_angle_limit){
        stop();
      }

      double motor_output;
      // 確保讀取的時候不會被中斷
      cli();
      motor_output = output;
      sei();

      // 分區 PWM 加成
      int current_zone = 0;
      if (abs(input) < zone1_limit) {
        motor_output *= zone1_gain;
        current_zone = 1;
      } else if (abs(input) < zone2_limit) {
        motor_output *= zone2_gain;
        current_zone = 2;
      } else {
        motor_output *= zone3_gain;
        current_zone = 3;
      }

      setMotorSpeed(motor_output);

      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print(" Angle: "); Serial.print(input, 2);
        Serial.print(" | Output: "); Serial.print(motor_output, 2);
        Serial.print(" | Zone: "); Serial.print(current_zone);
        Serial.print(" | Error: "); Serial.println(setpoint - input, 2);
      }
    }
  }
}