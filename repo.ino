#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include <PID_v1.h>
// 徑度角度的轉換常數
//const float DEG_TO_RAD = 0.0174533f; // (PI / 180.0) 因為原本就有定義了 重新定義會出錯先註解

// 移動與轉向參數
float PWM_MOVE_SPEED = 20.0;   // 前進後退
float PWM_TURN_SPEED = 15.0;   // 左右轉向

// 分區 PWM 設定
float zone1_limit = 2 * DEG_TO_RAD;        // |input| < 2
float zone2_limit = 3 * DEG_TO_RAD;        // |input| < 3
float zone1_gain = 0.95;                   // 小角度增益
float zone2_gain = 1.0;                    // 中角度增益
float zone3_gain = 1.1;                    // 大角度增益
float fall_angle_limit = 40 * DEG_TO_RAD;  // 超過 ±40 度就停車

// 用於扶正回復時的計數器
unsigned long recovery_counter = 0;

// 轉彎的馬達速度
double turn_speed = 0;
// 轉向控制變數
volatile float currentYaw = 0.0;  // 目前的航向角 (度)
float targetYaw = 0.0;            // 目標航向角 (度)
bool isAutoTurning = false;       // 是否正在自動轉彎
double turn_cmd_pwm = 0;          // 計算出來的轉向 PWM

// MPU6050 與 DMP設定
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// ISR 所需要的回傳值
volatile float return_output = 0.0;

// 用於ISR和Loop中傳遞當前的角度和小車已倒下旗標
volatile float currentDMPAngle = 0.0;
volatile bool is_fallen = false;
int current_zone = 0;
unsigned long moveStartTime = 0;  // 動作開始的時間
bool isAutoMoving = false;        // 是否正在自動移動中
const int MOVE_DURATION = 10000;  // 移動時間 (毫秒)，這裡設 1 秒
// 紀錄是否完成PID計算
volatile bool pid_computed = false;

// PID
double input, output, setpoint = -2.69 * DEG_TO_RAD;
double Kp = 2500.0, Ki = 100.0, Kd = 10000.0;  // 徑度的PID
// PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
float error = 0;       // 用於計算當前離平衡點的誤差值(P)
float last_error = 0;  // 上一次的誤差 (D)
float integral = 0;    // 累積的誤差 (I)
Quaternion q;
VectorFloat gravity;
float ypr[3];

// 馬達腳位
#define IN1M 7
#define IN2M 6
#define PWMA 9
#define IN3M 13
#define IN4M 12
#define PWMB 10
#define STBY 8
// led 定義
const int led = A0;

void initMotorPins() {
  pinMode(IN1M, OUTPUT);
  pinMode(IN2M, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(IN3M, OUTPUT);
  pinMode(IN4M, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

// 設定馬達輸出為零
void stop() {
  setMotorSpeed(0, 0);
}

// 設定馬達輸出,輸入為兩馬達的PWM訊號
void setMotorSpeed(float speedL, float speedR) {
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
  // 設定pwmLR並且消去小角度
  int pwmL = abs(speedL);
  int pwmR = abs(speedR);
  if (pwmL < 10) pwmL = 0;
  if (pwmR < 10) pwmR = 0;

  // 馬達A控制
  if (speedL < 0) {
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
  } else {
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
  }
  analogWrite(PWMA, pwmL);

  // 馬達B控制
  if (speedR < 0) {
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
  } else {
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
  }
  analogWrite(PWMB, pwmR);
}

float Lite_PID(float input, float setpoint) {
  // 計算(P)
  error = setpoint - input;

  // 算(I)
  // 這邊怪怪的
  if (abs(error) < (3 * DEG_TO_RAD)) {
    integral += error;
    integral = constrain(integral, -1.5, 1.5); // 防呆：抗積分飽和
  } else {
    integral = 0; // 角度太大時，清除積分
  }

  // 算(D)
  float derivative = error - last_error;

  // 計算PID output值
  float LPID_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // 紀錄兩次的error之間
  last_error = error;
  
  // 限制輸出範圍
  return constrain(LPID_output, -255, 255);
}

void setup() {
  Serial.begin(9600);  // 配合藍牙
  Wire.begin();

  //加速 I2C 到 400kHz，
  Wire.setClock(400000);

  initMotorPins();
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);  // 確保起始是滅的

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

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
    Serial.print("DMP Fail: ");
    Serial.println(devStatus);
    while (1)
      ;
  }

  // TIMER2設定
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 124;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);
  TIMSK2 |= (1 << OCIE2A);
  sei();
}
ISR(TIMER2_COMPA_vect) {
  // 宣告中斷秒數的變數
  static int count = 0;
  count++;

  // 檢查是否到中斷時間
  if (count < 10) {
    return;
  }
  count = 0;

  // 以下進入真正的中斷
  sei();
  digitalWrite(led, HIGH);

  // 宣告用於三筆平均濾波的變數
  static float angleBuffer[3] = { 0.0, 0.0, 0.0 };
  static int idx = 0;

  // 讀取 FIFO
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    digitalWrite(led, LOW);
    return;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    float ypr_temp[3];
    mpu.dmpGetYawPitchRoll(ypr_temp, &q, &gravity);
    // 取得原始徑度
    float raw_angle = ypr_temp[2];

    // 三筆平均濾波
    angleBuffer[idx] = raw_angle;
    idx = (idx + 1) % 3;
    currentDMPAngle = (angleBuffer[0] + angleBuffer[1] + angleBuffer[2]) / 3.0;

    input = currentDMPAngle;
  } else {
    digitalWrite(led, LOW);
    return;
  }

  // 判斷PID計算和是否傾倒偵測
  if (!is_fallen) {
    // 傾倒偵測 如果小車角度已經超過設定的傾倒角度 設定倒下旗標
    if (abs(input) > fall_angle_limit) {
      is_fallen = true;      // 設定倒下旗標
      return_output = 0;     // 停止馬達
      recovery_counter = 0;  // 清除回正計數器
    }
    // 計算PID並且使用分區增益
    else {
      output = Lite_PID(input,setpoint);
      // 分區增益邏輯
      current_zone = 0;
      if (abs(input) < zone1_limit) {
        output *= zone1_gain;
        current_zone = 1;
      } else if (abs(input) < zone2_limit) {
        output *= zone2_gain;
        current_zone = 2;
      } else {
        output *= zone3_gain;
        current_zone = 3;
      }

      output = constrain(output, -255, 255);
      return_output = output;
    }
  }
  // 已經倒下需要判斷是否扶正，並且讓recovery_counter計數
  else {
    return_output = 0;  // 確保馬達完全不動
    // 判斷是否扶正(與中心 < 5 角度)
    if (abs(input - setpoint) < (5 * DEG_TO_RAD)) {
      recovery_counter++;  // 回復計數器++

      // 判斷是否已經扶正1秒鐘 (10ms x 100times = 1000ms)
      if (recovery_counter >= 100) {
        is_fallen = false;     // 取消傾倒狀態
        recovery_counter = 0;  // 回復計數器歸零
      }
    } else {
      //recovery_counter = 0;
    }
  }

  // ==========================================================

  // 放置PID計算完成旗標，告訴Loop可以正常工作
  pid_computed = true;
  digitalWrite(led, LOW);
}

void loop() {
  static float speed_L = 0;
  static float speed_R = 0;
  // 移動目標 (預設為 0)
  static float move_pwm_target = 0;
  static float turn_pwm_target = 0;

  // 1. 讀取指令
  if (Serial.available()) {
    char cmd = Serial.read();

    // --- 前進/後退 ---
    if (cmd == 'w') {
      move_pwm_target = PWM_MOVE_SPEED;  // 使用變數 (正)
      Serial.println("Move: Forward");
    } else if (cmd == 'x') {
      move_pwm_target = -PWM_MOVE_SPEED;  // 使用變數 (負)
      Serial.println("Move: Backward");
    }

    // --- 左轉/右轉 ---
    else if (cmd == 'a') {
      turn_pwm_target = PWM_TURN_SPEED;  // 使用變數
      Serial.println("Turn: Left");
    } else if (cmd == 'd') {
      turn_pwm_target = -PWM_TURN_SPEED;  // 使用變數 (負)
      Serial.println("Turn: Right");
    }

    // --- 停止 ---
    else if (cmd == 's') {
      move_pwm_target = 0;
      turn_pwm_target = 0;
      Serial.println("Stop");
    }
  }
  // 1. 傾倒檢查
  if (is_fallen) {
    stop();
    isAutoMoving = false;
  }
  // 2. 執行馬達 (只有當 ISR 算好時才動作)
  if (pid_computed) {
    float motor_cmd = return_output;
    speed_L = motor_cmd + turn_speed;
    speed_R = motor_cmd - turn_speed;
    setMotorSpeed(speed_L, speed_R);
    pid_computed = false;
  }

  //輸出matlab
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    // 格式：時間, 角度, PID輸出, 左輪PWM
    Serial.print(millis());
    Serial.print(",");
    Serial.print(currentDMPAngle * 180.0 / 3.14159, 2);
    Serial.print(",");
    Serial.print(return_output);
    Serial.print(",");
    Serial.println(speed_L);
  }
}