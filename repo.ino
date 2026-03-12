#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PinChangeInterrupt.h>


// ★★★ 硬體腳位定義 ★★★
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

// Encoder 腳位定義
#define ENC_PIN_L 2   // 左輪
#define ENC_PIN_R A1  // 右輪


// ★★★ 系統常數和PID參數 ★★★
// Setpoint
float input, output, base_setpoint = -0.8 * DEG_TO_RAD, adj_setpoint = base_setpoint;
// Roll
float Roll_Kp_Small = 2500.0, Roll_Ki_Small = 100.0, Roll_Kd_Small = 10000.0;  // ROLL誤差小
float Roll_Kp_Mid = 3000.0, Roll_Ki_Mid = 50.0, Roll_Kd_Mid = 10500.0;         // 誤差中
float Roll_Kp_Large = 3500.0, Roll_Ki_Large = 0.0, Roll_Kd_Large = 11000.0;    // 誤差大
float limit_small = 1.0 * DEG_TO_RAD;                                          // 誤差 < 2 度，使用 Small
float limit_mid = 3.0 * DEG_TO_RAD;                                            // 誤差 < 3 度，使用 Mid
float Roll_Kp_Move = 2500.0, Roll_Ki_Move = 0.0, Roll_Kd_Move = 10000.0;       // 移動時Roll徑度的PID                                                                                                                              // 前後移動時的PWM加值
// Pitch
float Pitch_Kp_Small = 0.0, Pitch_Ki_Small = 0.0, Pitch_Kd_Small = 0.0;  // Pitch徑度的PID
// Yaw
float Yaw_Kp_Small = 0.0, Yaw_Ki_Small = 0.0, Yaw_Kd_Small = 0.0;  // Yaw徑度的PID
// Orient
float Orient_Kp = 150.0, Orient_Ki = 0.0, Orient_Kd = 0.0;
// Track
float Track_Kp = 3.0, Track_Ki = 0.0, Track_Kd = 0.0;
// Speed
float Speed_Kp = 1.5, Speed_Ki = 0.05;

float target_setpoint = base_setpoint;      // 最終目標角度
float max_lean_angle = 2.0 * DEG_TO_RAD;    // 最大傾斜角度
float angle_step_size = 0.02 * DEG_TO_RAD;  // 平滑過渡的步伐 (越小越柔順)

float fall_angle_limit = 40 * DEG_TO_RAD;  // 超過 ±40 度就停車

// ★★★ 全域變數 ★★★
// MPU6050 與 DMP設定
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;

//紀錄馬達轉向
volatile bool is_L_Forward = true;
volatile bool is_R_Forward = true;

// 轉向控制變數
volatile float currentYaw = 0.0;       // 目前航向角
volatile float currentDMPAngle = 0.0;  // 目前俯仰角 (Roll)
float target_Yaw = 0.0;                // 目標航向角

volatile bool is_fallen = false;
volatile bool pid_computed = false;
volatile float return_output = 0.0;  // ISR 所需要的回傳值
static bool is_navigating = false;   // 是否正在自動導航
unsigned long recovery_counter = 0;  // 用於扶正回復時的計數器

//PID orient track的誤差值
float Roll_error = 0, Roll_last_error = 0, Roll_integral = 0, Roll_derivative = 0;      // 前兩為誤差值，後面兩個是I和D的儲存的誤差值
float Pitch_error = 0, Pitch_last_error = 0, Pitch_integral = 0, Pitch_derivative = 0;  // 前兩為誤差值，後面兩個是I和D的儲存的誤差值
float Yaw_error = 0, Yaw_last_error = 0, Yaw_integral = 0, Yaw_derivative = 0;          // 前兩為誤差值，後面兩個是I和D的儲存的誤差值
float Orient_error = 0, Orient_last_error = 0, Orient_integral = 0, Orient_derivative = 0;
float Track_error = 0, Track_last_error = 0, Track_integral = 0, Track_derivative = 0;
float Speed_integral = 0.0;  //車速的I值計算用
float target_speed = 0.0;    // 期望車速 (正前進，負後退，0煞車定點)
float filtered_speed = 0.0;  // 濾波後的真實車速

float targetYaw = 0.0;       // 目標航向角 (度)
bool isAutoTurning = false;  // 是否正在自動轉彎
float turn_cmd_pwm = 0;      // 計算出來的轉向 PWM
float speed_L, speed_R;

// Encoder 計數變數
volatile long encoder_count_L = 0;
volatile long encoder_count_R = 0;

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
  speedL = speedL * 1.037;  //兩輪出場校正
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
  // 設定pwmLR並且消去小角度
  int pwmL = abs(speedL);
  int pwmR = abs(speedR);
  if (pwmL < 10) pwmL = 0;
  if (pwmR < 10) pwmR = 0;
  // 兩輪前進後退四種狀態
  if (speedL >= 0 && speedR >= 0) {
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
    is_L_Forward = true;
    is_R_Forward = true;
  } else if (speedL < 0 && speedR < 0) {
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
    is_L_Forward = false;
    is_R_Forward = false;
  } else if (speedL < 0 && speedR >= 0) {
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
    is_L_Forward = false;
    is_R_Forward = true;
  } else if (speedL >= 0 && speedR < 0) {
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
    is_L_Forward = true;
    is_R_Forward = false;
  }
  analogWrite(PWMA, pwmL);
  analogWrite(PWMB, pwmR);
}

void calculate_Target_Yaw_Angle(float x, float y) {
  //使用 atan2 算出徑度 (Radians)
  float target_rad = atan2(y, x);
  target_Yaw = target_rad;
  float display_deg = target_rad / DEG_TO_RAD;
}

float Lite_PID(float input, float adj_setpoint) {
  float Roll_PID_output = 0.0;
  // ======== ROLL =======
  // 計(P)
  Roll_error = adj_setpoint - input;
  // 算(I)
  if (abs(Roll_error) < (3 * DEG_TO_RAD)) {
    Roll_integral += Roll_error;
  } else {
    Roll_integral = 0;  // 角度太大時，清除積分
  }
  // 算(D)
  Roll_derivative = Roll_error - Roll_last_error;
  // ====================

  // ========PITCH=======
  // 計(P)
  Pitch_error = adj_setpoint - input;
  // 算(I)
  Pitch_integral += Pitch_error;
  Pitch_integral = constrain(Pitch_integral, -1, 1);
  // 算(D)
  Pitch_derivative = Pitch_error - Pitch_last_error;
  // ====================

  // ======== Yaw =======
  // 計(P)
  Yaw_error = adj_setpoint - input;
  // 算(I)
  Yaw_integral += Yaw_error;
  Yaw_integral = constrain(Yaw_integral, -1, 1);
  // 算(D)
  Yaw_derivative = Yaw_error - Yaw_last_error;
  // ====================

  if (is_navigating) {
    // 移動 PID控制
    Roll_PID_output = (Roll_Kp_Move * Roll_error) + (Roll_Ki_Move * Roll_integral) + (Roll_Kd_Move * Roll_derivative);
  } else {
    float abs_err = abs(Roll_error);

    if (abs_err < limit_small) {
      // 1. 小角度區間
      Roll_PID_output = (Roll_Kp_Small * Roll_error) + (Roll_Ki_Small * Roll_integral) + (Roll_Kd_Small * Roll_derivative);
    } else if (abs_err < limit_mid) {
      // 2. 中角度區間
      Roll_PID_output = (Roll_Kp_Mid * Roll_error) + (Roll_Ki_Mid * Roll_integral) + (Roll_Kd_Mid * Roll_derivative);
    } else {
      // 3. 大角度區間
      Roll_PID_output = (Roll_Kp_Large * Roll_error) + (Roll_Ki_Large * Roll_integral) + (Roll_Kd_Large * Roll_derivative);
    }
  }

  // 紀錄兩次的error之間
  Roll_last_error = Roll_error;
  // 限制輸出範圍
  return constrain(Roll_PID_output, -255, 255);
}
// 左輪中斷 encoder
void Code_left() {
  encoder_count_L += is_L_Forward ? 1 : -1;
}

// 右輪中斷 encoder
void Code_right() {
  encoder_count_R += is_R_Forward ? 1 : -1;
}

float Speed_PI() {
  // 讀取兩輪的encoder
  float current_speed = (encoder_count_L + encoder_count_R) / 2.0;
  encoder_count_L = 0;
  encoder_count_R = 0;
  static int speed_index = 0;
  static float speed_history[3] = { 0.0, 0.0, 0.0 };

  // 如果不在巡航模式，把歷史陣列與積分全部清空
  if (!is_navigating) {
    Speed_integral = 0;
    speed_history[0] = 0;
    speed_history[1] = 0;
    speed_history[2] = 0;
    return 0.0;  // 煞車定點回傳 0
  }

  // ★ 用你的方法：直接把最新速度存進目前的 index，然後讓 index++
  speed_history[speed_index] = current_speed;
  speed_index++;

  // 當指標超過陣列大小時，讓它歸零循環
  if (speed_index >= 3) {
    speed_index = 0;
  }

  // 算出 3 次的平均值
  filtered_speed = (speed_history[0] + speed_history[1] + speed_history[2]) / 3.0;
  // 計算 PI 誤差與積分
  float Speed_error = target_speed - filtered_speed;
  if (abs(Speed_error) < 3.0) {
    Speed_integral += Speed_error;
  }

  // // 積分限幅
  // Speed_integral = constrain(Speed_integral, -3000, 3000);

  // 計算速度環輸出
  float speed_output = (Speed_Kp * Speed_error) + (Speed_Ki * Speed_integral);

  // 將輸出轉換為角度量級，並限制最大傾角
  speed_output = speed_output * 0.001;
  speed_output = constrain(speed_output, -2.0 * DEG_TO_RAD, 2.0 * DEG_TO_RAD);

  return speed_output;
}

void setup() {
  Serial.begin(9600);  // 配合藍牙
  Wire.begin();

  //加速 I2C 到 400kHz，
  Wire.setClock(400000);

  initMotorPins();
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);  // 確保起始是滅的
  // 設定 Encoder 上拉模式
  pinMode(ENC_PIN_L, INPUT_PULLUP);
  pinMode(ENC_PIN_R, INPUT_PULLUP);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(1930);
  mpu.setYAccelOffset(-1670);
  mpu.setZAccelOffset(1540);
  mpu.setXGyroOffset(99);
  mpu.setYGyroOffset(-5);
  mpu.setZGyroOffset(-56);

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
  // 設定 Encoder 上拉模式
  pinMode(ENC_PIN_L, INPUT_PULLUP);
  pinMode(ENC_PIN_R, INPUT_PULLUP);

  // 左輪使用標準中斷
  attachInterrupt(0, Code_left, CHANGE);
  // 右輪使用 PinChangeInt
  attachPCINT(digitalPinToPCINT(ENC_PIN_R), Code_right, CHANGE);

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
    currentDMPAngle = ypr_temp[2];
    currentYaw = ypr_temp[0];
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
      output = Lite_PID(input, adj_setpoint);
      output = constrain(output, -255, 255);
      return_output = output;
    }
  }
  // 已經倒下需要判斷是否扶正，並且讓recovery_counter計數
  else {
    return_output = 0;  // 確保馬達完全不動
    // 判斷是否扶正(與中心 < 5 角度)
    if (abs(input - adj_setpoint) < (5 * DEG_TO_RAD)) {
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
  // 定義目標座標
  // static float target_x = 2.0;
  // static float target_y = 1.0;

  // --- 2. 狀態變數 ---
  // (刪除了 move_req)
  static float turn_req = 0.0;  // 轉向請求
  static int auto_stop_count = 0;  // 計算自動暫停的秒數
  //讀取指令
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'W' || cmd == 'w') {
      is_navigating = true;
      target_speed = 5.0;  // 設定目標速度為正 (前進)
      Serial.println("前進");
    } else if (cmd == 'x' || cmd == 'x') {
      is_navigating = true;
      target_speed = -5.0;  // 設定目標速度為負 (後退)
      Serial.println("後退");
    } else if (cmd == 'S' || cmd == 's') {
      is_navigating = false;
      target_speed = 0.0;  // 目標速度歸零 (定點煞車)
      Serial.println("煞車");
    }
  }

  // 傾倒檢查
  if (is_fallen) {
    stop();
    is_navigating = false;
    turn_req = 0;
    target_speed = 0.0;
    Speed_integral = 0;
    adj_setpoint = base_setpoint;
  }

  // 執行馬達與 目標平滑過渡器
  if (pid_computed && !is_fallen) {

    static int speed_tick = 0;
    
    speed_tick++;

    if (is_navigating && target_speed != 0.0) {
      auto_stop_count++;

      if (auto_stop_count * 10 >= (2000)) {  // *10是為了更直覺的看幾秒鐘
        target_speed = 0.0;                  // 時間到，自動煞車
        auto_stop_count = 0;                     // 計時器歸零
        Serial.println("移動結束");
      }
    }

    if (speed_tick >= 4) {
      speed_tick = 0;
      float angle_compensation = Speed_PI();
      adj_setpoint = base_setpoint - angle_compensation;
    }

    float motor_cmd = return_output;
    speed_L = motor_cmd + turn_req;
    speed_R = motor_cmd - turn_req;

    setMotorSpeed(speed_L, speed_R);
    pid_computed = false;  // 等待下一次中斷
  }
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 200) {  // 縮短到 50ms 抓一次資料，波形更細膩
  //   lastPrint = millis();

  //   Serial.print(millis());
  //   Serial.print(",");
  //   Serial.println(adj_setpoint * RAD_TO_DEG, 2);  // 變數 1: 當前傾斜角度
  // }
  //輸出matlab
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 200) {
  //   lastPrint = millis();
  //   // 格式：時間, 角度, PID輸出, 左輪PWM
  //   Serial.print(millis());
  //   Serial.print(",ROLL角 : ");
  //   Serial.print(currentDMPAngle * 180.0 / 3.14159, 2);
  //   Serial.print(",YAW角 : ");
  //   Serial.print(currentYaw * 180.0 / 3.14159, 2);
  //   Serial.print(",eL : ");
  //   Serial.print(encoder_count_L);
  //   Serial.print(",eR : ");
  //   Serial.print(encoder_count_R);
  //   if (is_navigating) {
  //     Serial.print(",模式 : 自動巡航");
  //   } else {
  //     Serial.print(",模式 : 平衡模式");
  //   }
  //   Serial.println(return_output);

  // }
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 200) {
  //   lastPrint = millis();
  //   Serial.print("ROLL角:"); Serial.print(currentDMPAngle * RAD_TO_DEG, 2);
  //   Serial.print(" | 目標速:"); Serial.print(target_speed);
  //   Serial.print(" | 真實速:"); Serial.print(filtered_speed);
  //   Serial.print(" | 補償角:"); Serial.println((base_setpoint - adj_setpoint) * RAD_TO_DEG, 2);
  // }
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    // 輸出格式: 時間, 目標角度, 真實角度, 目標速度, 真實速度
    Serial.print(millis());
    Serial.print(",");
    Serial.print(adj_setpoint * RAD_TO_DEG, 2);  // 指標 1: 大腦給的「目標傾角」
    Serial.print(",");
    Serial.print(currentDMPAngle * RAD_TO_DEG, 2);  // 指標 2: 小腦量到的「真實傾角」
    Serial.print(",");
    Serial.print(target_speed);  // 指標 3: 長官下的「目標速度」
    Serial.print(",");
    Serial.println(filtered_speed);  // 指標 4: Encoder 算出的「真實速度」
  }
}