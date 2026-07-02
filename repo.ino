#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PinChangeInterrupt.h>

//隨時可刪掉
volatile float out_P = 0.0;
volatile float out_I = 0.0;
volatile float out_D = 0.0;
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
float input, output, base_setpoint = -1.5 * DEG_TO_RAD;
volatile float adj_setpoint = base_setpoint;
//69.8
// Roll
// float Roll_Kp_Small = 2500.0, Roll_Ki_Small = 100.0, Roll_Kd_Small = 10000.0;  // ROLL誤差小
float Roll_Kp_Small = 2700.0, Roll_Ki_Small = 165.0, Roll_Kd_Small = 9000.0;  // ROLL誤差小
float Roll_Kp_Mid = 3500.0, Roll_Ki_Mid = 0.0, Roll_Kd_Mid = 10500.0;         // 誤差中
float Roll_Kp_Large = 4000.0, Roll_Ki_Large = 0.0, Roll_Kd_Large = 12500.0;   // 誤差大
float limit_small = 2.0 * DEG_TO_RAD;
float limit_mid = 4.0 * DEG_TO_RAD;
float Roll_Kp_Move = 2700.0, Roll_Ki_Move = 165.0, Roll_Kd_Move = 10000.0;  // 移動時Roll徑度的PID                                                                                                                              // 前後移動時的PWM加值
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

// 移動可調參數
float move_lean_angle = -0.65 * DEG_TO_RAD;  // 巡航時的傾角（統一命名，方便之後調整）
float decel_fraction = 0.0;                // 最後 35% 的距離開始減速，可依測試調整

// 核心監控變數
volatile unsigned long overlap_count = 0;       // 記錄總共發生幾次重疊衝突
volatile unsigned long isr_execution_time = 0;  // 記錄中斷核心計算花了幾秒

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
volatile float currentDMPAngle = 0.0;  // 目前俯仰角
float target_Yaw = 0.0;                // 目標航向角

volatile bool is_fallen = true;
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
volatile float speed_L, speed_R;

// Encoder 計數變數
volatile long encoder_count_L = 0;
volatile long encoder_count_R = 0;


// 狀態變數
volatile float turn_req = 0.0;
long start_pos = 0;
long target_ticks = 0;
volatile float target_angle_offset = 0.0;
volatile float current_angle_offset = 0.0;
float transition_speed = 0.02 * DEG_TO_RAD;
unsigned long wait_timer = 0;

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
  if (abs(Roll_error) < limit_small) {
    Roll_integral += Roll_error;
    Roll_integral = constrain(Roll_integral, -0.5, 0.5);
  } else {
    Roll_integral = Roll_integral * 0.9;
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

  Speed_integral += Speed_error;
  Speed_integral = constrain(Speed_integral, -1500.0, 1500.0);
  // 計算速度環輸出
  float speed_output = (Speed_Kp * Speed_error) + (Speed_Ki * Speed_integral);

  // 將輸出轉換為角度量級，並限制最大傾角
  speed_output = speed_output * 0.001;
  speed_output = constrain(speed_output, -2.0 * DEG_TO_RAD, 2.0 * DEG_TO_RAD);

  return speed_output;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  initMotorPins();
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);  // 確保LED起始是滅的
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

  // 左輪使用標準中斷
  attachInterrupt(0, Code_left, CHANGE);
  // 右輪使用 PinChangeInt
  attachPCINT(digitalPinToPCINT(ENC_PIN_R), Code_right, CHANGE);

  // TIMER2設定
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 148;
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
  if (count < 6) {
    return;
  }
  count = 0;
  static volatile bool busy_flag = false;

  if (busy_flag) {
    overlap_count++;  // 如果發現上鎖了，重疊次數 +1
    return;
  }
  busy_flag = true;  // 進入計算，立刻上鎖！

  // 以下進入真正的中斷
  sei();
  digitalWrite(led, HIGH);
  unsigned long startTime = micros();
  // 讀取 FIFO
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    digitalWrite(led, LOW);
    isr_execution_time = micros() - startTime;
    busy_flag = false;
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
    isr_execution_time = micros() - startTime;
    busy_flag = false;
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
  // 平滑的重心轉移
  // 計算目前角度與目標角度的差距
  float error = target_angle_offset - current_angle_offset;

  // 如果差距大於一步的距離，就往目標跨一步
  if (abs(error) > transition_speed) {
    // 判斷方向：如果目標在正前方就加，在後面就減
    if (error > 0) current_angle_offset += transition_speed;
    else current_angle_offset -= transition_speed;
  }
  // 如果差距已經小於一步的距離，直接貼齊目標，防止超過
  else {
    current_angle_offset = target_angle_offset;
  }
  adj_setpoint = base_setpoint + current_angle_offset;

  // 最後輸出給馬達
  float motor_cmd = return_output;
  speed_L = motor_cmd + turn_req;
  speed_R = motor_cmd - turn_req;

  setMotorSpeed(speed_L, speed_R);
  // ==========================================================
  isr_execution_time = micros() - startTime;
  // 放置PID計算完成旗標，告訴Loop可以正常工作
  pid_computed = true;
  busy_flag = false;
  digitalWrite(led, LOW);
}

//一些原本放在loop裡面的變數提出來
enum NavState { IDLE,
                CALC_PATH,
                TURNING,
                MOVING,
                WAIT };
NavState nav_state = IDLE;

// 定義座標 (X, Y)
float waypoints[2][2] = {
  { 0.0, 0.0 },  // 起點
  { 1.0, 0.0 }   //
};
int current_wp = 1;
int total_wps = 2;

float UNIT_TO_CM = 10.0;
float current_X = 0.0;   // 絕對 X 座標
float current_Y = 0.0;   // 絕對 Y 座標
long last_odom_pos = 0;  // 紀錄上一次計算時的輪胎位置


// 處理導航模式
void processCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'G' || cmd == 'g') {
      nav_state = CALC_PATH;
      current_wp = 1;
      is_navigating = true;

      // 每次啟動，重置絕對座標
      current_X = 0.0;
      current_Y = 0.0;
      last_odom_pos = (encoder_count_L + encoder_count_R) / 2;
      Serial.println("啟動");
    } else if (cmd == 'W' || cmd == 'w') {
      is_navigating = true;
      nav_state = MOVING;
      target_angle_offset = move_lean_angle;
      target_ticks = 243;
      start_pos = (encoder_count_L + encoder_count_R) / 2;
      Serial.println("前進 15cm");
    } else if (cmd == 'S' || cmd == 's') {
      nav_state = IDLE;
      is_navigating = false;
      target_angle_offset = 0.0;
      turn_req = 0.0;
      Roll_integral = 0;
      Serial.println("煞車");
    }
  }
}

// 傾倒保護
void checkstatus() {
  if (is_fallen) {
    stop();
    nav_state = IDLE;
    is_navigating = false;
    turn_req = 0;
    target_angle_offset = 0.0;
    current_angle_offset = 0.0;
    Roll_integral = 0;
    adj_setpoint = base_setpoint;
  }
}

// 導航模式
void goNavigation() {
  if (!is_navigating) return;

  if (nav_state == CALC_PATH) {
    float dx = waypoints[current_wp][0] - current_X;
    float dy = waypoints[current_wp][1] - current_Y;

    target_Yaw = atan2(dy, dx);
    float distance_cm = sqrt(dx * dx + dy * dy) * UNIT_TO_CM;

    target_ticks = distance_cm * 16.18;

    Serial.print("前往點 ");
    Serial.print(current_wp);
    Serial.print(" | 距離: ");
    Serial.println(distance_cm);
    nav_state = TURNING;
  } else if (nav_state == TURNING) {
    float yaw_error = target_Yaw - currentYaw;
    while (yaw_error > PI) yaw_error -= TWO_PI;
    while (yaw_error < -PI) yaw_error += TWO_PI;

    turn_req = yaw_error * 30.0;

    // 轉向完成
    if (abs(yaw_error) < (3.0 * DEG_TO_RAD)) {
      turn_req = 0.0;
      start_pos = (encoder_count_L + encoder_count_R) / 2;
      target_angle_offset = move_lean_angle;
      nav_state = MOVING;
      Serial.println("開始直行...");
    }
  } else if (nav_state == MOVING) {
    long current_pos = (encoder_count_L + encoder_count_R) / 2;
    long moved_distance = current_pos - start_pos;
    long remaining_ticks = target_ticks - moved_distance;
    long decel_start_ticks = target_ticks * decel_fraction;

    turn_req = 0.0;

    if (remaining_ticks <= 0) {
      // 抵達終點
      target_angle_offset = 0.0;
      turn_req = 0.0;
      Serial.println("抵達");
      wait_timer = millis();
      nav_state = WAIT;
    } else if (decel_start_ticks > 0 && remaining_ticks < decel_start_ticks) {
      // 減速區：傾角隨剩餘距離線性收斂到 0
      float decel_ratio = (float)remaining_ticks / decel_start_ticks;
      decel_ratio = constrain(decel_ratio, 0.15, 1.0);  // 保留最低15%動力，避免還沒到就停下不動
      target_angle_offset = move_lean_angle * decel_ratio;
    } else {
      // 巡航區：維持固定傾角
      target_angle_offset = move_lean_angle;
    }
  }
}

// void executeBalance() {
//   pid_computed = false;
// }

// void printInfo() {
//   static unsigned long lastPrint = 0;
//   if (millis() - lastPrint > 100) {
//     lastPrint = millis();

//     // 輸出即時的XY跟角度
//     Serial.print("X: ");
//     Serial.print(current_Y, 2);
//     Serial.print(" | Y: ");
//     Serial.print(current_X, 2);
//     Serial.print(" | 角度: ");
//     Serial.print(currentDMPAngle * RAD_TO_DEG, 1);
//     //
//     // 輸出剩餘距離
//     if (is_navigating && nav_state == MOVING) {
//       long current_pos = (encoder_count_L + encoder_count_R) / 2;
//       long moved_distance = current_pos - start_pos;
//       long remaining_ticks = abs(target_ticks) - abs(moved_distance);

//       float remaining_cm = remaining_ticks / 16.18;
//       if (remaining_cm < 0) remaining_cm = 0;

//       Serial.print(" | 和目標還差: ");
//       Serial.print(remaining_cm, 1);
//       Serial.println(" cm");
//     } else {
//       // 換行結尾
//       Serial.println();
//     }
//   }
// }
// matlab所使用的
void printInfo() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();

    // 格式固定為：時間,目前角度,PID原力,最終左輪PWM
    Serial.print(millis());
    Serial.print(",");
    Serial.print(currentDMPAngle * RAD_TO_DEG, 2);
    Serial.print(",");
    Serial.print(return_output, 2);
    Serial.print(",");
    Serial.println(speed_L, 2);
  }
}

void updateOdometry() {
  long current_pos = (encoder_count_L + encoder_count_R) / 2;
  long delta_ticks = current_pos - last_odom_pos;
  last_odom_pos = current_pos;  // 記住這次的位置給下一次用

  if (delta_ticks != 0) {
    //tick -> 公分 (1公分 = 16.18 Ticks)
    float delta_cm = delta_ticks / 16.18;

    // 再把公分換算成「地圖座標單位」
    float delta_units = delta_cm / UNIT_TO_CM;

    // 透過三角函數與當下的車頭朝向，拆解出 X 與 Y 的移動量並累加
    current_X += delta_units * cos(currentYaw);
    current_Y += delta_units * sin(currentYaw);
  }
}

void loop() {
  processCommand();  // 聽取藍牙指令
  checkstatus();     // 確認狀態

  // 當PID計算完成後判斷有無傾倒才開始計算
  if (pid_computed && !is_fallen) {
    updateOdometry();
    goNavigation();  // 指令
    pid_computed = false;
  }

  printInfo();  // 輸出數據
}