#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PinChangeInterrupt.h>

// ★★★ 硬體腳位定義 ★★★
#define IN1M 7
#define IN2M 6
#define PWMA 9
#define IN3M 13
#define IN4M 12
#define PWMB 10
#define STBY 8
const int led = A0;

#define ENC_PIN_L 2   // 左輪編碼器
#define ENC_PIN_R A1  // 右輪編碼器

// ★★★ 正統串級 PID 參數 (計算嚴格使用 SI 制：rad, rad/s) ★★★

// 1. 直立內環 (Balance PD)
float Kp_balance_normal = 1275.0;  // 比例推力 (對抗傾斜) 1300 > x >1200
float Kd_balance_normal = 70.0;    // 微分阻尼 (抑制前後震盪) 80 > x >60

float Kp_balance_large = 1450.0;  // 1500 > x > 1400
float Kd_balance_large = 70.0;    // 大角度緊急阻尼 (增加 50%，強制吸震防甩倒)

// 2. 優先級切換門檻 (以實測發散臨界點設計之遲滯區間)
const float LARGE_ANGLE_ENTER = 52.2 * DEG_TO_RAD;  // 傾角誤差 > 3.5° 進入搶救 (即傾角小於 -5.6° 或大於 +1.4°)
const float LARGE_ANGLE_EXIT = 51.7 * DEG_TO_RAD;   // 傾角誤差收斂至 < 2.0° 退出搶救
bool is_emergency_state = false;                    // 大角度狀態旗標

// 3. 速度外環 (Speed PI) - 分頻執行
float Kp_speed = 2.8;    // 速度比例增益 2.8 > x >2.6
float Ki_speed = 0.005;  // 速度積分增益 (消除重心偏移溜車)

// float Kp_speed = 2;       // 速度比例增益 2.8 > x >2.6 (原本)
// float Ki_speed = 0.005;   // 速度積分增益 (消除重心偏移溜車)(原本)
#define SPEED_LOOP_DIV 4  // 角度環實測週期10.13ms，4倍≈40.5ms，落在40~50ms需求區間

// 3. 重心與保護常數
float base_setpoint = -2.0 * DEG_TO_RAD;  // 機械重心零點 (rad)
volatile float adj_setpoint = base_setpoint;
float fall_limit = 35.0 * DEG_TO_RAD;  // 傾倒保護限制 (超過 ±35 度斷電)

// 系統全域變數
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;

volatile bool is_fallen = true;
unsigned long recovery_counter = 0;

volatile long encoder_count_L = 0;
volatile long encoder_count_R = 0;
volatile bool is_L_Forward = true;
volatile bool is_R_Forward = true;
volatile float debug_pwm = 0.0;  // 用於 Serial 輸出的即時 PWM 數值
// 濾波與中間狀態
float current_angle = 0.0;              // 當前車身傾角 (rad)
float current_gyro_rate = 0.0;          // 當前俯仰角速度 (rad/s)
float filtered_speed = 0.0;             // 低通濾波後的車輪速度
float speed_integral = 0.0;             // 速度環積分
float speed_angle_target = 0.0;         // 速度環計算出的傾角補償量 (rad)
float target_speed = 0.0;               // 目標速度 (原地平衡為 0)
volatile float target_speed_cmd = 0.0;  // 鍵盤/藍牙目標期望速度

// ★★★ 航向角閉環參數 ★★★
float current_yaw = 0.0;  // 當前車頭朝向角 (rad)
float target_yaw = 0.0;   // 鎖定的目標朝向角 (rad)
float initial_yaw = 0.0;
bool yaw_initialized = false;  // 初始朝向鎖定旗標
volatile bool is_homing = false;
// 偏航 PD 參數 (閉環跟隨需要足夠剛性，建議 Kp=35~45, Kd=6~8)
float Kp_yaw = 38.0;
float Kd_yaw = 7.0;
// 轉向指令：改為「目標旋轉角速度」 (rad/s，0 代表直行鎖定)
volatile float target_yaw_rate_cmd = 0.0;
float Heading_Control_PD(float current_yaw_rate);

// ★★★ 底層驅動函式 ★★★
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

void setMotorSpeed(float speedL, float speedR) {
  speedL = constrain(speedL * 1.037, -255, 255);
  speedR = constrain(speedR, -255, 255);

  int deadzone = 5;  //
  int pwmL = (speedL > 0) ? (speedL + deadzone) : ((speedL < 0) ? (speedL - deadzone) : 0);
  int pwmR = (speedR > 0) ? (speedR + deadzone) : ((speedR < 0) ? (speedR - deadzone) : 0);

  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  // 左輪方向
  if (pwmL >= 0) {
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
    is_L_Forward = true;
  } else {
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    is_L_Forward = false;
  }

  // 右輪方向
  if (pwmR >= 0) {
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
    is_R_Forward = true;
  } else {
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
    is_R_Forward = false;
  }

  analogWrite(PWMA, abs(pwmL));
  analogWrite(PWMB, abs(pwmR));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, LOW);
  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, LOW);
}

// 編碼器中斷
void Code_left() {
  encoder_count_L += is_L_Forward ? 1 : -1;
}
void Code_right() {
  encoder_count_R += is_R_Forward ? 1 : -1;
}

// ★★★ 核心控制演算法 ★★★

// 航向角維持與轉向閉環控制
float Heading_Control_PD(float current_yaw_rate) {
  // 1. 計算目標航向與當前航向誤差
  float yaw_error = target_yaw - current_yaw;

  // 2. 處理 ±180 度 (±PI) 跨界跳變問題 (永遠走最短路徑)
  while (yaw_error > PI) yaw_error -= 2.0 * PI;
  while (yaw_error < -PI) yaw_error += 2.0 * PI;

  // 3. PD 運算：P 追趕目標角度，D 抑制旋轉角速度甩動
  float turn_output = (Kp_yaw * yaw_error) - (Kd_yaw * current_yaw_rate);

  // 4. 動態限幅保護 (最大差速不超過 45，保證馬達保有平衡裕度)
  return constrain(turn_output, -45.0, 45.0);
}

// 1. 直立平衡內環 (支援小角度常態 / 大角度救援雙模式)
float Balance_PD(float angle, float target_angle, float gyro_rate, bool emergency) {
  float error = target_angle - angle;

  // 依優先級模式選用增益
  float kp = emergency ? Kp_balance_large : Kp_balance_normal;
  float kd = emergency ? Kd_balance_large : Kd_balance_normal;

  // D 項直接使用角速度提供純物理阻尼
  float balance_output = (kp * error) - (kd * gyro_rate);
  return constrain(balance_output, -255, 255);
}
// 2. 速度位移外環 (PI 控制器 - 具備重心自適應能力)
float Speed_PI(float target_spd) {
  // 採樣當前左右輪速度平均值 (Ticks / 40.5ms)
  float raw_speed = (encoder_count_L + encoder_count_R) / 2.0;
  encoder_count_L = 0;
  encoder_count_R = 0;

  // 一階低通濾波
  filtered_speed = (filtered_speed * 0.7) + (raw_speed * 0.3);

  // 速度偏差
  float speed_error = filtered_speed - target_spd;

  // ★★★ 核心修正 1：放寬累積門檻，讓溜車時能持續尋找重心 ★★★
  // 只有在速度誤差極大（高速行駛或急煞車 > 12.0 ticks）時才凍結積分
  if (abs(speed_error) < 12.0) {
    speed_integral += speed_error;
  } else {
    speed_integral *= 0.95;  // 溫和衰減，不再暴力清空
  }

  // ★★★ 核心修正 2：大幅放大積分上限 (提供 ±2.5° 的重心修正頻寬) ★★★
  // 原本 250 限制換算只有 0.07 度，放大到 3000 可提供約 2.5 度的補償能力
  speed_integral = constrain(speed_integral, -3000.0, 3000.0);

  // 外環輸出：轉換為直立環的「目標傾斜補償角 (rad)」
  // Kp_speed = 2.0, Ki_speed = 0.015 (約可輸出 ±0.045 rad ≈ ±2.6°)
  float angle_offset = (Kp_speed * speed_error + 0.015 * speed_integral) * 0.001;

  // 限幅最大補償 ±3.5 度
  return constrain(angle_offset, -3.5 * DEG_TO_RAD, 3.5 * DEG_TO_RAD);
}
// ★★★ SETUP ★★★
void setup() {
  Serial.begin(9600);  // 採用高速序列埠傳輸
  Wire.begin();
  Wire.setClock(400000);
  initMotorPins();
  stopMotors();

  pinMode(led, OUTPUT);
  pinMode(ENC_PIN_L, INPUT_PULLUP);
  pinMode(ENC_PIN_R, INPUT_PULLUP);

  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();

  // 寫入 MPU6050 晶片校準值
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
    Serial.println("DMP 啟動成功，進入正統雙閉環模式");
  } else {
    while (1)
      ;
  }

  attachInterrupt(0, Code_left, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PIN_R), Code_right, CHANGE);

  // TIMER2 CTC 中斷設定：單次比較中斷0.596ms，div_count累積17次才執行本體
  // 角度環實際執行週期 = 0.596ms × 17 ≈ 10.13ms
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
// ★★★ TIMER2 控制中斷 (高精度時序調度) ★★★
ISR(TIMER2_COMPA_vect) {
  static int div_count = 0;
  div_count++;
  if (div_count < 17) return;
  div_count = 0;

  static volatile bool is_computing = false;
  if (is_computing) return;
  is_computing = true;

  sei();  // 允許中斷巢狀以保證編碼器計數準確

  // 讀取 DMP 封包
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    is_computing = false;
    return;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    float ypr_temp[3];
    mpu.dmpGetYawPitchRoll(ypr_temp, &q, &gravity);

    // 取得當前姿態 (SI 制：rad, rad/s)
    current_angle = -ypr_temp[2];
    int16_t raw_gyro[3];
    mpu.dmpGetGyro(raw_gyro, fifoBuffer);
    current_gyro_rate = -(raw_gyro[0] / 16.4) * DEG_TO_RAD;

    // ★★★ 讀取DMP Yaw 航向角 ★★★
    current_yaw = ypr_temp[0];  // DMP 的第 0 元素正是 Yaw 角 (rad)
    float current_yaw_rate = (raw_gyro[2] / 16.4) * DEG_TO_RAD;
    // 開機站立時，自動將開機朝向設為初始目標航向
    if (!yaw_initialized && !is_fallen) {
      target_yaw = current_yaw;
      initial_yaw = current_yaw;  // ★ 記住開機的第一瞬間車頭朝向
      yaw_initialized = true;
    }
    // 計算當前相對於平衡點的絕對誤差
    float angle_err = abs(current_angle - base_setpoint);

    // =========================================================================
    // 【優先級 1：最高】極限傾倒安全保護（超過 ±35° 立即強制切斷馬達）
    // =========================================================================
    if (angle_err > fall_limit) {
      is_fallen = true;
      is_emergency_state = false;
      stopMotors();
      speed_integral = 0.0;
      filtered_speed = 0.0;
      speed_angle_target = 0.0;
      adj_setpoint = base_setpoint;
      encoder_count_L = 0;
      encoder_count_R = 0;
      recovery_counter = 0;
      target_speed_cmd = 0.0;
      target_speed = 0.0;
      yaw_initialized = false;
    }
    // =========================================================================
    // 正常運行防線（未摔倒）
    // =========================================================================
    else {
      // 扶正安全鎖定解除
      if (is_fallen) {
        if (angle_err < (3.0 * DEG_TO_RAD)) {
          recovery_counter++;
          if (recovery_counter > 20) {
            is_fallen = false;
            recovery_counter = 0;
          }
        }
      }

      if (!is_fallen) {
        // --- 狀態切換判定 (遲滯區間，避免在臨界點抖動) ---
        if (!is_emergency_state && (angle_err > LARGE_ANGLE_ENTER)) {
          is_emergency_state = true;
        } else if (is_emergency_state && (angle_err < LARGE_ANGLE_EXIT)) {
          is_emergency_state = false;
        }

        // =======================================================================
        // 【優先級 2：次高】大角度緊急姿態救援（Control Authority Override）
        // =======================================================================
        if (is_emergency_state) {
          // 1. 剝奪速度外環權限：清除積分債、清空補償角、中斷巡航
          speed_integral = 0.0;
          speed_angle_target = 0.0;
          target_speed = 0.0;
          adj_setpoint = base_setpoint;  // 唯一目標鎖死物理重心零點
          target_yaw = current_yaw;
          encoder_count_L = 0;
          encoder_count_R = 0;
          filtered_speed = 0.0;

          // 2. 呼叫大角度高阻尼 PD 控制器
          float emergency_pwm = Balance_PD(current_angle, adj_setpoint, current_gyro_rate, true);
          debug_pwm = emergency_pwm;  // ★ 記錄大角度輸出的 PWM
          // 3. 屏蔽轉向指令 (turn_cmd)，馬達 100% 頻寬全力用於拉正車身
          setMotorSpeed(emergency_pwm, emergency_pwm);
        }
        // =======================================================================
        // 【優先級 3：正常】小角度精密多速率串級控制（Normal Operation）
        // =======================================================================
        else {
          // [外環] 速度 PI 分頻運算 (約 40.5ms 執行一次)
          static int speed_loop_counter = 0;
          speed_loop_counter++;
          if (speed_loop_counter >= SPEED_LOOP_DIV) {
            speed_loop_counter = 0;

            // ★ 轉向期間持續凍結速度外環，切斷平衡環偏心干擾
            if (abs(target_yaw_rate_cmd) > 0.1 || is_homing) {
              speed_angle_target = 0.0;
              target_speed = 0.0;
              filtered_speed = 0.0;
              speed_integral = 0.0;
              encoder_count_L = 0;
              encoder_count_R = 0;
            } else {
              if (target_speed < target_speed_cmd) {
                target_speed = min(target_speed + 0.05, target_speed_cmd);
              } else if (target_speed > target_speed_cmd) {
                target_speed = max(target_speed - 0.05, target_speed_cmd);
              }
              speed_angle_target = Speed_PI(target_speed);
            }
          }

          // 串級疊加：直立環目標角
          adj_setpoint = base_setpoint + speed_angle_target;

          float balance_pwm = Balance_PD(current_angle, adj_setpoint, current_gyro_rate, false);
          debug_pwm = balance_pwm;

          // ★★★ 核心補齊：四態航向調度（自轉、直行校正、自動回正、靜止）★★★
          float final_turn = 0.0;

          // 1.【手動原地自轉 (A / D)】
          if (abs(target_yaw_rate_cmd) > 0.1) {
            is_homing = false;  // 手動操作時打斷回正程序
            target_yaw += target_yaw_rate_cmd * 0.01013;
            while (target_yaw > PI) target_yaw -= 2.0 * PI;
            while (target_yaw < -PI) target_yaw += 2.0 * PI;
            final_turn = Heading_Control_PD(current_yaw_rate);
          }
          // 2.【直行前進/後退 (W / X)】-> 啟用航向閉環，保持直線不偏斜
          else if (abs(target_speed_cmd) > 0.1) {
            is_homing = false;
            final_turn = Heading_Control_PD(current_yaw_rate);
          }
          // 3.【按下 C 鍵自動旋轉回正】
          else if (is_homing) {
            target_yaw = initial_yaw;  // 目標鎖死開機時的初始正前方

            // 計算當前與初始朝向誤差
            float homing_err = target_yaw - current_yaw;
            while (homing_err > PI) homing_err -= 2.0 * PI;
            while (homing_err < -PI) homing_err += 2.0 * PI;

            // 誤差小於 1.5 度時視為已回正，自動煞停定格
            if (abs(homing_err) < (1.5 * DEG_TO_RAD)) {
              is_homing = false;
              final_turn = 0.0;
            } else {
              // 尚未轉正，持續閉環輸出差速旋轉車身
              final_turn = Heading_Control_PD(current_yaw_rate);
            }
          }
          // 4.【靜止自平衡】-> 差速徹底關閉，目標隨時貼齊當前角度，消除晃動
          else {
            final_turn = 0.0;
            target_yaw = current_yaw;
          }

          // 合成巡航與差速轉向輸出
          float motor_L = balance_pwm + final_turn;
          float motor_R = balance_pwm - final_turn;
          setMotorSpeed(motor_L, motor_R);
        }
      }
    }
  }
  is_computing = false;
}

// ★★★ 主迴圈 (鍵盤/手把控制與狀態推播) ★★★
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 300;  // 超過 300ms 沒訊號自動煞停 (斷線防暴衝)

void loop() {
  // 1. 鍵盤 / 序列埠指令監聽
  while (Serial.available() > 0) {
    char c = Serial.read();

    // 忽略換行字元
    if (c == '\r' || c == '\n') continue;

    if (!is_fallen) {
      last_cmd_time = millis();  // 刷新指令接收時間

      switch (c) {
        // 前進 (W)
        case 'w':
        case 'W':
          target_speed_cmd = -4.5;
          break;

        // 後退 (X) - 修正：改由 X 觸發
        case 'x':
        case 'X':
          target_speed_cmd = 5.5;
          break;

        // 原地左旋 (A) - 設定目標角速度為 -2.2 rad/s (約每秒旋轉 126 度)
        case 'a':
        case 'A':
          target_yaw_rate_cmd = -2.2;
          break;

        // 原地右旋 (D) - 設定目標角速度為 +2.2 rad/s
        case 'd':
        case 'D':
          target_yaw_rate_cmd = 2.2;
          break;

        // 自動回正鍵 (C)
        case 'c':
        case 'C':
          is_homing = true;      // 啟動回正程序
          speed_integral = 0.0;  // 清空溜車積分
          Serial.println(F("\n=============================="));
          Serial.println(F(">> [指令確認] 正在自動旋轉回正中..."));
          Serial.println(F("==============================\n"));
          break;

        // 煞停 / 暫停 (S 或 空白鍵)
        case 's':
        case 'S':
        case ' ':
          target_speed_cmd = 0.0;
          target_yaw_rate_cmd = 0.0;
          speed_integral = 0.0;
          break;

        default: break;
      }
    }
  }

  // ★ 雙重防護：若傳輸線鬆脫或 Python 漏發放開訊號，逾時自動煞車
  if (!is_fallen && (target_speed_cmd != 0.0 || target_yaw_rate_cmd != 0.0)) {
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
      target_speed_cmd = 0.0;
      target_yaw_rate_cmd = 0.0;
      speed_integral = 0.0;
    }
  }

  // 2. 狀態遙測推播 (每 100ms 輸出一次)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("角度: ");
    Serial.print(current_angle * RAD_TO_DEG, 1);
    Serial.print("° | PWM: ");
    Serial.print(debug_pwm, 0);  // ★ 印出整數 PWM 數值
    Serial.print(" | 狀態: ");
    if (is_fallen) {
      Serial.println("【跌倒保護】");
    } else if (is_emergency_state) {
      Serial.println("【大角度緊急救援】");
    } else {
      Serial.println("【常態雙閉環】");
    }
  }
}