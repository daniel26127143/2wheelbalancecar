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
float Kp_balance = 1275.0;   // 比例推力 (對抗傾斜) 1300 > x >1200
float Kd_balance = 70.0;     // 微分阻尼 (抑制前後震盪) 80 > x >60

// 2. 速度外環 (Speed PI) - 分頻執行
float Kp_speed = 2.3;        // 速度比例增益 2.8 > x >2.6
float Ki_speed = 0.015;       // 速度積分增益 (消除重心偏移溜車)
#define SPEED_LOOP_DIV 4      // 角度環實測週期10.13ms，4倍≈40.5ms，落在40~50ms需求區間

// 3. 重心與保護常數
float base_setpoint = -1.8 * DEG_TO_RAD; // 機械重心零點 (rad)
volatile float adj_setpoint = base_setpoint;
float fall_limit = 35.0 * DEG_TO_RAD;     // 傾倒保護限制 (超過 ±35 度斷電)

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

// 濾波與中間狀態
float current_angle = 0.0;     // 當前車身傾角 (rad)
float current_gyro_rate = 0.0; // 當前俯仰角速度 (rad/s)
float filtered_speed = 0.0;    // 低通濾波後的車輪速度
float speed_integral = 0.0;    // 速度環積分
float speed_angle_target = 0.0;// 速度環計算出的傾角補償量 (rad)
float target_speed = 0.0;      // 目標速度 (原地平衡為 0)
float turn_cmd = 0.0;          // 轉向命令

// ★★★ 底層驅動函式 ★★★
void initMotorPins() {
  pinMode(IN1M, OUTPUT); pinMode(IN2M, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(IN3M, OUTPUT); pinMode(IN4M, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void setMotorSpeed(float speedL, float speedR) {
  speedL = constrain(speedL * 1.037, -255, 255);
  speedR = constrain(speedR, -255, 255);

  int deadzone = 5; // 
  int pwmL = (speedL > 0) ? (speedL + deadzone) : ((speedL < 0) ? (speedL - deadzone) : 0);
  int pwmR = (speedR > 0) ? (speedR + deadzone) : ((speedR < 0) ? (speedR - deadzone) : 0);

  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  // 左輪方向
  if (pwmL >= 0) {
    digitalWrite(IN1M, LOW); digitalWrite(IN2M, HIGH); is_L_Forward = true;
  } else {
    digitalWrite(IN1M, HIGH); digitalWrite(IN2M, LOW); is_L_Forward = false;
  }

  // 右輪方向
  if (pwmR >= 0) {
    digitalWrite(IN3M, LOW); digitalWrite(IN4M, HIGH); is_R_Forward = true;
  } else {
    digitalWrite(IN3M, HIGH); digitalWrite(IN4M, LOW); is_R_Forward = false;
  }

  analogWrite(PWMA, abs(pwmL));
  analogWrite(PWMB, abs(pwmR));
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(IN1M, LOW); digitalWrite(IN2M, LOW);
  digitalWrite(IN3M, LOW); digitalWrite(IN4M, LOW);
}

// 編碼器中斷
void Code_left()  { encoder_count_L += is_L_Forward ? 1 : -1; }
void Code_right() { encoder_count_R += is_R_Forward ? 1 : -1; }

// ★★★ 核心控制演算法 ★★★

// 1. 直立平衡內環 (PD 控制器 - 高頻調度)
float Balance_PD(float angle, float target_angle, float gyro_rate) {
  float error = target_angle - angle;
  // D 項直接使用角速度 (提供精確物理阻尼)
  float balance_output = (Kp_balance * error) - (Kd_balance * gyro_rate);
  return constrain(balance_output, -255, 255);
}

// 2. 速度位移外環 (PI 控制器 - 低頻調度)
float Speed_PI(float target_spd) {
  // 採樣當前左右輪速度平均值 (Ticks / 43ms)
  float raw_speed = (encoder_count_L + encoder_count_R) / 2.0;
  encoder_count_L = 0;
  encoder_count_R = 0;

  // 一階低通濾波，抑制齒輪敲擊高頻抖動
  filtered_speed = (filtered_speed * 0.7) + (raw_speed * 0.3);

  // 速度偏差 (以車身傾角修正消除溜車)
  float speed_error = filtered_speed - target_spd;
  speed_integral += speed_error;
  speed_integral = constrain(speed_integral, -800.0, 800.0); // 嚴防積分飽和

  // 外環輸出：轉換為直立環的「目標傾斜補償角 (rad)」
  float angle_offset = (Kp_speed * speed_error + Ki_speed * speed_integral) * 0.001;
  return constrain(angle_offset, -4.0 * DEG_TO_RAD, 4.0 * DEG_TO_RAD); // 限幅最大補償 ±4 度
}

// ★★★ SETUP ★★★
void setup() {
  Serial.begin(115200); // 採用高速序列埠傳輸
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
    while (1);
  }

  attachInterrupt(0, Code_left, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PIN_R), Code_right, CHANGE);

  // TIMER2 CTC 中斷設定：單次比較中斷0.596ms，div_count累積17次才執行本體
  // 角度環實際執行週期 = 0.596ms × 17 ≈ 10.13ms
  cli();
  TCCR2A = 0; TCCR2B = 0; TCNT2 = 0;
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

  sei(); // 允許中斷巢狀以保證編碼器計數準確

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

    // 1. 傾倒安全檢查
    if (abs(current_angle - base_setpoint) > fall_limit) {
      is_fallen = true;
      stopMotors();
      speed_integral = 0.0;
      encoder_count_L = 0;   // 跌倒期間車輪仍可能被搬動而觸發編碼器，順便歸零避免扶正瞬間假速度暴衝
      encoder_count_R = 0;
      recovery_counter = 0;
    } 
    // 2. 正常自平衡控制
    else {
      // 扶正判斷
      if (is_fallen) {
        if (abs(current_angle - base_setpoint) < (3.0 * DEG_TO_RAD)) {
          recovery_counter++;
          if (recovery_counter > 20) { // 20次 × 10.13ms ≈ 0.2秒，穩定維持0.2秒後解除保護
            is_fallen = false;
            recovery_counter = 0;
          }
        }
      }

      if (!is_fallen) {
        // [外環] 速度 PI 分頻運算 (約 43ms 執行一次)
        static int speed_loop_counter = 0;
        speed_loop_counter++;
        if (speed_loop_counter >= SPEED_LOOP_DIV) {
          speed_loop_counter = 0;
          speed_angle_target = Speed_PI(target_speed);
        }

        // 串級疊加：直立環目標角 = 機械基準角 + 速度環補償角
        adj_setpoint = base_setpoint + speed_angle_target;

        // [內環] 直立環 PD 運算 (約 3.58ms 執行一次)
        float balance_pwm = Balance_PD(current_angle, adj_setpoint, current_gyro_rate);

        // 合成最終馬達輸出
        float motor_L = balance_pwm + turn_cmd;
        float motor_R = balance_pwm - turn_cmd;
        setMotorSpeed(motor_L, motor_R);
      }
    }
  }

  is_computing = false;
}

// ★★★ 主迴圈 (僅負責監控與人機介面，不介入控制時序) ★★★
void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    // 顯示轉換為「度 (deg)」輸出，便於工程肉眼觀測
    Serial.print("角度: ");
    Serial.print(current_angle * RAD_TO_DEG, 2);
    Serial.print("° | 目標: ");
    Serial.print(adj_setpoint * RAD_TO_DEG, 2);
    Serial.print("° | 速度濾波: ");
    Serial.print(filtered_speed, 1);
    Serial.print(" | 狀態: ");
    Serial.println(is_fallen ? "倒下保護" : "平衡運行");
  }
}
