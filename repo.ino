#include "I2Cdev.h"
#include <PID_v1.h> // PID 控制函式庫
#include "MPU6050_6Axis_MotionApps20.h" // 使用 DMP 的 MPU6050 函式庫

MPU6050 mpu;

// MPU 控制/狀態變數
bool dmpReady = false;              // DMP 初始化成功與否
uint8_t mpuIntStatus;               // MPU 中斷狀態
uint8_t devStatus;                  // DMP 初始化狀態碼 (0 為成功)
uint16_t packetSize;                // DMP 資料包大小
uint16_t fifoCount;                 // FIFO 中的位元組數
uint8_t fifoBuffer[64];            // FIFO 資料緩衝區

// 姿態與重力計算變數
Quaternion q;                       // 四元數
VectorFloat gravity;                // 重力向量
float ypr[3];                       // Yaw, Pitch, Roll

// ----------- PID 參數設定（需手動調整）-----------
double setpoint = 182;              // 小車平衡角度（從序列監控取得）
double Kp = 20;                     // 比例參數（調平衡反應強度）
double Ki = 140;                    // 積分參數（抑制誤差累積）
double Kd = 0.8;                    // 微分參數（抑制震盪）

double input, output;               // PID 輸入與輸出變數

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// MPU 中斷旗標（由中斷服務程式設定）
volatile bool mpuInterrupt = false;

// MPU 中斷觸發處理（僅設置旗標）
void dmpDataReady() {
  mpuInterrupt = true;
}

// 馬達前進
void Forward() {
  analogWrite(6, output);
  analogWrite(9, 0);
  analogWrite(10, output);
  analogWrite(11, 0);
  Serial.print("F");
}

// 馬達後退
void Reverse() {
  analogWrite(6, 0);
  analogWrite(9, -output);  // 注意：output 為負值
  analogWrite(10, 0);
  analogWrite(11, -output);
  Serial.print("R");
}

// 馬達停止
void Stop() {
  analogWrite(6, 0);
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(11, 0);
  Serial.print("S");
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("初始化 I2C 裝置..."));
  mpu.initialize();  // 啟動 MPU6050

  Serial.println(F("檢查 MPU6050 是否連線..."));
  Serial.println(mpu.testConnection() ? F("成功") : F("失敗"));

  // 初始化 DMP
  devStatus = mpu.dmpInitialize();

  // 設定陀螺儀與加速度計偏移值（視硬體調整）
  mpu.setXGyroOffset(57);
  mpu.setYGyroOffset(-29);
  mpu.setZGyroOffset(3);
  mpu.setZAccelOffset(967);

  if (devStatus == 0) {
    Serial.println(F("啟用 DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("啟用外部中斷 INT0..."));
    attachInterrupt(0, dmpDataReady, RISING); // D2 腳位

    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize(); // 計算 DMP 資料包大小

    // 設定 PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);                 // 每 10ms 更新一次
    pid.SetOutputLimits(-255, 255);        // 對應 PWM 最大範圍
  } else {
    Serial.print(F("DMP 初始化失敗，錯誤碼："));
    Serial.println(devStatus);
    while (1) {
      Serial.println(F("請檢查 MPU 接線或硬體是否正常"));
      delay(1000);  // 停止執行
    }
  }

  // 馬達控制腳位設定為輸出
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // 預設關閉馬達
  Stop();
}

void loop() {
  // 如果 DMP 還沒準備好，就不執行主邏輯
  if (!dmpReady) return;

  // 若無中斷觸發且 FIFO 不足一包資料，就繼續運作 PID 控制
  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();  // 執行 PID 控制
    Serial.print("角度："); Serial.print(input);
    Serial.print(" => PID 輸出："); Serial.println(output);

    if (input > 150 && input < 200) { // 角度偏移合理範圍
      if (output > 0)
        Forward(); // 向前傾倒，驅動前進
      else if (output < 0)
        Reverse(); // 向後傾倒，驅動後退
    } else {
      Stop(); // 角度過大，直接停車
    }
  }

  // 清除中斷旗標
  mpuInterrupt = false;

  // 讀取中斷狀態
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // 處理 FIFO 溢出錯誤
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO 溢出錯誤！已重置"));
    return;
  }

  // 有有效的 DMP 資料可讀取
  if (mpuIntStatus & 0x02) {
    // 確保 FIFO 至少一包資料
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // 讀取 DMP 資料包
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // 解析 DMP 資料
    mpu.dmpGetQuaternion(&q, fifoBuffer);          // 四元數
    mpu.dmpGetGravity(&gravity, &q);               // 重力向量
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);     // 姿態角度

    // 將 Pitch（俯仰）角轉換為角度作為 PID 輸入
    input = ypr[1] * 180 / M_PI + 180;  // 轉換為 0~360 度範圍
  }
}
