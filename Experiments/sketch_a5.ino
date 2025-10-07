#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 imu(Wire);

// 馬達控制腳位
#define IN1M 7
#define IN2M 6
#define PWMA 9
#define IN3M 13
#define IN4M 12
#define PWMB 10
#define STBY 8

// 控制參數
float Kp = 2500.0;
float Kd = 20.0;
float baseSetPoint = 0.0;

const int sampleCount = 3;
const int MIN_PWM = 10;

unsigned long lastTime = 0;
const int interval = 1;

float prevGyroY = 0;

void initMotorPins() {
  pinMode(IN1M, OUTPUT); pinMode(IN2M, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(IN3M, OUTPUT); pinMode(IN4M, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

float degToRad(float deg) {
  return deg * 3.1415926 / 180.0;
}

void setMotorSpeed(float speed) {
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);
  if (pwm < MIN_PWM) pwm = 0;

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
  Serial.begin(115200);
  initMotorPins();
  Wire.begin();

  if (imu.begin() != 0) {
    Serial.println("MPU6050 初始化失敗");
    while (1);
  }

  Serial.println("校正中...");
  delay(1000);
  imu.calcOffsets();
  Serial.println("完成");
}

void loop() {
  if (millis() - lastTime >= interval) {
    lastTime = millis();

    imu.update();

    // 平均 pitch 角度
    float sumRad = 0;
    for (int i = 0; i < sampleCount; i++) {
      imu.update();
      float pitchDeg = imu.getAngleX();
      sumRad += degToRad(pitchDeg);
      delay(2);
    }
    float pitchRad = sumRad / sampleCount;

    // 當前角速度（rad/s）
    float gyroY = degToRad(imu.getGyroY());

    // PD 控制
    float error = pitchRad - baseSetPoint;
    float output = Kp * error - Kd * gyroY;

    // === 進階補償：偵測加速倒下 ===
    float gyroAccel = gyroY - prevGyroY; // 角速度的變化量

    bool tippingForward = (pitchRad > 0 && gyroY > 0);
    bool tippingBackward = (pitchRad < 0 && gyroY < 0);
    bool speedingUp = abs(gyroAccel) > 0.01;  // 調整這個門檻

    if ((tippingForward || tippingBackward) && speedingUp) {
      output *= 1.5;  // 補償加大輸出（放大倍數可微調）
    }

    // 限制輸出範圍
    output = constrain(output, -255, 255);

    setMotorSpeed(output);
    prevGyroY = gyroY;  // 更新前一筆速度

    // 除錯資訊
    Serial.print("Pitch(rad): "); Serial.print(pitchRad, 4);
    Serial.print(" | GyroY: "); Serial.print(gyroY, 4);
    Serial.print(" | GyroAccel: "); Serial.print(gyroAccel, 4);
    Serial.print(" | Output: "); Serial.println(output);
  }
}
