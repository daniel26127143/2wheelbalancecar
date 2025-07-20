// ===== 1. 定義馬達引腳 =====
#define IN1M   7    // 馬達 A 方向引腳 1
#define IN2M   6    // 馬達 A 方向引腳 2
#define PWMA   9    // 馬達 A PWM 控制引腳

#define IN3M   13   // 馬達 B 方向引腳 1
#define IN4M   12   // 馬達 B 方向引腳 2
#define PWMB   10   // 馬達 B PWM 控制引腳

#define STBY   8    // TB6612 使能引腳（須拉高）

// ===== 2. 初始化引腳 =====
void initMotorPins() {
  pinMode(IN1M, OUTPUT);
  pinMode(IN2M, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(IN3M, OUTPUT);
  pinMode(IN4M, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);

  // 使能驅動器
  digitalWrite(STBY, HIGH);

  // 預設停止狀態
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, LOW);
  analogWrite(PWMB, 0);
}

// ===== 3. 簡易測試馬達 =====
// 正轉、反轉、停止，每段持續 1 秒
void testMotors() {
  Serial.println("測試A馬達");
  // 馬達 A 正轉
  digitalWrite(IN1M, HIGH);
  digitalWrite(IN2M, LOW);
  analogWrite(PWMA, 200);
  delay(1000);

  // 馬達 A 停止
  analogWrite(PWMA, 0);
  delay(500);

  // 馬達 A 反轉
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, HIGH);
  analogWrite(PWMA, 200);
  delay(1000);

  // 停止
  analogWrite(PWMA, 0);
  delay(500);
  Serial.println("測試B馬達");
  // 馬達 B 正轉
  digitalWrite(IN3M, HIGH);
  digitalWrite(IN4M, LOW);
  analogWrite(PWMB, 200);
  delay(1000);

  // 停止
  analogWrite(PWMB, 0);
  delay(500);

  // 馬達 B 反轉
  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, HIGH);
  analogWrite(PWMB, 200);
  delay(1000);

  // 停止
  analogWrite(PWMB, 0);
  delay(500);
}

// ===== 4. 主程式框架 =====
void setup() {
  Serial.begin(9600);
  initMotorPins();
  Serial.println("開始測試馬達引腳...");
}

void loop() {
  testMotors();
  Serial.println("測試完成，暫停 5 秒");
  delay(5000);
}

