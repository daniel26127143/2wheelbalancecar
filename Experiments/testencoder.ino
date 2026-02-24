#include <Wire.h>

// ==========================================
// 硬體設定
// ==========================================
// 根據說明書計算的理論值
const float TICKS_PER_REV = 660.0;   // 轉一圈的理論 Ticks (330*2)
const float TICKS_PER_METER = 3235.0; // 走一公尺的理論 Ticks

// Encoder 腳位 
#define ENC_PIN_L 2   
#define ENC_PIN_R A1  

// 馬達腳位
#define IN1M 7
#define IN2M 6
#define PWMA 9
#define IN3M 13
#define IN4M 12
#define PWMB 10
#define STBY 8

// CT變數
volatile long count_L = 0;
volatile long count_R = 0;

// ==========================================
// 初始化
// ==========================================
void initMotors() {
  pinMode(IN1M, OUTPUT); pinMode(IN2M, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(IN3M, OUTPUT); pinMode(IN4M, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  
  // Encoder 輸入設定
  pinMode(ENC_PIN_L, INPUT_PULLUP);
  pinMode(ENC_PIN_R, INPUT_PULLUP); // A1 當數位腳用
}

void setSpeed(int speed) {
  // 設定 PWM 100 前進
  digitalWrite(IN1M, LOW); digitalWrite(IN2M, HIGH); analogWrite(PWMA, speed);
  digitalWrite(IN3M, LOW); digitalWrite(IN4M, HIGH); analogWrite(PWMB, speed);
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(9600);
  initMotors();
  
  // 設定 Timer2 中斷 (每 0.5ms 讀取一次 Encoder)
  cli();
  TCCR2A = 0; TCCR2B = 0; TCNT2 = 0; OCR2A = 124;
  TCCR2A |= (1 << WGM21); TCCR2B |= (1 << CS22); TIMSK2 |= (1 << OCIE2A);
  sei();

  Serial.println("=== Encoder Test Start ===");
  Serial.println("Motor Running at PWM 100...");
  Serial.println("Please count the wheel rotations!");
  delay(1000);
}

// ==========================================
// Timer ISR (讀取 Encoder)
// ==========================================
ISR(TIMER2_COMPA_vect) {
  static byte lastL = 0;
  byte currL = (PIND & 0x04);
  if (currL != lastL) { count_L++; lastL = currL; }
  static byte lastR = 0;
  byte currR = (PINC & 0x02); 
  if (currR != lastR) { count_R++; lastR = currR; }
}
// ==========================================
// Loop (執行 50 次自動測試)
// ==========================================
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 't' || cmd == 'T') {
      int test_times = 100;           // 測試次數
      unsigned long total_L = 0;     // 用來加總左輪的數據
      unsigned long total_R = 0;     // 用來加總右輪的數據

      Serial.println("\n測試開始");
      
      for (int i = 1; i <= test_times; i++) {
        // 每次測試前歸零
        count_L = 0;
        count_R = 0;
        
        // 啟動馬達 使用pwm=100
        setSpeed(100);
        
        // 讓它跑 10 秒鐘
        delay(10000); 
        
        // 停止馬達
        setSpeed(0);
        
        // 抓取這 10 秒的數值
        long current_L = count_L;
        long current_R = count_R;
        
        // 輸出當次數據
        Serial.print("第 "); Serial.print(i); Serial.print(" 次 | ");
        Serial.print("左輪 Ticks: "); Serial.print(current_L);
        Serial.print(" | 右輪 Ticks: "); Serial.println(current_R);
        
        // 將當次數據加入總和
        total_L += current_L;
        total_R += current_R;
        
        // 煞車緩衝 1 秒，等輪子完全停下再跑下一次
        delay(1000); 
      }
      
      // 計算平均值
      Serial.println("測試完成");
      
      float avg_L = (float)total_L / test_times;
      float avg_R = (float)total_R / test_times;
      
      Serial.print("平均 左輪 Ticks: "); Serial.println(avg_L, 2);
      Serial.print("平均 右輪 Ticks: "); Serial.println(avg_R, 2);
      
      if (avg_L > 0 && avg_R > 0) {
        float ratio_R = avg_L / avg_R;
        float ratio_L = avg_R / avg_L;
        
        if (avg_L > avg_R) {
          Serial.print("avg_L > avg_R R需x");
          Serial.println(ratio_R, 4);
        } else if (avg_R > avg_L) {
          Serial.print("avg_R > avg_L L需x");
          Serial.println(ratio_L, 4);
        } else {
          Serial.println("?");
        }
      }
      Serial.println("輸入T重新測試");
    }
  }
}