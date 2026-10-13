// ============================================================================
// rec.h — 診斷錄影（配合 controller/bal_rec.py；從 repo.ino 原封不動搬出來）
//   r 鍵：站立錄影（每 10 ms 一筆）；m 鍵：移動錄影（每 40 ms 一筆）。錄 REC_N 筆後由 loop() 印出 REC: 行
//   用法：repo.ino 裡 #define ENABLE_REC 1 才會啟用（約多 1.3 KB Flash，會超過 UNO 上限，要先拿掉別的功能）
//   ENABLE_REC 為 0 時，下面三個巨集都是空的，編譯後不佔任何空間，也不會多做任何運算
// ============================================================================
#ifndef REC_H
#define REC_H

#ifndef ENABLE_REC
#define ENABLE_REC 0
#endif

#if ENABLE_REC

// [v28] 診斷錄影：按 r 錄 100 筆（每個控制週期約 10 ms 一筆），錄完在 loop() 印出 REC: 行
//       每筆 = 角度(0.01°)、DMP 陀螺儀(°/s)、目標角(0.05°)、直立環輸出、左輪 PWM、暫存器 X 軸陀螺儀(°/s)
#define REC_N 64
static int16_t rec_ang[REC_N];
static int8_t rec_gyro[REC_N];   // 陀螺儀 (°/s)，直立環 D 項實際用的值
static int8_t rec_sp[REC_N];     // 目前直立目標角 adj_setpoint (0.05°)
static int8_t rec_bal[REC_N];    // 直立環輸出 balance_pwm
static int8_t rec_pwm[REC_N];
static int8_t rec_dl[REC_N];
static volatile uint8_t rec_state = 0;  // 0 閒置，1 錄影中，2 錄完待輸出
static uint8_t rec_i = 0;
static uint8_t rec_mode = 0;   // 0 = 站立模式（每週期一筆）；1 = 移動模式（每 4 週期一筆，錄 2.6 s，欄位換成速度外環資料）
static uint8_t rec_div = 0;

// 在控制中斷（ISR）最後呼叫：記錄「這個週期實際送出的 PWM」
static inline void Rec_Capture(MPU6050& mpu, float current_angle, float current_gyro_rate, float adj_setpoint,
                               float debug_balance_pwm, float debug_v, float debug_tv, float debug_sa, float debug_bs,
                               int debug_pwm_L) {
  if (rec_state == 1 && (rec_mode == 0 || (rec_div++ & 3) == 0)) {
    rec_ang[rec_i] = (int16_t)(current_angle * RAD_TO_DEG * 100.0f);
    if (rec_mode == 0) {
      rec_gyro[rec_i] = (int8_t)constrain(current_gyro_rate * RAD_TO_DEG, -127.0f, 127.0f);
      rec_sp[rec_i] = (int8_t)constrain(adj_setpoint * RAD_TO_DEG * 20.0f, -127.0f, 127.0f);
      rec_bal[rec_i] = (int8_t)constrain(debug_balance_pwm, -127.0f, 127.0f);
      rec_dl[rec_i] = (int8_t)constrain(mpu.getRotationX() / 16.4f, -127.0f, 127.0f);  // 直接讀暫存器的 X 軸陀螺儀 (°/s)，和上面 DMP 封包的陀螺儀對照
    } else {  // 移動模式：這四欄改記 速度×4、目標速度×4、速度外環補償角×20(°)、起步推力×50(°)
      rec_gyro[rec_i] = (int8_t)constrain(debug_v * 4.0f, -127.0f, 127.0f);
      rec_sp[rec_i] = (int8_t)constrain(debug_tv * 4.0f, -127.0f, 127.0f);
      rec_bal[rec_i] = (int8_t)constrain(debug_sa * 20.0f, -127.0f, 127.0f);
      rec_dl[rec_i] = (int8_t)constrain(debug_bs * 50.0f, -127.0f, 127.0f);
    }
    rec_pwm[rec_i] = (int8_t)constrain(debug_pwm_L, -127, 127);
    if (++rec_i >= REC_N) rec_state = 2;
  }
}

// 收到字元時呼叫：r / m 開始錄影。回傳 true 代表這個字元已被錄影功能吃掉
static inline bool Rec_HandleKey(char c) {
  if (c == 'r' || c == 'R' || c == 'm' || c == 'M') {  // [v28] 診斷錄影：r = 站立（10 ms 一筆），m = 移動（40 ms 一筆）
    if (rec_state == 0) {
      rec_mode = (c == 'm' || c == 'M') ? 1 : 0;
      rec_div = 0;
      rec_i = 0;
      rec_state = 1;
      Serial.println(F(">> REC 開始"));
    }
    return true;
  }
  return false;
}

// 在 loop() 呼叫：錄完（rec_state == 2）就把資料印出來
static inline void Rec_Output(float kd_normal) {
  if (rec_state == 2) {  // [v28] 輸出錄影：REC:序號,角度(0.01°),陀螺儀(°/s),目標角(0.05°),直立環輸出,PWM_L,dL
    Serial.print(F("REC:MODE,"));
    Serial.println(rec_mode);
    Serial.print(F("REC:KD,"));
    Serial.println(kd_normal, 0);
    for (uint8_t i = 0; i < REC_N; i++) {
      Serial.print(F("REC:"));
      Serial.print(i); Serial.print(',');
      Serial.print(rec_ang[i]); Serial.print(',');
      Serial.print(rec_gyro[i]); Serial.print(',');
      Serial.print(rec_sp[i]); Serial.print(',');
      Serial.print(rec_bal[i]); Serial.print(',');
      Serial.print(rec_pwm[i]); Serial.print(',');
      Serial.println(rec_dl[i]);
    }
    Serial.println(F("REC:END"));
    rec_state = 0;
  }
}

#define REC_CAPTURE(...) Rec_Capture(__VA_ARGS__)
#define REC_KEY(c)       Rec_HandleKey(c)
#define REC_OUTPUT(kd)   Rec_Output(kd)

#else

#define REC_CAPTURE(...) ((void)0)
#define REC_KEY(c)       (false)
#define REC_OUTPUT(kd)   ((void)0)

#endif  // ENABLE_REC
#endif  // REC_H
