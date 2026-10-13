// ============================================================================
// tuning.h — 現場調參按鍵（從 repo.ino 的 loop() 原封不動搬出來）
//   o       ：速度外環 開 / 關（關閉時為 setpoint 量測模式，每次關閉位移歸零）
//   g / h   ：直立環 Kd -10 / +10（上限 150；緊急模式 Kd = 1.2 倍）
//   j / l   ：速度外環 Kp_speed -0.5 / +0.5（範圍 0.5 ~ 10）
//   [ / ]   ：base_setpoint -0.1° / +0.1°
//   用法：repo.ino 裡 #define ENABLE_TUNING 1 才會啟用；0 時下面的巨集是 (false)，
//         按鍵整段不編譯，Kd / Kp_speed / base_setpoint / speed_loop_enabled 也變成 const（約省 0.7 KB Flash）。
//   注意：ENABLE_TUNING 為 0 時這四個參數不能再用按鍵改，要改只能改程式後重新上傳。
//   本檔要在 Kd_balance_normal、Kp_speed、base_setpoint、speed_loop_enabled、odo_ticks2 宣告之後才 #include。
// ============================================================================
#ifndef TUNING_H
#define TUNING_H

#ifndef ENABLE_TUNING
#define ENABLE_TUNING 0
#endif

#if ENABLE_TUNING

// 回傳 true 代表這個字元是調參鍵、已處理完畢
static bool Tune_Key(char c) {
  switch (c) {
    // [v10] setpoint 量測：o = 開/關速度外環，[ = setpoint -0.1°，] = setpoint +0.1°
    case 'o':
    case 'O':
      speed_loop_enabled = !speed_loop_enabled;
      if (!speed_loop_enabled) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { odo_ticks2 = 0; }  // [v11] 每次關閉外環時位移歸零
      }
      Serial.println(speed_loop_enabled ? F(">> 速度外環 ON") : F(">> 速度外環 OFF（setpoint 量測模式）"));
      return true;
    // [v28] g / h：直立環 Kd -10 / +10（緊急模式 Kd = 1.2 倍）
    case 'g':
    case 'h': {
      float kd = constrain(Kd_balance_normal + ((c == 'h') ? 10.0f : -10.0f), 0.0f, 150.0f);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { Kd_balance_normal = kd; Kd_balance_large = kd * 1.2f; }
      Serial.print(F(">> Kd = "));
      Serial.println(kd, 0);
      return true;
    }
    case 'j':
    case 'l': {  // [v29] 速度外環 Kp：j 減 0.5 / l 加 0.5（k 已是馬達斷電）
      float kp = constrain(Kp_speed + ((c == 'l') ? 0.5f : -0.5f), 0.5f, 10.0f);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { Kp_speed = kp; }
      Serial.print(F(">> Kp_speed = "));
      Serial.println(kp, 1);
      return true;
    }
    case '[':
    case ']': {
      float step = (c == ']') ? (0.1f * DEG_TO_RAD) : (-0.1f * DEG_TO_RAD);
      float sp_deg;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        base_setpoint += step;
        sp_deg = base_setpoint * RAD_TO_DEG;
      }
      Serial.print(F(">> base_setpoint = "));
      Serial.println(sp_deg, 2);
      return true;
    }
    default:
      return false;
  }
}
#define TUNE_KEY(c) Tune_Key(c)

#else

#define TUNE_KEY(c) (false)

#endif  // ENABLE_TUNING
#endif  // TUNING_H
