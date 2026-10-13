// ============================================================================
// nav_cmd.h — 導航指令的序列埠解析與回報（只給 Arduino 用，從 repo.ino 原封不動搬出來）
//   Nav_HandleLine()：解析 #T / #M / #C / #F / #S / #A 一行指令，丟給 nav 狀態機；[v32] #D 連續遙控也在這裡
//   Nav_Report()     ：導航完成 / 逾時 / 中止時印出 NAV:... 回報
// 這個檔案要在 repo.ino 的全域變數（nav、odo_x、current_yaw ...）宣告之後才能 #include。
// ============================================================================
#ifndef NAV_CMD_H
#define NAV_CMD_H

// [v27] 導航指令：「#」開頭、換行結尾，例如 "#T 45.0\n"、"#M 20\n"、"#A\n"
static char nav_buf[20];
static uint8_t nav_len = 0;
static bool nav_in_cmd = false;
static unsigned long last_rx_ms = 0;   // 最後收到任何字元的時間（導航時的斷線保護）

// 小型數字解析（不用 atof，省下約 1.5 KB 程式空間）：支援 "-12.5"、" 45"、"+3"
float Nav_ParseNum(const char*& p) {
  while (*p == ' ' || *p == ',') p++;
  bool neg = false;
  if (*p == '-' || *p == '+') { neg = (*p == '-'); p++; }
  float v = 0.0f, scale = 1.0f;
  bool frac = false;
  for (; *p; p++) {
    if (*p == '.') { frac = true; continue; }
    if (*p < '0' || *p > '9') break;
    if (frac) { scale *= 0.1f; v += (*p - '0') * scale; }
    else v = v * 10.0f + (*p - '0');
  }
  return neg ? -v : v;
}

static bool Nav_HasNum(const char* p) {
  while (*p == ' ' || *p == ',') p++;
  return (*p >= '0' && *p <= '9') || *p == '-' || *p == '+' || *p == '.';
}

void Nav_HandleLine() {
  nav_buf[nav_len] = '\0';
  char k = nav_buf[0];
  if (k >= 'a' && k <= 'z') k -= 32;
  const char* p = nav_buf + 1;
  float v = Nav_ParseNum(p);
  bool has_h = Nav_HasNum(p);
  float h = has_h ? Nav_ParseNum(p) : 0.0f;
  float w = (has_h && Nav_HasNum(p)) ? Nav_ParseNum(p) : 0.0f;  // [v30] #C 的第三個數字：前進目標速度
  if (k == 'D') {  // [v32] #D <速度> <角速度>：連續遙控。速度 + 前進（ticks/40ms，同 #M 的速度單位）、角速度 + 右轉（rad/s）
    if (is_fallen) return;                              // 倒地中直接忽略（10 Hz 重送，不回報以免洗版）
    if (Nav_Active(&nav)) nav.req_abort = NAV_AB_USER;  // 遙控接手：中止導航
    last_cmd_time = millis();
    is_homing = false;
    float dv = Nav_Clamp(v, -20.0f, 20.0f);            // [v34] 前進/後退上限皆 20（只是防呆；實際速度由速度環與馬達決定）
    bool spin = (dv < 0.5f && dv > -0.5f);              // 幾乎不前進 → 原地自轉（同 A / D）
    float lim = spin ? 1.0f : 0.6f;                     // 自轉上限 ±1.0 rad/s；邊走邊轉上限 ±0.6 rad/s
    float dw = Nav_Clamp(has_h ? h : 0.0f, -lim, lim);
    target_speed_cmd = spin ? 0.0f : -dv;               // 內部前進為負
    target_yaw_rate_cmd = spin ? dw : 0.0f;
    drive_w = spin ? 0.0f : dw;
    return;
  }
  if (k == 'A' || k == 'S') {
    nav.req_abort = NAV_AB_CMD;
    return;  // 結果由 Nav_Report() 回報 NAV:ABORT CMD
  }
  if (k != 'T' && k != 'M' && k != 'C' && k != 'F') {
    Serial.println(F("NAV:REJECT ?"));
    return;
  }
  if (k == 'C' && (!has_h || h == 0.0f || v <= 0.0f)) {  // [v30] #C <弧長 > 0> <半徑 ≠ 0>
    Serial.println(F("NAV:REJECT C"));
    return;
  }
  if (is_fallen) {
    Serial.println(F("NAV:REJECT FALLEN"));
    return;
  }
  // 開始導航前清掉手動指令
  target_speed_cmd = 0.0f;
  target_yaw_rate_cmd = 0.0f;
  is_homing = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    nav.req_value = (k == 'T') ? (v * DEG_TO_RAD) : (v * TICKS_PER_CM);
    nav.req_heading = (k == 'C') ? 1.0f / (h * TICKS_PER_CM) : h * DEG_TO_RAD;  // [v30] #C：曲率 rad/tick
    nav.req_has_heading = (k == 'M') && has_h;
    nav.req_tv = (k == 'C') ? w : 0.0f;
    nav.req_kind = k;  // 最後才寫：ISR 看到 kind 時 value 已經就緒
  }
  Serial.print(F("NAV:START "));
  Serial.print(k);
  Serial.print(' ');
  Serial.print(v, 1);
  if ((k == 'M' || k == 'C') && has_h) {
    Serial.print(' ');
    Serial.print(h, 1);
  }
  Serial.println();
}

void Nav_Report() {
  if (nav.result == NAV_RES_NONE) return;
  uint8_t res, kind, reason;
  float err, px, py, h;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    res = nav.result;
    kind = nav.result_kind;
    reason = nav.result_reason;
    err = nav.result_err;
    nav.result = NAV_RES_NONE;
    px = odo_x / TICKS_PER_CM;
    py = odo_y / TICKS_PER_CM;
    h = wrapPi(current_yaw - odo_yaw0) * RAD_TO_DEG;
  }
  if (res == NAV_RES_DONE) Serial.print(F("NAV:DONE "));
  else if (res == NAV_RES_TIMEOUT) Serial.print(F("NAV:TIMEOUT "));
  else {
    Serial.print(F("NAV:ABORT "));
    switch (reason) {
      case NAV_AB_USER: Serial.print(F("USER ")); break;
      case NAV_AB_FALL: Serial.print(F("FALL ")); break;
      case NAV_AB_EMERG: Serial.print(F("EMERG ")); break;
      case NAV_AB_LINK: Serial.print(F("LINK ")); break;
      default: Serial.print(F("CMD ")); break;
    }
  }
  Serial.print((char)(kind ? kind : '-'));
  Serial.print(F(" err="));
  Serial.print(kind == 'T' ? err * RAD_TO_DEG : err / TICKS_PER_CM, 1);  // 轉向：度；直走：公分
  Serial.print(F(" PX:"));
  Serial.print(px, 1);
  Serial.print(F(" PY:"));
  Serial.print(py, 1);
  Serial.print(F(" H:"));
  Serial.println(h, 1);
}

#endif
