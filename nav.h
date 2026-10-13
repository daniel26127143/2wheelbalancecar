// ============================================================================
// nav.h — 座標導航邏輯（純邏輯，不依賴 Arduino，電腦上也能編譯測試）
//
// 兩個動作（由 Python 腳本依「實際位置」算好後送來）：
//   #T <角度>          轉到指定方向（度，相對座標原點的 X 軸；往右 / D 方向為正）
//   #M <距離> [角度]   直走指定距離（公分，負數 = 後退）；有給角度就鎖在那個方向
//   #C <弧長> <半徑>   邊走邊轉（走圓弧）：弧長公分（只能前進）、半徑公分（正 = 右轉，負 = 左轉）
//                      目標方向 = 起點方向 + 已走弧長 / 半徑，並把角速度前饋給航向環
//                      可再加第三個數字 = 前進目標速度（預設 5.8，上限 12），例如 #C 94 -15 7
//                      弧線進行中又收到 #C：不停車，直接串接成新的弧線（連續走過多個點用）
//   #F                 結束弧線：沿著圓弧慢慢減速停下（#C 的弧長設很長時，用這個指令叫它停）
//   #S（或 #A）        中止
// 韌體每 10 ms 呼叫 Nav_TurnRate()（轉向中），每 40 ms 呼叫 Nav_MoveSpeed()（直走中）
// 完成 / 逾時 / 中止時 result 會被設定，由 loop() 印出 NAV:... 回報給 Python
//
// 單位：位置 ticks、角度 rad、速度 = 速度外環的目標速度單位（ticks/40ms，負 = 前進）
// ============================================================================
#ifndef NAV_H
#define NAV_H

#include <math.h>
#include <stdint.h>

// ---------------- 參數（實測後主要調這幾個） ----------------
// 轉向
#define NAV_TURN_KP        1.5f    // 角度誤差 → 目標角速度 (1/s)：誤差 30° → 0.79 rad/s
#define NAV_TURN_RATE_MAX  0.8f    // 最大角速度 (rad/s)，與 C 鍵回正相同
#define NAV_TURN_RATE_MIN  0.25f   // 最小角速度：誤差小時也要給，否則麥克拉姆輪可能轉不動
#define NAV_TURN_TOL       (2.5f * 0.0174533f)  // 到達判定：誤差 < 2.5°
#define NAV_TURN_STILL     0.15f   // 到達判定：角速度 < 0.15 rad/s
#define NAV_TURN_SETTLE    30      // 上面兩個條件連續成立 30 × 10 ms = 0.3 s 才算完成
#define NAV_TURN_TIMEOUT   600     // 600 × 10 ms = 6 s 還轉不到就放棄（回報 TIMEOUT）
#define NAV_TURN_LEAD      0.3f    // 預估提前量 (s)：用「誤差 − 角速度 × 0.3 s」決定轉速，提早減速避免轉過頭
                                   // （實測放開 A/D 後車身還會多轉約 35°，延遲很大）
// 直走
#define NAV_TV_MAX         5.8f    // 前進的最大目標速度（v29：4.0 會走走停停；改成和遙控 W 鍵同速 5.8 就很順，D 項修好後 5.8 不會倒）
#define NAV_SLOW_CM        20.0f   // 剩下多少公分開始減速（目標速度 ∝ 剩餘距離）
#define NAV_TV_MIN         3.0f    // 前進還沒到時的最小目標速度（實測 < 3 就會卡住）
#define NAV_TV_MAX_BACK    5.0f    // 後退的最大目標速度（v29：陀螺儀修好後前後退速度比約相同 ≈2.3 倍，2.5 實測會卡住來回晃）
#define NAV_TV_MIN_BACK    2.5f    // 後退的最小目標速度（v29：1.2~2.0 推不動，需要 ≥2.5）
#define NAV_POS_TOL_CM     2.0f    // 剩餘距離 < 2 cm → 目標速度歸零、進入停車
#define NAV_SETTLE_TOL_CM  3.0f    // 停車判定：剩餘距離 < 3 cm（用位置判斷，不被煞車後的前後晃動影響）
#define NAV_STILL_V        6.0f    // 停車判定：|平均速度| < 6.0（只是排除「還在衝」，煞車後晃動可到 ±10）
#define NAV_V_FILT         0.8f    // 停車判定用的速度再平均一次（每 40 ms：新 = 0.8 × 舊 + 0.2 × 量測）
#define NAV_SETTLE_UPD     12      // 連續 12 × 40 ms = 0.5 s 靜止才算完成
#define NAV_SETTLE_MAX     75      // 停車階段最多 3 s；逾時時若剩餘距離 < 2 × NAV_POS_TOL_CM 仍算完成
#define NAV_MOVE_TIME_BASE 100     // 直走逾時：100 × 40 ms = 4 s ...
#define NAV_MOVE_TIME_PER_CM 9     // ... 再加每公分 9 × 40 ms = 0.36 s
#define NAV_TV_ARC_CAP     12.0f   // 弧線 #C 可指定的最大前進目標速度（5.8 = 遙控 W 鍵同速；實測 8 可以，更高的沒測過）
#define NAV_DT_V           0.0405f // 速度外環週期 (s)（4 × 10.13 ms），弧線時由位移換算前進速度
#define NAV_FF_FILT        0.7f    // 弧線角速度前饋的濾波：新 = 0.7 × 舊 + 0.3 × 量測

enum { NAV_IDLE = 0, NAV_TURN = 1, NAV_MOVE = 2, NAV_STOP = 3 };
enum { NAV_RES_NONE = 0, NAV_RES_DONE = 1, NAV_RES_TIMEOUT = 2, NAV_RES_ABORT = 3 };
enum { NAV_AB_NONE = 0, NAV_AB_USER = 1, NAV_AB_FALL = 2, NAV_AB_EMERG = 3, NAV_AB_LINK = 4, NAV_AB_CMD = 5 };

struct Nav {
  // ---- loop() 寫入、ISR 讀取的請求 ----
  volatile uint8_t req_kind;      // 0 = 無，'T' 轉向，'M' 直走，'A' 中止
  volatile float req_value;       // T：目標方向 (rad)；M：距離 (ticks)
  volatile float req_heading;     // M：直走方向 (rad，相對原點)
  volatile uint8_t req_has_heading; // M：有沒有指定方向（沒有就用目前方向）
  volatile uint8_t req_abort;     // 中止原因（NAV_AB_*）
  // ---- ISR 內部狀態 ----
  uint8_t state;
  uint8_t kind;                   // 目前動作 'T' / 'M'
  float target_heading;           // 轉向目標 (rad，相對原點)
  float move_heading;             // 直走方向 (rad，相對原點，開始直走時鎖定)
  float move_dist;                // 直走距離 (ticks，負 = 後退)
  float sx, sy;                   // 直走起點 (ticks)
  float remaining;                // 剩餘角度 (rad) 或距離 (ticks)，遙測用
  uint16_t timer;
  uint16_t timeout;
  uint8_t settle;
  float v_avg;                    // 停車判定用的平均速度
  // 弧線 #C（直走時 curv = 0）
  float curv;                     // 曲率 (rad / tick)：每前進 1 tick 車頭轉多少，正 = 右轉
  float h0;                       // 弧線起點的方向 (rad，相對原點)
  float px, py;                   // 上一次的位置 (ticks)，用來累積弧長
  float s_arc;                    // 已走弧長 (ticks)
  float yaw_ff;                   // 航向環的角速度前饋 (rad/s)，ISR 的航向環讀取
  float tv_fwd;                   // 這一次前進的最大目標速度（#M 固定 NAV_TV_MAX；#C 可指定）
  volatile float req_tv;          // #C 的速度請求（0 = 用預設）
  // ---- ISR 寫入、loop() 讀取的結果 ----
  volatile uint8_t result;        // NAV_RES_*
  volatile uint8_t result_kind;   // 'T' / 'M'
  volatile uint8_t result_reason; // 中止原因
  volatile float result_err;      // 最後誤差 (rad 或 ticks)
  uint8_t last_reason;            // 最近一次結束的原因（不會被 loop() 清掉，ISR 判斷要不要歸零速度用）
};

static inline float Nav_Wrap(float a) {
  while (a > 3.14159265f) a -= 6.2831853f;
  while (a < -3.14159265f) a += 6.2831853f;
  return a;
}
static inline float Nav_Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline void Nav_Init(Nav* n) {
  n->req_kind = 0; n->req_value = 0; n->req_abort = 0;
  n->req_heading = 0; n->req_has_heading = 0;
  n->state = NAV_IDLE; n->kind = 0;
  n->target_heading = n->move_heading = n->move_dist = 0;
  n->sx = n->sy = 0; n->remaining = 0;
  n->timer = n->timeout = 0; n->settle = 0; n->v_avg = 0;
  n->curv = n->h0 = n->px = n->py = n->s_arc = n->yaw_ff = 0;
  n->tv_fwd = NAV_TV_MAX; n->req_tv = 0;
  n->result = NAV_RES_NONE; n->result_kind = 0; n->result_reason = 0; n->result_err = 0;
  n->last_reason = 0;
}

static inline bool Nav_Active(const Nav* n) { return n->state != NAV_IDLE; }

static inline void Nav_Finish(Nav* n, uint8_t res, uint8_t reason, float err) {
  n->result_kind = n->kind;
  n->result_reason = reason;
  n->result_err = err;
  n->last_reason = reason;
  n->curv = 0; n->yaw_ff = 0;
  n->result = res;          // 最後才寫，loop() 看到 result 時其他欄位已經就緒
  n->state = NAV_IDLE;
  n->kind = 0;
}

// ISR 每次開始時呼叫：處理 loop() 送來的請求，並在需要時中止
//   yaw_rel：目前方向（相對原點，rad）；x, y：目前位置 (ticks)；ticks_per_cm：估逾時時間用
static inline void Nav_Service(Nav* n, float yaw_rel, float x, float y, float ticks_per_cm) {
  if (n->req_abort) {
    uint8_t r = n->req_abort;
    n->req_abort = 0;
    n->req_kind = 0;
    if (Nav_Active(n)) Nav_Finish(n, NAV_RES_ABORT, r, n->remaining);
    return;
  }
  uint8_t k = n->req_kind;
  if (!k) return;
  float v = n->req_value;
  n->req_kind = 0;
  n->timer = 0;
  n->settle = 0;
  if (k == 'T') {
    n->kind = 'T';
    n->state = NAV_TURN;
    n->curv = 0; n->yaw_ff = 0;
    n->target_heading = Nav_Wrap(v);
    n->remaining = Nav_Wrap(n->target_heading - yaw_rel);
    n->timeout = NAV_TURN_TIMEOUT;
  } else if (k == 'F') {           // 結束弧線：把終點改成「現在再往前一段減速距離」，之後走一般的減速、停車流程
    if (n->state == NAV_MOVE && n->curv != 0.0f) {
      float slow = NAV_SLOW_CM * (n->tv_fwd > NAV_TV_MAX ? n->tv_fwd / NAV_TV_MAX : 1.0f) * ticks_per_cm;
      if (n->move_dist - n->s_arc > slow) n->move_dist = n->s_arc + slow;
    }
  } else if (k == 'M' || k == 'C') {
    bool chain = (k == 'C' && n->state == NAV_MOVE && n->curv != 0.0f);  // 弧線進行中又收到 #C：不停車、方向不歸零，直接接成新的弧線
    float mh = chain ? n->move_heading : yaw_rel;
    float yf = chain ? n->yaw_ff : 0.0f;
    n->kind = k;
    n->state = NAV_MOVE;
    n->curv = 0; n->yaw_ff = yf; n->s_arc = 0; n->h0 = mh; n->px = x; n->py = y;
    n->tv_fwd = (k == 'C' && n->req_tv > 0.0f) ? Nav_Clamp(n->req_tv, NAV_TV_MIN, NAV_TV_ARC_CAP) : NAV_TV_MAX;
    if (k == 'C') {                // 弧線：req_heading 放的是曲率 (rad/tick)
      n->curv = n->req_heading;
      n->move_heading = mh;
    } else {
      n->move_heading = n->req_has_heading ? Nav_Wrap(n->req_heading) : yaw_rel;  // 指定方向或目前方向
    }
    n->move_dist = v;
    n->sx = x;
    n->sy = y;
    n->v_avg = 0.0f;
    n->remaining = v;
    float cm = fabsf(v) / ticks_per_cm;
    n->timeout = NAV_MOVE_TIME_BASE + (uint16_t)(cm * NAV_MOVE_TIME_PER_CM);
  }
}

// 轉向中每 10 ms 呼叫一次 → 回傳目標角速度 (rad/s，正 = D 右旋)
//   yaw_rate：實際角速度 (rad/s)
static inline float Nav_TurnRate(Nav* n, float yaw_rel, float yaw_rate) {
  if (n->state != NAV_TURN) return 0.0f;
  float err = Nav_Wrap(n->target_heading - yaw_rel);
  n->remaining = err;
  float aerr = fabsf(err);
  if (aerr < NAV_TURN_TOL && fabsf(yaw_rate) < NAV_TURN_STILL) {
    if (++n->settle >= NAV_TURN_SETTLE) {
      Nav_Finish(n, NAV_RES_DONE, 0, err);
      return 0.0f;
    }
  } else {
    n->settle = 0;
  }
  if (++n->timer >= n->timeout) {
    Nav_Finish(n, NAV_RES_TIMEOUT, 0, err);
    return 0.0f;
  }
  if (aerr < NAV_TURN_TOL) return 0.0f;       // 在容許範圍內：不再推，等它停穩
  float pred = err - yaw_rate * NAV_TURN_LEAD;  // 照目前角速度再轉一下會剩多少
  if (pred * err <= 0.0f) return 0.0f;         // 預估會自己滑到 → 先不推
  float rate = Nav_Clamp(NAV_TURN_KP * pred, -NAV_TURN_RATE_MAX, NAV_TURN_RATE_MAX);
  if (fabsf(rate) < NAV_TURN_RATE_MIN) rate = (err > 0) ? NAV_TURN_RATE_MIN : -NAV_TURN_RATE_MIN;
  return rate;
}

// 直走 / 停車中每 40 ms 呼叫一次 → 回傳速度外環的目標速度（負 = 前進）
//   v：濾波後實際速度 (ticks/40ms，負 = 前進)；ticks_per_cm：距離換算
static inline float Nav_MoveSpeed(Nav* n, float x, float y, float v, float ticks_per_cm) {
  if (n->state != NAV_MOVE && n->state != NAV_STOP) return 0.0f;
  // 沿行進方向的投影 → 超過目標時剩餘距離變負，會自動往回修
  float s;
  if (n->curv != 0.0f) {
    // 弧線：累積沿「目前切線方向」的位移 = 已走弧長；目標方向 = 起點方向 + 曲率 × 弧長（走完就不再轉）
    float ds = (x - n->px) * cosf(n->move_heading) + (y - n->py) * sinf(n->move_heading);
    n->px = x; n->py = y;
    n->s_arc += ds;
    float sh = Nav_Clamp(n->s_arc, 0.0f, n->move_dist);   // 起步先倒退的那一下不轉；走完後方向固定
    n->move_heading = Nav_Wrap(n->h0 + n->curv * sh);
    float ff = (n->state == NAV_MOVE && n->s_arc < n->move_dist && ds > 0.0f) ? n->curv * ds / NAV_DT_V : 0.0f;
    n->yaw_ff = (n->state == NAV_MOVE && n->s_arc < n->move_dist) ? NAV_FF_FILT * n->yaw_ff + (1.0f - NAV_FF_FILT) * ff : 0.0f;
    s = n->s_arc;
  } else {
    s = (x - n->sx) * cosf(n->move_heading) + (y - n->sy) * sinf(n->move_heading);
  }
  float r = n->move_dist - s;
  n->remaining = r;
  float tol = NAV_POS_TOL_CM * ticks_per_cm;

  if (++n->timer >= n->timeout) {
    // 停車階段逾時：位置已經夠近就算完成（車身晃動不該讓整個動作失敗）
    bool ok = (n->state == NAV_STOP) && fabsf(r) < (n->curv != 0.0f ? 5.0f : 2.0f) * tol;  // 弧線衝比較快，容許多滑一點
    Nav_Finish(n, ok ? NAV_RES_DONE : NAV_RES_TIMEOUT, 0, r);
    return 0.0f;
  }
  if (n->state == NAV_MOVE) {
    if (fabsf(r) < tol) {                 // 到了：目標速度歸零，交給既有的煞車 / 停車邏輯
      n->state = NAV_STOP;
      n->v_avg = v;
      n->settle = 0;
      n->timer = 0;
      n->timeout = NAV_SETTLE_MAX;
      return 0.0f;
    }
    float vmax = (r > 0) ? n->tv_fwd : NAV_TV_MAX_BACK;    // 前進 / 後退各用自己的速度範圍
    float vmin = (r > 0) ? NAV_TV_MIN : NAV_TV_MIN_BACK;
    float slow = NAV_SLOW_CM * ((r > 0 && n->tv_fwd > NAV_TV_MAX) ? n->tv_fwd / NAV_TV_MAX : 1.0f);  // 跑得越快，越早開始減速
    float k = vmax / (slow * ticks_per_cm);                 // 剩 slow 公分時剛好開始減速
    float fwd = Nav_Clamp(k * r, -vmax, vmax);              // 往前為正
    if (fabsf(fwd) < vmin) fwd = (r > 0) ? vmin : -vmin;
    return -fwd;                          // 速度外環：負 = 前進
  }
  // NAV_STOP：等車子停穩（用平均速度，避免被平衡晃動誤判）
  n->v_avg = NAV_V_FILT * n->v_avg + (1.0f - NAV_V_FILT) * v;
  if (fabsf(r) < (n->curv != 0.0f ? 2.0f : 1.0f) * NAV_SETTLE_TOL_CM * ticks_per_cm && fabsf(n->v_avg) < NAV_STILL_V) {
    if (++n->settle >= NAV_SETTLE_UPD) Nav_Finish(n, NAV_RES_DONE, 0, r);
  } else {
    n->settle = 0;
  }
  return 0.0f;
}

#endif
