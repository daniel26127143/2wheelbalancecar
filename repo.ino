// ============================================================================
// [v34] 針對 10/13 實車 log（單純按住 W 5 秒、預設 7.2 → 速度衝到目標的 3.7 倍、放開後衝到 6 倍、最後倒地）的三項修改，各自有開關：
//   ENABLE_EMF_FF          反電動勢前饋：速度快時直立環的 PWM 被馬達反電動勢抵消（V 20 時約 54 PWM），拉不回來 → 把這部分補回去
//   ENABLE_OVERSPEED_BRAKE 超速煞車：實際速度比目標快很多時，速度外環 P 增益 ×1.6、補償角上限 3.5° → 4.0°
//   ENABLE_FAST_START      快速起步：目標速度 < 3 的區間（車子本來就推不動）斜坡加快，縮短起步前的空等（log 約 0.8 s）
//   三個都設 0 = 與 v33 行為相同。ENABLE_TUNING 預設改成 0（調參鍵已不需要，省 Flash）。
// ============================================================================
// [v32] 新增 #D <速度> <角速度>：連續（無段）遙控，像遙控汽車一樣同時調整前進速度與轉向（見 nav_cmd.h）
// [v31] 拆檔：功能與 v30 完全相同，只是把三塊程式搬到同資料夾的標頭檔（Arduino IDE 會自動一起編譯）
//   nav.h       座標導航邏輯（Nav_*，純計算）          ← 原本貼在這個檔案開頭
//   nav_cmd.h   導航指令解析 Nav_HandleLine / 回報 Nav_Report ← 原本在 loop() 前面
//   rec.h       診斷錄影（r / m 鍵，配合 bal_rec.py）  ← 原本散在 ISR 與 loop()
//   三個檔案都要和 repo.ino 放在同一個資料夾，缺一個就編譯不過
// ============================================================================
// ============================================================================
// repo.ino（v30 整合版）= nav_v27（座標導航）+ 陀螺儀修正 + 診斷錄影，全部在這一個檔案，不需要 nav.h
//   1) 陀螺儀修正：原本 dmpGetGyro 的 X 軸值實測幾乎永遠是 0（直立環 D 項失效），
//      改直接讀 GYRO_XOUT 暫存器。Kd 從 80 改成 40 起步，用 g / h 鍵 -10 / +10 調整
//   2) 診斷錄影（配合 controller/bal_rec.py）：r = 站立錄影、m = 移動錄影
//   3) 導航：#T <角度> / #M <距離> [角度] / #S，與 controller/nav.py 搭配
//   5) [v30] 弧線 #C <弧長cm> <半徑cm>（正=右轉、負=左轉）：邊走邊轉，用來繞圈
//   4) 編碼器固定使用 A/B 雙相（左 B = A0、右 B = D4），不再有 USE_QUADRATURE 開關
//   舊版 repo.ino（v25b）備份在 backup/repo_v25b_20261011.ino.txt
//   Flash 約 94%：若之後要加功能，可以先把診斷錄影（REC_ 開頭的程式）拿掉
// ============================================================================
// ============================================================================
// nav_v27：quad_v26 + 座標導航（導航邏輯在同資料夾的 nav.h）
//   Python 腳本（controller/nav_script.py）依實際位置算好後送：
//     #T <角度>          轉到指定方向（度；0 = 起點車頭方向，往右 / D 方向為正）
//     #M <距離> [角度]   直走（公分；負 = 後退）；有給角度就鎖在那個方向，沒給就用目前方向
//     #S（或 #A）        中止
//   收到時回報 NAV:START ...；完成時回報 NAV:DONE T err=<度> PX:<cm> PY:<cm> H:<度>
//   （或 NAV:TIMEOUT / NAV:ABORT <原因> / NAV:REJECT <原因>）；導航中遙測多 NV（1 轉向 2 直走 3 停車）、NR（剩餘量）
//   導航中按 W/X/A/D/S/C 任一鍵立即中止；超過 1 s 沒收到任何字元（Python 心跳）也會中止
// ----------------------------------------------------------------------------
// quad_v26：B 相接好、方向測試通過後使用的版本（從 v25 延伸）
//   1) 編碼器固定用 A/B 雙相判斷方向（不再用 PWM 正負猜）
//   2) 拿掉單相編碼器時代的補丁：自轉後丟計數、方向鎖、依角速度判斷慣性旋轉、自轉後嚴格變化率限制
//   3) 新增虛擬座標（里程計）：X = 起點車頭方向（往前為正）、Y = 往右為正，單位 cm；按 z 歸零
//   4) 遙測：新增 PX / PY（虛擬座標）；OL 只在速度外環關閉時顯示、G 只在轉向權限 < 1 時顯示（省頻寬）
// B 相腳位與方向已用 enc_test 實測確認（10/10）；使用前只剩校正 TICKS_PER_CM
// ============================================================================
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PinChangeInterrupt.h>
#include <util/atomic.h>
#include "nav.h"  // [v27] 座標導航邏輯（純計算，不碰硬體；v31 起放在獨立檔案，電腦上也能編譯測試）

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
// ★★★ [v25] 編碼器改用 A/B 雙相判斷方向 ★★★
// 原本只接 A 相：單相編碼器分不出正反轉，只能用「PWM 正負」猜方向
//   → PWM 換方向但輪子還靠慣性轉時會記錯號（自轉結束、平衡來回晃時最嚴重），
//     之前的丟計數、方向鎖、變化率限制都是在補這個洞
// 馬達的 6pin（GND / A / VCC / B / M- / M+）本來就有 B 相，接上後每個 tick 的方向都是真的
// [v26] 固定使用 A/B 雙相（B 相沒接好的話請用 v25 的 repo.ino，並把 USE_QUADRATURE 設 0）
#define ENC_B_PIN_L A0        // 左輪 B 相 → A0（板子沒有走線，用杜邦線從馬達座第 3 腳拉過來）
#define ENC_B_PIN_R 4         // 右輪 B 相 → D4（板子上 DJ-R 第 5 腳本來就接到 D4，右輪 A 相在 A1）
// 方向測試：按 k 斷電、扶著車子往前（W 方向）推，EL、ER 都要變「負」；哪一輪變正，就把那一輪改成 (-1)
// ⚠ 請填 v25 方向測試的結果（兩邊程式要一致）
#define ENC_L_SIGN (-1)   // 10/10 enc_test 實測：往前時 A0 與 D2 同電位
#define ENC_R_SIGN (-1)   // 10/10 enc_test 實測：往前時 D4 與 A1 同電位
// [v26] 距離校正：按 k 斷電、扶著車子往前推剛好 50 cm，TICKS_PER_CM = (|EL| + |ER|) / 2 / 50
//       預設 30 是估計值（11 PPR × 30 減速比 × A 相雙邊緣 ÷ 輪周約 22 cm），請實測後修改
#define TICKS_PER_CM 36.2f   // 10/10 實測：每圈 660 tick（11 線霍爾 × 1:30 × 2），輪徑 5.8 cm → 660 ÷ (π × 5.8)
volatile long enc_total_L = 0;  // [v25] 方向測試用累計 tick（倒地 / 斷電時顯示在遙測 EL / ER）
volatile long enc_total_R = 0;

// [v30] 診斷錄影開關：加了弧線 #C 之後 Flash 放不下，預設關閉（0）。
//       要用 bal_rec.py 做站立 / 移動診斷時改成 1，但此時程式約 32.4 KB，超過 UNO 上限 32,256 B，
//       必須先拿掉別的功能才能上傳；平常請保持 0。
#define ENABLE_REC 0
#include "rec.h"  // [v31] 診斷錄影（r / m 鍵），原本散在 ISR 與 loop() 的程式現在都在這個檔案

// [v33] 現場調參開關：1 = 保留 g/h（Kd）、j/l（Kp_speed）、[ ]（setpoint）、o（速度外環開關）這些按鍵，行為和 v32 完全相同；
//       0 = 這些按鍵整段不編譯，同時 Kd / Kp_speed / base_setpoint / speed_loop_enabled 變成常數，編譯器可以直接代入，
//       省下約 0.7 KB Flash 給新功能。參數定案後再改成 0；要重新調參就改回 1。按鍵處理放在 tuning.h。
#define ENABLE_TUNING 0   // [v34] 預設 0（要重新用 g/h/j/l 調參時改回 1；1 時 Flash 只剩約 70 B，放不下 v34）
#if ENABLE_TUNING
#define TUNE_VAR
#define TUNE_VOL volatile
#else
#define TUNE_VAR const
#define TUNE_VOL const
#endif

// ★★★ [v34] 三項修改的開關與參數 ★★★
#define ENABLE_EMF_FF 1
#define ENABLE_OVERSPEED_BRAKE 1
#define ENABLE_FAST_START 1
// 反電動勢前饋：|V| 超過 EMF_V_DB（靜止時 V 會在 ±7 內抖動，不補）的部分，每 1 單位補 EMF_K 的 PWM，最多 EMF_FF_MAX。
//   實測約 2.7 PWM / 單位（10/13 log：V -16 對應 PWM 約 -45），只補約 2/3，補滿會變成正回授（速度越快推越大）
#define EMF_K 1.8f
#define EMF_V_DB 8.0f
#define EMF_FF_MAX 50.0f
// 超速煞車：|V| > |目標| + OVERSPEED_MARGIN 且方向相同（含目標 0）才算超速
#define OVERSPEED_MARGIN 8.0f
#define OVERSPEED_SA_MAX (4.0f * DEG_TO_RAD)
#if ENABLE_OVERSPEED_BRAKE
#define SPEED_PI_CLAMP OVERSPEED_SA_MAX
#else
#define SPEED_PI_CLAMP (3.5f * DEG_TO_RAD)
#endif
// 快速起步：|目標速度| < FAST_START_TV 時，起步斜坡改用 FAST_START_STEP（每 40 ms）
#define FAST_START_TV 3.0f
#define FAST_START_STEP 0.90f

// ★★★ 正統串級 PID 參數 (計算嚴格使用 SI 制：rad, rad/s) ★★★

// 1. 直立內環 (Balance PD)
float Kp_balance_normal = 1275.0;  // 比例推力 (對抗傾斜) 1300 > x >1200
TUNE_VAR float Kd_balance_normal = 40.0;    // 微分阻尼 (抑制前後震盪)。[v28] D 項修好後第一次真的生效，從 40 起步，用 g / h 鍵 -10 / +10 調整（上限 150）

float Kp_balance_large = 1400.0;  // 1500 > x > 1400
TUNE_VAR float Kd_balance_large = 48.0;    // 大角度緊急阻尼 (增加 50%，強制吸震防甩倒)

// 2. 優先級切換門檻 (以實測發散臨界點設計之遲滯區間)
const float LARGE_ANGLE_ENTER = 7 * DEG_TO_RAD;  // 傾角誤差 > 3.5° 進入搶救 (即傾角小於 -5.6° 或大於 +1.4°)
const float LARGE_ANGLE_EXIT = 4.5 * DEG_TO_RAD;   // 傾角誤差收斂至 < 2.0° 退出搶救
bool is_emergency_state = false;                    // 大角度狀態旗標

// 3. 速度外環 (Speed PI) - 分頻執行
TUNE_VAR float Kp_speed = 2.8;    // 速度比例增益 2.8 > x >2.6
float Ki_speed = 0.015;  // 速度積分增益 (消除重心偏移溜車)
                         // [v26] 舊版這個變數沒被用到（0.005），Speed_PI 裡實際寫死 0.015 → 改成真的使用、數值不變

// float Kp_speed = 2;       // 速度比例增益 2.8 > x >2.6 (原本)
// float Ki_speed = 0.005;   // 速度積分增益 (消除重心偏移溜車)(原本)
#define SPEED_LOOP_DIV 4  // 角度環實測週期10.13ms，4倍≈40.5ms，落在40~50ms需求區間
// [v9~v23] 單相編碼器時代的自轉補丁（v26 已移除）：
//   自轉結束後丟棄計數 0.15 s、依實際角速度鎖方向、自轉後 0.5~1.5 s 的嚴格變化率限制
//   → 都是因為「分不出輪子正反轉」而加的；接上 B 相後速度讀值永遠正確，不再需要
// [v26] 只保留：自轉「進行中」補償角變化率 ≤ 0.15°/40ms（讓自轉時直立目標角平順）
#define SA_SLEW_SPIN (0.15f * DEG_TO_RAD)
#define SA_SLEW_FREE (0.5f * DEG_TO_RAD)         // 其他時候：每 40 ms 最多 0.5°（只擋單次異常跳變）
// ★★★ [v24] 自轉「定點」：自轉時記住車身位移，往哪邊漂就往反方向補 ★★★
// 實測（205032~212925，共 23 次自轉）：8 次沒能原地轉（A 方向 6 次），全部是同一個過程：
//   自轉中速度外環只有 P 項（積分凍結），「維持不動」的力道很弱（≈0.06°/tick）
//   → 自轉產生的一點前後干擾就讓車身慢慢往後漂（V +5~+10）
//   → 平衡輸出 B 跟著變大（+20~+65）→ 內側輪 PWM = B − 轉向差速 掉進死區（|PWM| < 20）停住
//   → 變成「繞著停住的那一輪轉」，看起來就是沒辦法原地轉
//   正常的自轉 B 都在 ±10 以內、兩輪 PWM 都 > 30
// 作法：自轉期間累積平移量 spin_dx，補償角加上 SPIN_HOLD_K × spin_dx（等於自轉時加一個較強的「回原位」力道）
//       → 漂移最多約 150~300 ticks 就被拉住，B 不會一路變大；放開後約 0.5 s 內平滑退掉
#define SPIN_HOLD_K   (0.004f * DEG_TO_RAD)      // 每偏移 1 tick 補 0.004°（100 ticks → 0.4°）
#define SPIN_HOLD_MAX (1.5f * DEG_TO_RAD)        // 定點補償上限 1.5°
#define SPIN_HOLD_DECAY 0.93f                    // 自轉結束後每 40 ms 剩 93%（約 0.5 s 退掉）

// ★★★ [v13] 前後移動參數（速度單位：左右輪平均 ticks / 40ms，負號 = 前進 W 方向）★★★
#define MOVE_SPEED_FWD   (-5.8f)   // W 目標速度（v17 實測：-4.5 → 實際最高約 -10，約 2.3 倍）
#define MOVE_SPEED_BACK  (7.0f)    // X 目標速度（v17 實測：5.5 → 實際最高約 10，約 1.9 倍）
                                   // [v18] 兩個方向的放大倍率不同，各自調整讓實際最高速都約 12（快約 20%）
                                   // [v20] v19 實測巡航 W 約 -11.5、X 約 11.5 → 再快約 12%（-5.2→-5.8、6.3→7.0）
#define MOVE_ACCEL_STEP  0.30f     // 起步：目標速度每 40 ms 最多增加多少（0→5.8 約 0.8 s）
                                   // [v20] 0.20 → 0.30；反向（W 放開馬上按 X）時也用這個斜率穿過 0
                                   // [v18] 0.10 時按下後要 0.6~2.2 s 才開始真正移動
                                   // [v14] 0.05 時前 2 秒目標太小、被平衡晃動淹沒，車子原地前後晃不前進
#define MOVE_DECEL_STEP  0.15f     // 煞車：目標速度每 40 ms 最多減少多少（4.5→0 約 1.2 s）
                                   // [v14] 0.05 時放開後目標還要 3.6 s 才歸零 → 滑行距離是行駛距離的 1.5~2.5 倍
// [v15] 移動中與停下後一小段時間「凍結速度積分」（只用 P 項）
//       實測：W/X 期間實際速度一直超過目標，積分把這段「多走的距離」記下來，
//       停下後積分約 -0.5°，把車往回拉 116~313 ticks（看起來像停下後往回退）
//       凍結後積分只保留站立時學到的重心補償，不再記移動過程的誤差
#define MOVE_SETTLE_UPDATES 50     // [v16] 目標速度歸零後的「停車階段」50 × 40ms ≈ 2 s
// [v16] 移動中 + 停車階段的速度 P 倍率
//       實測 v15：積分已凍結（停下時積分 ≈ 0），但仍往回退 24~455 ticks
//       原因：速度外環 P 太弱 —— 維持速度 V 本身就需要約 0.08°×V 的補償角，
//       P(0.16°/tick) 扣掉這部分只剩一半在「修正速度」→ 實際速度 ≈ 2.2 倍目標、
//       停下後越過 0 反向，且要 2~3 s 才慢慢停
//       站立時維持原本 Kp_speed，只有移動/停車階段加強
#define MOVE_KP_SCALE 1.6f
// [v19] 只在「煞車 / 停車階段」使用 MOVE_KP_SCALE；起步與等速巡航用原本的 P
//       實測 v18：巡航時 P×1.6 會把平衡本身 ±3 的速度晃動放大成 SA ±0.8° 的抖動（看起來不穩），
//       而且 P 越大、實際速度越接近目標 → 速度上不去

// ★★★ [v21] 煞車柔化（放開 W/X 後的停車過程）★★★
// v20 實測：放開時實際速度約 20（v19 約 15），煞車時 SA 0.1 s 內從 2.2° 跳到 3.5° 上限並卡住約 0.6 s
//   → 放開瞬間輪子先往前衝（V 20→25），接著車身後仰到偏離 3~4.5°，再彈回來 → 停車時晃得比 v19 大
// 1) P 倍率不再瞬間切到 ×1.6，改成約 0.24 s 內線性加上去（煞車「踩下去」比較柔）
// 2) 煞車 / 停車階段補償角上限 3.5° → 3.0°（限制最大後仰角，代價是滑行距離多一點）
#define BRAKE_RAMP_UPDATES 6                   // 6 × 40 ms ≈ 0.24 s 從 ×1.0 升到 ×MOVE_KP_SCALE
#define BRAKE_SA_MAX (3.0f * DEG_TO_RAD)       // 煞車 / 停車階段的補償角上限

// ★★★ [v20] 起步防卡死（只在 W/X 起步、車子「卡住不走」時作用）★★★
// v19 實測：目標速度早已加到最大，車子卻原地前後踱步 1~3.6 s 才突然衝出去
//   原因：麥克拉姆輪滾子摩擦大，要先多傾一點才「推得動」；P 項在 V=0 時只給約 0.8°，
//         再被站立時殘留的積分抵掉約 0.3° → 推力不夠 → 卡在原地；積分在移動中又是凍結的，沒有東西會繼續加力
//         最後偶然被晃動帶出去時一次衝很快 → 這也是移動中前後晃（ANG 晃動 1.2~1.5°）的主因
// 作法：卡住時另外慢慢加一個「起步推力」，一跑起來（速度 ≥ 目標）就自動退掉，放開按鍵後很快歸零
//       → 不會像積分那樣記住位置，所以不會造成停下後往回退
#define BOOST_START_TV   2.0f                   // 目標速度超過 2 才開始判斷有沒有卡住
#define BOOST_STALL_FRAC 0.5f                   // 實際速度 < 目標的一半 = 卡住 → 推力增加
#define BOOST_RATE       (0.03f * DEG_TO_RAD)   // 每 40 ms 加 0.03°（約 0.75°/s）
#define BOOST_MAX        (1.0f * DEG_TO_RAD)    // 最多額外 1.0°
#define BOOST_LEAK_MOVE  0.85f                  // 速度已達目標：每 40 ms 剩 85%（約 0.3 s 退掉）
#define BOOST_LEAK_STOP  0.70f                  // 放開按鍵 / 煞車中：每 40 ms 剩 70%（約 0.15 s 退掉）

// 3. 重心與保護常數
TUNE_VOL float base_setpoint = -1.45 * DEG_TO_RAD;  // 機械重心零點 (rad) -1.5 < x < -1.25
// [v23] 充電後（電池拆裝過）實測靜止平均傾角 -1.45~-1.52、速度外環平均修正 ≈ 0 → 取 -1.45
// [v10] setpoint 量測模式：按 o 關閉/開啟速度外環；按 [ / ] 即時把 setpoint 減/加 0.1°
TUNE_VOL bool speed_loop_enabled = true;
// [v11] k = 馬達強制斷電（遙測照常輸出，用來手扶找平衡角）
volatile bool motor_kill = false;
// [v11→v13] 累積位移 (ticks ×2，左右輪相加)：按下 W/X 起步或關閉外環時歸零
//            遙測 DX = 從那一刻起車子走了多少 ticks（含煞車滑行），可直接看煞車距離
volatile long odo_ticks2 = 0;
volatile float debug_v = 0.0f;   // [v13] 實際速度（濾波後，ticks/40ms）
volatile float debug_tv = 0.0f;  // [v13] 斜坡後的目標速度
float move_boost = 0.0f;         // [v20] 起步推力大小 (rad, ≥0)
int8_t boost_dir = 0;            // [v20] 起步推力對應的指令方向（+1 = X，-1 = W）
float v_slow = 0.0f;             // [v20] 判斷「卡住」用的慢速度（再濾一次，避免被晃動誤判）
volatile float debug_bs = 0.0f;  // [v20] 起步推力 (deg，已含正負號，遙測 BS 欄位)；[v24] 再加上自轉定點補償
float spin_dx = 0.0f;            // [v24] 自轉期間累積的平移量 (ticks，左右輪平均)
float last_raw_speed = 0.0f;     // [v24] Speed_PI 最近一次的原始速度 (ticks/40ms)
// ★★★ [v26] 虛擬座標（里程計）★★★
// 每 40 ms 用「左右輪平均位移 × 目前車頭方向（DMP Yaw）」累加 → 車子在地上的位置
// 原點 = 站起來那一刻（或按 z）的位置；X 軸 = 當時的車頭方向（往前為正），Y = 往右為正
float odo_x = 0.0f;                    // 往前 (ticks)
float odo_y = 0.0f;                    // 往右 (ticks)
float odo_yaw0 = 0.0f;                 // 原點的車頭方向 (rad)
volatile bool odo_reset_req = false;   // 按 z 時設為 true，由控制中斷負責歸零（避免和計算同時進行）
Nav nav;                               // [v27] 導航狀態（ISR 與 loop() 共用，見 nav.h）
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
// [v26] enc_dir_lock（自轉時鎖編碼器方向）已移除：B 相直接量方向
volatile float debug_sa = 0.0;   // [v9] 速度外環輸出的補償角 (deg)，遙測 SA 欄位
volatile float debug_pwm = 0.0;  // 用於 Serial 輸出的即時 PWM 數值
volatile float debug_balance_pwm = 0.0;
volatile float debug_final_turn = 0.0;
volatile float debug_motor_L = 0.0;
volatile float debug_motor_R = 0.0;
volatile int debug_pwm_L = 0;
volatile int debug_pwm_R = 0;
volatile float debug_yaw_rate = 0.0;
volatile float debug_yaw = 0.0;
volatile float debug_target_yaw = 0.0;
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
volatile float drive_w = 0.0f;  // [v32] #D 連續轉向的目標角速度 (rad/s，右轉為正)；與 target_speed_cmd 同時作用，不會像 A/D 一樣把前進速度壓成 0

// ★★★ [重構新增] 轉向權限整形 / 輸出仲裁參數 ★★★
#define CTRL_DT 0.01013f      // 角度環實際週期 (s)
#define PWM_LIMIT 255.0f      // 8-bit PWM 上限
// 死區補償：維持原版調好 Kp/Kd 時的 5（此值直接影響直立環，改動必須重調平衡參數）
#define MOTOR_DZ_L 5.0f
#define MOTOR_DZ_R 5.0f
#define DZ_BLEND 2.0f         // |指令| < 2 時線性過渡，避免過零點 ±DZ 跳變
#define MOTOR_TRIM_L 1.037f   // 左輪增益校正（沿用原值）

float Kff_yaw = 30.0f;                           // 角速度前饋 (PWM per rad/s)：讓自轉不再靠「堆誤差」推動
const float TURN_MAX = 45.0f;                    // 轉向差速絕對上限
const float YAW_LEAD_MAX = 15.0f * DEG_TO_RAD;   // 目標航向最多超前實際 15°（Anti-windup）
const float YAW_RATE_SLEW = 4.0f;                // 目標角速度加速斜率 rad/s^2（0→1 rad/s 約 0.25 s）
const float YAW_RATE_DECEL = 2.0f;               // [v12] 減速斜率 rad/s^2（1→0 rad/s 約 0.5 s，放開 A/D 時煞得比較緩）
const float HOMING_KP = 2.0f;                    // C 鍵回正：角度誤差 → 角速度 (1/s)
const float HOMING_RATE_MAX = 0.8f;              // C 鍵回正最大角速度 (rad/s)
const float TILT_FULL = 2.5f * DEG_TO_RAD;       // 傾角誤差 ≤ 2.5°：轉向權限 100%
const float TILT_ZERO = 6.0f * DEG_TO_RAD;       // 傾角誤差 ≥ 6.0°：轉向權限 0%（全力救車）
float yaw_rate_ref = 0.0f;                       // 斜坡整形後的目標角速度 (rad/s)，僅 ISR 使用
float drive_ref = 0.0f;                          // [v32] drive_w 經斜坡後的值，僅 ISR 使用

// ★★★ [v22] 直行航向鎖定：停車不再偏方向 ★★★
// v21 實測（172142）：134 s 內 29 次 W/X，車頭累積轉了約 44°
//   - 放開按鍵後的煞車滑行（約 1.5~2 s）沒有鎖航向：X 每次停車偏 +1~3°，W 偏 -0.6~1.7° → 共 22°
//   - 每次按 W/X 都重新鎖「當下」的方向 → 上一次留下的誤差不會被修回來，一直累積
// 1) 航向鎖定延長：按住 W/X → 煞車滑行（目標速度歸零前）→ 停下後再 1 s
// 2) 航向記憶：下一次 W/X 沿用上一次的航向（同一條直線來回），
//    只有按過 A/D/C、倒地、緊急救援，或車頭已經被轉開超過 10°（例如被手轉）才重新鎖當下方向
#define HEADING_HOLD_TICKS 100                         // 停下後繼續鎖航向 100 × 10ms = 1 s
const float HEADING_MEM_MAX = 10.0f * DEG_TO_RAD;      // 偏離記憶航向超過 10° 就不沿用
float heading_mem = 0.0f;         // 記住的直行航向 (rad)
bool heading_mem_valid = false;
bool heading_locked = false;      // 目前是否處於直行鎖航向狀態
volatile float debug_tilt_gain = 1.0f;

float Heading_Control_PD(float current_yaw_rate, float rate_ref);
float Turn_Authority_Limit(float balance, float turn, float tilt_err);

static inline float wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

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

// 死區補償：與原版相同的 out = |u| + DZ（直立環增益與原版完全一致）
// 唯一差異：|u| < DZ_BLEND 時從 0 線性爬升，過零點連續
static inline float deadzoneComp(float u, float dz) {
  float a = fabsf(u);
  float out;
  if (a < DZ_BLEND) {
    out = a * (dz + DZ_BLEND) / DZ_BLEND;  // 在 a = DZ_BLEND 處與下式銜接
  } else {
    out = a + dz;
  }
  if (out > PWM_LIMIT) out = PWM_LIMIT;
  return (u >= 0.0f) ? out : -out;
}

void setMotorSpeed(float speedL, float speedR) {
  speedL *= MOTOR_TRIM_L;

  // 最終安全網：任一輪超出上限時「兩輪同比例縮放」，保留 L/R 比例（差速方向不變）
  float peak = max(fabsf(speedL), fabsf(speedR));
  if (peak > PWM_LIMIT) {
    float k = PWM_LIMIT / peak;
    speedL *= k;
    speedR *= k;
  }

  float outL = deadzoneComp(speedL, MOTOR_DZ_L);
  float outR = deadzoneComp(speedR, MOTOR_DZ_R);
  int pwmL = (int)(outL + (outL >= 0.0f ? 0.5f : -0.5f));
  int pwmR = (int)(outR + (outR >= 0.0f ? 0.5f : -0.5f));
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  debug_pwm_L = pwmL;
  debug_pwm_R = pwmR;

  // 左輪方向
  if (pwmL >= 0) {
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
  } else {
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
  }

  // 右輪方向
  if (pwmR >= 0) {
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
  } else {
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
  }

  analogWrite(PWMA, abs(pwmL));
  analogWrite(PWMB, abs(pwmR));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  debug_pwm_L = 0;
  debug_pwm_R = 0;
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, LOW);
  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, LOW);
}

// 編碼器中斷
void Code_left() {
  // A 相變化時讀 B 相：A、B 同電位 / 不同電位 分別代表兩個轉向
  int8_t d = (digitalRead(ENC_PIN_L) == digitalRead(ENC_B_PIN_L)) ? ENC_L_SIGN : -ENC_L_SIGN;
  encoder_count_L += d;
  enc_total_L += d;
}
void Code_right() {
  int8_t d = (digitalRead(ENC_PIN_R) == digitalRead(ENC_B_PIN_R)) ? ENC_R_SIGN : -ENC_R_SIGN;
  encoder_count_R += d;
  enc_total_R += d;
}

// ★★★ 核心控制演算法 ★★★

// 航向角維持與轉向閉環控制
// rate_ref：斜坡整形後的目標角速度 (rad/s)；直行鎖航向時傳 0
float Heading_Control_PD(float current_yaw_rate, float rate_ref) {
  // 1. 航向誤差（最短路徑）
  float yaw_error = wrapPi(target_yaw - current_yaw);

  // 2. Anti-windup：車轉不動時目標不准一直往前跑
  //    原版 target_yaw 無限積分 → 誤差可累積到數十度 → 輸出長期頂死 ±45，甚至超過 π 後反向
  if (yaw_error > YAW_LEAD_MAX) {
    yaw_error = YAW_LEAD_MAX;
    target_yaw = wrapPi(current_yaw + YAW_LEAD_MAX);
  } else if (yaw_error < -YAW_LEAD_MAX) {
    yaw_error = -YAW_LEAD_MAX;
    target_yaw = wrapPi(current_yaw - YAW_LEAD_MAX);
  }

  // 3. 前饋 + P(角度) + D(角速度誤差)
  //    原版 D = -Kd·ω 在穩態自轉時會「反抗」指令角速度，只能靠 P 堆大誤差硬推；
  //    改成 Kd·(ω_ref - ω)，穩態時 D≈0，主要推力由前饋提供，P 只修正小偏差
  float turn_output = Kff_yaw * rate_ref
                      + Kp_yaw * yaw_error
                      + Kd_yaw * (rate_ref - current_yaw_rate);

  return constrain(turn_output, -TURN_MAX, TURN_MAX);
}

// 轉向權限仲裁：直立平衡永遠優先
float Turn_Authority_Limit(float balance, float turn, float tilt_err) {
  // ① 傾角動態壓制 (Tilt Suppression)：傾角誤差越大，轉向權限線性降到 0
  float g;
  if (tilt_err <= TILT_FULL) {
    g = 1.0f;
  } else if (tilt_err >= TILT_ZERO) {
    g = 0.0f;
  } else {
    g = (TILT_ZERO - tilt_err) / (TILT_ZERO - TILT_FULL);
  }
  turn *= g;
  debug_tilt_gain = g;

  // ② 飽和比例壓縮（平衡優先版）：只壓縮轉向量，讓 |balance| + |turn| ≤ PWM_LIMIT
  //    → 兩輪平均值（直立力矩）永遠等於 balance，差速依剩餘餘裕等比例縮小
  float a_bal = fabsf(balance);
  float a_turn = fabsf(turn);
  if (a_turn > 0.0f && (a_bal + a_turn) > PWM_LIMIT) {
    float headroom = PWM_LIMIT - a_bal;
    if (headroom < 0.0f) headroom = 0.0f;
    turn *= headroom / a_turn;
  }
  return turn;
}

#if ENABLE_EMF_FF
// [v34] 反電動勢前饋（V 與 PWM 同號：V 負 = 前進 = PWM 負）
static inline float emfFF(float v) {
  float a = fabsf(v);
  if (a <= EMF_V_DB) return 0.0f;
  float ff = EMF_K * (a - EMF_V_DB);
  if (ff > EMF_FF_MAX) ff = EMF_FF_MAX;
  return (v >= 0.0f) ? ff : -ff;
}
#endif

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
// allow_integral = false：只用 P 項抑制平移（轉向期間用），積分保持不動
// [v26] 里程計：ds = 左右輪平均位移 (ticks，正 = X 鍵後退方向)
void Odo_Update(float ds) {
  float h = wrapPi(current_yaw - odo_yaw0);  // 相對原點的車頭方向（D 右旋為正）
  float fwd = -ds;                            // 前進（W）為正
  odo_x += fwd * cosf(h);
  odo_y += fwd * sinf(h);
}

#define STOP_ASSIST_V 5.0f    // [v29] 目標速度 0 時，|速度| 超過這個值才啟動積分加速
#define STOP_KI_MULT  4.0f    // [v29] 積分加快倍數（原本約 8 秒才停得住，約 2 秒）
float Speed_PI(float target_spd, bool allow_integral, float p_scale) {
  // 採樣當前左右輪速度平均值 (Ticks / 40.5ms)
  long cL, cR;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    cL = encoder_count_L;
    cR = encoder_count_R;
    encoder_count_L = 0;
    encoder_count_R = 0;
  }
  odo_ticks2 += cL + cR;  // [v13] 里程累積
  float raw_speed = (cL + cR) / 2.0;
  last_raw_speed = raw_speed;  // [v24]
  Odo_Update(raw_speed);       // [v26] 虛擬座標

  // 一階低通濾波
  filtered_speed = (filtered_speed * 0.7) + (raw_speed * 0.3);

  // 速度偏差
  float speed_error = filtered_speed - target_spd;

  // ★★★ 核心修正 1：放寬累積門檻，讓溜車時能持續尋找重心 ★★★
  // 只有在速度誤差極大（高速行駛或急煞車 > 12.0 ticks）時才凍結積分
  if (!allow_integral) {
    // 轉向期間：積分凍結（保留重心補償），不累積也不衰減
  } else if (abs(speed_error) < 12.0) {
    // [v29] 停車助推：目標速度 0 但車子還在滾（轉彎後常見，V 可滾 9 以上好幾秒），積分加快 STOP_KI_MULT 倍
    float ki_mult = (target_spd == 0.0f && fabsf(filtered_speed) > STOP_ASSIST_V) ? STOP_KI_MULT : 1.0f;
    speed_integral += speed_error * ki_mult;
  } else {
    speed_integral *= 0.95;  // 溫和衰減，不再暴力清空
  }

  // ★★★ 核心修正 2：大幅放大積分上限 (提供 ±2.5° 的重心修正頻寬) ★★★
  // 原本 250 限制換算只有 0.07 度，放大到 3000 可提供約 2.5 度的補償能力
  speed_integral = constrain(speed_integral, -3000.0, 3000.0);

  // 外環輸出：轉換為直立環的「目標傾斜補償角 (rad)」
  // Kp_speed = 2.0, Ki_speed = 0.015 (約可輸出 ±0.045 rad ≈ ±2.6°)
  float angle_offset = (Kp_speed * p_scale * speed_error + Ki_speed * speed_integral) * 0.001;

  // 限幅最大補償（[v34] 超速煞車開啟時放寬到 ±4.0°，一般時候後面仍限制在 ±3.5°）
  return constrain(angle_offset, -SPEED_PI_CLAMP, SPEED_PI_CLAMP);
}
// ★★★ SETUP ★★★
void setup() {
  Serial.begin(9600);  // 採用高速序列埠傳輸
  Nav_Init(&nav);      // [v27]
  Wire.begin();
  Wire.setClock(400000);
  initMotorPins();
  stopMotors();

  // pinMode(led, OUTPUT);  // [10/10] A0 改當左輪 B 相輸入，不再設成輸出
  pinMode(ENC_PIN_L, INPUT_PULLUP);
  pinMode(ENC_PIN_R, INPUT_PULLUP);
  pinMode(ENC_B_PIN_L, INPUT_PULLUP);  // [v25] B 相
  pinMode(ENC_B_PIN_R, INPUT_PULLUP);

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
    Serial.print(F("v34 EMF/OVS/FAST="));  // [v34] 確認有沒有燒到新版
    Serial.print(ENABLE_EMF_FF);
    Serial.print(ENABLE_OVERSPEED_BRAKE);
    Serial.println(ENABLE_FAST_START);
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
    // [v28] 修正 D 項：10/11 實測 dmpGetGyro 的 X 軸值幾乎永遠是 0（直立環的 D 項等於沒有），
    //       改直接讀 GYRO_XOUT 暫存器（±2000°/s → 16.4 LSB/(°/s)，已驗證量值與角度微分一致 ±10%）。
    //       暫存器讀到的方向和「角度對時間的微分」相反，所以前面的負號保留，結果 = 角度的變化率 (rad/s)
    current_gyro_rate = -(mpu.getRotationX() / 16.4f) * DEG_TO_RAD;

    // ★★★ 讀取DMP Yaw 航向角 ★★★
    current_yaw = ypr_temp[0];
    // [v3] 航向角速度改由 DMP Yaw 微分求得
    //      實測 raw_gyro[2]/16.4 與 Yaw 實際變化率差約 -22 倍（方向相反、量值過小），不可用
    static float last_yaw_for_rate = 0.0f;
    static float yaw_rate_filt = 0.0f;
    static bool yaw_rate_ready = false;
    if (!yaw_rate_ready) {
      last_yaw_for_rate = current_yaw;
      yaw_rate_ready = true;
    }
    float yaw_rate_raw = wrapPi(current_yaw - last_yaw_for_rate) / CTRL_DT;
    last_yaw_for_rate = current_yaw;
    yaw_rate_filt = 0.6f * yaw_rate_filt + 0.4f * yaw_rate_raw;  // 一階低通，壓掉封包時間抖動
    float current_yaw_rate = yaw_rate_filt;
    debug_yaw_rate = current_yaw_rate;
    debug_yaw = current_yaw;

    // 開機站立時，自動將開機朝向設為初始目標航向
    if (!yaw_initialized && !is_fallen) {
      target_yaw = current_yaw;
      initial_yaw = current_yaw;
      yaw_initialized = true;
      odo_reset_req = true;  // [v26] 每次站起來 = 新的座標原點
    }
    if (odo_reset_req) {     // [v26] 座標歸零（站起來或按 z）
      odo_x = 0.0f;
      odo_y = 0.0f;
      odo_yaw0 = current_yaw;
      odo_reset_req = false;
    }
    float angle_err = fabsf(current_angle - base_setpoint);

    // =========================================================================
    // 【優先級 1：最高】極限傾倒安全保護
    // =========================================================================
    if (angle_err > fall_limit) {
      if (Nav_Active(&nav)) Nav_Finish(&nav, NAV_RES_ABORT, NAV_AB_FALL, nav.remaining);  // [v27]
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
      drive_w = 0.0f;  // [v32]
      drive_ref = 0.0f;
      target_speed = 0.0;
      move_boost = 0.0f;  // [v20]
      v_slow = 0.0f;
      spin_dx = 0.0f;     // [v24]
      yaw_initialized = false;
      yaw_rate_ref = 0.0f;  // [新增] 轉向斜坡歸零
      is_homing = false;
      heading_mem_valid = false;  // [v22]
      heading_locked = false;
    }
    // =========================================================================
    // 正常運行防線（未摔倒）
    // =========================================================================
    else {
      // [v11] 馬達強制斷電：視同跌倒，但繼續更新姿態與遙測
      if (motor_kill) {
        if (Nav_Active(&nav)) Nav_Finish(&nav, NAV_RES_ABORT, NAV_AB_USER, nav.remaining);  // [v27]
        if (!is_fallen) {
          speed_integral = 0.0;
          filtered_speed = 0.0;
          speed_angle_target = 0.0;
          target_speed = 0.0;
          move_boost = 0.0f;  // [v20]
          v_slow = 0.0f;
          spin_dx = 0.0f;     // [v24]
          yaw_initialized = false;
          yaw_rate_ref = 0.0f;
          is_homing = false;
          heading_mem_valid = false;  // [v22]
          heading_locked = false;
          is_emergency_state = false;
        }
        is_fallen = true;
        recovery_counter = 0;
        stopMotors();
      }
      if (is_fallen) {
        if (angle_err < (3.0 * DEG_TO_RAD)) {
          recovery_counter++;
          if (recovery_counter > 20) {
            is_fallen = false;
            recovery_counter = 0;
            // [v25] 倒地 / 斷電期間累積的計數丟掉，避免站起來第一次速度計算出現假的大速度
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
              encoder_count_L = 0;
              encoder_count_R = 0;
            }
            filtered_speed = 0.0f;
          }
        }
      }

      if (!is_fallen) {
        if (!is_emergency_state && (angle_err > LARGE_ANGLE_ENTER)) {
          is_emergency_state = true;
        } else if (is_emergency_state && (angle_err < LARGE_ANGLE_EXIT)) {
          is_emergency_state = false;
        }

        // =======================================================================
        // 【優先級 2】大角度緊急姿態救援：轉向完全屏蔽
        // =======================================================================
        if (is_emergency_state) {
          if (Nav_Active(&nav)) Nav_Finish(&nav, NAV_RES_ABORT, NAV_AB_EMERG, nav.remaining);  // [v27]
          speed_integral = 0.0;
          speed_angle_target = 0.0;
          target_speed = 0.0;
          move_boost = 0.0f;  // [v20]
          v_slow = 0.0f;
          spin_dx = 0.0f;     // [v24]
          adj_setpoint = base_setpoint;
          target_yaw = current_yaw;
          yaw_rate_ref = 0.0f;  // [新增]
          heading_mem_valid = false;  // [v22]
          heading_locked = false;
          encoder_count_L = 0;
          encoder_count_R = 0;
          filtered_speed = 0.0;

          float emergency_pwm = Balance_PD(current_angle, adj_setpoint, current_gyro_rate, true);
          debug_pwm = emergency_pwm;
          debug_balance_pwm = emergency_pwm;
          debug_final_turn = 0.0;
          debug_tilt_gain = 0.0f;
          debug_motor_L = emergency_pwm;
          debug_motor_R = emergency_pwm;
          debug_target_yaw = target_yaw;
          setMotorSpeed(emergency_pwm, emergency_pwm);
        }
        // =======================================================================
        // 【優先級 3】常態串級控制
        // =======================================================================
        else {
          // ---------- [A] 轉向指令整形：決定期望角速度 → 斜坡限制 ----------
          // [v27] 導航：處理 loop() 送來的 #T / #M / #A 請求
          float nav_yaw_rel = wrapPi(current_yaw - odo_yaw0);
          Nav_Service(&nav, nav_yaw_rel, odo_x, odo_y, TICKS_PER_CM);
          float rate_des = 0.0f;
          if (fabsf(target_yaw_rate_cmd) > 0.1f) {  // A / D 手動自轉
            is_homing = false;
            rate_des = target_yaw_rate_cmd;
          } else if (nav.state == NAV_TURN) {  // [v27] 導航轉向：依角度誤差給角速度（同 C 鍵回正的作法）
            is_homing = false;
            rate_des = Nav_TurnRate(&nav, nav_yaw_rel, current_yaw_rate);
          } else if (fabsf(target_speed_cmd) > 0.1f) {  // W / X 直行：打斷回正（與原版優先序相同）
            is_homing = false;
          } else if (is_homing) {  // C 鍵回正：轉成「有限角速度」的軌跡
            float homing_err = wrapPi(initial_yaw - current_yaw);
            if (fabsf(homing_err) < (1.5f * DEG_TO_RAD)) {
              is_homing = false;
            } else {
              rate_des = constrain(HOMING_KP * homing_err, -HOMING_RATE_MAX, HOMING_RATE_MAX);
            }
          }
          // [v12] 加速用 YAW_RATE_SLEW，減速（往 0 靠近）用較緩的 YAW_RATE_DECEL
          bool decelerating = (fabsf(rate_des) < fabsf(yaw_rate_ref)) || (rate_des * yaw_rate_ref < 0.0f);
          const float slew_step = (decelerating ? YAW_RATE_DECEL : YAW_RATE_SLEW) * CTRL_DT;
          yaw_rate_ref += constrain(rate_des - yaw_rate_ref, -slew_step, slew_step);
          bool turning = (rate_des != 0.0f) || (fabsf(yaw_rate_ref) > 0.01f);

          // ---------- [B] 速度外環：轉向期間仍以「目標速度 0」運作（只用 P 項） ----------
          static int speed_loop_counter = 0;
          speed_loop_counter++;
          if (speed_loop_counter >= SPEED_LOOP_DIV) {
            speed_loop_counter = 0;
            float sa_prev = speed_angle_target;  // [v12] 供變化率限制使用
            if (!speed_loop_enabled) {
              // [v10] 量測模式：速度外環完全關閉，直立環只追 base_setpoint
              speed_angle_target = 0.0f;
              speed_integral = 0.0f;
              filtered_speed = 0.0f;
              move_boost = 0.0f;  // [v20]
              v_slow = 0.0f;
              spin_dx = 0.0f;     // [v24]
              long cL, cR;
              ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                cL = encoder_count_L;
                cR = encoder_count_R;
                encoder_count_L = 0;
                encoder_count_R = 0;
              }
              odo_ticks2 += cL + cR;  // [v11] 累積位移
              Odo_Update((cL + cR) / 2.0f);  // [v26] 虛擬座標
            } else if (turning) {
              // 自轉中（含加減速段）：左右輪平均 = 車身「平移」速度（B 相量方向，永遠正確）
              target_speed = 0.0f;
              move_boost = 0.0f;  // [v20]
              v_slow = 0.0f;
              speed_angle_target = Speed_PI(0.0f, false, 1.0f);
              // [v24] 自轉定點：累積平移量，往漂移的反方向補（與 P 項同號：往 + 漂 → SA 往正）
              spin_dx += last_raw_speed;
              float hold = constrain(SPIN_HOLD_K * spin_dx, -SPIN_HOLD_MAX, SPIN_HOLD_MAX);
              spin_dx = hold / SPIN_HOLD_K;  // 上限處不再累積（anti-windup）
              speed_angle_target = constrain(speed_angle_target + hold, -3.5f * DEG_TO_RAD, 3.5f * DEG_TO_RAD);
            } else {
              // [v27] 導航直走：目標速度由「剩餘距離」決定（越接近越慢），之後照一般 W/X 的斜坡、煞車、停車邏輯
              static bool nav_drove = false;
              if (nav.state == NAV_MOVE || nav.state == NAV_STOP) {
                target_speed_cmd = Nav_MoveSpeed(&nav, odo_x, odo_y, filtered_speed, TICKS_PER_CM);
                nav_drove = true;
              } else if (nav_drove) {
                nav_drove = false;
                if (nav.last_reason != NAV_AB_USER) target_speed_cmd = 0.0f;  // 手動中止時保留使用者新按的鍵
              }
              // [v13] 起步/煞車分開斜率：往 0 靠近用 MOVE_DECEL_STEP
              // [v20] 反向（上一次的目標速度還沒歸零就按了相反方向）改用 MOVE_ACCEL_STEP 穿過 0
              //       v19 實測：W/X 交替時，按下時 TV 常還有 1.2~1.9 → 要先花 0.5 s 慢慢歸零才開始反向
              bool reversing = (target_speed_cmd * target_speed < 0.0f);
              bool braking = reversing || (fabsf(target_speed_cmd) < fabsf(target_speed));
              float step = (braking && !reversing) ? MOVE_DECEL_STEP : MOVE_ACCEL_STEP;
#if ENABLE_FAST_START
              // [v34] 目標速度還在 ±FAST_START_TV 內時車子本來就推不動（log：TV 到約 -3 才開始動），不必慢慢爬
              if (step == MOVE_ACCEL_STEP && fabsf(target_speed) < FAST_START_TV) step = FAST_START_STEP;
#endif
              if (target_speed < target_speed_cmd) {
                target_speed = min(target_speed + step, target_speed_cmd);
              } else if (target_speed > target_speed_cmd) {
                target_speed = max(target_speed - step, target_speed_cmd);
              }
              // [v15→v16] 移動 / 停車階段管理
              //   積分：移動中凍結；目標歸零後，等車速第一次越過 0（= 滑行最遠點）才恢復，
              //         讓積分從「真正停下的位置」開始記，而不是從往回退的半路
              //   P：移動中 + 停車階段 ×MOVE_KP_SCALE
              static uint8_t move_settle = 0;
              static bool integral_hold = false;
              static int8_t move_dir = 0;
              bool cmd_active = (target_speed != 0.0f) || (target_speed_cmd != 0.0f);
              if (cmd_active) {
                move_settle = MOVE_SETTLE_UPDATES;
                integral_hold = true;
                if (target_speed > 0.0f) move_dir = 1;
                else if (target_speed < 0.0f) move_dir = -1;
              } else if (move_settle > 0) {
                move_settle--;
              }
              if (!cmd_active && integral_hold) {
                if (filtered_speed * move_dir <= 0.0f || move_settle == 0) integral_hold = false;
              }
              bool braking_phase = (target_speed_cmd == 0.0f) ||
                                   (fabsf(target_speed_cmd) < fabsf(target_speed)) ||
                                   (target_speed_cmd * target_speed < 0.0f);
              // [v19] 只在煞車 / 停車階段加強 P；[v21] 改成約 0.24 s 線性加上去
              bool brake_gain = (move_settle > 0 && braking_phase);
              static uint8_t brake_ramp = 0;
              if (brake_gain) {
                if (brake_ramp < BRAKE_RAMP_UPDATES) brake_ramp++;
              } else {
                brake_ramp = 0;
              }
              float p_scale = 1.0f + (MOVE_KP_SCALE - 1.0f) * brake_ramp / (float)BRAKE_RAMP_UPDATES;
#if ENABLE_OVERSPEED_BRAKE
              // [v34] 超速：實際速度遠超過目標（且方向相同，含目標 0 的放開後滑行）→ P 增益至少 ×MOVE_KP_SCALE
              bool overspeed = (fabsf(filtered_speed) > fabsf(target_speed) + OVERSPEED_MARGIN) &&
                               (filtered_speed * target_speed >= 0.0f);
              if (overspeed && p_scale < MOVE_KP_SCALE) p_scale = MOVE_KP_SCALE;
#endif
              speed_angle_target = Speed_PI(target_speed, !integral_hold, p_scale);

              // [v20] 起步防卡死：卡住時慢慢加推力，跑起來後退掉
              v_slow = 0.8f * v_slow + 0.2f * filtered_speed;
              int8_t cmd_dir = (target_speed_cmd > 0.0f) ? 1 : ((target_speed_cmd < 0.0f) ? -1 : 0);
              if (cmd_dir != 0 && cmd_dir != boost_dir) {  // 換方向：推力從 0 重新開始
                move_boost = 0.0f;
                boost_dir = cmd_dir;
              }
              if (cmd_dir != 0 && !braking_phase && fabsf(target_speed) >= BOOST_START_TV) {
                float progress = v_slow * cmd_dir;  // 往指令方向的實際速度
                float tv_abs = fabsf(target_speed);
                if (progress < BOOST_STALL_FRAC * tv_abs) {
                  move_boost = min(move_boost + BOOST_RATE, BOOST_MAX);  // 卡住：加推力
                } else if (progress >= tv_abs) {
                  move_boost *= BOOST_LEAK_MOVE;  // 已經跑起來：退掉
                }                                 // 介於中間：維持
              } else {
                move_boost *= BOOST_LEAK_STOP;    // 放開 / 煞車：快速歸零
              }
              // 方向：與 P 項在 V=0 時同號（W → SA 往正、X → SA 往負）
              speed_angle_target -= boost_dir * move_boost;
              // [v24] 自轉定點補償：自轉結束後平滑退掉（不突然消失，避免直立目標角跳變）
              spin_dx *= SPIN_HOLD_DECAY;
              speed_angle_target += SPIN_HOLD_K * spin_dx;
              // [v21] 煞車 / 停車階段用較小的上限（限制最大後仰），其他時候維持 ±3.5°
              float sa_lim = brake_gain ? BRAKE_SA_MAX : (3.5f * DEG_TO_RAD);
#if ENABLE_OVERSPEED_BRAKE
              if (overspeed && sa_lim < OVERSPEED_SA_MAX) sa_lim = OVERSPEED_SA_MAX;  // [v34]
#endif
              speed_angle_target = constrain(speed_angle_target, -sa_lim, sa_lim);
            }
            // [v12] 補償角變化率限制（量測模式除外）
            if (speed_loop_enabled) {
              float sa_slew = turning ? SA_SLEW_SPIN : SA_SLEW_FREE;  // [v26] 只在自轉進行中限制
              speed_angle_target = constrain(speed_angle_target,
                                             sa_prev - sa_slew,
                                             sa_prev + sa_slew);
            }
            debug_v = filtered_speed;   // [v13]
            debug_tv = target_speed;
            debug_bs = (-boost_dir * move_boost + SPIN_HOLD_K * spin_dx) * RAD_TO_DEG;  // [v20] 起步推力 + [v24] 自轉定點
          }

          // ---------- [C] 直立內環 ----------
          adj_setpoint = base_setpoint + speed_angle_target;
          debug_sa = speed_angle_target * RAD_TO_DEG;
          float balance_pwm = Balance_PD(current_angle, adj_setpoint, current_gyro_rate, false);
#if ENABLE_EMF_FF
          balance_pwm = constrain(balance_pwm + emfFF(filtered_speed), -PWM_LIMIT, PWM_LIMIT);  // [v34]
#endif
          debug_pwm = balance_pwm;

          // ---------- [D] 航向環 ----------
          float final_turn = 0.0f;
          // [v22] 直行（含煞車滑行）期間都算「移動中」，停下後再鎖 1 s
          static uint8_t heading_hold = 0;
          bool move_active = (fabsf(target_speed_cmd) > 0.1f) || (target_speed != 0.0f);
          if (move_active) heading_hold = HEADING_HOLD_TICKS;
          else if (heading_hold > 0) heading_hold--;

          // [v32] #D 連續轉向：drive_w 經斜坡（3 rad/s²）後得到 drive_ref，非自轉時才生效，速度外環照常運作
          {
            const float st = 3.0f * CTRL_DT;
            float dd = drive_w - drive_ref;
            drive_ref += (dd > st) ? st : ((dd < -st) ? -st : dd);
          }
          bool driving = !turning && (fabsf(drive_ref) > 0.005f);

          if (turning) {  // 自轉 / 回正（含加減速斜坡段）
            heading_mem_valid = false;  // [v22] 方向被刻意改變：記憶作廢
            heading_locked = false;
            heading_hold = 0;
            target_yaw = wrapPi(target_yaw + yaw_rate_ref * CTRL_DT);
            final_turn = Heading_Control_PD(current_yaw_rate, yaw_rate_ref);
          } else if (nav.state == NAV_MOVE || nav.state == NAV_STOP) {  // [v27] 導航直走：鎖在出發時的方向
            target_yaw = wrapPi(odo_yaw0 + nav.move_heading);
            heading_locked = true;
            heading_mem = target_yaw;
            heading_mem_valid = true;
            final_turn = Heading_Control_PD(current_yaw_rate, nav.yaw_ff);  // [v30] 直走時 yaw_ff = 0；#C 弧線時 = 曲率 × 前進速度
          } else if (driving) {  // [v32] #D：邊走邊轉（目標航向跟著 drive_ref 前進，角速度前饋 = drive_ref）
            if (!heading_locked) {
              target_yaw = current_yaw;
              heading_locked = true;
            }
            target_yaw = wrapPi(target_yaw + drive_ref * CTRL_DT);
            final_turn = Heading_Control_PD(current_yaw_rate, drive_ref);
          } else if (move_active || heading_hold > 0) {  // 直行 + 煞車 + 停下後 1 s：鎖航向
            if (!heading_locked) {  // [v22] 這一次移動剛開始：決定要鎖哪個方向
              if (heading_mem_valid && fabsf(wrapPi(heading_mem - current_yaw)) < HEADING_MEM_MAX) {
                target_yaw = heading_mem;  // 沿用上一次的直線
              } else {
                target_yaw = current_yaw;
              }
              heading_locked = true;
            }
            final_turn = Heading_Control_PD(current_yaw_rate, 0.0f);
            heading_mem = target_yaw;
            heading_mem_valid = true;
          } else {  // 靜止：不做航向閉環
            heading_locked = false;
            target_yaw = current_yaw;
            final_turn = 0.0f;
          }

          // ---------- [E] 轉向權限仲裁 + 輸出 ----------
          float tilt_err = fabsf(current_angle - adj_setpoint);
          float turn_cmd = Turn_Authority_Limit(balance_pwm, final_turn, tilt_err);

          float motor_L = balance_pwm + turn_cmd;
          float motor_R = balance_pwm - turn_cmd;

          debug_balance_pwm = balance_pwm;
          debug_final_turn = turn_cmd;
          debug_motor_L = motor_L;
          debug_motor_R = motor_R;
          debug_target_yaw = target_yaw;
          setMotorSpeed(motor_L, motor_R);
        }
      }
    }
    // [v28] 診斷錄影（放在所有控制輸出之後，記錄「這個週期實際送出的 PWM」；ENABLE_REC 為 0 時整行不會編譯）
    REC_CAPTURE(mpu, current_angle, current_gyro_rate, adj_setpoint, debug_balance_pwm, debug_v, debug_tv, debug_sa, debug_bs, debug_pwm_L);
  }
  is_computing = false;
}

// ★★★ 主迴圈 (鍵盤/手把控制與狀態推播) ★★★
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 300;  // 超過 300ms 沒訊號自動煞停 (斷線防暴衝)

#include "nav_cmd.h"  // [v31] 導航指令解析 Nav_HandleLine() 與回報 Nav_Report()
#include "tuning.h"  // [v33] 現場調參按鍵 Tune_Key()（ENABLE_TUNING 為 0 時是空的）

void loop() {
  // 1. 鍵盤 / 序列埠指令監聽
  while (Serial.available() > 0) {
    char c = Serial.read();
    last_rx_ms = millis();  // [v27] 任何字元都算心跳（Python 導航時每 0.1 s 送一個 '.'）

    // [v27] 導航指令列
    if (nav_in_cmd) {
      if (c == '\r' || c == '\n') {
        nav_in_cmd = false;
        Nav_HandleLine();
      } else if (nav_len < sizeof(nav_buf) - 1) {
        nav_buf[nav_len++] = c;
      }
      continue;
    }
    if (c == '#') {
      nav_in_cmd = true;
      nav_len = 0;
      continue;
    }

    // 忽略換行字元
    if (c == '\r' || c == '\n') continue;

    // [v27] 導航中按任何移動鍵 → 立即中止導航（使用者接手）
    if (Nav_Active(&nav) && (c == 'w' || c == 'W' || c == 'x' || c == 'X' || c == 'a' || c == 'A' ||
                             c == 'd' || c == 'D' || c == 's' || c == 'S' || c == ' ' || c == 'c' || c == 'C')) {
      nav.req_abort = NAV_AB_USER;
    }

    // [v11] k：馬達強制斷電開/關（跌倒狀態下也要能切換，所以放在 is_fallen 判斷之前）
    // [v26] z：虛擬座標歸零（目前位置 = 原點、目前車頭 = X 軸）
    if (c == 'z' || c == 'Z') {
      odo_reset_req = true;
      Serial.println(F(">> 座標歸零"));
      continue;
    }

    if (REC_KEY(c)) continue;  // [v28] 診斷錄影 r / m 鍵（ENABLE_REC 為 0 時整行不會編譯）

    if (c == 'k' || c == 'K') {
      motor_kill = !motor_kill;
      if (motor_kill) {  // [v25] 進入斷電模式時把方向測試累計歸零
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { enc_total_L = 0; enc_total_R = 0; }
      }
      Serial.println(motor_kill ? F(">> 馬達強制斷電（手扶量測模式）") : F(">> 馬達解除斷電（扶正後自動啟動）"));
      continue;
    }

    if (!is_fallen) {
      last_cmd_time = millis();  // 刷新指令接收時間
      if (c != '.') drive_w = 0.0f;  // [v32] 任何按鍵都取消 #D（'.' 是心跳，不算）

      switch (c) {
        // 前進 (W)
        case 'w':
        case 'W':
          if (target_speed_cmd != MOVE_SPEED_FWD) {  // [v13] 新的一次前進：里程歸零
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { odo_ticks2 = 0; }
          }
          target_speed_cmd = MOVE_SPEED_FWD;
          break;

        // 後退 (X) - 修正：改由 X 觸發
        case 'x':
        case 'X':
          if (target_speed_cmd != MOVE_SPEED_BACK) {  // [v13] 新的一次後退：里程歸零
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { odo_ticks2 = 0; }
          }
          target_speed_cmd = MOVE_SPEED_BACK;
          break;

        // 原地左旋 (A) - 設定目標角速度為 -2.2 rad/s (約每秒旋轉 126 度)
        case 'a':
        case 'A':
          target_yaw_rate_cmd = -1.0;
          break;

        // 原地右旋 (D) - 設定目標角速度為 +2.2 rad/s
        case 'd':
        case 'D':
          target_yaw_rate_cmd = 1.0;
          break;

        // 自動回正鍵 (C)
        case 'c':
        case 'C':
          is_homing = true;  // 啟動回正程序
          // [重構] 不再清 speed_integral：它保存重心補償，清掉會讓直立目標角瞬間跳變
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
          break;

        // [v33] o / g / h / j / l / [ ] 已搬到 tuning.h（ENABLE_TUNING 為 0 時整行不會編譯）
        default: (void)TUNE_KEY(c); break;
      }
    }
  }

  // ★ 雙重防護：若傳輸線鬆脫或 Python 漏發放開訊號，逾時自動煞車
  // [v27] 導航中：改用 1 s 心跳逾時（導航時速度指令由中斷自己給，不靠按鍵重送）
  if (Nav_Active(&nav) && !nav.req_abort && millis() - last_rx_ms > 1000) {
    nav.req_abort = NAV_AB_LINK;
  }
  Nav_Report();  // [v27] 導航完成 / 中止回報

  REC_OUTPUT(Kd_balance_normal);  // [v28] 診斷錄影輸出（ENABLE_REC 為 0 時整行不會編譯）

  if (!is_fallen && !Nav_Active(&nav) && (target_speed_cmd != 0.0 || target_yaw_rate_cmd != 0.0)) {
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
      target_speed_cmd = 0.0;
      target_yaw_rate_cmd = 0.0;
      drive_w = 0.0f;  // [v32]
    }
  }

  // 2. 狀態與控制診斷推播 (每 200ms 輸出一次)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    float snapshot_angle;
    float snapshot_debug_pwm;
    float snapshot_sa;
    float snapshot_balance_pwm;
    float snapshot_final_turn;
    float snapshot_motor_L;
    float snapshot_motor_R;
    int snapshot_pwm_L;
    int snapshot_pwm_R;
    float snapshot_yaw_rate;
    float snapshot_yaw;
    float snapshot_target_yaw;
    float snapshot_tilt_gain;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      snapshot_angle = current_angle;
      snapshot_debug_pwm = debug_pwm;
      snapshot_sa = debug_sa;
      snapshot_balance_pwm = debug_balance_pwm;
      snapshot_final_turn = debug_final_turn;
      snapshot_motor_L = debug_motor_L;
      snapshot_motor_R = debug_motor_R;
      snapshot_pwm_L = debug_pwm_L;
      snapshot_pwm_R = debug_pwm_R;
      snapshot_yaw_rate = debug_yaw_rate;
      snapshot_yaw = debug_yaw;
      snapshot_target_yaw = debug_target_yaw;
      snapshot_tilt_gain = debug_tilt_gain;
    }
    Serial.print("ANG:");
    Serial.print(snapshot_angle * RAD_TO_DEG, 1);
    Serial.print(" SA:");  // [v9] 原 BPWM（與 B 重複）改為速度外環補償角 (deg)
    Serial.print(snapshot_sa, 2);
    Serial.print(" B:");
    Serial.print(snapshot_balance_pwm, 1);
    Serial.print(" T:");
    Serial.print(snapshot_final_turn, 1);
    Serial.print(" PWM:");
    Serial.print(snapshot_pwm_L);
    Serial.print('/');
    Serial.print(snapshot_pwm_R);
    Serial.print(" YR:");
    Serial.print(snapshot_yaw_rate, 2);
    Serial.print(" Y:");
    Serial.print(snapshot_yaw, 2);
    Serial.print(" TY:");
    Serial.print(snapshot_target_yaw, 2);
    if (snapshot_tilt_gain < 0.995f) {  // [v26] 轉向權限被壓縮時才顯示（省頻寬）
      Serial.print(" G:");
      Serial.print(snapshot_tilt_gain, 2);
    }
    Serial.print(" SP:");  // [v10] 目前 base_setpoint (deg)
    Serial.print(base_setpoint * RAD_TO_DEG, 2);
    if (!speed_loop_enabled) Serial.print(" OL:0");  // [v26] 速度外環關閉時才顯示
    long dx;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { dx = odo_ticks2 / 2; }
    Serial.print(" DX:");  // [v13] 起步（或關閉外環）後的累積位移 (ticks)
    Serial.print(dx);
    float sv, stv, sbs;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sv = debug_v; stv = debug_tv; sbs = debug_bs; }
    Serial.print(" V:");   // [v13] 實際速度 (ticks/40ms)
    Serial.print(sv, 1);
    Serial.print(" TV:");  // [v13] 目標速度 (ticks/40ms)
    Serial.print(stv, 2);
    Serial.print(" BS:");  // [v20] 起步推力 (deg)；[v24] 自轉時為定點補償 (deg)
    Serial.print(sbs, 2);
    float ox, oy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ox = odo_x; oy = odo_y; }
    Serial.print(" PX:");  // [v26] 虛擬座標 (cm)：往前為正（Y 已被航向角使用，所以用 PX / PY）
    Serial.print(ox / TICKS_PER_CM, 1);
    Serial.print(" PY:");  // [v26] 往右為正
    Serial.print(oy / TICKS_PER_CM, 1);
    uint8_t nst;
    float nrm;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { nst = nav.state; nrm = nav.remaining; }
    if (nst != NAV_IDLE) {  // [v27] 導航中：NV 狀態（1 轉向 / 2 直走 / 3 停車）、NR 剩餘量（度或公分）
      Serial.print(" NV:");
      Serial.print(nst);
      Serial.print(" NR:");
      Serial.print(nst == NAV_TURN ? nrm * RAD_TO_DEG : nrm / TICKS_PER_CM, 1);
    }
    if (is_fallen) {  // [v25] 倒地 / 斷電時多印左右輪累計 tick（方向測試用）
      long el, er;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { el = enc_total_L; er = enc_total_R; }
      Serial.print(" EL:");
      Serial.print(el);
      Serial.print(" ER:");
      Serial.print(er);
    }
    Serial.print(" | 狀態:");
    if (is_fallen) {
      Serial.println("【跌倒保護】");
    } else if (is_emergency_state) {
      Serial.println("【大角度緊急救援】");
    } else {
      Serial.println("【常態雙閉環】");
    }
  }
}
