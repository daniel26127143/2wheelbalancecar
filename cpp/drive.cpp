// ============================================================================
// drive.cpp — 像遙控汽車一樣開小車（對應 Python 的 drive.py）
//
// 需要韌體 v32 以上的 #D 指令。編譯：  g++ -std=c++17 -O2 drive.cpp -o drive.exe
//
//   drive.exe                  鍵盤（方向盤式：按住轉向越轉越大，放開自動回中）
//   drive.exe --port COM5      指定序列埠（預設 COM3）
//   drive.exe --dry-run        不連線，只印出每個畫面會送出的指令（確認按鍵與數值）
//
// 鍵盤
//   W / ↑   油門前進（按住越按越快，放開慢慢減速）      X / ↓   後退
//   A / ←   向左轉（按住越轉越大，放開自動回中）         D / →   向右轉
//   S / 空白  立刻停（油門與方向歸零）                    C       方向回中
//   Z       座標歸零                                      Q / Esc 結束（會先停車）
//   Y       設定航向（韌體 v37 以上）：按下當下的車頭方向 = 航向 setpoint；車子停著時偏離超過約 4° 就自己轉回來。
//           沒按過 Y 就不做靜止航向保持；按 A / D / C 或導航後設定作廢，要再按一次 Y
//   U / I   設定航向 -0.25° / +0.25°（韌體 v37e 以上；要先按過 Y，把車子調成沿著你的線走）
//   J / L   速度外環 Kp_speed -0.5 / +0.5     G / H   直立環 Kd -10 / +10   （韌體 ENABLE_TUNING 1 才有效）
//   車子不動（油門 0）時按 A / D = 原地自轉
//
// 怎麼轉：方向值 steer（-1～1）代表「曲率」，角速度 w = steer ÷ 最小半徑 × 前進速度。
// 所以車速越慢轉得越小，車速越快轉得越大（和真的遙控車一樣）；--min-radius 是方向打到底時的轉彎半徑（cm）。
//
// 和 Python 版的差別：沒有 USB 手把（--pad）。其他行為、預設值、記錄檔格式都相同。
// ============================================================================
#include <algorithm>
#include <cerrno>
#include <set>

#include "link.h"

using namespace rc;

// 目標速度 1 單位 ≈ 實際幾 cm/s（直走速度 5.8 約 5.3 cm/s，圓弧速度 8 約 7～8 cm/s）
constexpr double CM_PER_S_PER_UNIT = 0.9;

struct Params {
    double vmax = 18.0;       // 前進最高目標速度（韌體上限 20）；10/13 autotest：W 13 / 18 各 3～4 次都沒倒；選 18 是使用者決定
    double vback = 18.0;      // 後退最高目標速度（韌體上限 20）；10/13 autotest 掃 6～20 都沒倒：9 以上才穩
    double vmin = 17.0;       // 前進最低目標速度（和 vmax 只差 1，一推油門就直接跑）
    double vmin_back = 17.0;  // 後退最低目標速度；目標太低（3.5～5）速度環來回晃、幾乎退不動，所以和 vback 只差 1
    double min_radius = 30.0; // 方向打到底的轉彎半徑 cm（實車只測過 30）
    double wmax = 0.5;        // 邊走邊轉的角速度上限 rad/s（韌體上限 0.6）
    double w_low = 0.12;      // 低速時方向有打的最小角速度 rad/s（否則車速慢時完全轉不動）
    double spin_rate = 0.8;   // 油門 0 時，方向打到底的原地自轉角速度 rad/s（韌體上限 1.0）
    double hz = 10.0;         // 送指令頻率；韌體 300 ms 沒收到就自動煞車
    double thr_up = 2.0;      // 按住油門的加速斜率（單位/秒）
};

// 把油門 / 方向輸入變成 (v, w)。純計算，不碰序列埠。
// thr：油門位置 -1（後退到底）～ 1（前進到底）；steer：方向 -1（左到底）～ 1（右到底）
class DriveModel {
public:
    // 鍵盤的斜率（單位/秒）
    double THR_UP;                       // 按住油門：約 0.5 s 從 0 到滿（太慢會長時間停在低速區，車子卡住頓挫）
    static constexpr double THR_RELEASE = 8.0;   // 放開油門：約 0.12 s 回到 0（vmin 17 時油門在 1.0→0.05 之間目標速度幾乎不變；韌體另外會緩衝）
    static constexpr double STEER_IN = 1.6;      // 按住方向：約 0.6 s 從 0 打到底
    static constexpr double STEER_RELEASE = 3.0; // 放開方向：約 0.3 s 回中

    double thr = 0.0, steer = 0.0;
    Params p;

    explicit DriveModel(const Params& params) : THR_UP(params.thr_up), p(params) {}

    void stop() { thr = steer = 0.0; }
    void center() { steer = 0.0; }

    // thr_dir / steer_dir 為 -1、0、1（目前按住的方向）
    void update_keys(double dt, int thr_dir, int steer_dir) {
        thr = thr_dir != 0 ? approach(thr, (double)thr_dir, THR_UP * dt) : approach(thr, 0.0, THR_RELEASE * dt);
        steer = steer_dir != 0 ? approach(steer, (double)steer_dir, STEER_IN * dt) : approach(steer, 0.0, STEER_RELEASE * dt);
    }

    // 回傳 (v, w)：v = 前進目標速度（韌體單位，前進為正），w = 角速度 rad/s（右轉為正）
    void output(double& v, double& w) const {
        double a = std::fabs(thr);
        if (a <= 0.05) v = 0.0;
        else if (thr > 0) v = p.vmin + (p.vmax - p.vmin) * a;               // 油門一推就從 vmin 起跳（低於 3 的目標速度車子推不動）
        else v = -(p.vmin_back + (p.vback - p.vmin_back) * a);
        if (v == 0.0) {                                                      // 沒有前進：原地自轉（韌體會當成 A / D）
            double ws = steer * p.spin_rate;
            w = std::fabs(ws) >= 0.1 ? ws : 0.0;
            return;
        }
        // 曲率 = 方向值 ÷ 最小半徑；角速度 = 曲率 × 速度（倒車時速度為負，轉向方向自然反過來）
        double v_cm = v * CM_PER_S_PER_UNIT;
        w = (steer / p.min_radius) * v_cm;
        double floor_w = p.w_low * std::fabs(steer);                         // 低速也要有一點轉向
        if (std::fabs(w) < floor_w) w = floor_w * (steer > 0 ? 1 : -1) * (v > 0 ? 1 : -1);
        w = clampd(w, -p.wmax, p.wmax);
    }
};

inline std::string format_cmd(double v, double w) { return fmt("#D %.1f %.2f\n", v, w); }

// ---------------------------------------------------------------------------
// 鍵盤：每 5 ms 取樣一次目前按住的鍵，記下「按下」事件
//   Windows：GetAsyncKeyState（和 pynput 一樣是全域的，不論哪個視窗在前景都收得到）
//   其他系統：只用來測試——從環境變數 DRIVE_TEST_KEYS 讀腳本，例如 "0.5:w 2.0:-w 3.0:q"
// ---------------------------------------------------------------------------
class Keyboard {
public:
    std::set<std::string> down;                 // 目前按住的 'w' 'x' 'a' 'd' 'up' 'down' 'left' 'right'
    std::vector<std::string> events;            // 單次事件：'stop' 'center' 'zero' 'heading' 'quit' 'tune:j'…

    Keyboard() {
#ifndef _WIN32
        if (const char* s = std::getenv("DRIVE_TEST_KEYS"))
            for (auto& tok : split_ws(s)) {
                size_t c = tok.find(':');
                if (c != std::string::npos) script_.push_back({std::atof(tok.substr(0, c).c_str()), tok.substr(c + 1)});
            }
        t0_ = now_s();
#endif
    }

    // 取樣一次；latch 用來記住兩次 tick 之間「曾經按下」的鍵（避免短按漏掉）
    void sample() {
#ifdef _WIN32
        struct K { int vk; const char* name; };
        static const K movers[] = {{'W', "w"}, {'X', "x"}, {'A', "a"}, {'D', "d"}, {VK_UP, "up"}, {VK_DOWN, "down"}, {VK_LEFT, "left"}, {VK_RIGHT, "right"}};
        static const K singles[] = {{'S', "stop"}, {VK_SPACE, "stop"}, {'C', "center"}, {'Z', "zero"}, {'Y', "heading"}, {'U', "nudge-"}, {'I', "nudge+"}, {'J', "tune:j"}, {'L', "tune:l"},
                                    {'G', "tune:g"}, {'H', "tune:h"}, {'Q', "quit"}, {VK_ESCAPE, "quit"}};
        down.clear();
        for (auto& k : movers) {
            if (GetAsyncKeyState(k.vk) & 0x8000) { down.insert(k.name); latch_.insert(k.name); }
        }
        for (auto& k : singles) {
            bool pressed = (GetAsyncKeyState(k.vk) & 0x8000) != 0;
            bool& was = prev_[k.vk];
            if (pressed && !was) events.push_back(k.name);
            was = pressed;
        }
#else
        double t = now_s() - t0_;
        while (next_ < script_.size() && script_[next_].first <= t) {
            const std::string& k = script_[next_].second;
            if (k == "w" || k == "x" || k == "a" || k == "d") { down.insert(k); latch_.insert(k); }
            else if (k.size() > 1 && k[0] == '-') down.erase(k.substr(1));
            else if (k == "s") events.push_back("stop");
            else if (k == "c") events.push_back("center");
            else if (k == "z") events.push_back("zero");
            else if (k == "y") events.push_back("heading");
            else if (k == "u") events.push_back("nudge-");
            else if (k == "i") events.push_back("nudge+");
            else if (k == "q") events.push_back("quit");
            else if (k.size() == 1) events.push_back("tune:" + k);
            next_++;
        }
#endif
    }

    // 兩次 tick 之間按住過的方向（包含已經放開、但曾經按下的短按）
    int thr_dir() const { return (held("w") || held("up") ? 1 : 0) - (held("x") || held("down") ? 1 : 0); }
    int steer_dir() const { return (held("d") || held("right") ? 1 : 0) - (held("a") || held("left") ? 1 : 0); }
    void end_tick() { latch_.clear(); }
    std::vector<std::string> pop_events() { std::vector<std::string> e; e.swap(events); return e; }

private:
    bool held(const char* n) const { return down.count(n) || latch_.count(n); }
    std::set<std::string> latch_;
#ifdef _WIN32
    std::map<int, bool> prev_;
#else
    std::vector<std::pair<double, std::string>> script_;
    size_t next_ = 0;
    double t0_ = 0;
#endif
};

// 一般狀態列：油門 方向 -> v w | 車子回報
inline std::string status_line(const DriveModel& m, double v, double w, const Fields* tel, const std::string& state) {
    std::string s = fmt("油門 %+.2f  方向 %+.2f -> v %+.1f  w %+.2f rad/s", m.thr, m.steer, v, w);
    if (tel) {
        auto get = [&](const char* k) { auto it = tel->find(k); return it == tel->end() ? std::string("?") : it->second; };
        s += "  | " + (state.empty() ? std::string("?") : state) + " ANG " + get("ANG") + " V " + get("V") + " PX " + get("PX") + " PY " + get("PY");
    }
    return s;
}

// 把字串補空白到至少 n 個「顯示寬度」（中文算 2 格），讓 \r 重繪時蓋掉上一行殘留
inline std::string pad_to(const std::string& s, size_t n) {
    size_t w = 0;
    for (unsigned char c : s) {
        if ((c & 0xC0) == 0x80) continue;
        w += c >= 0xE0 ? 2 : 1;
    }
    return w >= n ? s : s + std::string(n - w, ' ');
}

static void usage() {
    std::puts(
        "用法：drive [選項]\n"
        "  --port COM5        序列埠（預設 COM3）\n"
        "  --baud N           鮑率（預設 9600）\n"
        "  --vmax V           前進最高目標速度（預設 18，上限 20）\n"
        "  --vback V          後退最高目標速度（預設 18，上限 20）\n"
        "  --vmin V           前進最低目標速度（預設 17）\n"
        "  --vmin-back V      後退最低目標速度（預設 17）\n"
        "  --thr-up V         按住油門的加速斜率（單位/秒，預設 2.0）\n"
        "  --min-radius CM    方向打到底的轉彎半徑 cm（預設 30）\n"
        "  --wmax W           邊走邊轉的角速度上限 rad/s（預設 0.5，上限 0.6）\n"
        "  --hz N             送指令頻率（預設 10）\n"
        "  --dry-run          不連線，只印出會送出的指令");
}

int main(int argc, char** argv) {
    init_console();
    Params p;
    std::string port = COM_PORT;
    int baud = BAUD_RATE;
    bool dry = false, thr_up_set = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s 後面少了數值\n", a.c_str()); std::exit(2); }
            return argv[++i];
        };
        auto num = [&]() -> double {
            std::string s = next();
            try { return to_double(s); } catch (...) { std::fprintf(stderr, "%s 的數值看不懂：%s\n", a.c_str(), s.c_str()); std::exit(2); }
        };
        if (a == "--port") port = next();
        else if (a == "--baud") baud = (int)num();
        else if (a == "--vmax") p.vmax = num();
        else if (a == "--vback") p.vback = num();
        else if (a == "--vmin") p.vmin = num();
        else if (a == "--vmin-back") p.vmin_back = num();
        else if (a == "--thr-up") { p.thr_up = num(); thr_up_set = true; }
        else if (a == "--min-radius") p.min_radius = num();
        else if (a == "--wmax") p.wmax = num();
        else if (a == "--hz") p.hz = num();
        else if (a == "--dry-run") dry = true;
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else { std::fprintf(stderr, "不認得的選項：%s（--help 看說明）\n", a.c_str()); return 2; }
    }
    (void)thr_up_set;

    if (!(p.vmax >= 0.5 && p.vmax <= 20.0) || !(p.vback >= 0.5 && p.vback <= 20.0) || !(p.wmax >= 0.1 && p.wmax <= 0.6)) {
        std::fputs("數值超出範圍：--vmax 0.5～20、--vback 0.5～20、--wmax 0.1～0.6（韌體也會限制）\n", stderr);
        return 2;
    }
    if (p.min_radius < 10) { std::fputs("--min-radius 至少 10 cm\n", stderr); return 2; }
    if (p.vmin > p.vmax || p.vmin_back > p.vback) {
        std::fputs("最低速度不能高於最高速度（--vmin <= --vmax，--vmin-back <= --vback）\n", stderr);
        return 2;
    }
    if (p.hz < 1 || p.hz > 50) { std::fputs("--hz 要在 1～50\n", stderr); return 2; }

    DriveModel model(p);
    Keyboard kb;

    SerialLink* link = nullptr;
    TelemetryLogger* logger = nullptr;
    Fields tel;
    std::string tel_state;
    bool have_tel = false;
    double t_start = now_s();
    std::string active_cmd = "IDLE";

    if (!dry) {
        try {
            link = new SerialLink(port, baud);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "無法開啟序列埠 %s：%s\n", port.c_str(), e.what());
            return 1;
        }
        logger = new TelemetryLogger();
        std::printf(">> 記錄檔：%s\n", logger->path.c_str());
    }

    std::puts("==================================================================");
    std::puts(" 遙控模式：W/↑ 前進  X/↓ 後退  A/← 左轉  D/→ 右轉  S/空白 停  C 回中  Z 座標歸零  Y 設定航向  U/I 微調航向  Q/Esc 結束");
    std::puts(" 車子站穩（遙測顯示【常態雙閉環】）後再開始；倒地時指令會被忽略");
    std::puts("==================================================================");
    std::fflush(stdout);

    std::signal(SIGINT, on_sigint);
    const double dt = 1.0 / p.hz;
    double nxt = now_s();

    // 讀序列埠：遙測 → 更新狀態列；NAV: / >> 開頭的訊息直接印出；全部寫進 CSV
    auto pump = [&]() {
        if (!link) return;
        std::string line;
        while (link->read_line(line)) {
            std::string st;
            Fields f;
            if (parse_telemetry(line, st, f)) {
                tel = f;
                tel_state = st;
                have_tel = true;
            } else if (starts_with(line, "NAV:") || starts_with(line, ">>")) {
                std::printf("\n%s\n", line.c_str());
            }
            logger->log(now_s() - t_start, active_cmd, line);
        }
    };

    int exit_code = 0;
    try {
        for (;;) {
            check_interrupt();
            for (auto& ev : kb.pop_events()) {
                if (ev == "quit") throw Interrupted{};
                if (ev == "stop") model.stop();
                else if (ev == "center") model.center();
                else if (ev == "zero" && link) link->send("z");
                else if (ev == "nudge-" && link) link->send("u");   // 設定角度 -0.25°
                else if (ev == "nudge+" && link) link->send("i");   // 設定角度 +0.25°
                else if (ev == "heading" && link) link->send("y");   // 韌體回 ">> 航向設定 xx.x"，pump() 會印出來
                else if (starts_with(ev, "tune:") && link) link->send(ev.substr(5));
            }
            model.update_keys(dt, kb.thr_dir(), kb.steer_dir());
            kb.end_tick();

            double v, w;
            model.output(v, w);
            std::string cmd = format_cmd(v, w);
            if (logger) { logger->set_drive(model.thr, model.steer, v, w); logger->set_cmd(strip(cmd)); }   // [v37g] 輸入訊號寫進 CSV
            if (link) {
                link->send(cmd);
                active_cmd = (v != 0.0 || w != 0.0) ? fmt("D%+.1f/%+.2f", v, w) : "IDLE";
            }
            if (dry) {
                std::printf("%s    %s\n", strip(cmd).c_str(), status_line(model, v, w, nullptr, "").c_str());
            } else {
                std::printf("\r%s", pad_to(status_line(model, v, w, have_tel ? &tel : nullptr, tel_state), 110).c_str());
            }
            std::fflush(stdout);

            // 睡到下一個 tick；睡的期間每 5 ms 取樣鍵盤、讀序列埠
            nxt += dt;
            for (;;) {
                kb.sample();
                pump();
                if (g_interrupted) break;
                double left = nxt - now_s();
                if (left <= 0) break;
                sleep_s(std::min(left, 0.005));
            }
            if (nxt < now_s() - 1.0) nxt = now_s();                      // 被卡住太久就重新對時，不要連續補送
        }
    } catch (const Interrupted&) {
    } catch (const std::exception& e) {
        std::fprintf(stderr, "\n>> 發生錯誤：%s\n", e.what());
        exit_code = 1;
    }

    std::puts("\n>> 停車並結束...");
    if (link) {
        try {
            for (int i = 0; i < 3; i++) { link->send("#D 0 0\n"); sleep_s(0.05); }
            link->send("s");
        } catch (...) {}
        sleep_s(0.2);
        pump();
        link->close();
        logger->close();
        std::printf(">> 記錄檔：%s\n", logger->path.c_str());
        delete link;
        delete logger;
    }
    return exit_code;
}
