// ============================================================================
// nav.cpp — 座標導航執行器 + 選單（對應 Python 的 nav.py 與 run_nav.py，搭配韌體 nav_v27 以上）
//
// 編譯：  g++ -std=c++17 -O2 nav.cpp -o nav.exe
//
// 用法
//   nav.exe                                       沒有參數 = 選單（選形狀 / 輸入座標 → 預覽 / 模擬 / 實車），VS Code 按 F5 就是這個
//   nav.exe "0,0 3,0 3,3 0,3 0,0"                 依序走到這些座標（預設座標 1 = 10 cm）
//   nav.exe routes\square.txt                     路線寫在檔案裡（每行一個點：x,y；# 開頭是註解）
//   nav.exe "0,0 1,1" --scale 5                   座標 1 = 5 cm
//   nav.exe "0,0 3,0 3,3" --dry-run               只印出每一段要轉幾度、走幾公分，不連線
//   nav.exe "0,0 3,0 3,3" --sim                   用電腦模擬的車跑完整流程（不需要實車）
//   nav.exe ... --port COM5                       指定序列埠（預設 COM3）
//   nav.exe ... --no-zero                         不先把座標歸零（沿用車上目前的座標）
//   nav.exe "T90 T180 T-90 T0" --steps            單一動作測試：依序轉到 90°、180°、-90°、0°
//   nav.exe "M20 M-20" --steps                    單一動作測試：往前 20 cm 再退回（M20@45 = 沿 45° 走 20 cm）
//   nav.exe "0,0 5,3 8,-2 2,4" --smooth           平滑路線：點與點之間走圓弧、不停車（--dry-run 先看每段半徑；--speed 7 加快）
//   nav.exe x --circle 15                         邊走邊轉繞圓：半徑 15 cm、向左一圈（--right 向右、--laps 2 兩圈）
//   nav.exe x --circle 15 --laps 0                一直繞，按 Enter 才沿著圓弧慢慢停下（--stop-after 20 = 20 秒後自動停）
//   nav.exe "C47@-15" --steps                     底層指令：C<弧長cm>@<半徑cm>，半徑負 = 左轉、正 = 右轉
//
// 座標系（與韌體虛擬座標相同）：開始時先送 z，把「目前位置」設為原點、「目前車頭方向」設為 X 軸；
//   X = 往前為正，Y = 往右為正；角度 0° = 起點車頭方向，往右轉為正。
//
// 每一段的流程：用目前位置算出目標方向與距離 → 送 #T 原地轉向 → 用實際位置重算 → 送 #M 直走；
//   每一段都用「實際位置」重算，上一段停歪或自轉時漂移，下一段會自動修正。
// 安全：執行中按 Ctrl+C 立即中止（送 #S 和 s）；韌體端倒地、緊急救援、超過 1 s 沒收到電腦訊號都會自動中止。
// ============================================================================
#include <algorithm>
#include <cerrno>
#include <iostream>
#include <optional>

#ifdef _WIN32
#include <conio.h>
#endif

#include "link.h"
#include "sim.h"

using namespace rc;

// =============================================================================
// 1. 設定
// =============================================================================
constexpr double DEFAULT_SCALE_CM = 10.0;     // 座標 1 = 幾公分
constexpr double SKIP_DIST_CM = 1.0;          // 離目標小於這個距離就不走
constexpr double TURN_TIMEOUT_S = 12.0;
constexpr double MOVE_TIMEOUT_S = 20.0;       // 直走等待回應的下限；實際用 move_timeout(距離)
constexpr double HEARTBEAT_S = 0.2;           // 每 0.2 s 送 '.'（韌體超過 1 s 沒收到會中止導航）
constexpr double SETTLE_AFTER_DONE_S = 0.4;   // 每個動作完成後等一下再送下一個
constexpr double RETRY_CM = 3.0;              // 到達後誤差超過這個值就再修正一次
constexpr int RETRIES = 1;                    // 每個點最多修正幾次
constexpr double MAX_TURN_STEP_DEG = 30.0;    // 大於這個角度的轉彎拆成多次（[v37f] 50 → 30，每一小步之後等車子靜止並檢查位置被帶偏多少）
constexpr double TURN_DRIFT_ABORT_CM = 10.0;  // 一次轉彎位置被帶偏超過這個距離就中止
constexpr double SMOOTH_MIN_R_CM = 20.0;      // 平滑路線：弧線半徑小於這個值就改用「原地轉向 + 直走」
constexpr double SMOOTH_SWITCH_CM = 5.0;      // 平滑路線：這一段剩下多少公分時，就接上下一段（[v37g] 8 → 5：太早接下一段，每個彎會少轉、下一個彎再多轉補回來）
constexpr double STRAIGHT_R_CM = 20000.0;     // 幾乎直線時用的半徑
constexpr double FOREVER_ARC_CM = 1000.0;     // 「一直繞到按 Enter」用的弧長
constexpr double MAX_ARC_CM = 1000.0;         // 單一個弧線指令的弧長上限
constexpr double MAX_LEG_CM = 150.0;          // 單段直走超過這個距離就中止
constexpr double SETTLE_V = 4.0;              // 轉完後要等 |V| 小於這個值才送下一個指令
constexpr double REALIGN_DEG = 3.0;           // 轉完後重算的方向和車頭差超過這個角度，就再轉一次（[v37f] 10 → 3：直走前車頭要更準）

using Pose = std::pair<double, double>;

static std::string g(double x) { return fmt("%g", x); }

// 依「字元數」（不是位元組、也不是顯示寬度）補空白，和 Python 的 str.ljust 一樣
static std::string ljust(const std::string& s, size_t n) {
    size_t chars = 0;
    for (unsigned char c : s)
        if ((c & 0xC0) != 0x80) chars++;
    return chars >= n ? s : s + std::string(n - chars, ' ');
}

// =============================================================================
// 2. 路線
// =============================================================================
struct Pt {
    double x = 0, y = 0;   // 座標單位（還沒乘 scale）；turn_only 時 y = 目標角度
    bool back = false;     // 座標後面加 b = 這一段用倒車過去（不先轉頭）
    bool turn_only = false;// h<角度> = 在目前位置轉到指定的絕對方向（0 = 起點車頭，右轉為正）
};

static std::vector<Pt> parse_route(const std::string& text_or_file) {
    std::vector<std::string> items;
    bool is_file = false;
    try {
        std::error_code ec;
        is_file = fs::is_regular_file(fs::u8path(text_or_file), ec);
    } catch (...) {}
    if (is_file) {
        std::ifstream in(fs::u8path(text_or_file), std::ios::binary);
        std::string line;
        bool first = true;
        while (std::getline(in, line)) {
            if (first && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);   // 去掉 BOM
            first = false;
            size_t h = line.find('#');
            if (h != std::string::npos) line.erase(h);
            line = strip(line);
            if (!line.empty()) items.push_back(line);
        }
    } else {
        items = split_ws(replace_all(text_or_file, ";", " "));
    }
    std::vector<Pt> pts;
    for (std::string it : items) {
        it = strip(it);
        while (!it.empty() && (it.front() == '(' || it.front() == ')')) it.erase(0, 1);
        while (!it.empty() && (it.back() == '(' || it.back() == ')')) it.pop_back();
        Pt p;
        if (it.size() > 1 && (it[0] == 'h' || it[0] == 'H')) {
            p.turn_only = true;
            p.y = to_double(it.substr(1));
            pts.push_back(p);
            continue;
        }
        if (!it.empty() && (it.back() == 'b' || it.back() == 'B')) {
            p.back = true;
            it.pop_back();
        }
        auto parts = split_ws(replace_all(it, ",", " "));
        if (parts.size() != 2) throw NavError("看不懂的座標：'" + it + "'（格式：x,y 或 x,yb）");
        p.x = to_double(parts[0]);
        p.y = to_double(parts[1]);
        pts.push_back(p);
    }
    if (pts.empty()) throw NavError("路線是空的");
    return pts;
}

// 目前位置 → 目標：回傳 (方向°, 距離 cm)；方向 0° = X 軸，往右（+Y）為正
static std::pair<double, double> plan(Pose pose, Pose target) {
    double dx = target.first - pose.first, dy = target.second - pose.second;
    return {std::atan2(dy, dx) / DEG, std::hypot(dx, dy)};
}

static void dry_run(const std::vector<Pt>& route, double scale) {
    std::printf("座標 1 = %s cm；起點 = 第一個點（實際執行時會先把目前位置歸零）\n", g(scale).c_str());
    Pose pose = route[0].turn_only ? Pose{0, 0} : Pose{route[0].x * scale, route[0].y * scale};
    double heading = 0.0, total = 0.0;
    std::puts(" 段  從 → 到                  轉到(°)  需要轉(°)  直走(cm)");
    for (size_t i = 1; i < route.size(); i++) {
        const Pt& p = route[i];
        if (p.turn_only) {
            double turn = wrap_deg(p.y - heading);
            std::string head = fmt(" %2zu  在 (%s,%s) 轉到 %s°", i, g(pose.first / scale).c_str(), g(pose.second / scale).c_str(), g(p.y).c_str());
            std::printf("%s%8.1f %9.1f\n", ljust(head, 30).c_str(), p.y, turn);
            heading = p.y;
            continue;
        }
        Pose tgt{p.x * scale, p.y * scale};
        auto [b, d] = plan(pose, tgt);
        if (d < SKIP_DIST_CM) {
            std::printf(" %2zu  已在目標上，略過\n", i);
            continue;
        }
        double hd = p.back ? wrap_deg(b + 180) : b;                      // 倒車：車頭朝反方向
        double turn = wrap_deg(hd - heading);
        std::string head = fmt(" %2zu  (%s,%s) → (%s,%s)", i, g(pose.first / scale).c_str(), g(pose.second / scale).c_str(), g(p.x).c_str(), g(p.y).c_str());
        std::printf("%s%8.1f %9.1f %10.1f%s\n", ljust(head, 30).c_str(), hd, turn, p.back ? -d : d, p.back ? "  倒車" : "");
        pose = tgt;
        heading = hd;
        total += d;
    }
    std::printf("總路徑 %.1f cm\n", total);
}

// =============================================================================
// 3. 導航器（讀序列埠、送心跳、等回應）——單執行緒：所有等待迴圈都呼叫 pump()
// =============================================================================
class Navigator {
public:
    Link* link;
    TelemetryLogger* logger;
    double k;                          // 模擬倍速：等待時間要跟著縮短
    std::deque<std::string> events;    // NAV: 開頭的訊息
    bool have_pose = false;
    Pose pose{0, 0};                   // (PX, PY) cm
    double heading = 0.0;              // 車頭方向（°，相對原點），由 NAV:DONE 的 H: 更新
    std::string state;
    double v = 0.0;                    // 最近一筆遙測的實際速度 V
    bool have_yaw = false;
    double yaw = 0.0;                  // 最近一筆遙測的絕對車頭方向 Y (rad)
    double yaw0 = 0.0;                 // 座標歸零當下的 Y；車頭相對方向 = Y - yaw0
    int nv = 0;                        // 導航狀態（0 閒置、1 轉向、2 直走、3 停車）
    double nr = 0.0;                   // 剩餘量（直走：公分）
    long tel_count = 0;

    Navigator(Link* l, TelemetryLogger* lg, double time_scale = 1.0) : link(l), logger(lg), k(time_scale), t0_(now_s()) {}

    double elapsed() const { return std::round((now_s() - t0_) * k * 1000.0) / 1000.0; }   // 模擬時換算回模擬時間

    // 讀序列埠、處理訊息、必要時送心跳；Ctrl+C 時丟出 Interrupted
    void pump() {
        check_interrupt();
        std::string line;
        while (link->read_line(line)) handle(line);
        double t = now_s();
        if (t >= next_hb_) {
            link->send(".");
            next_hb_ = t + HEARTBEAT_S / k;
        }
    }

    // 睡 sec 秒（模擬時依倍速縮短），睡的期間照常讀序列埠、送心跳
    void sleep(double sec) {
        double end = now_s() + sec / k;
        do {
            pump();
            sleep_s(std::min(0.002, std::max(0.0, end - now_s())));
        } while (now_s() < end);
    }

    Pose wait_telemetry(int n = 1, double timeout = 3.0) {
        long target = tel_count + n;
        double end = now_s() + timeout / k;
        while (now_s() < end) {
            pump();
            if (tel_count >= target) {
                if (!have_pose) throw NavError("遙測裡沒有 PX / PY（韌體版本太舊？需要 quad_v26 以上）");
                return pose;
            }
            sleep_s(0.002);
        }
        throw NavError("收不到遙測（檢查連線 / 序列埠）");
    }

    std::optional<std::string> wait_event(double timeout) {
        double end = now_s() + timeout / k;
        while (now_s() < end) {
            pump();
            while (!events.empty()) {
                std::string ev = events.front();
                events.pop_front();
                if (starts_with(ev, "NAV:START")) continue;
                return ev;
            }
            sleep_s(0.002);
        }
        return std::nullopt;
    }

    // [v37g] 送指令並記到 CSV 的 IN_CMD 欄
    void send_cmd(const std::string& text) {
        if (logger) logger->set_cmd(text);
        link->send(text);
    }

    Pose command(const std::string& text, double timeout) {
        send_cmd("#" + text + "\n");
        auto ev = wait_event(timeout);
        if (!ev) {
            link->send("#S\n");
            throw NavError(fmt("#%s 等不到回應（%.0f s）", text.c_str(), timeout));
        }
        if (!starts_with(*ev, "NAV:DONE")) throw NavError(fmt("#%s 失敗：%s", text.c_str(), ev->c_str()));
        read_heading(*ev);
        sleep(SETTLE_AFTER_DONE_S);
        return wait_telemetry(1);
    }

    // 從 "NAV:DONE ... H:12.3" 讀出車頭方向
    void read_heading(const std::string& ev) {
        for (auto& tok : split_ws(ev))
            if (starts_with(tok, "H:")) heading = std::atof(tok.c_str() + 2);
    }

    // 車頭相對座標原點的方向（°，右轉為正）；由遙測的 Y 算，連續弧線行進中也看得到
    double rel_heading() const { return wrap_deg(((have_yaw ? yaw : yaw0) - yaw0) / DEG); }

    // 把電腦端的資訊（例如目標點）寫進記錄檔，analyze.py 會用來算誤差
    void note(const std::string& text) {
        if (logger) logger->log(elapsed(), "NAV", text);
    }

    void abort() {
        try {
            link->send("#S\n");
            link->send("s");
        } catch (...) {}
    }

private:
    double t0_;
    double next_hb_ = 0.0;

    void handle(const std::string& line) {
        double el = elapsed();
        if (logger) logger->log(el, "NAV", line);
        if (starts_with(line, "NAV:")) {
            std::printf("  [%6.1fs] %s\n", el, line.c_str());
            events.push_back(line);
            return;
        }
        std::string st;
        Fields f;
        if (!parse_telemetry(line, st, f)) {
            if (!strip(line).empty()) std::printf("  [%6.1fs] %s\n", el, line.c_str());
            return;
        }
        state = st;
        auto num = [&](const char* key, double& out) {
            auto it = f.find(key);
            if (it == f.end() || it->second.empty()) return false;
            out = std::atof(it->second.c_str());
            return true;
        };
        double px, py;
        if (num("PX", px) && num("PY", py)) {
            pose = {px, py};
            have_pose = true;
        }
        num("V", v);
        if (num("Y", yaw)) have_yaw = true;
        nv = 0;
        nr = 0.0;
        double tmp;
        if (num("NV", tmp)) nv = (int)tmp;
        num("NR", nr);
        tel_count++;
    }
};

static double move_timeout(double cm) { return std::max(MOVE_TIMEOUT_S, 6.0 + 0.45 * std::fabs(cm)); }

// 等速度小於 SETTLE_V 連續 need 筆遙測（轉彎剛結束時車子還在晃，這時送指令容易失控）
static bool wait_still(Navigator& nav, double timeout = 4.0, int need = 3) {
    int ok = 0;
    double end = now_s() + timeout / nav.k;
    while (now_s() < end && ok < need) {
        nav.wait_telemetry(1, 2);
        ok = std::fabs(nav.v) < SETTLE_V ? ok + 1 : 0;
    }
    return ok >= need;
}

// 轉到 target_deg：大角度拆成 ≤MAX_TURN_STEP_DEG 的小步，每步之後等車子靜止；位置被帶偏太多就中止
static Pose safe_turn(Navigator& nav, double target_deg) {
    Pose pose = nav.wait_telemetry(1);
    for (int n = 0; n < 8; n++) {
        double diff = wrap_deg(target_deg - nav.heading);
        double step = std::fabs(diff) > MAX_TURN_STEP_DEG ? nav.heading + std::copysign(std::min(std::fabs(diff), MAX_TURN_STEP_DEG * 0.9), diff) : target_deg;
        Pose before = nav.wait_telemetry(1);
        pose = nav.command(fmt("T %.1f", wrap_deg(step)), TURN_TIMEOUT_S);
        wait_still(nav);
        pose = nav.wait_telemetry(1);
        double drift = std::hypot(pose.first - before.first, pose.second - before.second);
        if (drift > TURN_DRIFT_ABORT_CM)
            throw NavError(fmt("轉彎時位置被帶偏 %.1f cm（超過 %s），車子可能失控，已中止", drift, g(TURN_DRIFT_ABORT_CM).c_str()));
        if (step == target_deg) break;
    }
    return pose;
}

static void wait_standing(Navigator& nav) {
    std::puts("等待車子站穩…");
    double end = now_s() + 15 / nav.k;
    int stable = 0;
    while (now_s() < end && stable < 5) {
        nav.wait_telemetry(1, 5);
        stable = nav.state == "NORMAL" ? stable + 1 : 0;
    }
    if (stable < 5) throw NavError("車子沒有在站立狀態（遙測不是【常態雙閉環】）");
}

static void zero_pose(Navigator& nav) {
    nav.heading = 0.0;
    nav.send_cmd("z");
    nav.sleep(0.5);
    nav.wait_telemetry(2);
    if (nav.have_yaw) nav.yaw0 = nav.yaw;
    std::puts(">> 座標歸零：目前位置 = (0, 0)，目前車頭 = X 軸");
}

// =============================================================================
// 4. 單一動作（--steps）
// =============================================================================
struct Step {
    char kind;          // 'T' 轉向 / 'M' 直走 / 'C' 圓弧
    double v;           // T：角度；M：公分；C：弧長 cm
    bool has_h = false;
    double h = 0;       // M：方向；C：半徑 cm（正 = 右轉、負 = 左轉）
    bool has_tv = false;
    double tv = 0;      // C：前進目標速度
};

// "T90 T180 M20 M-20@0 C94@-15" → steps
static std::vector<Step> parse_steps(const std::string& text) {
    std::vector<Step> steps;
    for (auto& tok : split_ws(replace_all(replace_all(text, ",", " "), ";", " "))) {
        char kind = (char)std::toupper((unsigned char)tok[0]);
        std::string rest = tok.substr(1);
        if (!(kind == 'T' || kind == 'M' || kind == 'C') || rest.empty() || (kind == 'C' && rest.find('@') == std::string::npos))
            throw NavError("看不懂的動作：" + tok + "（格式：T<角度>、M<公分>、M<公分>@<方向> 或 C<弧長>@<半徑>）");
        Step s{kind, 0};
        if (kind == 'C') {
            size_t at = rest.find('@');
            std::string d = rest.substr(0, at), rad = rest.substr(at + 1), tvs = "0";
            size_t at2 = rad.find('@');
            if (at2 != std::string::npos) { tvs = rad.substr(at2 + 1); rad = rad.substr(0, at2); }
            double dv = to_double(d), rv = to_double(rad), tv = to_double(tvs);
            if (dv <= 0 || rv == 0 || dv > MAX_ARC_CM || !(tv >= 0 && tv <= NAV.TV_ARC_CAP))
                throw NavError(fmt("%s：弧長要 > 0 且 ≤ %s cm，半徑不能是 0，速度要在 0（預設）~ %s（想一直繞、按 Enter 才停，請用 --laps 0）",
                                   tok.c_str(), g(MAX_ARC_CM).c_str(), g(NAV.TV_ARC_CAP).c_str()));
            s.v = dv;
            s.has_h = true;
            s.h = rv;
            if (tv != 0) { s.has_tv = true; s.tv = tv; }
        } else if (kind == 'M' && rest.find('@') != std::string::npos) {
            size_t at = rest.find('@');
            s.v = to_double(rest.substr(0, at));
            s.has_h = true;
            s.h = to_double(rest.substr(at + 1));
        } else {
            s.v = to_double(rest);
        }
        steps.push_back(s);
    }
    return steps;
}

static void print_steps(const std::vector<Step>& steps) {
    for (auto& s : steps)
        std::printf("  #%c %s%s%s\n", s.kind, g(s.v).c_str(), s.has_h ? (" " + g(s.h)).c_str() : "", s.has_tv ? (" " + g(s.tv)).c_str() : "");
}

static void run_steps(Navigator& nav, const std::vector<Step>& steps, bool zero = true) {
    wait_standing(nav);
    if (zero) zero_pose(nav);
    for (auto& s : steps) {
        std::string cmd;
        if (s.kind == 'T') cmd = fmt("T %.1f", s.v);
        else if (s.kind == 'C') cmd = fmt("C %.1f %.1f", s.v, s.h) + (s.has_tv ? " " + g(s.tv) : "");
        else cmd = fmt("M %.1f", s.v) + (s.has_h ? fmt(" %.1f", s.h) : "");
        Pose before = nav.wait_telemetry(1);
        double h0 = nav.heading;
        double t0 = now_s();
        Pose pose = nav.command(cmd, s.kind == 'T' ? TURN_TIMEOUT_S : move_timeout(s.v));   // C 的弧長也用同一個逾時公式
        double dt = (now_s() - t0) * nav.k;
        double moved = std::hypot(pose.first - before.first, pose.second - before.second);
        if (s.kind == 'T') {
            double err = wrap_deg(s.v - nav.heading);
            std::printf("  #%s：%.1f s，航向 %.1f° → %.1f°，誤差 %+.1f°，位置被帶偏 %.1f cm\n", cmd.c_str(), dt, h0, nav.heading, err, moved);
        } else if (s.kind == 'C') {
            std::printf("  #%s：%.1f s，位置 (%.1f, %.1f) → (%.1f, %.1f)，起終點相距 %.1f cm，航向 %.1f° → %.1f°（弧長 %.1f、半徑 %s、應轉 %+.0f°）\n",
                        cmd.c_str(), dt, before.first, before.second, pose.first, pose.second, moved, h0, nav.heading, s.v, g(std::fabs(s.h)).c_str(),
                        s.v / s.h / DEG);
        } else {
            std::printf("  #%s：%.1f s，位置 (%.1f, %.1f) → (%.1f, %.1f)，走了 %.1f cm（要求 %.1f），航向 %.1f°\n", cmd.c_str(), dt, before.first,
                        before.second, pose.first, pose.second, moved, std::fabs(s.v), nav.heading);
        }
    }
    std::puts(">> 全部動作完成。實際位置 / 角度請用捲尺、量角器確認，和上面的數字比較。");
}

// Windows 主控台：有按任何鍵（Enter / 空白鍵…）就回傳 true；其他系統回傳 false（這時請改用 --stop-after）
static bool stop_key_pressed() {
#ifdef _WIN32
    bool hit = false;
    while (_kbhit()) {
        _getwch();
        hit = true;
    }
    return hit;
#else
    return false;
#endif
}

// 一直繞圓，直到你按 Enter（或 --stop-after 秒數到）：送 #F，車子沿著圓弧慢慢減速停下
static void run_arc_until_stop(Navigator& nav, double radius_cm, double speed, std::optional<double> stop_after, bool zero) {
    wait_standing(nav);
    if (zero) zero_pose(nav);
    std::string cmd = fmt("C %s %.1f", g(FOREVER_ARC_CM).c_str(), radius_cm) + (speed != 0 ? " " + g(speed) : "");
    nav.wait_telemetry(1);
    double t0 = now_s();
    nav.send_cmd("#" + cmd + "\n");
    stop_key_pressed();                                                  // 清掉之前按過的鍵
    std::puts(">> 繞圈中……按 Enter（或空白鍵）：沿著圓弧慢慢停下；Ctrl+C：緊急中止");
    std::optional<double> stopping_at;
    std::optional<std::string> ev;
    for (;;) {
        ev = nav.wait_event(0.2);
        if (ev) break;
        double el = (now_s() - t0) * nav.k;
        if (!stopping_at && (stop_key_pressed() || (stop_after && el >= *stop_after))) {
            nav.send_cmd("#F\n");
            stopping_at = el;
            std::printf("  [%6.1fs] >> 收到停止，沿著圓弧減速中……\n", el);
        }
        if (stopping_at && el - *stopping_at > 40) throw NavError("按停止後 40 秒還沒停下來");
        if (el > move_timeout(FOREVER_ARC_CM)) throw NavError("繞圓等不到回應");
    }
    if (!starts_with(*ev, "NAV:DONE")) throw NavError("#" + cmd + " 失敗：" + *ev);
    nav.read_heading(*ev);
    Pose pose = nav.wait_telemetry(1);
    double el = (now_s() - t0) * nav.k;
    std::printf("  #%s：共 %.1f s，停在 (%.1f, %.1f) cm，車頭 %.1f°\n", cmd.c_str(), el, pose.first, pose.second, nav.heading);
}

// =============================================================================
// 5. 平滑路線（--smooth）
// =============================================================================
struct ArcPlan {
    enum Kind { SKIP, ARC, NONE } kind;
    double length = 0, radius = 0, angle = 0;
};

// 從目前位置、車頭方向出發，用「和車頭相切的一段圓弧」走到目標點。
// 半徑正 = 右轉、負 = 左轉；NONE = 目標在背後，或需要的半徑小於 rmin → 改用原地轉向 + 直走
static ArcPlan plan_arc(Pose pose, double heading_deg, Pose target, double rmin) {
    double dx = target.first - pose.first, dy = target.second - pose.second;
    double d = std::hypot(dx, dy);
    if (d < SKIP_DIST_CM) return {ArcPlan::SKIP};
    double th = heading_deg * DEG;
    double fwd = dx * std::cos(th) + dy * std::sin(th);                  // 目標在車頭前方多遠
    double lat = -dx * std::sin(th) + dy * std::cos(th);                 // 目標在車頭右邊多遠（左邊為負）
    if (fwd <= 0) return {ArcPlan::NONE};
    if (std::fabs(lat) < 0.3) return {ArcPlan::ARC, d, STRAIGHT_R_CM, 0.0};   // 幾乎正前方：走直線
    double r = d * d / (2 * std::fabs(lat));
    if (r < rmin) return {ArcPlan::NONE};
    double phi = 2 * std::atan2(std::fabs(lat), fwd);                    // 弧角（rad）
    return {ArcPlan::ARC, r * phi, std::copysign(r, lat), std::copysign(phi / DEG, lat)};
}

static void smooth_dry_run(const std::vector<Pt>& route, double scale, double rmin) {
    Pose pose{0, 0};
    double heading = 0.0, total = 0.0;
    std::printf("座標 1 = %s cm；起點 = (0, 0)、車頭朝 X 軸；最小弧線半徑 %s cm（更小的轉彎改成原地轉 + 直走）\n", g(scale).c_str(), g(rmin).c_str());
    std::puts(" 段  目標點                方式      半徑(cm)  轉角(°)  長度(cm)");
    for (size_t i = 1; i < route.size(); i++) {
        const Pt& p = route[i];
        Pose tgt{p.x * scale, p.y * scale};
        ArcPlan pl = plan_arc(pose, heading, tgt, rmin);
        std::string head = fmt(" %2zu  (%s, %s)", i, g(p.x).c_str(), g(p.y).c_str());
        if (pl.kind == ArcPlan::SKIP) {
            std::printf("%s  已在目標上，略過\n", head.c_str());
            continue;
        }
        if (pl.kind == ArcPlan::NONE) {
            auto [b, d] = plan(pose, tgt);
            std::printf("%s%s%8.1f\n", ljust(head, 30).c_str(), ljust(fmt("轉向+直走  朝 %.0f° 轉 %+.0f°", b, wrap_deg(b - heading)), 22).c_str(), d);
            heading = b;
            total += d;
        } else {
            bool straight = std::fabs(pl.radius) >= STRAIGHT_R_CM;
            std::string side = straight ? "直線" : (pl.radius > 0 ? "右弧" : "左弧");
            std::string mid = straight ? fmt("%9s  %+7d  ", "-", 0) : fmt("%9.1f  %+7.0f  ", std::fabs(pl.radius), pl.angle);
            std::printf("%s%s%s%8.1f\n", ljust(head, 30).c_str(), ljust(side, 9).c_str(), mid.c_str(), pl.length);
            heading = wrap_deg(heading + pl.angle);
            total += pl.length;
        }
        pose = tgt;
    }
    std::printf("總路徑約 %.1f cm\n", total);
}

// 等目前這段弧線快到終點（剩餘 ≤ switch_cm）。回傳 true = 'near'，false = 'done'（這段已經完成並停下）
static bool wait_arc_near(Navigator& nav, double switch_cm, double timeout) {
    double end = now_s() + timeout / nav.k;
    long last = -1;
    while (now_s() < end) {
        nav.pump();
        while (!nav.events.empty()) {                                    // 先看有沒有結束 / 失敗的事件
            std::string ev = nav.events.front();
            nav.events.pop_front();
            if (starts_with(ev, "NAV:START")) continue;
            if (starts_with(ev, "NAV:DONE")) return false;
            throw NavError("弧線失敗：" + ev);
        }
        if (nav.tel_count != last) {
            last = nav.tel_count;
            if (nav.nv == 3 || (nav.nv == 2 && nav.nr <= switch_cm)) return true;
        }
        sleep_s(0.02);
    }
    throw NavError("等不到這段弧線的進度（逾時）");
}

// 送出 #C 之後：等韌體回 NAV:START，再等 2 筆新的遙測，避免拿到上一段的舊狀態
static void wait_arc_started(Navigator& nav, double timeout = 4.0) {
    double end = now_s() + timeout / nav.k;
    while (now_s() < end) {
        nav.pump();
        if (nav.events.empty()) { sleep_s(0.002); continue; }
        std::string ev = nav.events.front();
        nav.events.pop_front();
        if (starts_with(ev, "NAV:START")) {
            nav.wait_telemetry(2);
            return;
        }
        if (starts_with(ev, "NAV:REJECT") || starts_with(ev, "NAV:ABORT") || starts_with(ev, "NAV:TIMEOUT"))
            throw NavError("弧線指令被拒絕或失敗：" + ev);
    }
    throw NavError("送出弧線指令後，車子沒有回應（沒收到 NAV:START）");
}

struct Result {
    double x, y;
    Pose pose;
    double err;
};

static std::vector<Result> run_route(Navigator& nav, const std::vector<Pt>& route, double scale, bool zero = true);

static Pose run_smooth(Navigator& nav, const std::vector<Pt>& route, double scale, double rmin, double switch_cm, double speed, bool zero = true) {
    for (auto& p : route)
        if (p.turn_only || p.back) throw NavError("平滑路線（--smooth）不支援倒車（b）和原地轉向（h角度），請拿掉");
    wait_standing(nav);
    if (zero) zero_pose(nav);
    bool active = false;
    std::string tv = speed != 0 ? " " + g(speed) : "";
    for (size_t i = 0; i < route.size(); i++) {
        Pose tgt{route[i].x * scale, route[i].y * scale};
        bool last = (i == route.size() - 1);
        if (active && !wait_arc_near(nav, switch_cm, 40)) active = false;
        Pose pose = nav.wait_telemetry(1);
        double hd = nav.rel_heading();
        ArcPlan pl = plan_arc(pose, hd, tgt, rmin);
        if (pl.kind == ArcPlan::SKIP) {
            std::printf("點 %zu：(%s, %s) 已在目標上\n", i, g(route[i].x).c_str(), g(route[i].y).c_str());
            continue;
        }
        nav.note(fmt("NAV:PLAN %zu %.1f %.1f", i, tgt.first, tgt.second));
        if (nav.logger) nav.logger->set_goal(tgt.first, tgt.second);
        if (pl.kind == ArcPlan::NONE) {                                  // 目標在背後 / 弧線半徑太小 → 停下來，原地轉 + 直走
            if (active) {
                std::printf("點 %zu：轉彎太急，先停下來再原地轉向\n", i);
                if (wait_arc_near(nav, 0.0, 40)) nav.wait_event(20);
                active = false;
                wait_still(nav);
            }
            std::printf("點 %zu：前往 (%s, %s)，改用原地轉向 + 直走\n", i, g(route[i].x).c_str(), g(route[i].y).c_str());
            run_route(nav, {Pt{tgt.first, tgt.second, false, false}}, 1.0, false);
            continue;
        }
        std::printf("點 %zu：前往 (%s, %s) = (%.1f, %.1f) cm，目前 (%.1f, %.1f) 車頭 %.0f°，%s\n", i, g(route[i].x).c_str(), g(route[i].y).c_str(), tgt.first,
                    tgt.second, pose.first, pose.second, hd,
                    std::fabs(pl.radius) < STRAIGHT_R_CM ? fmt("走圓弧 半徑 %.0f cm、轉 %+.0f°、長 %.0f cm", std::fabs(pl.radius), pl.angle, pl.length).c_str()
                                                         : fmt("直線 %.0f cm", pl.length).c_str());
        if (pl.length > MAX_ARC_CM) throw NavError(fmt("這一段弧長 %.0f cm（超過 %s），位置估計可能壞了，已中止", pl.length, g(MAX_ARC_CM).c_str()));
        nav.send_cmd(fmt("#C %.1f %.1f", pl.length, pl.radius) + tv + "\n");
        wait_arc_started(nav);
        active = true;
        if (last) {
            auto ev = nav.wait_event(move_timeout(pl.length));
            if (!ev) {
                nav.link->send("#S\n");
                throw NavError("最後一段等不到完成（逾時）");
            }
            if (!starts_with(*ev, "NAV:DONE")) throw NavError("最後一段失敗：" + *ev);
            active = false;
        }
    }
    Pose pose = nav.wait_telemetry(1);
    std::printf(">> 路線走完：最後位置 (%.1f, %.1f) cm，車頭 %.1f°\n", pose.first, pose.second, nav.rel_heading());
    return pose;
}

// =============================================================================
// 6. 一般路線（原地轉向 + 直走）
// =============================================================================
static std::vector<Result> run_route(Navigator& nav, const std::vector<Pt>& route, double scale, bool zero) {
    wait_standing(nav);
    if (zero) zero_pose(nav);
    std::vector<Result> results;
    for (size_t i = 0; i < route.size(); i++) {
        const Pt& pt = route[i];
        if (pt.turn_only) {
            Pose pose = nav.wait_telemetry(1);
            std::printf("點 %zu：在原地轉到 %s°\n", i, g(pt.y).c_str());
            pose = safe_turn(nav, wrap_deg(pt.y));
            std::printf("  → 車頭 %.1f°（目標 %s°，誤差 %+.1f°）\n", nav.heading, g(pt.y).c_str(), wrap_deg(pt.y - nav.heading));
            results.push_back({pose.first / scale, pose.second / scale, pose, 0.0});
            continue;
        }
        Pose tgt{pt.x * scale, pt.y * scale};
        Pose pose = nav.wait_telemetry(1);
        auto [b, d] = plan(pose, tgt);
        if (d < SKIP_DIST_CM) {
            std::printf("點 %zu：(%s, %s) 已在目標上\n", i, g(pt.x).c_str(), g(pt.y).c_str());
            results.push_back({pt.x, pt.y, pose, 0.0});
            continue;
        }
        nav.note(fmt("NAV:PLAN %zu %.1f %.1f", i, tgt.first, tgt.second));
        if (nav.logger) nav.logger->set_goal(tgt.first, tgt.second);
        std::printf("點 %zu：前往 (%s, %s) = (%.1f, %.1f) cm，目前 (%.1f, %.1f)，方向 %.1f°、距離 %.1f cm\n", i, g(pt.x).c_str(), g(pt.y).c_str(), tgt.first,
                    tgt.second, pose.first, pose.second, b, d);
        double err = d;
        for (int attempt = 0; attempt < 1 + RETRIES; attempt++) {
            std::tie(b, d) = plan(pose, tgt);
            if (d < SKIP_DIST_CM) break;
            // 修正時如果目標在背後，就倒車過去（不要原地轉 180°）
            bool back = pt.back || (attempt > 0 && std::fabs(wrap_deg(b - nav.heading)) > 100);
            if (attempt > 0) std::printf("  修正第 %d 次：還差 %.1f cm%s\n", attempt, d, back ? "（倒車）" : "");
            double hd = back ? wrap_deg(b + 180) : b;
            pose = safe_turn(nav, hd);
            for (int n = 0; n < 2; n++) {                                // 轉完用實際位置重算；偏太多就再轉一次
                std::tie(b, d) = plan(pose, tgt);
                hd = back ? wrap_deg(b + 180) : b;
                if (std::fabs(wrap_deg(hd - nav.heading)) <= REALIGN_DEG) break;
                std::printf("  轉向時被帶偏，重新對準 %.1f°\n", hd);
                pose = safe_turn(nav, hd);
            }
            if (d > MAX_LEG_CM) throw NavError(fmt("這一段要走 %.1f cm（超過 %s），位置估計可能壞了，已中止", d, g(MAX_LEG_CM).c_str()));
            pose = nav.command(fmt("M %.1f %.1f", back ? -d : d, hd), move_timeout(d));
            err = std::hypot(pose.first - tgt.first, pose.second - tgt.second);
            std::printf("  → 到達 (%.1f, %.1f) cm，誤差 %.1f cm\n", pose.first, pose.second, err);
            if (err <= RETRY_CM) break;
        }
        results.push_back({pt.x, pt.y, pose, err});
    }
    return results;
}

// =============================================================================
// 7. 命令列
// =============================================================================
struct Args {
    std::string route;
    bool steps = false, dry_run = false, sim = false, no_zero = false, right = false, smooth = false;
    double scale = DEFAULT_SCALE_CM, sim_speed = 5.0, speed = 0.0, laps = 1.0;
    double min_radius = SMOOTH_MIN_R_CM, switch_cm = SMOOTH_SWITCH_CM;
    int seed = -1, baud = BAUD_RATE;
    std::string port = COM_PORT, keys;
    std::optional<double> circle, stop_after;
};

static void usage() {
    std::puts(
        "用法：nav [路線] [選項]      （不給任何參數 = 選單）\n"
        "  路線               例如 \"0,0 1,1 1,-1 -1,-1\"，或路線檔名；加 --steps 時是動作清單\n"
        "  --steps            改成逐一執行動作，例如 \"T90 T180 T-90 T0\" 或 \"M20 M-20\"（M<公分>@<方向> 可指定方向）\n"
        "  --scale N          座標 1 = 幾公分（預設 10）\n"
        "  --dry-run          只印出計畫，不連線\n"
        "  --sim              用模擬車執行（不需要實車）\n"
        "  --sim-speed N      模擬倍速（預設 5 倍）\n"
        "  --seed N           模擬車隨機參數的種子\n"
        "  --port COMx        序列埠（預設 COM3）      --baud N  鮑率（預設 9600）\n"
        "  --no-zero          不先把座標歸零\n"
        "  --circle R         繞圓：半徑 R cm（--right 向右、--laps 圈數、--speed 速度、--stop-after 秒）\n"
        "  --smooth           平滑路線：點與點之間走圓弧、不停車（--min-radius、--switch-cm 調整）\n"
        "  --keys 字元        開始前先送出的按鍵，例如 jj（Kp_speed -1.0）、h（Kd +10）");
}

// 解析命令列；錯誤時印訊息並回傳 false
static bool parse_args(const std::vector<std::string>& v, Args& a, bool& help) {
    help = false;
    bool have_route = false;
    for (size_t i = 0; i < v.size(); i++) {
        std::string s = v[i], inline_val;
        bool has_inline = false;
        if (starts_with(s, "--")) {
            size_t eq = s.find('=');
            if (eq != std::string::npos) { inline_val = s.substr(eq + 1); s = s.substr(0, eq); has_inline = true; }
        }
        auto val = [&]() -> std::string {
            if (has_inline) return inline_val;
            if (i + 1 >= v.size()) throw NavError(s + " 後面少了數值");
            return v[++i];
        };
        auto num = [&]() -> double { return to_double(val()); };
        if (s == "-h" || s == "--help") { help = true; return true; }
        else if (s == "--steps") a.steps = true;
        else if (s == "--dry-run") a.dry_run = true;
        else if (s == "--sim") a.sim = true;
        else if (s == "--no-zero") a.no_zero = true;
        else if (s == "--right") a.right = true;
        else if (s == "--smooth") a.smooth = true;
        else if (s == "--scale") a.scale = num();
        else if (s == "--sim-speed") a.sim_speed = num();
        else if (s == "--seed") a.seed = (int)num();
        else if (s == "--baud") a.baud = (int)num();
        else if (s == "--port") a.port = val();
        else if (s == "--circle") a.circle = num();
        else if (s == "--min-radius") a.min_radius = num();
        else if (s == "--switch-cm") a.switch_cm = num();
        else if (s == "--speed") a.speed = num();
        else if (s == "--laps") a.laps = num();
        else if (s == "--stop-after") a.stop_after = num();
        else if (s == "--keys") a.keys = val();
        else if (starts_with(s, "--")) throw NavError("不認得的選項：" + s + "（--help 看說明）");
        else if (!have_route) { a.route = v[i]; have_route = true; }
        else throw NavError("多餘的參數：" + v[i]);
    }
    if (!have_route) {
        if (a.circle && *a.circle != 0) a.route = "T0";
        else throw NavError("少了路線（例如 \"0,0 3,0 3,3 0,3 0,0\"）；--help 看說明");
    }
    return true;
}

// 回傳 0 = 成功；1 = 中止 / 失敗；2 = 參數錯誤
static int nav_main(const std::vector<std::string>& argv) {
    Args a;
    bool help;
    try {
        parse_args(argv, a, help);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }
    if (help) { usage(); return 0; }

    std::optional<std::pair<double, double>> forever;                    // (有號半徑, 速度)
    std::vector<Step> steps;
    std::vector<Pt> route;
    bool use_steps = a.steps;
    try {
        if (a.circle && *a.circle != 0) {
            double signed_r = a.right ? std::fabs(*a.circle) : -std::fabs(*a.circle);
            const char* side = a.right ? "向右（順時針）" : "向左（逆時針）";
            use_steps = true;
            if (a.laps <= 0) {
                forever = {signed_r, a.speed};
                a.route = "T0";                                          // 佔位，不會用到
                std::printf(">> 繞圓：半徑 %s cm、%s，一直繞到你按 Enter\n", g(std::fabs(*a.circle)).c_str(), side);
            } else {
                if (a.stop_after) std::printf(">> 注意：--stop-after 只在 --laps 0 時有效，這次會繞完 %s 圈\n", g(a.laps).c_str());
                double arc = 2 * PI * std::fabs(*a.circle) * a.laps;
                a.route = fmt("C%.1f@%s", arc, g(signed_r).c_str()) + (a.speed != 0 ? "@" + g(a.speed) : "");
                std::printf(">> 繞圓：半徑 %s cm、%s、%s 圈，弧長 %.1f cm\n", g(std::fabs(*a.circle)).c_str(), side, g(a.laps).c_str(), arc);
            }
        }
        if (use_steps) steps = parse_steps(a.route);
        else route = parse_route(a.route);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }

    if (a.dry_run && forever) {
        std::printf("  #C %s %s%s   （按 Enter 後再送 #F，沿著圓弧慢慢停下）\n", g(FOREVER_ARC_CM).c_str(), g(forever->first).c_str(),
                    forever->second != 0 ? (" " + g(forever->second)).c_str() : "");
        return 0;
    }
    if (a.dry_run && use_steps) { print_steps(steps); return 0; }
    if (a.dry_run && a.smooth) { smooth_dry_run(route, a.scale, a.min_radius); return 0; }
    if (a.dry_run) { dry_run(route, a.scale); return 0; }

    g_interrupted = false;
    std::signal(SIGINT, on_sigint);
    Link* link = nullptr;
    TelemetryLogger* logger = nullptr;
    double k = 1.0;
    try {
        if (a.sim) {
            link = new SimRobot(a.sim_speed, a.seed);
            logger = new TelemetryLogger("sim");
            k = a.sim_speed;
            std::printf(">> 模擬模式（%s 倍速），記錄檔：%s\n", g(a.sim_speed).c_str(), logger->path.c_str());
        } else {
            link = new SerialLink(a.port, a.baud);
            logger = new TelemetryLogger();
            std::printf(">> 已連線 %s，記錄檔：%s\n", a.port.c_str(), logger->path.c_str());
            std::puts(">> 按 Ctrl+C 可隨時中止");
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "無法開啟序列埠 %s：%s\n", a.port.c_str(), e.what());
        delete link;
        delete logger;
        std::signal(SIGINT, SIG_DFL);
        return 1;
    }

    Navigator nav(link, logger, k);
    int code = 0;
    try {
        for (char ch : a.keys) {                                         // 調參按鍵（韌體會回報 >> Kp_speed = x 之類）
            link->send(std::string(1, ch));
            nav.sleep(0.4);
        }
        if (forever) {
            run_arc_until_stop(nav, forever->first, forever->second, a.stop_after, !a.no_zero);
        } else if (use_steps) {
            run_steps(nav, steps, !a.no_zero);
        } else if (a.smooth) {
            run_smooth(nav, route, a.scale, a.min_radius, a.switch_cm, a.speed, !a.no_zero);
        } else {
            auto results = run_route(nav, route, a.scale, !a.no_zero);
            std::puts("\n==== 結果 ====");
            std::puts(" 點   目標(cm)          到達(cm)          誤差(cm)");
            double max_err = 0.0;
            for (size_t i = 0; i < results.size(); i++) {
                auto& r = results[i];
                std::printf(" %2zu  (%6.1f,%6.1f)   (%6.1f,%6.1f)   %6.1f\n", i, r.x * a.scale, r.y * a.scale, r.pose.first, r.pose.second, r.err);
                if (i > 0) max_err = std::max(max_err, r.err);
            }
            std::printf("最大誤差 %.1f cm（這是車上里程計自己的估計；實際位置請用捲尺量）\n", max_err);
        }
    } catch (const Interrupted&) {
        std::puts("\n>> 中止！");
        nav.abort();
        code = 1;
    } catch (const std::exception& e) {
        std::printf("\n>> 中止：%s\n", e.what());
        nav.abort();
        code = 1;
    }
    sleep_s(0.3 / k);
    sleep_s(0.1);
    link->close();
    logger->close();
    std::printf(">> 記錄檔：%s（可用 python analyze.py 這個檔名 分析）\n", logger->path.c_str());
    delete link;
    delete logger;
    std::signal(SIGINT, SIG_DFL);
    return code;
}

// =============================================================================
// 8. 選單（沒有參數時；對應 run_nav.py）
// =============================================================================
struct Preset {
    const char* name;
    const char* desc;
    const char* route;
};
// 內建形狀：座標單位 1 = 10 cm（可在選單裡改倍率）
static const Preset PRESETS[] = {
    {"正方形", "前進30 → 右30 → 後30 → 回起點", "0,0 3,0 3,3 0,3 0,0"},
    {"長方形", "90 × 45 cm，轉彎回起點", "0,0 9,0 9,4.5 0,4.5 0,0"},
    {"三角形", "30 cm 直角三角形", "0,0 3,0 3,3 0,0"},
    {"之字形", "每段 20 cm，轉 45°", "0,0 2,0 4,2 6,0"},
    {"去程再倒車回來", "之字形去，倒車原路回來", "0,0 1.4,1.4 2.8,0 4.2,1.4 2.8,0b 1.4,1.4b 0,0b"},
    {"直線往返", "前進 50 cm 再倒車回來", "0,0 5,0 0,0b"},
};
constexpr int N_PRESETS = (int)(sizeof(PRESETS) / sizeof(PRESETS[0]));

static bool g_eof = false;
static std::string ask(const std::string& prompt, const std::string& def = "") {
    std::fputs(prompt.c_str(), stdout);
    std::fflush(stdout);
    std::string s;
    if (!std::getline(std::cin, s)) { g_eof = true; return def; }
    s = strip(s);
    return s.empty() ? def : s;
}

static double ask_float(const std::string& prompt, double def) {
    for (;;) {
        std::string s = ask(prompt, "");
        if (g_eof || s.empty()) return def;
        char* end = nullptr;
        double v = std::strtod(s.c_str(), &end);
        if (*end == '\0' && v > 0) return v;
        std::puts("  請輸入大於 0 的數字");
    }
}

static fs::path routes_dir() {
    fs::path d = exe_dir();
    fs::path p = d.parent_path() / "controller" / "routes";
    return fs::exists(p) ? p : d / "routes";
}

struct Picked {
    std::string name;
    std::vector<std::string> args;
    bool can_smooth, preset;
};

// 回傳 false = 離開
static bool pick_route(Picked& out) {
    std::puts("\n==================== 座標導航 ====================");
    for (int i = 0; i < N_PRESETS; i++) std::printf("  [%d] %s %s\n", i + 1, ljust(PRESETS[i].name, 8).c_str(), PRESETS[i].desc);
    std::puts("  [c] 自己輸入座標      [o] 繞圓      [f] 讀路線檔      [q] 離開");
    for (;;) {
        std::string s = lower(ask("請選擇："));
        if (g_eof || s == "q" || s == "quit" || s == "exit") return false;
        if (!s.empty() && std::all_of(s.begin(), s.end(), ::isdigit) && std::atoi(s.c_str()) >= 1 && std::atoi(s.c_str()) <= N_PRESETS) {
            const Preset& p = PRESETS[std::atoi(s.c_str()) - 1];
            out = {p.name, {p.route}, true, true};
            return true;
        }
        if (s == "c") {
            std::puts("  格式：x,y 以空白分隔；座標後加 b = 倒車過去。例：0,0 3,0 3,3 0,3 0,0");
            std::string route = ask("  座標：");
            if (!route.empty()) { out = {"自訂座標", {route}, true, false}; return true; }
            continue;
        }
        if (s == "o") {
            double r = ask_float("  半徑（cm，預設 15）：", 15.0);
            bool right = lower(ask("  方向 [l]左（逆時針）/ [r]右（順時針），預設 l：", "l")).rfind("r", 0) == 0;
            double laps = ask_float("  圈數（預設 1）：", 1.0);
            std::vector<std::string> args = {"T0", "--circle", g(r), "--laps", g(laps)};
            if (right) args.push_back("--right");
            out = {fmt("繞圓 R=%gcm", r), args, false, false};
            return true;
        }
        if (s == "f") {
            std::vector<std::string> names;
            std::error_code ec;
            for (auto& e : fs::directory_iterator(routes_dir(), ec))
                if (e.path().extension() == ".txt") names.push_back(path_utf8(e.path().filename()));
            std::sort(names.begin(), names.end());
            std::string list;
            for (auto& n : names) list += (list.empty() ? "" : ", ") + n;
            std::printf("  routes 資料夾：%s\n", list.empty() ? "（空）" : list.c_str());
            std::string f = ask("  檔名（例 square.txt）：");
            if (!f.empty()) {
                std::error_code ec2;
                std::string path = fs::is_regular_file(fs::u8path(f), ec2) ? f : path_utf8(routes_dir() / fs::u8path(f));
                out = {f, {path}, true, false};
                return true;
            }
            continue;
        }
        if (g_eof) return false;
        std::puts("  看不懂，請再選一次");
    }
}

static char pick_mode() {
    std::puts("\n模式：[1] 預覽（只印計畫，不連車）  [2] 模擬車  [3] 實車");
    for (;;) {
        std::string s = ask("請選擇（預設 1）：", "1");
        if (g_eof) return '1';
        if (s == "1" || s == "2" || s == "3") return s[0];
        std::puts("  請輸入 1、2 或 3");
    }
}

// 各段直線長度（座標單位）；h<角度> 這種原地轉向不算
static std::vector<double> route_legs(const std::string& route_text) {
    std::vector<double> legs;
    bool have_prev = false;
    double px = 0, py = 0;
    for (auto& p : parse_route(route_text)) {
        if (p.turn_only) continue;
        if (have_prev) legs.push_back(std::hypot(p.x - px, p.y - py));
        px = p.x;
        py = p.y;
        have_prev = true;
    }
    return legs;
}

// 問大小，並檢查每段長度；回傳 座標1=幾公分
static double ask_scale(const std::string& route_text, bool preset) {
    auto legs = route_legs(route_text);
    for (;;) {
        double scale = preset ? 10.0 * ask_float("大小倍率（1 = 原尺寸；0.5 = 縮小一半；2 = 放大兩倍；預設 1）：", 1.0)
                              : ask_float("座標 1 = 幾公分（預設 10）：", 10.0);
        if (legs.empty() || g_eof) return scale;
        double longest = *std::max_element(legs.begin(), legs.end()) * scale, shortest = *std::min_element(legs.begin(), legs.end()) * scale;
        std::printf("  → 最長一段 %.0f cm，最短一段 %.0f cm\n", longest, shortest);
        if (longest > MAX_LEG_CM) {
            std::printf("  ✗ 超過單段上限 %s cm（超過會被中止），請填小一點\n", g(MAX_LEG_CM).c_str());
            continue;
        }
        if (shortest < 15)
            std::puts("  ⚠ 有一段短於 15 cm：實測每次原地轉彎車子會前後晃 3～7 cm，路線太小容易因「轉彎位置被帶偏」中止；建議每段 ≥ 30 cm");
        return scale;
    }
}

// 回傳 false = 離開選單
static bool run_once() {
    Picked pk;
    if (!pick_route(pk)) return false;
    char mode = pick_mode();
    bool is_circle = std::find(pk.args.begin(), pk.args.end(), "--circle") != pk.args.end();
    std::vector<std::string> args = pk.args;
    if (!is_circle) {
        try {
            double scale = ask_scale(args[0], pk.preset);
            args.push_back("--scale");
            args.push_back(g(scale));
        } catch (const std::exception& e) {
            std::printf(">> 路線有問題：%s\n", e.what());
            return true;
        }
        if (pk.can_smooth && lower(ask("平滑路線（點與點之間走圓弧不停車）？[y/N]：")).rfind("y", 0) == 0) args.push_back("--smooth");
    }
    if (mode == '1') args.push_back("--dry-run");
    else if (mode == '2') args.push_back("--sim");
    else {
        std::puts("\n>> 實車：車子站穩、前後左右留 1 公尺以上；其他程式（drive、Arduino 序列埠監控視窗等）要先關掉；執行中 Ctrl+C 可中止");
        if (lower(ask(">> 要開始「" + pk.name + "」嗎？按 Enter 開始，輸入 n 取消：")).rfind("n", 0) == 0) return true;
    }
    std::string cmd = "nav";
    for (auto& s : args) cmd += s.find(' ') != std::string::npos ? " \"" + s + "\"" : " " + s;
    std::printf(">> 等同命令：%s\n\n", cmd.c_str());
    try {
        nav_main(args);
    } catch (const std::exception& e) {                                  // 單次失敗不要讓選單結束
        std::printf(">> 發生錯誤：%s\n", e.what());
    }
    return true;
}

int main(int argc, char** argv) {
    init_console();
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty()) {
        while (!g_eof && run_once()) {}
        std::puts("\n>> 結束");
        return 0;
    }
    return nav_main(args);
}
