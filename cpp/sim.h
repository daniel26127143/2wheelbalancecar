// ============================================================================
// sim.h — 模擬車（對應 nav.py 的 NavCore 與 SimRobot；--sim 用）
//   NavCore  ：韌體 nav.h 的逐行對照版（角度是「相對座標原點」的 rad，位置單位 ticks）
//   SimRobot ：簡化車體模型（延遲、靜摩擦、平衡晃動、自轉漂移都是隨機的），
//              吃導航程式送的字元，吐出和韌體同格式的遙測與 NAV 訊息。
//              用來確認流程，不代表實車表現。
// 和 Python 版的差別：不開背景執行緒——每次 read_line() 時依牆上時鐘把模擬往前推進。
// ============================================================================
#pragma once

#include <random>

#include "link.h"

namespace rc {

// 與 nav.h 的 #define 相同
struct NavParams {
    double TURN_KP = 1.5, TURN_RATE_MAX = 0.8, TURN_RATE_MIN = 0.25, TURN_TOL = 2.5 * 0.0174533;
    double TURN_STILL = 0.15;
    int TURN_SETTLE = 30, TURN_TIMEOUT = 600;
    double TURN_LEAD = 0.3;
    double TV_MAX = 5.8, TV_MAX_BACK = 5.0, SLOW_CM = 20.0, TV_MIN = 3.0, TV_MIN_BACK = 2.5, POS_TOL_CM = 2.0, SETTLE_TOL_CM = 3.0,
           STILL_V = 6.0, V_FILT = 0.8;
    int SETTLE_UPD = 12, SETTLE_MAX = 75, MOVE_TIME_BASE = 100, MOVE_TIME_PER_CM = 9;
    double DT_V = 0.0405, FF_FILT = 0.7, TV_ARC_CAP = 15.0;   // [v37g] 10/27 12 → 15（同韌體 nav.h）
};
inline const NavParams NAV;

enum NavMode { NAV_IDLE = 0, NAV_TURN = 1, NAV_MOVE = 2, NAV_STOP = 3 };
enum NavRes { RUNNING = 0, DONE = 1, TIMEOUT = 2 };

struct StepResult {
    double out;
    int res;
};

class NavCore {
public:
    int mode = NAV_IDLE;
    int timer = 0, settle = 0, timeout = 0;
    double remaining = 0.0;
    double target_heading = 0.0, move_heading = 0.0;
    double curv = 0.0, h0 = 0.0, px = 0.0, py = 0.0, s_arc = 0.0, yaw_ff = 0.0;
    double tv_fwd = NAV.TV_MAX;
    double move_dist = 0.0, sx = 0.0, sy = 0.0, v_avg = 0.0;
    double tpc;

    explicit NavCore(double ticks_per_cm = 30.0) : tpc(ticks_per_cm) {}

    void stop() {
        mode = NAV_IDLE;
        timer = settle = timeout = 0;
        curv = yaw_ff = 0.0;
    }

    void start_turn(double target_rel, double yaw_rel = 0.0) {
        mode = NAV_TURN;
        timer = settle = 0;
        curv = yaw_ff = 0.0;
        target_heading = wrap_rad(target_rel);
        remaining = wrap_rad(target_heading - yaw_rel);
        timeout = NAV.TURN_TIMEOUT;
    }

    void start_move(double dist, double heading_rel, double x, double y) {
        mode = NAV_MOVE;
        timer = settle = 0;
        move_heading = wrap_rad(heading_rel);
        move_dist = dist;
        sx = x;
        sy = y;
        curv = yaw_ff = s_arc = 0.0;
        tv_fwd = NAV.TV_MAX;
        h0 = move_heading;
        px = x;
        py = y;
        v_avg = 0.0;
        remaining = dist;
        double cm = std::fabs(dist) / tpc;
        timeout = NAV.MOVE_TIME_BASE + (int)(cm * NAV.MOVE_TIME_PER_CM);
    }

    // #C：dist = 弧長 (ticks)；curv = 曲率 (rad/tick，正 = 右轉)；yaw_rel = 目前方向 (rad)；tv = 前進目標速度（0 = 預設）
    void start_arc(double dist, double curv_, double yaw_rel, double x, double y, double tv = 0.0) {
        bool chain = mode == NAV_MOVE && curv != 0.0;                  // 弧線進行中又收到 #C：不停車、方向不歸零，直接接成新的弧線
        double mh = chain ? move_heading : wrap_rad(yaw_rel);
        double yf = chain ? yaw_ff : 0.0;
        start_move(dist, mh, x, y);
        tv_fwd = tv > 0.0 ? clampd(tv, NAV.TV_MIN, NAV.TV_ARC_CAP) : NAV.TV_MAX;
        curv = curv_;
        yaw_ff = yf;
    }

    // #F：結束弧線，把終點改成「現在再往前一段減速距離」
    void finish_arc() {
        timer = settle = 0;
        if (mode == NAV_MOVE && curv != 0.0) {
            double slow = NAV.SLOW_CM * (tv_fwd > NAV.TV_MAX ? tv_fwd / NAV.TV_MAX : 1.0) * tpc;
            if (move_dist - s_arc > slow) move_dist = s_arc + slow;
        }
    }

    StepResult turn_step(double yaw_rel, double yaw_rate) {
        double err = wrap_rad(target_heading - yaw_rel);
        remaining = err;
        double aerr = std::fabs(err);
        if (aerr < NAV.TURN_TOL && std::fabs(yaw_rate) < NAV.TURN_STILL) {
            settle++;
            if (settle >= NAV.TURN_SETTLE) return {0.0, DONE};
        } else {
            settle = 0;
        }
        timer++;
        if (timer >= timeout) return {0.0, TIMEOUT};
        if (aerr < NAV.TURN_TOL) return {0.0, RUNNING};
        double pred = err - yaw_rate * NAV.TURN_LEAD;
        if (pred * err <= 0.0) return {0.0, RUNNING};
        double r = clampd(NAV.TURN_KP * pred, -NAV.TURN_RATE_MAX, NAV.TURN_RATE_MAX);
        if (std::fabs(r) < NAV.TURN_RATE_MIN) r = err > 0 ? NAV.TURN_RATE_MIN : -NAV.TURN_RATE_MIN;
        return {r, RUNNING};
    }

    StepResult move_step(double x, double y, double v) {
        double s;
        if (curv != 0.0) {
            // 弧線：累積沿目前切線方向的位移 = 已走弧長；目標方向 = 起點方向 + 曲率 × 弧長（走完就不再轉）
            double ds = (x - px) * std::cos(move_heading) + (y - py) * std::sin(move_heading);
            px = x;
            py = y;
            s_arc += ds;
            double sh = clampd(s_arc, 0.0, move_dist);
            move_heading = wrap_rad(h0 + curv * sh);
            bool running = mode == NAV_MOVE && s_arc < move_dist;
            double ff = (running && ds > 0.0) ? curv * ds / NAV.DT_V : 0.0;
            yaw_ff = running ? NAV.FF_FILT * yaw_ff + (1.0 - NAV.FF_FILT) * ff : 0.0;
            s = s_arc;
        } else {
            s = (x - sx) * std::cos(move_heading) + (y - sy) * std::sin(move_heading);
        }
        double r = move_dist - s;
        remaining = r;
        double tol = NAV.POS_TOL_CM * tpc;
        timer++;
        if (timer >= timeout) {
            bool ok = mode == NAV_STOP && std::fabs(r) < (curv != 0.0 ? 5.0 : 2.0) * tol;
            return {0.0, ok ? DONE : TIMEOUT};
        }
        if (mode == NAV_MOVE) {
            if (std::fabs(r) < tol) {
                mode = NAV_STOP;
                v_avg = v;
                settle = 0;
                timer = 0;
                timeout = NAV.SETTLE_MAX;
                return {0.0, RUNNING};
            }
            double vmax = r > 0 ? tv_fwd : NAV.TV_MAX_BACK;
            double vmin = r > 0 ? NAV.TV_MIN : NAV.TV_MIN_BACK;
            double slow = NAV.SLOW_CM * ((r > 0 && tv_fwd > NAV.TV_MAX) ? tv_fwd / NAV.TV_MAX : 1.0);
            double k = vmax / (slow * tpc);
            double fwd = clampd(k * r, -vmax, vmax);
            if (std::fabs(fwd) < vmin) fwd = r > 0 ? vmin : -vmin;
            return {-fwd, RUNNING};
        }
        v_avg = NAV.V_FILT * v_avg + (1.0 - NAV.V_FILT) * v;
        if (std::fabs(r) < (curv != 0.0 ? 2.0 : 1.0) * NAV.SETTLE_TOL_CM * tpc && std::fabs(v_avg) < NAV.STILL_V) {
            settle++;
            if (settle >= NAV.SETTLE_UPD) return {0.0, DONE};
        } else {
            settle = 0;
        }
        return {0.0, RUNNING};
    }
};

// 在「牆上時鐘 × 倍速」的模擬時間裡執行的模擬車；Link 介面讓導航程式把它當成序列埠用
class SimRobot : public Link {
public:
    static constexpr double DT = 0.01013;
    static constexpr double TICKS_PER_CM = 36.2;

    SimRobot(double speed = 5.0, int seed = -1) : speed_(speed), nav_(TICKS_PER_CM) {
        if (seed >= 0) rng_.seed((unsigned)seed);
        else rng_.seed(std::random_device{}());
        G = 1.5 + 1.0 * uni();                              // 實測：導航速度 V ≈ 1.5~2.5 倍目標速度
        tau_v = 0.35 + 0.3 * uni();
        tau_coast = 0.5 + 0.4 * uni();
        stick_tv = 0.6 + 0.6 * uni();
        tau_yaw = 0.25 + 0.2 * uni();
        stick_rate = 0.12 + 0.12 * uni();
        spin_drift = (uni() - 0.5) * 8.0;
        yaw = 0.3 + (-0.05 + 0.1 * uni());
        wall0_ = now_s();
    }

    // --- 序列埠介面 ---
    void send(const std::string& s) override {
        for (char c : s) inbox_.push_back(c);
    }
    bool read_line(std::string& out) override {
        advance();
        if (outbox_.empty()) return false;
        out = outbox_.front();
        outbox_.pop_front();
        return true;
    }
    void close() override {}

private:
    double speed_;
    NavCore nav_;
    std::mt19937_64 rng_;
    double G, tau_v, tau_coast, stick_tv, tau_yaw, stick_rate, spin_drift;
    double x = 0.0, y = 0.0, yaw0 = 0.3, yaw = 0.3;
    double yaw_rate = 0.0, v = 0.0, target_speed = 0.0, cmd = 0.0, ref = 0.0, osc = 0.0;
    double vf = 0.0, vm = 0.0;
    double t = 0.0, next_tel = 0.0, wall0_ = 0.0;
    int k = 0;
    bool in_line = false;
    std::string buf_;
    std::deque<char> inbox_;
    std::deque<std::string> outbox_;

    double uni() { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_); }
    double gauss(double mu, double sigma) { return std::normal_distribution<double>(mu, sigma)(rng_); }

    // --- 韌體行為 ---
    std::string pose_str() const {
        return fmt("PX:%.1f PY:%.1f H:%.1f", x / TICKS_PER_CM, y / TICKS_PER_CM, wrap_rad(yaw - yaw0) / DEG);
    }
    double yaw_rel() const { return wrap_rad(yaw - yaw0); }

    // 同韌體 Nav_Report()：NAV:DONE T err=1.2 PX:.. PY:.. H:..
    void finish(const std::string& tag) {
        char kind = nav_.mode == NAV_TURN ? 'T' : (nav_.curv != 0.0 ? 'C' : 'M');
        double err = kind == 'T' ? nav_.remaining / DEG : nav_.remaining / TICKS_PER_CM;
        outbox_.push_back(fmt("%s%c err=%.1f %s", tag.c_str(), kind, err, pose_str().c_str()));
        nav_.stop();
        cmd = 0.0;
    }

    void on_line(const std::string& line) {
        char c = line.empty() ? '\0' : (char)std::toupper((unsigned char)line[0]);
        std::vector<double> nums;
        for (auto& tok : split_ws(replace_all(line.substr(line.empty() ? 0 : 1), ",", " "))) {
            char* end = nullptr;
            double d = std::strtod(tok.c_str(), &end);
            if (end != tok.c_str() && *end == '\0') nums.push_back(d);
        }
        if (c == '\0' || c == 'S' || c == 'A') {
            if (nav_.mode != NAV_IDLE) finish("NAV:ABORT CMD ");
        } else if (c == 'T' && !nums.empty()) {
            nav_.start_turn(nums[0] * DEG, yaw_rel());
            outbox_.push_back(fmt("NAV:START T %.1f", nums[0]));
        } else if (c == 'M' && !nums.empty()) {
            double heading = nums.size() > 1 ? nums[1] * DEG : yaw_rel();
            nav_.start_move(nums[0] * TICKS_PER_CM, heading, x, y);
            outbox_.push_back(fmt("NAV:START M %.1f", nums[0]) + (nums.size() > 1 ? fmt(" %.1f", nums[1]) : ""));
        } else if (c == 'F') {
            nav_.finish_arc();
            outbox_.push_back("NAV:START F 0.0");
        } else if (c == 'C' && nums.size() > 1 && nums[0] > 0 && nums[1] != 0) {
            nav_.start_arc(nums[0] * TICKS_PER_CM, 1.0 / (nums[1] * TICKS_PER_CM), yaw_rel(), x, y, nums.size() > 2 ? nums[2] : 0.0);
            outbox_.push_back(fmt("NAV:START C %.1f %.1f", nums[0], nums[1]));
        } else {
            outbox_.push_back("NAV:REJECT ?");
        }
    }

    // 依牆上時鐘把模擬推進到「現在 × 倍速」
    void advance() {
        double target = (now_s() - wall0_) * speed_;
        int guard = 0;
        while (t < target && guard++ < 20000) iterate();
    }

    void iterate() {
        // 處理收到的字元
        while (!inbox_.empty()) {
            char c = inbox_.front();
            inbox_.pop_front();
            if (in_line) {
                if (c == '\r' || c == '\n') {
                    in_line = false;
                    on_line(buf_);
                } else {
                    buf_ += c;
                }
            } else if (c == '#') {
                in_line = true;
                buf_.clear();
            } else if (c == 'z' || c == 'Z') {
                x = y = 0.0;
                yaw0 = yaw;
                outbox_.push_back(">> 座標歸零");
            } else if (std::strchr("wxadscWXADSC ", c) && c != '\0' && nav_.mode != NAV_IDLE) {
                finish("NAV:ABORT USER ");
            }
        }
        // 10 ms 控制週期
        double rate_des = 0.0;
        if (nav_.mode == NAV_TURN) {
            StepResult r = nav_.turn_step(yaw_rel(), yaw_rate + gauss(0, 0.03));
            rate_des = r.out;
            if (r.res != RUNNING) finish(r.res == DONE ? "NAV:DONE " : "NAV:TIMEOUT ");
        }
        bool decel = std::fabs(rate_des) < std::fabs(ref) || rate_des * ref < 0;
        double st = (decel ? 2.0 : 4.0) * DT;
        ref += std::max(-st, std::min(st, rate_des - ref));
        if (nav_.mode == NAV_MOVE || nav_.mode == NAV_STOP)             // 直走時航向鎖在指定方向
            ref = std::max(-0.8, std::min(0.8, nav_.yaw_ff + 3.0 * wrap_rad(yaw0 + nav_.move_heading - yaw)));
        k++;
        if (k >= 4) {
            k = 0;
            osc += 0.0405 * 2 * PI * 2.0;
            double v_meas = v + 3.0 * std::sin(osc) + gauss(0, 1.0);
            if (nav_.mode == NAV_MOVE || nav_.mode == NAV_STOP) {
                vf = 0.7 * vf + 0.3 * v_meas;                           // 同韌體 filtered_speed
                StepResult r = nav_.move_step(x, y, vf);
                cmd = r.out;
                if (r.res != RUNNING) finish(r.res == DONE ? "NAV:DONE " : "NAV:TIMEOUT ");
            }
            speed_update(nav_.mode == NAV_TURN && std::fabs(ref) > 0.3);
            vm = v_meas;
        }
        tick();
        t += DT;
        if (t >= next_tel) {
            next_tel += 0.2;
            outbox_.push_back(telemetry());
        }
    }

    void speed_update(bool spinning) {
        bool reversing = cmd * target_speed < 0;
        bool braking = reversing || std::fabs(cmd) < std::fabs(target_speed);
        double step = (braking && !reversing) ? 0.15 : 0.30;
        if (target_speed < cmd) target_speed = std::min(target_speed + step, cmd);
        else if (target_speed > cmd) target_speed = std::max(target_speed - step, cmd);
        double vt = G * target_speed;
        if (std::fabs(target_speed) < stick_tv && std::fabs(v) < 1.0) vt = 0.0;
        if (spinning) vt = spin_drift;
        double tau = (target_speed == 0 && !spinning) ? tau_coast : tau_v;
        v += (vt - v) * (0.0405 / tau);
    }

    void tick() {
        double target = ref;
        if (std::fabs(ref) < stick_rate && std::fabs(yaw_rate) < 0.1) target = 0.0;
        yaw_rate += (target - yaw_rate) * (DT / tau_yaw);
        yaw = wrap_rad(yaw + yaw_rate * DT);
        double fwd = -v / 4.0;
        double h = yaw - yaw0;
        x += fwd * std::cos(h);
        y += fwd * std::sin(h);
    }

    std::string telemetry() {
        double tgt = yaw0 + (nav_.mode == NAV_TURN ? nav_.target_heading : nav_.move_heading);
        std::string line = fmt("ANG:%.1f SA:0.00 B:0.0 T:0.0 PWM:0/0 YR:%.2f Y:%.2f TY:%.2f SP:-1.45 DX:0 V:%.1f TV:%.2f BS:0.00 PX:%.1f PY:%.1f",
                               -1.45 + gauss(0, 0.6), yaw_rate, yaw, wrap_rad(tgt), vm, target_speed, x / TICKS_PER_CM, y / TICKS_PER_CM);
        if (nav_.mode != NAV_IDLE) {
            double nr = nav_.mode == NAV_TURN ? nav_.remaining / DEG : nav_.remaining / TICKS_PER_CM;
            line += fmt(" NV:%d NR:%.1f", nav_.mode, nr);
        }
        return line + " | 狀態:【常態雙閉環】";
    }
};

}  // namespace rc
