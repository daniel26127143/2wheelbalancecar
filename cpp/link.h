// ============================================================================
// link.h — 小車操控軟體的共用層（對應 Python 的 control.py）
//   1. 工具函式   —— 時間、格式化、角度換算、Ctrl+C 旗標
//   2. 遙測解析   —— 把 Arduino 每行遙測拆成欄位（和 control.py 的 parse_telemetry 相同）
//   3. CSV 記錄   —— 格式和 Python 版完全一樣，analyze.py 可以直接讀
//   4. 序列埠連線 —— Windows（Win32 API）；Linux 版只為了在沒有車的環境測試
//
// 整個程式是「單執行緒」：所有等待迴圈都會呼叫 pump() 讀序列埠、送心跳，
// 所以不需要 std::thread / mutex（MinGW 的 win32 thread 版本也能編）。
// 需要 C++17。原始檔請用 UTF-8 儲存（g++ 預設；MSVC 要加 /utf-8）。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <csignal>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace rc {

namespace fs = std::filesystem;

// =============================================================================
// 1. 設定與工具
// =============================================================================
constexpr const char* COM_PORT = "COM3";
constexpr int BAUD_RATE = 9600;

struct NavError : std::runtime_error {          // 導航失敗（逾時、被帶偏、車子回報錯誤…）
    using std::runtime_error::runtime_error;
};
struct Interrupted {};                          // 使用者按 Ctrl+C

inline std::atomic<bool> g_interrupted{false};
extern "C" inline void on_sigint(int) { g_interrupted = true; }
inline void check_interrupt() {
    if (g_interrupted) throw Interrupted{};
}

inline std::string fmt(const char* f, ...) {
    va_list ap, ap2;
    va_start(ap, f);
    va_copy(ap2, ap);
    int n = std::vsnprintf(nullptr, 0, f, ap);
    va_end(ap);
    std::string s(n > 0 ? (size_t)n : 0, '\0');
    if (n > 0) std::vsnprintf(&s[0], s.size() + 1, f, ap2);
    va_end(ap2);
    return s;
}

inline double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
inline void sleep_s(double s) {                    // 不用 std::this_thread：舊版 MinGW（win32 thread 模型）沒有它
    if (s <= 0) return;
#ifdef _WIN32
    Sleep((DWORD)std::ceil(s * 1000.0));
#else
    timespec ts;
    ts.tv_sec = (time_t)s;
    ts.tv_nsec = (long)((s - (double)ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
#endif
}

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double approach(double cur, double target, double step) {     // cur 每次最多移動 step 靠近 target
    return cur < target ? std::min(cur + step, target) : std::max(cur - step, target);
}
constexpr double PI = 3.14159265358979323846;
constexpr double DEG = PI / 180.0;
inline double wrap_deg(double a) {                                   // → (-180, 180]，同 Python 的 (a+180)%360-180
    double m = std::fmod(a + 180.0, 360.0);
    if (m < 0) m += 360.0;
    return m - 180.0;
}
inline double wrap_rad(double a) {
    while (a > PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}

// 空白切字串（同 Python str.split()）
inline std::vector<std::string> split_ws(const std::string& s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && std::isspace((unsigned char)s[i])) i++;
        size_t j = i;
        while (j < s.size() && !std::isspace((unsigned char)s[j])) j++;
        if (j > i) out.push_back(s.substr(i, j - i));
        i = j;
    }
    return out;
}
inline std::string strip(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace((unsigned char)s[a])) a++;
    while (b > a && std::isspace((unsigned char)s[b - 1])) b--;
    return s.substr(a, b - a);
}
inline std::string replace_all(std::string s, const std::string& from, const std::string& to) {
    size_t p = 0;
    while ((p = s.find(from, p)) != std::string::npos) { s.replace(p, from.size(), to); p += to.size(); }
    return s;
}
inline std::string lower(std::string s) {
    for (auto& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}
inline bool starts_with(const std::string& s, const std::string& p) { return s.compare(0, p.size(), p) == 0; }

// Python 的 float()：整段都要是數字，否則丟 NavError
inline double to_double(const std::string& s) {
    char* end = nullptr;
    double v = std::strtod(s.c_str(), &end);
    if (s.empty() || end == s.c_str() || *end != '\0') throw NavError("看不懂的數字：'" + s + "'");
    return v;
}

// 主控台初始化：UTF-8 輸入輸出（中文才不會亂碼）、啟用跳脫序列
inline void init_console() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (h != INVALID_HANDLE_VALUE && GetConsoleMode(h, &mode)) SetConsoleMode(h, mode | 0x0004);  // ENABLE_VIRTUAL_TERMINAL_PROCESSING
#endif
}

// 執行檔所在資料夾
inline fs::path exe_dir() {
#ifdef _WIN32
    wchar_t buf[32768];
    DWORD n = GetModuleFileNameW(nullptr, buf, 32768);
    return fs::path(std::wstring(buf, n)).parent_path();
#else
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return fs::current_path();
    return fs::path(std::string(buf, (size_t)n)).parent_path();
#endif
}

// 記錄檔根目錄：<repo>/controller/logs（和 Python 版同一個地方，analyze.py 才讀得到）；
// 執行檔旁邊沒有 controller 資料夾時改用 <執行檔資料夾>/logs
inline fs::path log_root() {
    fs::path d = exe_dir();
    fs::path repo = d.parent_path();
    if (fs::exists(repo / "controller")) return repo / "controller" / "logs";
    return d / "logs";
}

inline std::string path_utf8(const fs::path& p) {
    auto u = p.u8string();
    return std::string(u.begin(), u.end());
}

// =============================================================================
// 2. 遙測解析
// =============================================================================
// 遙測欄位（與 repo.ino 的輸出順序相同；欄位意義見 control.py）
inline const std::vector<std::string>& telemetry_fields() {
    static const std::vector<std::string> f = {"ANG", "SA", "B", "T", "PWM_L", "PWM_R", "YR", "Y", "TY",
                                               "G", "SP", "OL", "DX", "V", "TV", "BS", "EL", "ER", "PX", "PY", "NV", "NR"};
    return f;
}

using Fields = std::map<std::string, std::string>;

// 回傳 true = 是遙測行，並填入 state（NORMAL / EMERGENCY / FALLEN / 其他原文）與各欄位（保留原字串）
inline bool parse_telemetry(const std::string& line, std::string& state, Fields& fields) {
    if (!starts_with(line, "ANG:")) return false;
    fields.clear();
    size_t i = 0, n = line.size();
    while (i < n) {
        if (!(line[i] >= 'A' && line[i] <= 'Z')) { i++; continue; }
        size_t j = i;
        while (j < n && line[j] >= 'A' && line[j] <= 'Z') j++;
        std::string key = line.substr(i, j - i);
        if (j >= n || line[j] != ':') { i = j; continue; }
        auto scan_num = [&](size_t p, size_t& e) -> bool {            // -?\d+(\.\d+)?
            size_t q = p;
            if (q < n && line[q] == '-') q++;
            size_t d0 = q;
            while (q < n && std::isdigit((unsigned char)line[q])) q++;
            if (q == d0) return false;
            if (q + 1 < n && line[q] == '.' && std::isdigit((unsigned char)line[q + 1])) {
                q++;
                while (q < n && std::isdigit((unsigned char)line[q])) q++;
            }
            e = q;
            return true;
        };
        size_t e1;
        if (!scan_num(j + 1, e1)) { i = j; continue; }
        std::string v1 = line.substr(j + 1, e1 - (j + 1)), v2;
        bool has2 = false;
        size_t e2;
        if (e1 < n && line[e1] == '/' && scan_num(e1 + 1, e2)) {
            v2 = line.substr(e1 + 1, e2 - (e1 + 1));
            has2 = true;
            e1 = e2;
        }
        if (key == "PWM") {
            fields["PWM_L"] = v1;
            fields["PWM_R"] = has2 ? v2 : "";
        } else {
            fields[key] = v1;
        }
        i = e1;
    }
    state = "UNKNOWN";
    size_t a = line.find("\xE3\x80\x90");                              // 【
    if (a != std::string::npos) {
        size_t b = line.find("\xE3\x80\x91", a + 3);                    // 】
        if (b != std::string::npos) {
            std::string s = line.substr(a + 3, b - (a + 3));
            if (s == "常態雙閉環") state = "NORMAL";
            else if (s == "大角度緊急救援") state = "EMERGENCY";
            else if (s == "跌倒保護") state = "FALLEN";
            else state = s;
        }
    }
    return true;
}

// =============================================================================
// 3. CSV 記錄（格式與 Python 版相同：UTF-8 含 BOM、CRLF 換行）
// =============================================================================
inline std::string csv_field(const std::string& s) {
    if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
    return "\"" + replace_all(s, "\"", "\"\"") + "\"";
}

// 時間戳：最多 3 位小數、至少 1 位（同 Python round(x,3) 的 repr：10.0、0.12）
inline std::string fmt_elapsed(double x) {
    std::string s = fmt("%.3f", x);
    while (s.size() > 1 && s.back() == '0' && s[s.size() - 2] != '.') s.pop_back();
    return s;
}

class TelemetryLogger {
public:
    std::string path;                                                   // 給人看的路徑（UTF-8）

    // sub = "" → logs/YYYYMMDD/；sub = "sim" → logs/sim/
    explicit TelemetryLogger(const std::string& sub = "") {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        char day[16], stamp[32];
        std::strftime(day, sizeof day, "%Y%m%d", &tm);
        std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &tm);
        fs::path dir = log_root() / (sub.empty() ? std::string(day) : sub);
        std::error_code ec;
        fs::create_directories(dir, ec);
        fs::path p = dir / (std::string("telemetry_") + stamp + ".csv");
        path = path_utf8(p);
        file_.open(p, std::ios::binary);
        if (!file_) throw std::runtime_error("無法建立記錄檔：" + path);
        file_ << "\xEF\xBB\xBF";
        std::string head = "Timestamp (s),Active_Cmd,State";
        for (auto& f : telemetry_fields()) head += "," + f;
        head += ",Raw_Message";
        head += ",IN_CMD,IN_THR,IN_STEER,IN_V,IN_W,GOAL_X,GOAL_Y\r\n";   // [v37g] 電腦端送出的輸入（放最後，舊的分析程式不受影響）
        file_ << head;
        file_.flush();
    }

    void log(double elapsed, const std::string& active_cmd, const std::string& line) {
        std::string state;
        Fields f;
        std::string row = fmt_elapsed(elapsed) + "," + csv_field(active_cmd) + ",";
        if (!parse_telemetry(line, state, f)) {                          // 系統訊息（DMP 啟動、按鍵確認、被切斷的半行…）
            row += "EVENT";
            for (size_t i = 0; i < telemetry_fields().size(); i++) row += ",";
        } else {
            row += csv_field(state);
            for (auto& name : telemetry_fields()) {
                auto it = f.find(name);
                row += "," + (it == f.end() ? std::string() : csv_field(it->second));
            }
        }
        row += "," + csv_field(line);
        row += "," + csv_field(in_cmd) + "," + in_thr + "," + in_steer + "," + in_v + "," + in_w + "," + goal_x + "," + goal_y + "\r\n";
        if (file_.is_open()) {
            file_ << row;
            file_.flush();                                              // 即時寫入，避免突然中斷掉資料
        }
    }

    void close() {
        if (file_.is_open()) file_.close();
    }

    // [v37g] 輸入訊號：之後每一列都會帶著「當下」的值，直到下一次更新
    //   IN_CMD   最近一次送給車子的指令（例如 "#D 18.0 0.00"、"#C 59.5 29.1"、"z"）
    //   IN_THR / IN_STEER  drive 的油門 / 方向（-1～1，鍵盤輸入）
    //   IN_V / IN_W        drive 送出的目標速度 / 角速度（rad/s）
    //   GOAL_X / GOAL_Y    nav 目前要去的點（cm）
    void set_cmd(const std::string& c) {
        in_cmd = c;
        while (!in_cmd.empty() && (in_cmd.back() == '\n' || in_cmd.back() == '\r')) in_cmd.pop_back();
    }
    void set_drive(double thr, double steer, double v, double w) {
        in_thr = fmt("%.2f", thr); in_steer = fmt("%.2f", steer); in_v = fmt("%.2f", v); in_w = fmt("%.3f", w);
    }
    void set_goal(double x, double y) { goal_x = fmt("%.1f", x); goal_y = fmt("%.1f", y); }
    void clear_goal() { goal_x.clear(); goal_y.clear(); }

private:
    std::ofstream file_;
    std::string in_cmd, in_thr, in_steer, in_v, in_w, goal_x, goal_y;
};

// =============================================================================
// 4. 序列埠連線
// =============================================================================
// 介面：真車（SerialLink）和模擬車（SimRobot，見 sim.h）都實作它
struct Link {
    virtual ~Link() {}
    virtual void send(const std::string& s) = 0;
    virtual bool read_line(std::string& out) = 0;   // 非阻塞；只回傳「完整的一行」（已去掉前後空白）；沒有就回傳 false
    virtual void close() = 0;
};

// 丟掉不合法的 UTF-8 位元組（同 Python decode(errors='ignore')）
inline std::string sanitize_utf8(const std::string& s) {
    std::string out;
    size_t i = 0, n = s.size();
    while (i < n) {
        unsigned char c = (unsigned char)s[i];
        size_t len = c < 0x80 ? 1 : (c >= 0xC2 && c <= 0xDF) ? 2 : (c >= 0xE0 && c <= 0xEF) ? 3 : (c >= 0xF0 && c <= 0xF4) ? 4 : 0;
        bool ok = len > 0 && i + len <= n;
        for (size_t k = 1; ok && k < len; k++) ok = ((unsigned char)s[i + k] & 0xC0) == 0x80;
        if (ok) { out.append(s, i, len); i += len; }
        else i++;
    }
    return out;
}

class SerialLink : public Link {
public:
    // 開啟後等 settle_s 秒讓 Arduino 重新啟動（開序列埠會 reset 板子），再清掉緩衝區
    SerialLink(const std::string& port, int baud, double settle_s = 2.0) {
        open_port(port, baud);
        if (std::getenv("ROBOT_NO_SETTLE")) settle_s = 0.05;           // 測試用
        sleep_s(settle_s);
        purge();
    }
    ~SerialLink() override { close(); }

    void send(const std::string& s) override {
        size_t off = 0;
        while (off < s.size()) {
#ifdef _WIN32
            DWORD w = 0;
            if (!WriteFile(h_, s.data() + off, (DWORD)(s.size() - off), &w, nullptr)) throw std::runtime_error("序列埠寫入失敗（斷線？）");
            off += w;
#else
            ssize_t w = ::write(fd_, s.data() + off, s.size() - off);
            if (w < 0) {
                if (errno == EAGAIN) { sleep_s(0.001); continue; }
                throw std::runtime_error("序列埠寫入失敗（斷線？）");
            }
            off += (size_t)w;
#endif
        }
    }

    bool read_line(std::string& out) override {
        fill();
        for (;;) {
            size_t i = buf_.find('\n');
            if (i == std::string::npos) {
                if (buf_.size() > 2048) buf_.clear();                   // 一直沒有換行（資料亂掉）就丟掉，避免無限變大
                return false;
            }
            std::string line = strip(sanitize_utf8(buf_.substr(0, i)));
            buf_.erase(0, i + 1);
            if (!line.empty()) { out = line; return true; }
        }
    }

    void close() override {
#ifdef _WIN32
        if (h_ != INVALID_HANDLE_VALUE) { CloseHandle(h_); h_ = INVALID_HANDLE_VALUE; }
#else
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
    }

private:
    std::string buf_;                                                   // 收到一半的行先放這裡，等收到換行才輸出

#ifdef _WIN32
    HANDLE h_ = INVALID_HANDLE_VALUE;

    void open_port(const std::string& port, int baud) {
        std::string name = starts_with(port, "\\\\.\\") ? port : "\\\\.\\" + port;   // COM10 以上一定要這個前綴
        h_ = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE)
            throw std::runtime_error(fmt("無法開啟 %s（Windows 錯誤碼 %lu；檢查埠號，或是否被 Arduino IDE / 其他程式占用）", port.c_str(), GetLastError()));
        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h_, &dcb)) throw std::runtime_error("GetCommState 失敗");
        dcb.BaudRate = (DWORD)baud;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;                           // 和 pyserial 一樣：開啟時拉高 DTR（Arduino UNO 會重啟）
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        if (!SetCommState(h_, &dcb)) throw std::runtime_error("SetCommState 失敗（baud 不支援？）");
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;                              // 非阻塞讀取：沒有資料就立刻回傳 0 個位元組
        to.ReadTotalTimeoutMultiplier = 0;
        to.ReadTotalTimeoutConstant = 0;
        to.WriteTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant = 1000;
        SetCommTimeouts(h_, &to);
    }
    void purge() { PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR); buf_.clear(); }
    void fill() {
        char tmp[512];
        for (;;) {
            DWORD got = 0;
            if (!ReadFile(h_, tmp, sizeof tmp, &got, nullptr)) throw std::runtime_error("序列埠讀取失敗（斷線？）");
            if (got == 0) break;
            buf_.append(tmp, got);
        }
    }
#else
    int fd_ = -1;

    static speed_t baud_const(int b) {
        switch (b) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: throw std::runtime_error("不支援的 baud");
        }
    }
    void open_port(const std::string& port, int baud) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("無法開啟 " + port);
        termios t{};
        tcgetattr(fd_, &t);
        cfmakeraw(&t);
        cfsetispeed(&t, baud_const(baud));
        cfsetospeed(&t, baud_const(baud));
        t.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd_, TCSANOW, &t);
    }
    void purge() { tcflush(fd_, TCIOFLUSH); buf_.clear(); }
    void fill() {
        char tmp[512];
        for (;;) {
            ssize_t n = ::read(fd_, tmp, sizeof tmp);
            if (n > 0) { buf_.append(tmp, (size_t)n); continue; }
            if (n == 0 || errno == EAGAIN || errno == EIO) break;
            throw std::runtime_error("序列埠讀取失敗");
        }
    }
#endif
};

}  // namespace rc
