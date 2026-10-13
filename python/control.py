# -*- coding: utf-8 -*-
"""
雙輪平衡車 鍵盤遙控 + 遙測記錄器

執行方式不變：python control.py
結構（由上到下）：
  1. 設定        —— 序列埠、記錄資料夾、心跳間隔
  2. 按鍵定義    —— 按鍵分成：長按移動 / 單擊指令 / 調校量測 / 系統
  3. 遙測解析    —— 把 Arduino 每行遙測拆成獨立欄位
  4. CSV 記錄    —— 依日期分資料夾存檔，每個遙測值一個欄位
  5. 序列埠連線  —— 開啟 / 傳送（多執行緒安全）/ 關閉
  6. 背景執行緒  —— 遙測接收、長按心跳
  7. 鍵盤控制    —— 按下 / 放開處理
  8. 主程式
"""

import csv
import os
import re
import threading
import time
from datetime import datetime

import serial
# pynput（鍵盤監聽）只有遙控時才需要，改成在用到時才載入，讓 nav.py 可以共用這個檔案的其他部分


# =============================================================================
# 1. 設定
# =============================================================================
COM_PORT = 'COM3'
BAUD_RATE = 9600
HEARTBEAT_INTERVAL = 0.1   # 長按時每 0.1 s 重送一次按鍵（Arduino 端 300 ms 沒收到會自動煞停）

# 記錄檔位置：controller/logs/YYYYMMDD/telemetry_YYYYMMDD_HHMMSS.csv
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.join(BASE_DIR, 'logs')


# =============================================================================
# 2. 按鍵定義（依用途分類）
# =============================================================================
# 長按類：按住期間持續送出（心跳），放開時自動送 's' 煞停
HOLD_KEYS = {
    'w': '前進',
    'x': '後退',
    'a': '左旋',
    'd': '右旋',
}

# 單擊類：按一下送一次
COMMAND_KEYS = {
    's': '緊急煞車',
    'c': '自動旋轉回正',
    'z': '虛擬座標歸零（quad_v26 起）',
}

# 調校 / 量測類：按一下送一次
TUNING_KEYS = {
    'o': '速度外環 開/關',
    '[': 'setpoint -0.1°',
    ']': 'setpoint +0.1°',
    'k': '馬達強制斷電 開/關',
    'g': 'Kd -10（直立環阻尼）',
    'h': 'Kd +10（直立環阻尼）',
    'j': 'Kp_speed -0.5（速度外環）',
    'l': 'Kp_speed +0.5（速度外環）',
}

STOP_CHAR = 's'


# =============================================================================
# 3. 遙測解析
# =============================================================================
# 遙測欄位（與 repo.ino 的輸出順序相同）
#   ANG 傾角(deg)  SA 速度外環補償角(deg)  B 直立環輸出  T 轉向差速
#   PWM 左/右實際 PWM  YR 航向角速度  Y 航向  TY 目標航向  G 轉向權限
#   SP setpoint(deg)  OL 速度外環開關  DX 起步後位移(ticks)  V 實際速度  TV 目標速度
#   BS 起步推力 / 自轉定點補償(deg，v20 起；舊韌體沒有這欄會留空)
#   EL / ER 倒地或斷電時的左右輪累計 tick（v25 起，編碼器方向測試用）
#   PX / PY 虛擬座標(cm，quad_v26 起)：PX 往前為正、PY 往右為正
#   NV / NR 導航模式（1 轉向、2 直走）/ 剩餘量（轉向：角度誤差°；直走：剩餘 cm），nav_v27 起
TELEMETRY_FIELDS = ['ANG', 'SA', 'B', 'T', 'PWM_L', 'PWM_R', 'YR', 'Y', 'TY',
                    'G', 'SP', 'OL', 'DX', 'V', 'TV', 'BS', 'EL', 'ER', 'PX', 'PY', 'NV', 'NR']

STATE_MAP = {
    '常態雙閉環': 'NORMAL',
    '大角度緊急救援': 'EMERGENCY',
    '跌倒保護': 'FALLEN',
}

_PAIR_RE = re.compile(r'([A-Z]+):(-?\d+(?:\.\d+)?)(?:/(-?\d+(?:\.\d+)?))?')
_STATE_RE = re.compile(r'【(.*?)】')


def parse_telemetry(line):
    """回傳 (state, fields)；不是遙測行時回傳 (None, None)"""
    if not line.startswith('ANG:'):
        return None, None
    fields = {}
    for key, v1, v2 in _PAIR_RE.findall(line):
        if key == 'PWM':
            fields['PWM_L'] = v1
            fields['PWM_R'] = v2
        else:
            fields[key] = v1
    m = _STATE_RE.search(line)
    state = STATE_MAP.get(m.group(1), m.group(1)) if m else 'UNKNOWN'
    return state, fields


# =============================================================================
# 4. CSV 記錄
# =============================================================================
class TelemetryLogger:
    HEADER = ['Timestamp (s)', 'Active_Cmd', 'State'] + TELEMETRY_FIELDS + ['Raw_Message']

    def __init__(self):
        now = datetime.now()
        day_dir = os.path.join(LOG_ROOT, now.strftime('%Y%m%d'))
        os.makedirs(day_dir, exist_ok=True)
        self.path = os.path.join(day_dir, f"telemetry_{now.strftime('%Y%m%d_%H%M%S')}.csv")
        self._file = open(self.path, mode='w', newline='', encoding='utf-8-sig')
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._file.flush()
        self._lock = threading.Lock()

    def log(self, elapsed, active_cmd, line):
        state, fields = parse_telemetry(line)
        if state is None:  # 系統訊息（DMP 啟動、按鍵確認、被切斷的半行…）
            row = [elapsed, active_cmd, 'EVENT'] + [''] * len(TELEMETRY_FIELDS) + [line]
        else:
            row = [elapsed, active_cmd, state] + [fields.get(f, '') for f in TELEMETRY_FIELDS] + [line]
        with self._lock:
            self._writer.writerow(row)
            self._file.flush()  # 即時寫入，避免突然中斷掉資料

    def close(self):
        with self._lock:
            self._file.close()


# =============================================================================
# 5. 序列埠連線
# =============================================================================
class RobotLink:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # 等待 Arduino 重新啟動就緒
        self.ser.reset_input_buffer()
        self._buf = bytearray()  # 收到一半的行先放這裡，等收到換行才輸出
        self._lock = threading.Lock()  # 心跳執行緒與鍵盤執行緒會同時寫入

    def send(self, ch):
        with self._lock:
            self.ser.write(ch.encode())

    def read_line(self):
        """只回傳「完整的一行」。藍牙 / 序列埠常把一行切成兩段送到，
        以前直接用 readline() 會把半行當成一行（欄位被切斷、NAV:DONE 可能漏掉）"""
        n = self.ser.in_waiting
        if n > 0:
            self._buf += self.ser.read(n)
        i = self._buf.find(b'\n')
        if i < 0:
            if len(self._buf) > 2048:   # 一直沒有換行（資料亂掉）就丟掉，避免無限變大
                self._buf.clear()
            return ''
        line = bytes(self._buf[:i])
        del self._buf[:i + 1]
        return line.decode('utf-8', errors='ignore').strip()

    def close(self):
        with self._lock:
            self.ser.close()


# =============================================================================
# 6. 背景執行緒
# =============================================================================
class Session:
    """集中保存執行狀態，讓各執行緒共用"""

    def __init__(self, link, logger):
        self.link = link
        self.logger = logger
        self.active_key = None
        self.running = True
        self.start_time = time.time()

    def elapsed(self):
        return round(time.time() - self.start_time, 3)

    def active_cmd(self):
        return self.active_key if self.active_key else 'IDLE'


def telemetry_reader(session):
    """接收 Arduino 遙測 → 顯示 + 寫入 CSV"""
    while session.running:
        try:
            line = session.link.read_line()
        except (serial.SerialException, OSError):
            break  # 序列埠斷線
        if line:
            elapsed = session.elapsed()
            print(f"[{elapsed}s][小車] {line}")
            try:
                session.logger.log(elapsed, session.active_cmd(), line)
            except ValueError:
                break  # 記錄檔已關閉（程式結束中）
        else:
            time.sleep(0.01)


def heartbeat_sender(session):
    """長按期間持續重送按鍵，避免 Arduino 端逾時煞停"""
    while session.running:
        key = session.active_key
        if key in HOLD_KEYS:
            try:
                session.link.send(key)
            except (serial.SerialException, OSError):
                break
        time.sleep(HEARTBEAT_INTERVAL)


# =============================================================================
# 7. 鍵盤控制
# =============================================================================
def make_handlers(session):
    from pynput import keyboard

    def on_press(key):
        try:
            k = key.char.lower()
        except AttributeError:
            return  # 特殊鍵（方向鍵、Shift…）不處理

        if k in HOLD_KEYS:
            if session.active_key != k:
                session.active_key = k
                session.link.send(k)
                print(f"--> [長按] {k.upper()} {HOLD_KEYS[k]}")
        elif k == STOP_CHAR:
            session.active_key = STOP_CHAR
            session.link.send(STOP_CHAR)
            print(f"--> [指令] S {COMMAND_KEYS['s']}")
        elif k in COMMAND_KEYS:
            session.link.send(k)
            print(f"--> [指令] {k.upper()} {COMMAND_KEYS[k]}")
        elif k in TUNING_KEYS:
            session.link.send(k)
            print(f"--> [調校] {k} {TUNING_KEYS[k]}")

    def on_release(key):
        if key == keyboard.Key.esc:
            print("\n正在停止小車並關閉記錄檔...")
            session.link.send(STOP_CHAR)
            time.sleep(0.1)
            return False  # 結束鍵盤監聽

        try:
            k = key.char.lower()
        except AttributeError:
            return
        if k == session.active_key:
            session.active_key = None
            session.link.send(STOP_CHAR)
            print(f"--> [放開] {k.upper()} -> 送出煞停 (S)")

    return on_press, on_release


def print_banner(logger):
    print("=" * 65)
    print("小車連線成功！鍵盤遙控 + CSV 遙測記錄")
    print(f">> 記錄檔：{logger.path}")
    print("  長按：" + " | ".join(f"[{k.upper()}] {v}" for k, v in HOLD_KEYS.items()))
    print("  單擊：" + " | ".join(f"[{k.upper()}] {v}" for k, v in COMMAND_KEYS.items()))
    print("  調校：" + " | ".join(f"[{k.upper()}] {v}" for k, v in TUNING_KEYS.items()))
    print("  系統：[ESC] 安全退出並儲存")
    print("=" * 65)


# =============================================================================
# 8. 主程式
# =============================================================================
def main():
    try:
        link = RobotLink(COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"無法開啟序列埠 {COM_PORT}，錯誤：{e}")
        return

    logger = TelemetryLogger()
    session = Session(link, logger)
    print_banner(logger)

    threading.Thread(target=telemetry_reader, args=(session,), daemon=True).start()
    threading.Thread(target=heartbeat_sender, args=(session,), daemon=True).start()

    on_press, on_release = make_handlers(session)
    try:
        from pynput import keyboard
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    finally:
        session.running = False
        time.sleep(0.2)  # 等背景執行緒結束最後一次讀寫
        link.close()
        logger.close()
        print(f">> 記錄完成！檔案已儲存至：{logger.path}")


if __name__ == '__main__':
    main()
