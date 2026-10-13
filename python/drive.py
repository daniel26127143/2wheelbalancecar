# -*- coding: utf-8 -*-
"""
drive.py — 像遙控汽車一樣開小車：前進速度與轉向都是連續（無段）可調

需要韌體 v32（repo.ino 的 #D 指令）。

  py drive.py                 鍵盤（方向盤式：按住轉向越轉越大，放開自動回中）
  py drive.py --pad           USB 手把（左搖桿：上下 = 油門，左右 = 方向；需要 pip install pygame）
  py drive.py --port COM5     指定序列埠（預設 COM3）
  py drive.py --dry-run       不連線，只印出每個畫面會送出的指令（確認按鍵與數值）

鍵盤
  W / ↑   油門前進（按住越按越快，放開慢慢減速）      X / ↓   後退
  A / ←   向左轉（按住越轉越大，放開自動回中）         D / →   向右轉
  S / 空白  立刻停（油門與方向歸零）                    C       方向回中
  Z       座標歸零                                      Q / Esc 結束（會先停車）
  J / L   速度外環 Kp_speed -0.5 / +0.5     G / H   直立環 Kd -10 / +10   （韌體 ENABLE_TUNING 1 才有效，新值會印在畫面上）
  車子不動（油門 0）時按 A / D = 原地自轉

怎麼轉：方向值 steer（-1～1）代表「曲率」，角速度 w = steer ÷ 最小半徑 × 前進速度。
所以車速越慢轉得越小，車速越快轉得越大（和真的遙控車一樣）；--min-radius 是方向打到底時的轉彎半徑（cm）。
"""

import argparse
import sys
import threading
import time

# 目標速度 1 單位 ≈ 實際幾 cm/s（直走速度 5.8 約 5.3 cm/s，圓弧速度 8 約 7～8 cm/s）
CM_PER_S_PER_UNIT = 0.9

DEFAULTS = dict(
    vmax=18.0,         # 前進最高目標速度（韌體上限 20）；10/13 autotest：W 13 / 18 各 3～4 次都沒倒，18 時 4 次平均 16.5 cm/s、次間只差 1 cm/s；選 18 是使用者決定
    vback=18.0,        # 後退最高目標速度（韌體上限 20）；10/13 autotest 掃 6～20 都沒倒：9 以上才穩、20 時偏角上限已用到 18%，選 18 是使用者決定
    vmin=17.0,         # 前進最低目標速度（和 vmax 只差 1，一推油門就直接跑；舊值 6.5 是 vmax 7.2 時代：目標太低車子會來回晃、走走停停）
    vmin_back=17.0,    # 後退最低目標速度；10/13 實測目標太低（3.5～5）速度環來回晃、幾乎退不動，所以和 vback 只差 1
    min_radius=30.0,   # 方向打到底的轉彎半徑 cm（實車只測過 30）
    wmax=0.5,          # 邊走邊轉的角速度上限 rad/s（韌體上限 0.6）
    w_low=0.12,        # 低速時方向有打的最小角速度 rad/s（否則車速慢時完全轉不動）
    spin_rate=0.8,     # 油門 0 時，方向打到底的原地自轉角速度 rad/s（韌體上限 1.0）
    hz=10.0,           # 送指令頻率；韌體 300 ms 沒收到就自動煞車
)


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def approach(cur, target, step):
    """cur 每次最多移動 step 靠近 target"""
    if cur < target:
        return min(cur + step, target)
    return max(cur - step, target)


class DriveModel:
    """把油門 / 方向輸入變成 (v, w)。純計算，不碰序列埠，方便測試。

    thr：油門位置 -1（後退到底）～ 1（前進到底）；steer：方向 -1（左到底）～ 1（右到底）
    """

    # 鍵盤的斜率（單位/秒）
    THR_UP = 2.0        # 按住油門：約 0.5 s 從 0 到滿（太慢會長時間停在低速區，車子卡住頓挫）
    THR_RELEASE = 1.8   # 放開油門：約 0.55 s 回到 0（韌體還會再緩衝）
    STEER_IN = 1.6      # 按住方向：約 0.6 s 從 0 打到底
    STEER_RELEASE = 3.0 # 放開方向：約 0.3 s 回中

    def __init__(self, **kw):
        p = dict(DEFAULTS)
        p.update(kw)
        self.p = p
        self.thr = 0.0
        self.steer = 0.0
        if 'thr_up' in kw and kw['thr_up']:
            self.THR_UP = kw['thr_up']

    def stop(self):
        self.thr = 0.0
        self.steer = 0.0

    def center(self):
        self.steer = 0.0

    def update_keys(self, dt, thr_dir, steer_dir):
        """鍵盤：thr_dir / steer_dir 為 -1、0、1（目前按住的方向）"""
        if thr_dir != 0:
            self.thr = approach(self.thr, float(thr_dir), self.THR_UP * dt)
        else:
            self.thr = approach(self.thr, 0.0, self.THR_RELEASE * dt)
        if steer_dir != 0:
            self.steer = approach(self.steer, float(steer_dir), self.STEER_IN * dt)
        else:
            self.steer = approach(self.steer, 0.0, self.STEER_RELEASE * dt)

    def set_analog(self, thr, steer):
        """手把：直接給 -1～1"""
        self.thr = clamp(thr, -1.0, 1.0)
        self.steer = clamp(steer, -1.0, 1.0)

    def output(self):
        """回傳 (v, w)：v = 前進目標速度（韌體單位，前進為正），w = 角速度 rad/s（右轉為正）"""
        p = self.p
        a = abs(self.thr)
        if a <= 0.05:
            v = 0.0
        elif self.thr > 0:                     # 油門一推就從 vmin 起跳（低於 3 的目標速度車子推不動）
            v = p['vmin'] + (p['vmax'] - p['vmin']) * a
        else:
            v = -(p['vmin_back'] + (p['vback'] - p['vmin_back']) * a)
        if v == 0.0:                           # 沒有前進：原地自轉（韌體會當成 A / D）
            w = self.steer * p['spin_rate']
            return v, (w if abs(w) >= 0.1 else 0.0)
        # 曲率 = 方向值 ÷ 最小半徑；角速度 = 曲率 × 速度（倒車時速度為負，轉向方向自然反過來）
        v_cm = v * CM_PER_S_PER_UNIT
        w = (self.steer / p['min_radius']) * v_cm
        floor = p['w_low'] * abs(self.steer)   # 低速也要有一點轉向
        if abs(w) < floor:
            w = floor * (1 if self.steer > 0 else -1) * (1 if v > 0 else -1)
        return v, clamp(w, -p['wmax'], p['wmax'])


def format_cmd(v, w):
    return f'#D {v:.1f} {w:.2f}\n'


# ---------------------------------------------------------------------------
# 以下是需要硬體 / 鍵盤的部分
# ---------------------------------------------------------------------------
class Inputs:
    """目前按住的鍵（pynput 在背景執行緒更新）"""

    def __init__(self):
        self.down = set()
        self.events = []        # 單次事件：'stop' 'center' 'zero' 'quit'
        self.lock = threading.Lock()

    def thr_dir(self):
        with self.lock:
            return (1 if self.down & {'w', 'up'} else 0) - (1 if self.down & {'x', 'down'} else 0)

    def steer_dir(self):
        with self.lock:
            return (1 if self.down & {'d', 'right'} else 0) - (1 if self.down & {'a', 'left'} else 0)

    def pop_events(self):
        with self.lock:
            ev, self.events = self.events, []
        return ev


KEY_NAMES = {'up': 'up', 'down': 'down', 'left': 'left', 'right': 'right', 'space': 'space', 'esc': 'esc'}


def start_keyboard(inputs):
    from pynput import keyboard

    def name_of(key):
        try:
            return key.char.lower()
        except AttributeError:
            return KEY_NAMES.get(getattr(key, 'name', ''), None)

    def on_press(key):
        n = name_of(key)
        if n is None:
            return
        with inputs.lock:
            if n in ('w', 'x', 'a', 'd', 'up', 'down', 'left', 'right'):
                inputs.down.add(n)
            elif n in ('s', 'space'):
                inputs.events.append('stop')
            elif n == 'c':
                inputs.events.append('center')
            elif n == 'z':
                inputs.events.append('zero')
            elif n in ('j', 'l', 'g', 'h'):
                inputs.events.append('tune:' + n)   # 現場調參：j/l = Kp_speed -/+0.5，g/h = Kd -/+10（韌體 ENABLE_TUNING 為 1 才有效）
            elif n in ('q', 'esc'):
                inputs.events.append('quit')

    def on_release(key):
        n = name_of(key)
        with inputs.lock:
            inputs.down.discard(n)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    return listener


class Pad:
    """USB 手把（pygame）。左搖桿：Y = 油門（上 = 前進），X = 方向"""

    DEAD = 0.10

    def __init__(self):
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError('找不到手把')
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f'>> 手把：{self.js.get_name()}（左搖桿；按鈕 0 = 停車，按鈕 7 / Start = 結束）')

    def _shape(self, a):
        if abs(a) < self.DEAD:
            return 0.0
        a = (abs(a) - self.DEAD) / (1.0 - self.DEAD) * (1 if a > 0 else -1)
        return a * abs(a) * 0.5 + a * 0.5      # 小幅度細緻、大幅度還是能打滿

    def read(self):
        self.pg.event.pump()
        thr = -self._shape(self.js.get_axis(1))
        steer = self._shape(self.js.get_axis(0))
        ev = []
        if self.js.get_button(0):
            ev.append('stop')
        if self.js.get_numbuttons() > 7 and self.js.get_button(7):
            ev.append('quit')
        return thr, steer, ev


def status_line(model, v, w, tel):
    s = f'油門 {model.thr:+.2f}  方向 {model.steer:+.2f} -> v {v:+.1f}  w {w:+.2f} rad/s'
    if tel:
        s += f"  | {tel.get('state', '?')} ANG {tel.get('ANG', '?')} V {tel.get('V', '?')} PX {tel.get('PX', '?')} PY {tel.get('PY', '?')}"
    return s


def main():
    ap = argparse.ArgumentParser(description='像遙控汽車一樣開小車（需要韌體 v32 的 #D 指令）')
    ap.add_argument('--port', default=None, help='序列埠（預設用 control.py 的 COM3）')
    ap.add_argument('--baud', type=int, default=None)
    ap.add_argument('--pad', action='store_true', help='用 USB 手把（需要 pygame）')
    ap.add_argument('--vmax', type=float, default=DEFAULTS['vmax'], help='前進最高目標速度（預設 18，上限 20）')
    ap.add_argument('--vback', type=float, default=DEFAULTS['vback'], help='後退最高目標速度（預設 18，上限 20）')
    ap.add_argument('--vmin', type=float, default=DEFAULTS['vmin'], help='前進最低目標速度（預設 17）')
    ap.add_argument('--vmin-back', type=float, default=DEFAULTS['vmin_back'], help='後退最低目標速度（預設 17）')
    ap.add_argument('--thr-up', type=float, default=None, help='按住油門的加速斜率（單位/秒，預設 2.0；越大越快到最高速）')
    ap.add_argument('--min-radius', type=float, default=DEFAULTS['min_radius'], help='方向打到底的轉彎半徑 cm（預設 30）')
    ap.add_argument('--wmax', type=float, default=DEFAULTS['wmax'], help='邊走邊轉的角速度上限 rad/s（預設 0.5，上限 0.6）')
    ap.add_argument('--hz', type=float, default=DEFAULTS['hz'], help='送指令頻率（預設 10）')
    ap.add_argument('--dry-run', action='store_true', help='不連線，只印出會送出的指令')
    a = ap.parse_args()

    if not (0.5 <= a.vmax <= 20.0) or not (0.5 <= a.vback <= 20.0) or not (0.1 <= a.wmax <= 0.6):
        raise SystemExit('數值超出範圍：--vmax 0.5～20、--vback 0.5～20、--wmax 0.1～0.6（韌體也會限制）')
    if a.min_radius < 10:
        raise SystemExit('--min-radius 至少 10 cm')

    if a.vmin > a.vmax or a.vmin_back > a.vback:
        raise SystemExit('最低速度不能高於最高速度（--vmin <= --vmax，--vmin-back <= --vback）')
    model = DriveModel(vmax=a.vmax, vback=a.vback, vmin=a.vmin, vmin_back=a.vmin_back, thr_up=a.thr_up,
                       min_radius=a.min_radius, wmax=a.wmax, hz=a.hz)
    inputs = Inputs()
    pad = Pad() if a.pad else None
    if not pad:
        start_keyboard(inputs)

    link = logger = session = None
    tel = {}
    if not a.dry_run:
        import control
        port = a.port or control.COM_PORT
        baud = a.baud or control.BAUD_RATE
        try:
            link = control.RobotLink(port, baud)
        except Exception as e:
            raise SystemExit(f'無法開啟序列埠 {port}：{e}')
        logger = control.TelemetryLogger()
        session = control.Session(link, logger)

        def reader():
            while session.running:
                try:
                    line = link.read_line()
                except Exception:
                    break
                if not line:
                    time.sleep(0.01)
                    continue
                st, fields = control.parse_telemetry(line)
                if st is not None:
                    tel.clear()
                    tel.update(fields)
                    tel['state'] = st
                elif line.startswith('NAV:') or line.startswith('>>'):
                    print('\n' + line)
                try:
                    logger.log(session.elapsed(), session.active_cmd(), line)
                except ValueError:
                    break

        threading.Thread(target=reader, daemon=True).start()
        print(f'>> 記錄檔：{logger.path}')

    print('=' * 66)
    print(' 遙控模式：W/↑ 前進  X/↓ 後退  A/← 左轉  D/→ 右轉  S/空白 停  C 回中  Z 座標歸零  Q/Esc 結束'
          if not pad else ' 手把模式：左搖桿控制，按鈕 0 停車，Start 結束')
    print(' 車子站穩（遙測顯示【常態雙閉環】）後再開始；倒地時指令會被忽略')
    print('=' * 66)

    dt = 1.0 / a.hz
    nxt = time.time()
    try:
        while True:
            for ev in inputs.pop_events():
                if ev == 'quit':
                    raise KeyboardInterrupt
                if ev == 'stop':
                    model.stop()
                elif ev == 'center':
                    model.center()
                elif ev == 'zero' and link:
                    link.send('z')
                elif ev.startswith('tune:') and link:
                    link.send(ev[5:])
            if pad:
                thr, steer, evs = pad.read()
                model.set_analog(thr, steer)
                if 'quit' in evs:
                    raise KeyboardInterrupt
                if 'stop' in evs:
                    model.stop()
            else:
                model.update_keys(dt, inputs.thr_dir(), inputs.steer_dir())
            v, w = model.output()
            cmd = format_cmd(v, w)
            if link:
                link.send(cmd)
                session.active_key = f'D{v:+.1f}/{w:+.2f}' if (v or w) else None
            if a.dry_run:
                print(cmd.strip(), '   ', status_line(model, v, w, None))
            else:
                sys.stdout.write('\r' + status_line(model, v, w, tel).ljust(110))
                sys.stdout.flush()
            nxt += dt
            time.sleep(max(0.0, nxt - time.time()))
    except KeyboardInterrupt:
        pass
    finally:
        print('\n>> 停車並結束...')
        if link:
            for _ in range(3):
                try:
                    link.send('#D 0 0\n')
                    time.sleep(0.05)
                except Exception:
                    break
            try:
                link.send('s')
            except Exception:
                pass
            session.running = False
            time.sleep(0.2)
            link.close()
            logger.close()
            print(f'>> 記錄檔：{logger.path}')


if __name__ == '__main__':
    main()
