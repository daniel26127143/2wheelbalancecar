# -*- coding: utf-8 -*-
"""
座標導航腳本執行器（搭配韌體 nav_v27）

用法（在 controller 資料夾執行）：
  python nav.py "0,0 1,1 1,-1 -1,-1"             依序走到這些座標（預設座標 1 = 10 cm）
  python nav.py route.txt                          路線寫在檔案裡（每行一個點：x,y；# 開頭是註解）
  python nav.py "0,0 1,1" --scale 5                座標 1 = 5 cm
  python nav.py "0,0 1,1 1,-1 -1,-1" --dry-run     只印出每一段要轉幾度、走幾公分，不連線
  python nav.py "0,0 1,1 1,-1 -1,-1" --sim         用電腦模擬的車跑完整流程（不需要實車）
  python nav.py ... --port COM5                    指定序列埠（預設用 control.py 的 COM_PORT）
  python nav.py ... --no-zero                      不先把座標歸零（沿用車上目前的座標）
  python nav.py "T90 T180 T-90 T0" --steps         單一動作測試：依序轉到 90°、180°、-90°、0°
  python nav.py "M20 M-20" --steps                 單一動作測試：往前 20 cm 再退回（M20@45 = 沿 45° 走 20 cm）
  python nav.py "0,0 5,3 8,-2 2,4" --smooth        平滑路線：點與點之間走圓弧、不停車（--dry-run 先看每段半徑；--speed 7 加快）
  python nav.py x --circle 15                      邊走邊轉繞圓：半徑 15 cm、向左一圈（--right 向右、--laps 2 兩圈）
  python nav.py x --circle 15 --laps 0             一直繞，按 Enter 才沿著圓弧慢慢停下（--stop-after 20 = 20 秒後自動停，測試用）
  python nav.py "C47@-15" --steps                  同上的底層指令：C<弧長cm>@<半徑cm>，半徑負 = 左轉、正 = 右轉（這裡是半圈）
  python nav.py x --circle 15 --speed 7            繞圓加快（前進目標速度 7；預設 5.8，上限 12）

座標系（與韌體虛擬座標相同）：
  開始時會先送 z，把「目前位置」設為原點、「目前車頭方向」設為 X 軸
  X = 往前為正，Y = 往右為正；角度 0° = 起點車頭方向，往右轉為正

每一段的流程：
  1. 用目前位置算出目標方向與距離
  2. 送 #T <角度>，等 NAV:DONE（原地轉向）
  3. 再用目前位置重算一次，送 #M <距離> <角度>，等 NAV:DONE（直走）
  → 每一段都用「實際位置」重算，上一段停歪或自轉時漂移，下一段會自動修正

安全：
  - 執行中按 Ctrl+C 立即中止（送 #S 和 s）
  - 韌體端：導航中按 W/X/A/D/S/C、倒地、緊急救援、超過 1 s 沒收到電腦訊號都會自動中止
  - 每段有逾時（電腦端：轉向 12 s、直走 20 s；韌體端：轉向 6 s、直走 4 s + 每公分 0.36 s）
  - 到達後誤差 > 3 cm 會自動再修正一次（目標在背後就倒車）；轉完方向偏 > 10° 會再轉一次
模擬（--sim）：車上的導航邏輯是 nav_v27/nav.h 的 Python 逐行對照版（nav_v27/sim/test_nav_port.py 驗證兩邊一致），
  車體是簡化模型（延遲、靜摩擦、平衡晃動、自轉漂移都是隨機的），用來確認流程，不代表實車表現
記錄：和 control.py 一樣存成 CSV（實車：logs/日期/；模擬：logs/sim/），可以直接用 analyze.py 分析
"""

import argparse
import math
import os
import queue
import random
import sys
import threading
import time
from datetime import datetime

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# =============================================================================
# 1. 設定
# =============================================================================
DEFAULT_SCALE_CM = 10.0      # 座標 1 = 幾公分
SKIP_DIST_CM = 1.0           # 離目標小於這個距離就不走
TURN_TIMEOUT_S = 12.0
MOVE_TIMEOUT_S = 20.0         # 直走等待回應的下限；實際用 move_timeout(距離)（韌體逾時 = 4 s + 0.36 s/cm）
HEARTBEAT_S = 0.2            # 每 0.2 s 送 '.'（韌體超過 1 s 沒收到會中止導航）
SETTLE_AFTER_DONE_S = 0.4    # 每個動作完成後等一下再送下一個
RETRY_CM = 3.0               # 到達後誤差超過這個值就再修正一次
RETRIES = 1                  # 每個點最多修正幾次
MAX_TURN_STEP_DEG = 50.0      # 大於這個角度的轉彎拆成多次（每次 ≤45°，偏移比一次轉 90° 小很多）
TURN_DRIFT_ABORT_CM = 10.0    # 一次轉彎位置被帶偏超過這個距離就中止（車子可能已經失控）
SMOOTH_MIN_R_CM = 20.0        # 平滑路線：弧線半徑小於這個值就改用「原地轉向 + 直走」（半徑 30 cm 實車測過；更小的沒測）
SMOOTH_SWITCH_CM = 8.0        # 平滑路線：這一段剩下多少公分時，就接上下一段（不停車）
STRAIGHT_R_CM = 20000.0       # 幾乎直線時用的半徑（曲率接近 0，但不為 0）
FOREVER_ARC_CM = 1000.0       # 「一直繞到按 Enter」用的弧長（約 10 圈半徑 15 cm 的圓，韌體逾時 = 4 s + 0.36 s/cm ≈ 6 分鐘）
MAX_ARC_CM = 1000.0           # 單一個弧線指令的弧長上限（約 5 圈半徑 30 cm 的圓；韌體逾時 = 4 s + 0.36 s/cm）
MAX_LEG_CM = 150.0            # 單段直走超過這個距離就中止（位置估計大概壞了）
SETTLE_V = 4.0                # 轉完後要等 |V| 小於這個值才送下一個指令
REALIGN_DEG = 10.0           # 轉完後重算的方向和車頭差超過這個角度，就再轉一次（自轉時可能被帶偏）


# =============================================================================
# 2. 路線
# =============================================================================
def parse_route(text_or_file):
    """回傳 [(x, y, 倒車?), ...]（座標單位，還沒乘 scale）；座標後面加 b 代表倒車過去，例如 "1,1 0,0b" """
    if os.path.isfile(text_or_file):
        with open(text_or_file, encoding='utf-8') as fh:
            items = []
            for line in fh:
                line = line.split('#')[0].strip()
                if line:
                    items.append(line)
    else:
        items = text_or_file.replace(';', ' ').split()
    pts = []
    for it in items:
        it = it.strip().strip('()')
        if it[:1] in 'hH' and len(it) > 1:      # h<角度> = 在目前位置轉到指定的絕對方向（0 = 起點車頭，右轉為正）
            pts.append((None, float(it[1:]), False))
            continue
        back = it.lower().endswith('b')     # 座標後面加 b = 這一段用倒車過去（不先轉頭）
        if back:
            it = it[:-1]
        parts = [p for p in it.replace(',', ' ').split() if p]
        if len(parts) != 2:
            raise ValueError(f'看不懂的座標：{it!r}（格式：x,y 或 x,yb）')
        pts.append((float(parts[0]), float(parts[1]), back))
    if not pts:
        raise ValueError('路線是空的')
    return pts


def plan(pose_cm, target_cm):
    """目前位置 → 目標：回傳 (方向°, 距離 cm)；方向 0° = X 軸，往右（+Y）為正"""
    dx = target_cm[0] - pose_cm[0]
    dy = target_cm[1] - pose_cm[1]
    return math.degrees(math.atan2(dy, dx)), math.hypot(dx, dy)


def dry_run(route, scale):
    print(f'座標 1 = {scale:g} cm；起點 = 第一個點（實際執行時會先把目前位置歸零）')
    pose = (route[0][0] * scale, route[0][1] * scale)
    heading = 0.0
    total = 0.0
    print(' 段  從 → 到                  轉到(°)  需要轉(°)  直走(cm)')
    for i, (x, y, back) in enumerate(route[1:], 1):
        if x is None:
            turn = (y - heading + 180) % 360 - 180
            print(f' {i:2d}  在 ({pose[0]/scale:g},{pose[1]/scale:g}) 轉到 {y:g}°'.ljust(30) + f'{y:8.1f} {turn:9.1f}')
            heading = y
            continue
        tgt = (x * scale, y * scale)
        b, d = plan(pose, tgt)
        if d < SKIP_DIST_CM:
            print(f' {i:2d}  已在目標上，略過')
            continue
        hd = (b + 180 + 180) % 360 - 180 if back else b       # 倒車：車頭朝反方向
        turn = (hd - heading + 180) % 360 - 180
        print(f' {i:2d}  ({pose[0]/scale:g},{pose[1]/scale:g}) → ({x:g},{y:g})'.ljust(30)
              + f'{hd:8.1f} {turn:9.1f} {-d if back else d:10.1f}' + ('  倒車' if back else ''))
        pose, heading, total = tgt, hd, total + d
    print(f'總路徑 {total:.1f} cm')


# =============================================================================
# 3. 模擬車（--sim）：nav_v27/nav.h 的 Python 逐行對照版 + 簡化車體模型
#    （nav_v27/sim/test_nav_port.py 會把這裡和 nav.h 的輸出逐步比對，確認兩邊一致）
# =============================================================================
DEG = math.pi / 180.0
# 與 nav.h 的 #define 相同
NAV = dict(TURN_KP=1.5, TURN_RATE_MAX=0.8, TURN_RATE_MIN=0.25, TURN_TOL=2.5 * 0.0174533,
           TURN_STILL=0.15, TURN_SETTLE=30, TURN_TIMEOUT=600, TURN_LEAD=0.3,
           TV_MAX=5.8, TV_MAX_BACK=5.0, SLOW_CM=20.0, TV_MIN=3.0, TV_MIN_BACK=2.5, POS_TOL_CM=2.0, SETTLE_TOL_CM=3.0, STILL_V=6.0, V_FILT=0.8,
           SETTLE_UPD=12, SETTLE_MAX=75, MOVE_TIME_BASE=100, MOVE_TIME_PER_CM=9, DT_V=0.0405, FF_FILT=0.7, TV_ARC_CAP=12.0)
NAV_IDLE, NAV_TURN, NAV_MOVE, NAV_STOP = 0, 1, 2, 3
RUNNING, DONE, TIMEOUT = 0, 1, 2


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class NavCore:
    """nav.h 的逐行對照版（角度都是「相對座標原點」的 rad，位置 ticks）"""

    def __init__(self, ticks_per_cm=30.0):
        self.tpc = ticks_per_cm
        self.stop()
        self.remaining = 0.0
        self.target_heading = self.move_heading = 0.0
        self.curv = self.h0 = self.px = self.py = self.s_arc = self.yaw_ff = 0.0
        self.tv_fwd = NAV['TV_MAX']

    def stop(self):
        self.mode = NAV_IDLE
        self.timer = self.settle = self.timeout = 0
        self.curv = self.yaw_ff = 0.0

    def start_turn(self, target_rel, yaw_rel=0.0):
        self.mode = NAV_TURN
        self.timer = self.settle = 0
        self.curv = self.yaw_ff = 0.0
        self.target_heading = wrap(target_rel)
        self.remaining = wrap(self.target_heading - yaw_rel)
        self.timeout = NAV['TURN_TIMEOUT']

    def start_move(self, dist, heading_rel, x, y):
        self.mode = NAV_MOVE
        self.timer = self.settle = 0
        self.move_heading = wrap(heading_rel)
        self.move_dist = dist
        self.sx, self.sy = x, y
        self.curv = self.yaw_ff = self.s_arc = 0.0
        self.tv_fwd = NAV['TV_MAX']
        self.h0 = self.move_heading
        self.px, self.py = x, y
        self.v_avg = 0.0
        self.remaining = dist
        cm = abs(dist) / self.tpc
        self.timeout = NAV['MOVE_TIME_BASE'] + int(cm * NAV['MOVE_TIME_PER_CM'])

    def start_arc(self, dist, curv, yaw_rel, x, y, tv=0.0):
        """#C：dist = 弧長 (ticks)；curv = 曲率 (rad/tick，正 = 右轉)；yaw_rel = 目前方向 (rad)；tv = 前進目標速度（0 = 預設）"""
        chain = self.mode == NAV_MOVE and self.curv != 0.0      # 弧線進行中又收到 #C：不停車、方向不歸零，直接接成新的弧線
        mh = self.move_heading if chain else wrap(yaw_rel)
        yf = self.yaw_ff if chain else 0.0
        self.start_move(dist, mh, x, y)
        self.tv_fwd = _clamp(tv, NAV['TV_MIN'], NAV['TV_ARC_CAP']) if tv > 0.0 else NAV['TV_MAX']
        self.curv = curv
        self.yaw_ff = yf

    def finish_arc(self):
        """#F：結束弧線，把終點改成「現在再往前一段減速距離」（同 nav.h 的 'F'）"""
        self.timer = self.settle = 0
        if self.mode == NAV_MOVE and self.curv != 0.0:
            slow = NAV['SLOW_CM'] * (self.tv_fwd / NAV['TV_MAX'] if self.tv_fwd > NAV['TV_MAX'] else 1.0) * self.tpc
            if self.move_dist - self.s_arc > slow:
                self.move_dist = self.s_arc + slow

    def turn_step(self, yaw_rel, yaw_rate):
        P = NAV
        err = wrap(self.target_heading - yaw_rel)
        self.remaining = err
        aerr = abs(err)
        if aerr < P['TURN_TOL'] and abs(yaw_rate) < P['TURN_STILL']:
            self.settle += 1
            if self.settle >= P['TURN_SETTLE']:
                return 0.0, DONE
        else:
            self.settle = 0
        self.timer += 1
        if self.timer >= self.timeout:
            return 0.0, TIMEOUT
        if aerr < P['TURN_TOL']:
            return 0.0, RUNNING
        pred = err - yaw_rate * P['TURN_LEAD']
        if pred * err <= 0.0:
            return 0.0, RUNNING
        r = _clamp(P['TURN_KP'] * pred, -P['TURN_RATE_MAX'], P['TURN_RATE_MAX'])
        if abs(r) < P['TURN_RATE_MIN']:
            r = P['TURN_RATE_MIN'] if err > 0 else -P['TURN_RATE_MIN']
        return r, RUNNING

    def move_step(self, x, y, v):
        P = NAV
        if self.curv != 0.0:
            # 弧線：累積沿目前切線方向的位移 = 已走弧長；目標方向 = 起點方向 + 曲率 × 弧長（走完就不再轉）
            ds = (x - self.px) * math.cos(self.move_heading) + (y - self.py) * math.sin(self.move_heading)
            self.px, self.py = x, y
            self.s_arc += ds
            sh = _clamp(self.s_arc, 0.0, self.move_dist)
            self.move_heading = wrap(self.h0 + self.curv * sh)
            running = self.mode == NAV_MOVE and self.s_arc < self.move_dist
            ff = self.curv * ds / P['DT_V'] if (running and ds > 0.0) else 0.0
            self.yaw_ff = P['FF_FILT'] * self.yaw_ff + (1.0 - P['FF_FILT']) * ff if running else 0.0
            s = self.s_arc
        else:
            s = (x - self.sx) * math.cos(self.move_heading) + (y - self.sy) * math.sin(self.move_heading)
        r = self.move_dist - s
        self.remaining = r
        tol = P['POS_TOL_CM'] * self.tpc
        self.timer += 1
        if self.timer >= self.timeout:
            ok = self.mode == NAV_STOP and abs(r) < (5.0 if self.curv != 0.0 else 2.0) * tol
            return 0.0, (DONE if ok else TIMEOUT)
        if self.mode == NAV_MOVE:
            if abs(r) < tol:
                self.mode = NAV_STOP
                self.v_avg = v
                self.settle = 0
                self.timer = 0
                self.timeout = P['SETTLE_MAX']
                return 0.0, RUNNING
            vmax = self.tv_fwd if r > 0 else P['TV_MAX_BACK']
            vmin = P['TV_MIN'] if r > 0 else P['TV_MIN_BACK']
            slow = P['SLOW_CM'] * (self.tv_fwd / P['TV_MAX'] if (r > 0 and self.tv_fwd > P['TV_MAX']) else 1.0)
            k = vmax / (slow * self.tpc)
            fwd = _clamp(k * r, -vmax, vmax)
            if abs(fwd) < vmin:
                fwd = vmin if r > 0 else -vmin
            return -fwd, RUNNING
        self.v_avg = P['V_FILT'] * self.v_avg + (1.0 - P['V_FILT']) * v
        if abs(r) < (2.0 if self.curv != 0.0 else 1.0) * P['SETTLE_TOL_CM'] * self.tpc and abs(self.v_avg) < P['STILL_V']:
            self.settle += 1
            if self.settle >= P['SETTLE_UPD']:
                return 0.0, DONE
        else:
            self.settle = 0
        return 0.0, RUNNING


class SimRobot(threading.Thread):
    """在背景跑的模擬車：吃 nav.py 送的字元，吐出和韌體同格式的遙測與 NAV 訊息"""
    DT = 0.01013
    TICKS_PER_CM = 36.2

    def __init__(self, speed=5.0, seed=None):
        super().__init__(daemon=True)
        rnd = random.Random(seed)
        self.G = 1.5 + 1.0 * rnd.random()    # 實測：導航速度 V ≈ 1.5~2.5 倍目標速度
        self.tau_v = 0.35 + 0.3 * rnd.random()
        self.tau_coast = 0.5 + 0.4 * rnd.random()
        self.stick_tv = 0.6 + 0.6 * rnd.random()
        self.tau_yaw = 0.25 + 0.2 * rnd.random()
        self.stick_rate = 0.12 + 0.12 * rnd.random()
        self.spin_drift = (rnd.random() - 0.5) * 8.0
        self.rnd = rnd
        self.speed = speed
        self.x = self.y = 0.0
        self.yaw0 = 0.3
        self.yaw = 0.3 + rnd.uniform(-0.05, 0.05)
        self.yaw_rate = self.v = self.target_speed = self.cmd = self.ref = self.osc = 0.0
        self.nav = NavCore(self.TICKS_PER_CM)
        self.inbox = queue.Queue()
        self.outbox = queue.Queue()
        self.running = True
        self.t = 0.0

    # --- 模擬序列埠介面 ---
    def send(self, s):
        for ch in s:
            self.inbox.put(ch)

    def read_line(self):
        try:
            return self.outbox.get(timeout=0.05)
        except queue.Empty:
            return ''

    def close(self):
        self.running = False

    # --- 韌體行為 ---
    def _pose_str(self):
        return 'PX:%.1f PY:%.1f H:%.1f' % (self.x / self.TICKS_PER_CM, self.y / self.TICKS_PER_CM,
                                           math.degrees(wrap(self.yaw - self.yaw0)))

    def _yaw_rel(self):
        return wrap(self.yaw - self.yaw0)

    def _finish(self, tag, kind_res):
        """同韌體 Nav_Report()：NAV:DONE T err=1.2 PX:.. PY:.. H:.."""
        kind = 'T' if self.nav.mode == NAV_TURN else ('C' if self.nav.curv != 0.0 else 'M')
        err = math.degrees(self.nav.remaining) if kind == 'T' else self.nav.remaining / self.TICKS_PER_CM
        self.outbox.put('%s%s err=%.1f %s' % (tag, kind, err, self._pose_str()))
        self.nav.stop()
        self.cmd = 0.0

    def _on_line(self, line):
        cmd = line[:1].upper()
        nums = [float(v) for v in line[1:].replace(',', ' ').split()]
        if cmd in 'SA':
            if self.nav.mode != NAV_IDLE:
                self._finish('NAV:ABORT CMD ', None)
        elif cmd == 'T' and nums:
            self.nav.start_turn(nums[0] * DEG, self._yaw_rel())
            self.outbox.put('NAV:START T %.1f' % nums[0])
        elif cmd == 'M' and nums:
            heading = nums[1] * DEG if len(nums) > 1 else self._yaw_rel()
            self.nav.start_move(nums[0] * self.TICKS_PER_CM, heading, self.x, self.y)
            self.outbox.put('NAV:START M %.1f' % nums[0] + (' %.1f' % nums[1] if len(nums) > 1 else ''))
        elif cmd == 'F':
            self.nav.finish_arc()
            self.outbox.put('NAV:START F 0.0')
        elif cmd == 'C' and len(nums) > 1 and nums[0] > 0 and nums[1] != 0:
            self.nav.start_arc(nums[0] * self.TICKS_PER_CM, 1.0 / (nums[1] * self.TICKS_PER_CM),
                               self._yaw_rel(), self.x, self.y, nums[2] if len(nums) > 2 else 0.0)
            self.outbox.put('NAV:START C %.1f %.1f' % (nums[0], nums[1]))
        else:
            self.outbox.put('NAV:REJECT ?')

    def run(self):
        buf, in_line, k, next_tel = '', False, 0, 0.0
        wall0 = time.time()
        while self.running:
            while not self.inbox.empty():
                c = self.inbox.get()
                if in_line:
                    if c in '\r\n':
                        in_line = False
                        self._on_line(buf)
                    else:
                        buf += c
                elif c == '#':
                    in_line, buf = True, ''
                elif c in 'zZ':
                    self.x = self.y = 0.0
                    self.yaw0 = self.yaw
                    self.outbox.put('>> 座標歸零')
                elif c in 'wxadscWXADSC ' and self.nav.mode != NAV_IDLE:
                    self._finish('NAV:ABORT USER ', None)
            # 10 ms 控制週期
            rate_des = 0.0
            if self.nav.mode == NAV_TURN:
                rate_des, res = self.nav.turn_step(self._yaw_rel(), self.yaw_rate + self.rnd.gauss(0, 0.03))
                if res != RUNNING:
                    self._finish('NAV:DONE ' if res == DONE else 'NAV:TIMEOUT ', res)
            decel = abs(rate_des) < abs(self.ref) or rate_des * self.ref < 0
            st = (2.0 if decel else 4.0) * self.DT
            self.ref += max(-st, min(st, rate_des - self.ref))
            if self.nav.mode in (NAV_MOVE, NAV_STOP):   # 直走時航向鎖在指定方向
                self.ref = max(-0.8, min(0.8, self.nav.yaw_ff + 3.0 * wrap(self.yaw0 + self.nav.move_heading - self.yaw)))
            k += 1
            if k >= 4:
                k = 0
                self.osc += 0.0405 * 2 * math.pi * 2.0
                v_meas = self.v + 3.0 * math.sin(self.osc) + self.rnd.gauss(0, 1.0)
                if self.nav.mode in (NAV_MOVE, NAV_STOP):
                    self.vf = 0.7 * getattr(self, 'vf', 0.0) + 0.3 * v_meas   # 同韌體 filtered_speed
                    self.cmd, res = self.nav.move_step(self.x, self.y, self.vf)
                    if res != RUNNING:
                        self._finish('NAV:DONE ' if res == DONE else 'NAV:TIMEOUT ', res)
                self._speed_update(self.nav.mode == NAV_TURN and abs(self.ref) > 0.3)
                self.vm = v_meas
            self._tick()
            self.t += self.DT
            if self.t >= next_tel:
                next_tel += 0.2
                self.outbox.put(self._telemetry())
            # 依模擬倍速對時
            lag = self.t / self.speed - (time.time() - wall0)
            if lag > 0.002:
                time.sleep(lag)

    def _speed_update(self, spinning):
        cmd = self.cmd
        reversing = cmd * self.target_speed < 0
        braking = reversing or abs(cmd) < abs(self.target_speed)
        step = 0.15 if (braking and not reversing) else 0.30
        if self.target_speed < cmd:
            self.target_speed = min(self.target_speed + step, cmd)
        elif self.target_speed > cmd:
            self.target_speed = max(self.target_speed - step, cmd)
        vt = self.G * self.target_speed
        if abs(self.target_speed) < self.stick_tv and abs(self.v) < 1.0:
            vt = 0.0
        if spinning:
            vt = self.spin_drift
        tau = self.tau_coast if (self.target_speed == 0 and not spinning) else self.tau_v
        self.v += (vt - self.v) * (0.0405 / tau)

    def _tick(self):
        target = self.ref
        if abs(self.ref) < self.stick_rate and abs(self.yaw_rate) < 0.1:
            target = 0.0
        self.yaw_rate += (target - self.yaw_rate) * (self.DT / self.tau_yaw)
        self.yaw = wrap(self.yaw + self.yaw_rate * self.DT)
        fwd = -self.v / 4.0
        h = self.yaw - self.yaw0
        self.x += fwd * math.cos(h)
        self.y += fwd * math.sin(h)

    def _telemetry(self):
        tgt = self.yaw0 + (self.nav.target_heading if self.nav.mode == NAV_TURN else self.nav.move_heading)
        line = ('ANG:%.1f SA:0.00 B:0.0 T:0.0 PWM:0/0 YR:%.2f Y:%.2f TY:%.2f SP:-1.45 DX:0 V:%.1f TV:%.2f BS:0.00 '
                'PX:%.1f PY:%.1f' % (-1.45 + self.rnd.gauss(0, 0.6), self.yaw_rate, self.yaw, wrap(tgt),
                                     getattr(self, 'vm', 0.0), self.target_speed,
                                     self.x / self.TICKS_PER_CM, self.y / self.TICKS_PER_CM))
        if self.nav.mode != NAV_IDLE:
            nr = math.degrees(self.nav.remaining) if self.nav.mode == NAV_TURN else self.nav.remaining / self.TICKS_PER_CM
            line += ' NV:%d NR:%.1f' % (self.nav.mode, nr)
        return line + ' | 狀態:【常態雙閉環】'


# =============================================================================
# 4. 執行器
# =============================================================================
class Navigator:
    def __init__(self, link, logger, time_scale=1.0):
        self.link = link
        self.logger = logger
        self.k = time_scale             # 模擬倍速：等待時間要跟著縮短
        self.events = queue.Queue()
        self.pose = None                # (PX, PY) cm
        self.heading = 0.0              # 車頭方向（°，相對原點），由 NAV:DONE 的 H: 更新
        self.state = None
        self.v = 0.0                    # 最近一筆遙測的實際速度 V
        self.yaw = None                 # 最近一筆遙測的絕對車頭方向 Y (rad)
        self.yaw0 = 0.0                 # 座標歸零當下的 Y；車頭相對方向 = Y - yaw0
        self.nv = 0                     # 導航狀態（0 閒置、1 轉向、2 直走、3 停車）
        self.nr = 0.0                   # 剩餘量（直走：公分）
        self.tel_count = 0
        self.running = True
        self.t0 = time.time()
        self.lock = threading.Lock()
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _reader(self):
        from control import parse_telemetry
        while self.running:
            try:
                line = self.link.read_line()
            except Exception:
                break
            if not line:
                time.sleep(0.005)
                continue
            el = round((time.time() - self.t0) * self.k, 3)   # 模擬時換算回模擬時間
            if self.logger:
                try:
                    self.logger.log(el, 'NAV', line)
                except ValueError:
                    break
            if line.startswith('NAV:'):
                print(f'  [{el:6.1f}s] {line}')
                self.events.put(line)
                continue
            state, f = parse_telemetry(line)
            if state is None:
                if line.strip():
                    print(f'  [{el:6.1f}s] {line}')
                continue
            with self.lock:
                self.state = state
                if 'PX' in f and 'PY' in f:
                    self.pose = (float(f['PX']), float(f['PY']))
                if 'V' in f:
                    try:
                        self.v = float(f['V'])
                    except ValueError:
                        pass
                try:
                    if 'Y' in f:
                        self.yaw = float(f['Y'])
                    self.nv = int(f['NV']) if 'NV' in f else 0
                    self.nr = float(f['NR']) if 'NR' in f else 0.0
                except ValueError:
                    pass
                self.tel_count += 1

    def _heartbeat(self):
        while self.running:
            try:
                self.link.send('.')
            except Exception:
                break
            time.sleep(HEARTBEAT_S / self.k)

    def wait_telemetry(self, n=1, timeout=3.0):
        with self.lock:
            target = self.tel_count + n
        end = time.time() + timeout / self.k
        while time.time() < end:
            with self.lock:
                if self.tel_count >= target:
                    return self.pose
            time.sleep(0.01)
        raise RuntimeError('收不到遙測（檢查連線 / 序列埠）')

    def wait_event(self, timeout):
        end = time.time() + timeout / self.k
        while time.time() < end:
            try:
                ev = self.events.get(timeout=0.05)
            except queue.Empty:
                continue
            if ev.startswith('NAV:START'):
                continue
            return ev
        return None

    def command(self, text, timeout):
        self.link.send('#' + text + '\n')
        ev = self.wait_event(timeout)
        if ev is None:
            self.link.send('#S\n')
            raise RuntimeError(f'#{text} 等不到回應（{timeout:.0f} s）')
        if not ev.startswith('NAV:DONE'):
            raise RuntimeError(f'#{text} 失敗：{ev}')
        for tok in ev.split():
            if tok.startswith('H:'):
                self.heading = float(tok[2:])
        time.sleep(SETTLE_AFTER_DONE_S / self.k)
        return self.wait_telemetry(1)

    def rel_heading(self):
        """車頭相對座標原點的方向（°，右轉為正）；由遙測的 Y 算，連續弧線行進中也看得到"""
        return wrap_deg(math.degrees((self.yaw if self.yaw is not None else self.yaw0) - self.yaw0))

    def note(self, text):
        """把電腦端的資訊（例如目標點）寫進記錄檔，analyze.py 會用來算誤差"""
        if self.logger:
            self.logger.log(round((time.time() - self.t0) * self.k, 3), 'NAV', text)

    def abort(self):
        try:
            self.link.send('#S\n')
            self.link.send('s')
        except Exception:
            pass

    def close(self):
        self.running = False


def move_timeout(cm):
    return max(MOVE_TIMEOUT_S, 6.0 + 0.45 * abs(cm))


def wait_still(nav, timeout=4.0, need=3):
    """等速度小於 SETTLE_V 連續 need 筆遙測（轉彎剛結束時車子還在晃，這時送指令容易失控）"""
    ok = 0
    end = time.time() + timeout / nav.k
    while time.time() < end and ok < need:
        nav.wait_telemetry(1, timeout=2)
        ok = ok + 1 if abs(nav.v) < SETTLE_V else 0
    return ok >= need


def safe_turn(nav, target_deg):
    """轉到 target_deg：大角度拆成 ≤MAX_TURN_STEP_DEG 的小步，每步之後等車子靜止；
    位置被帶偏太多就中止（回傳最後的位置）"""
    pose = nav.wait_telemetry(1)
    for _ in range(8):
        diff = wrap_deg(target_deg - nav.heading)
        step = nav.heading + math.copysign(min(abs(diff), MAX_TURN_STEP_DEG * 0.9), diff) \
            if abs(diff) > MAX_TURN_STEP_DEG else target_deg
        before = nav.wait_telemetry(1)
        pose = nav.command(f'T {wrap_deg(step):.1f}', TURN_TIMEOUT_S)
        wait_still(nav)
        pose = nav.wait_telemetry(1)
        drift = math.hypot(pose[0] - before[0], pose[1] - before[1])
        if drift > TURN_DRIFT_ABORT_CM:
            raise RuntimeError(f'轉彎時位置被帶偏 {drift:.1f} cm（超過 {TURN_DRIFT_ABORT_CM:g}），車子可能失控，已中止')
        if step == target_deg:
            break
    return pose


def wait_standing(nav):
    print('等待車子站穩…')
    end = time.time() + 15 / nav.k
    stable = 0
    while time.time() < end and stable < 5:
        nav.wait_telemetry(1, timeout=5)
        stable = stable + 1 if nav.state == 'NORMAL' else 0
    if stable < 5:
        raise RuntimeError('車子沒有在站立狀態（遙測不是【常態雙閉環】）')


def zero_pose(nav):
    nav.heading = 0.0
    nav.link.send('z')
    time.sleep(0.5 / nav.k)
    nav.wait_telemetry(2)
    if nav.yaw is not None:
        nav.yaw0 = nav.yaw
    print('>> 座標歸零：目前位置 = (0, 0)，目前車頭 = X 軸')


def parse_steps(text):
    """單一動作測試："T90 T180 M20 M-20@0 C94@-15" → [('T', 90, None), ('T', 180, None), ('M', 20, None), ('M', -20, 0), ('C', 94, -15)]
    C<弧長cm>@<半徑cm>[@<速度>]：邊走邊轉的圓弧，半徑正 = 右轉、負 = 左轉；速度 = 前進目標速度（預設 5.8，上限 12）"""
    steps = []
    for tok in text.replace(',', ' ').replace(';', ' ').split():
        kind, rest = tok[0].upper(), tok[1:]
        if kind not in 'TMC' or not rest or (kind == 'C' and '@' not in rest):
            raise SystemExit(f'看不懂的動作：{tok}（格式：T<角度>、M<公分>、M<公分>@<方向> 或 C<弧長>@<半徑>）')
        if kind == 'C':
            d, rad = rest.split('@', 1)
            tv = 0.0
            if '@' in rad:
                rad, tv = rad.split('@', 1)
            if float(d) <= 0 or float(rad) == 0 or float(d) > MAX_ARC_CM or not 0 <= float(tv) <= NAV['TV_ARC_CAP']:
                raise SystemExit(f'{tok}：弧長要 > 0 且 ≤ {MAX_ARC_CM:g} cm，半徑不能是 0，速度要在 0（預設）~ {NAV["TV_ARC_CAP"]:g}'
                                 '（想一直繞、按 Enter 才停，請用 --laps 0）')
            steps.append(('C', float(d), float(rad)) if tv == 0 else ('C', float(d), float(rad), float(tv)))
        elif kind == 'M' and '@' in rest:
            d, h = rest.split('@', 1)
            steps.append(('M', float(d), float(h)))
        else:
            steps.append((kind, float(rest), None))
    return steps


def run_steps(nav, steps, zero=True):
    """一次送一個 #T / #M，印出每個動作的結果（上車第一次測試用）"""
    wait_standing(nav)
    if zero:
        zero_pose(nav)
    rows = []
    for i, (kind, v, h, *rest) in enumerate(steps):
        tv = rest[0] if rest else 0.0
        cmd = (f'T {v:.1f}' if kind == 'T' else
               f'C {v:.1f} {h:.1f}' + (f' {tv:g}' if tv else '') if kind == 'C' else
               f'M {v:.1f}' + (f' {h:.1f}' if h is not None else ''))
        before = nav.wait_telemetry(1)
        h0 = nav.heading
        t0 = time.time()
        pose = nav.command(cmd, TURN_TIMEOUT_S if kind == 'T' else move_timeout(v))   # C 的弧長也用同一個逾時公式
        dt = (time.time() - t0) * nav.k
        if kind == 'T':
            err = wrap_deg(v - nav.heading)
            print(f'  #{cmd}：{dt:.1f} s，航向 {h0:.1f}° → {nav.heading:.1f}°，誤差 {err:+.1f}°，'
                  f'位置被帶偏 {math.hypot(pose[0] - before[0], pose[1] - before[1]):.1f} cm')
        elif kind == 'C':
            moved = math.hypot(pose[0] - before[0], pose[1] - before[1])
            print(f'  #{cmd}：{dt:.1f} s，位置 ({before[0]:.1f}, {before[1]:.1f}) → ({pose[0]:.1f}, {pose[1]:.1f})，'
                  f'起終點相距 {moved:.1f} cm，航向 {h0:.1f}° → {nav.heading:.1f}°（弧長 {v:.1f}、半徑 {abs(h):g}、'
                  f'應轉 {math.degrees(v / h):+.0f}°）')
        else:
            moved = math.hypot(pose[0] - before[0], pose[1] - before[1])
            print(f'  #{cmd}：{dt:.1f} s，位置 ({before[0]:.1f}, {before[1]:.1f}) → ({pose[0]:.1f}, {pose[1]:.1f})，'
                  f'走了 {moved:.1f} cm（要求 {abs(v):.1f}），航向 {nav.heading:.1f}°')
        rows.append((cmd, dt))
    print('>> 全部動作完成。實際位置 / 角度請用捲尺、量角器確認，和上面的數字比較。')
    return rows


def stop_key_pressed():
    """Windows 主控台：有按任何鍵（Enter / 空白鍵…）就回傳 True；其他系統回傳 False（這時請改用 --stop-after）"""
    try:
        import msvcrt
    except ImportError:
        return False
    hit = False
    while msvcrt.kbhit():
        msvcrt.getwch()
        hit = True
    return hit


def run_arc_until_stop(nav, radius_cm, speed, stop_after=None, zero=True):
    """一直繞圓，直到你按 Enter（或 --stop-after 秒數到）：送 #F，車子沿著圓弧慢慢減速停下"""
    wait_standing(nav)
    if zero:
        zero_pose(nav)
    cmd = f'C {FOREVER_ARC_CM:g} {radius_cm:.1f}' + (f' {speed:g}' if speed else '')
    nav.wait_telemetry(1)
    t0 = time.time()
    nav.link.send('#' + cmd + '\n')
    stop_key_pressed()                                    # 清掉之前按過的鍵
    print('>> 繞圈中……按 Enter（或空白鍵）：沿著圓弧慢慢停下；Ctrl+C：緊急中止')
    stopping_at = None
    while True:
        ev = nav.wait_event(0.2)
        if ev is not None:
            break
        el = (time.time() - t0) * nav.k
        if stopping_at is None and (stop_key_pressed() or (stop_after is not None and el >= stop_after)):
            nav.link.send('#F\n')
            stopping_at = el
            print(f'  [{el:6.1f}s] >> 收到停止，沿著圓弧減速中……')
        if stopping_at is not None and el - stopping_at > 40:
            raise RuntimeError('按停止後 40 秒還沒停下來')
        if el > move_timeout(FOREVER_ARC_CM):
            raise RuntimeError('繞圓等不到回應')
    if not ev.startswith('NAV:DONE'):
        raise RuntimeError(f'#{cmd} 失敗：{ev}')
    for tok in ev.split():
        if tok.startswith('H:'):
            nav.heading = float(tok[2:])
    pose = nav.wait_telemetry(1)
    el = (time.time() - t0) * nav.k
    print(f'  #{cmd}：共 {el:.1f} s，停在 ({pose[0]:.1f}, {pose[1]:.1f}) cm，車頭 {nav.heading:.1f}°')
    return pose


def plan_arc(pose, heading_deg, target, rmin):
    """從目前位置、車頭方向出發，用「和車頭相切的一段圓弧」走到目標點。
    回傳 ('skip',)、('arc', 弧長cm, 有號半徑cm, 轉角°)（半徑正 = 右轉、負 = 左轉），
    或 None（目標在背後，或需要的半徑小於 rmin → 改用原地轉向 + 直走）"""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    d = math.hypot(dx, dy)
    if d < SKIP_DIST_CM:
        return ('skip',)
    th = math.radians(heading_deg)
    fwd = dx * math.cos(th) + dy * math.sin(th)          # 目標在車頭前方多遠
    lat = -dx * math.sin(th) + dy * math.cos(th)         # 目標在車頭右邊多遠（左邊為負）
    if fwd <= 0:
        return None
    if abs(lat) < 0.3:                                   # 幾乎正前方：走直線（曲率接近 0）
        return ('arc', d, STRAIGHT_R_CM, 0.0)
    r = d * d / (2 * abs(lat))
    if r < rmin:
        return None
    phi = 2 * math.atan2(abs(lat), fwd)                  # 弧角（rad）
    return ('arc', r * phi, math.copysign(r, lat), math.copysign(math.degrees(phi), lat))


def smooth_dry_run(route, scale, rmin):
    """平滑路線的計畫：假設每一段都走得很準，列出每一段的圓弧半徑、轉角、長度"""
    pose, heading, total = (0.0, 0.0), 0.0, 0.0
    print(f'座標 1 = {scale:g} cm；起點 = (0, 0)、車頭朝 X 軸；最小弧線半徑 {rmin:g} cm（更小的轉彎改成原地轉 + 直走）')
    print(' 段  目標點                方式      半徑(cm)  轉角(°)  長度(cm)')
    for i, (x, y, _) in enumerate(route[1:], 1):
        tgt = (x * scale, y * scale)
        pl = plan_arc(pose, heading, tgt, rmin)
        if pl and pl[0] == 'skip':
            print(f' {i:2d}  ({x:g}, {y:g})  已在目標上，略過')
            continue
        if pl is None:
            b, d = plan(pose, tgt)
            print(f' {i:2d}  ({x:g}, {y:g})'.ljust(30) + f'轉向+直走  朝 {b:.0f}° 轉 {wrap_deg(b - heading):+.0f}°'.ljust(22) + f'{d:8.1f}')
            heading, total = b, total + d
        else:
            _, length, rad, ang = pl
            side = '直線' if abs(rad) >= STRAIGHT_R_CM else ('右弧' if rad > 0 else '左弧')
            print(f' {i:2d}  ({x:g}, {y:g})'.ljust(30) + f'{side}'.ljust(9)
                  + (f'{abs(rad):9.1f}  {ang:+7.0f}  ' if side != '直線' else f'{"-":>9}  {0:+7.0f}  ') + f'{length:8.1f}')
            heading, total = wrap_deg(heading + ang), total + length
        pose = tgt
    print(f'總路徑約 {total:.1f} cm')


def _wait_arc_near(nav, switch_cm, timeout):
    """等目前這段弧線快到終點（剩餘 ≤ switch_cm）。回傳 'near' 或 'done'（這段已經完成並停下）"""
    end = time.time() + timeout / nav.k
    last = -1
    while time.time() < end:
        while True:                                       # 先看有沒有結束 / 失敗的事件
            try:
                ev = nav.events.get_nowait()
            except queue.Empty:
                break
            if ev.startswith('NAV:START'):
                continue
            if ev.startswith('NAV:DONE'):
                return 'done'
            raise RuntimeError(f'弧線失敗：{ev}')
        with nav.lock:
            n, nv, nr = nav.tel_count, nav.nv, nav.nr
        if n != last:
            last = n
            if nv == 3 or (nv == 2 and nr <= switch_cm):
                return 'near'
        time.sleep(0.02)
    raise RuntimeError('等不到這段弧線的進度（逾時）')


def _wait_arc_started(nav, timeout=4.0):
    """送出 #C 之後：等韌體回 NAV:START，再等 2 筆新的遙測，避免拿到上一段的舊狀態"""
    end = time.time() + timeout / nav.k
    while time.time() < end:
        try:
            ev = nav.events.get(timeout=0.05)
        except queue.Empty:
            continue
        if ev.startswith('NAV:START'):
            nav.wait_telemetry(2)
            return
        if ev.startswith('NAV:REJECT') or ev.startswith('NAV:ABORT') or ev.startswith('NAV:TIMEOUT'):
            raise RuntimeError(f'弧線指令被拒絕或失敗：{ev}')
    raise RuntimeError('送出弧線指令後，車子沒有回應（沒收到 NAV:START）')


def run_smooth(nav, route, scale, rmin, switch_cm, speed, zero=True):
    """依序走過每個點，點與點之間走圓弧；不停車：前一段剩下 switch_cm 時，直接接上下一段"""
    for x, y, back in route:
        if x is None or back:
            raise SystemExit('平滑路線（--smooth）不支援倒車（b）和原地轉向（h角度），請拿掉')
    wait_standing(nav)
    if zero:
        zero_pose(nav)
    active = False
    done_points = 0
    tv = f' {speed:g}' if speed else ''
    for i, (x, y, _) in enumerate(route):
        tgt = (x * scale, y * scale)
        last = (i == len(route) - 1)
        if active:
            if _wait_arc_near(nav, switch_cm, 40) == 'done':
                active = False
        pose = nav.wait_telemetry(1)
        hd = nav.rel_heading()
        pl = plan_arc(pose, hd, tgt, rmin)
        if pl and pl[0] == 'skip':
            print(f'點 {i}：({x:g}, {y:g}) 已在目標上')
            continue
        nav.note(f'NAV:PLAN {i} {tgt[0]:.1f} {tgt[1]:.1f}')
        if pl is None:                                    # 目標在背後 / 弧線半徑太小 → 停下來，原地轉 + 直走
            if active:
                print(f'點 {i}：轉彎太急，先停下來再原地轉向')
                if _wait_arc_near(nav, 0.0, 40) != 'done':
                    nav.wait_event(20)
                active = False
                wait_still(nav)
            print(f'點 {i}：前往 ({x:g}, {y:g})，改用原地轉向 + 直走')
            run_route(nav, [(tgt[0], tgt[1], False)], 1.0, zero=False)
            done_points += 1
            continue
        _, length, rad, ang = pl
        print(f'點 {i}：前往 ({x:g}, {y:g}) = ({tgt[0]:.1f}, {tgt[1]:.1f}) cm，目前 ({pose[0]:.1f}, {pose[1]:.1f}) 車頭 {hd:.0f}°，'
              + (f'走圓弧 半徑 {abs(rad):.0f} cm、轉 {ang:+.0f}°、長 {length:.0f} cm' if abs(rad) < STRAIGHT_R_CM else f'直線 {length:.0f} cm'))
        if length > MAX_ARC_CM:
            raise RuntimeError(f'這一段弧長 {length:.0f} cm（超過 {MAX_ARC_CM:g}），位置估計可能壞了，已中止')
        nav.link.send(f'#C {length:.1f} {rad:.1f}{tv}\n')
        _wait_arc_started(nav)
        active = True
        done_points += 1
        if last:
            ev = nav.wait_event(move_timeout(length))
            if ev is None:
                nav.link.send('#S\n')
                raise RuntimeError('最後一段等不到完成（逾時）')
            if not ev.startswith('NAV:DONE'):
                raise RuntimeError(f'最後一段失敗：{ev}')
            active = False
    pose = nav.wait_telemetry(1)
    print(f'>> 路線走完：最後位置 ({pose[0]:.1f}, {pose[1]:.1f}) cm，車頭 {nav.rel_heading():.1f}°')
    return pose


def run_route(nav, route, scale, zero=True):
    wait_standing(nav)
    if zero:
        zero_pose(nav)
    results = []
    for i, (x, y, want_back) in enumerate(route):
        if x is None:
            pose = nav.wait_telemetry(1)
            print(f'點 {i}：在原地轉到 {y:g}°')
            pose = safe_turn(nav, wrap_deg(y))
            print(f'  → 車頭 {nav.heading:.1f}°（目標 {y:g}°，誤差 {wrap_deg(y - nav.heading):+.1f}°）')
            results.append((pose[0] / scale, pose[1] / scale, pose, 0.0))
            continue
        tgt = (x * scale, y * scale)
        pose = nav.wait_telemetry(1)
        b, d = plan(pose, tgt)
        if d < SKIP_DIST_CM:
            print(f'點 {i}：({x:g}, {y:g}) 已在目標上')
            results.append((x, y, pose, 0.0))
            continue
        nav.note(f'NAV:PLAN {i} {tgt[0]:.1f} {tgt[1]:.1f}')
        print(f'點 {i}：前往 ({x:g}, {y:g}) = ({tgt[0]:.1f}, {tgt[1]:.1f}) cm，'
              f'目前 ({pose[0]:.1f}, {pose[1]:.1f})，方向 {b:.1f}°、距離 {d:.1f} cm')
        err = d
        for attempt in range(1 + RETRIES):
            b, d = plan(pose, tgt)
            if d < SKIP_DIST_CM:
                break
            # 修正時如果目標在背後，就倒車過去（不要原地轉 180°）
            back = want_back or (attempt > 0 and abs(wrap_deg(b - nav.heading)) > 100)
            if attempt > 0:
                print(f'  修正第 {attempt} 次：還差 {d:.1f} cm' + ('（倒車）' if back else ''))
            hd = wrap_deg(b + 180) if back else b
            pose = safe_turn(nav, hd)
            for _ in range(2):                          # 轉完用實際位置重算；偏太多就再轉一次
                b, d = plan(pose, tgt)
                hd = wrap_deg(b + 180) if back else b
                if abs(wrap_deg(hd - nav.heading)) <= REALIGN_DEG:
                    break
                print(f'  轉向時被帶偏，重新對準 {hd:.1f}°')
                pose = safe_turn(nav, hd)
            if d > MAX_LEG_CM:
                raise RuntimeError(f'這一段要走 {d:.1f} cm（超過 {MAX_LEG_CM:g}），位置估計可能壞了，已中止')
            pose = nav.command(f'M {-d if back else d:.1f} {hd:.1f}', move_timeout(d))
            err = math.hypot(pose[0] - tgt[0], pose[1] - tgt[1])
            print(f'  → 到達 ({pose[0]:.1f}, {pose[1]:.1f}) cm，誤差 {err:.1f} cm')
            if err <= RETRY_CM:
                break
        results.append((x, y, pose, err))
    return results


class SimLogger:
    """模擬時的 CSV 記錄（存到 logs/sim/，不和實車記錄混在一起）"""

    def __init__(self):
        import control
        now = datetime.now()
        d = os.path.join(control.LOG_ROOT, 'sim')
        os.makedirs(d, exist_ok=True)
        self.inner = control.TelemetryLogger.__new__(control.TelemetryLogger)
        self.inner.path = os.path.join(d, f"telemetry_{now.strftime('%Y%m%d_%H%M%S')}.csv")
        import csv
        self.inner._file = open(self.inner.path, 'w', newline='', encoding='utf-8-sig')
        self.inner._writer = csv.writer(self.inner._file)
        self.inner._writer.writerow(control.TelemetryLogger.HEADER)
        self.inner._lock = threading.Lock()
        self.path = self.inner.path

    def log(self, *a):
        self.inner.log(*a)

    def close(self):
        self.inner.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description='座標導航腳本執行器（nav_v27）')
    ap.add_argument('route', help='路線，例如 "0,0 1,1 1,-1 -1,-1"，或路線檔名；加 --steps 時是動作清單')
    ap.add_argument('--steps', action='store_true',
                    help='改成逐一執行動作，例如 "T90 T180 T-90 T0" 或 "M20 M-20"（M<公分>@<方向> 可指定方向）')
    ap.add_argument('--scale', type=float, default=DEFAULT_SCALE_CM, help='座標 1 = 幾公分（預設 10）')
    ap.add_argument('--dry-run', action='store_true', help='只印出計畫，不連線')
    ap.add_argument('--sim', action='store_true', help='用模擬車執行（不需要實車）')
    ap.add_argument('--sim-speed', type=float, default=5.0, help='模擬倍速（預設 5 倍）')
    ap.add_argument('--seed', type=int, default=None, help='模擬車隨機參數的種子')
    ap.add_argument('--port', default=None, help='序列埠（預設用 control.py 的 COM_PORT）')
    ap.add_argument('--no-zero', action='store_true', help='不先把座標歸零')
    ap.add_argument('--circle', type=float, default=None, metavar='半徑cm',
                    help='邊走邊轉繞圓，例如 --circle 15（預設向左；加 --right 向右；--laps 圈數）。此時不用給路線，隨便填 x')
    ap.add_argument('--right', action='store_true', help='--circle 向右（順時針）')
    ap.add_argument('--smooth', action='store_true', help='平滑路線：點與點之間走圓弧、不停車（座標路線用；目標在背後或轉彎太急的段落自動改用原地轉 + 直走）')
    ap.add_argument('--min-radius', type=float, default=SMOOTH_MIN_R_CM, help=f'--smooth 的最小弧線半徑 cm（預設 {SMOOTH_MIN_R_CM:g}）')
    ap.add_argument('--switch-cm', type=float, default=SMOOTH_SWITCH_CM, help=f'--smooth 前一段剩下幾公分就接上下一段（預設 {SMOOTH_SWITCH_CM:g}）')
    ap.add_argument('--speed', type=float, default=0.0, help='--circle 的前進目標速度（預設 5.8 = 遙控 W 鍵同速；上限 12；實測 8 可以，更高請一次加 1）')
    ap.add_argument('--laps', type=float, default=1.0, help='--circle 的圈數（預設 1，可以 0.5 之類；0 = 一直繞，直到你按 Enter 才沿著圓弧慢慢停下）')
    ap.add_argument('--stop-after', type=float, default=None, help='--laps 0 時，過幾秒自動停（測試用，不用手按）')
    ap.add_argument('--keys', default='', help='開始前先送出的按鍵，例如 jj（Kp_speed -1.0）、h（Kd +10）')
    a = ap.parse_args(argv)

    sys.path.insert(0, BASE_DIR)
    forever = None
    if a.circle:
        signed_r = abs(a.circle) if a.right else -abs(a.circle)
        side = '向右（順時針）' if a.right else '向左（逆時針）'
        a.steps = True
        if a.laps <= 0:
            forever = (signed_r, a.speed)
            a.route = 'T0'                      # 佔位，不會用到
            print(f'>> 繞圓：半徑 {abs(a.circle):g} cm、{side}，一直繞到你按 Enter')
        else:
            if a.stop_after is not None:
                print('>> 注意：--stop-after 只在 --laps 0 時有效，這次會繞完 %g 圈' % a.laps)
            arc = 2 * math.pi * abs(a.circle) * a.laps
            a.route = f'C{arc:.1f}@{signed_r:g}' + (f'@{a.speed:g}' if a.speed else '')
            print(f'>> 繞圓：半徑 {abs(a.circle):g} cm、{side}、{a.laps:g} 圈，弧長 {arc:.1f} cm')
    steps = parse_steps(a.route) if a.steps else None
    route = None if a.steps else parse_route(a.route)
    if a.dry_run and forever:
        print(f'  #C {FOREVER_ARC_CM:g} {forever[0]:g}' + (f' {forever[1]:g}' if forever[1] else '') + '   （按 Enter 後再送 #F，沿著圓弧慢慢停下）')
        return 0
    if a.dry_run and steps:
        for kind, v, h, *rest in steps:
            print(f'  #{kind} {v:g}' + (f' {h:g}' if h is not None else '') + (f' {rest[0]:g}' if rest else ''))
        return 0
    if a.dry_run and a.smooth:
        smooth_dry_run(route, a.scale, a.min_radius)
        return 0
    if a.dry_run:
        dry_run(route, a.scale)
        return 0

    if a.sim:
        link = SimRobot(speed=a.sim_speed, seed=a.seed)
        link.start()
        logger = SimLogger()
        k = a.sim_speed
        print(f'>> 模擬模式（{a.sim_speed:g} 倍速），記錄檔：{logger.path}')
    else:
        import control
        port = a.port or control.COM_PORT
        link = control.RobotLink(port, control.BAUD_RATE)
        logger = control.TelemetryLogger()
        k = 1.0
        print(f'>> 已連線 {port}，記錄檔：{logger.path}')
        print('>> 按 Ctrl+C 可隨時中止')

    nav = Navigator(link, logger, time_scale=k)
    for ch in a.keys:                       # 調參按鍵（韌體會回報 >> Kp_speed = x 之類）
        link.send(ch)
        time.sleep(0.4 / k)
    code = 0
    try:
        if forever:
            run_arc_until_stop(nav, forever[0], forever[1], stop_after=a.stop_after, zero=not a.no_zero)
            return code
        if steps:
            run_steps(nav, steps, zero=not a.no_zero)
            return code
        if a.smooth:
            run_smooth(nav, route, a.scale, a.min_radius, a.switch_cm, a.speed, zero=not a.no_zero)
            return code
        results = run_route(nav, route, a.scale, zero=not a.no_zero)
        print('\n==== 結果 ====')
        print(' 點   目標(cm)          到達(cm)          誤差(cm)')
        for i, (x, y, pose, err) in enumerate(results):
            print(f' {i:2d}  ({x * a.scale:6.1f},{y * a.scale:6.1f})   ({pose[0]:6.1f},{pose[1]:6.1f})   {err:6.1f}')
        errs = [r[3] for r in results[1:]] or [0.0]
        print(f'最大誤差 {max(errs):.1f} cm（這是車上里程計自己的估計；實際位置請用捲尺量）')
    except KeyboardInterrupt:
        print('\n>> 中止！')
        nav.abort()
        code = 1
    except RuntimeError as e:
        print(f'\n>> 中止：{e}')
        nav.abort()
        code = 1
    finally:
        time.sleep(0.3 / k)
        nav.close()
        time.sleep(0.1)
        link.close()
        logger.close()
        print(f'>> 記錄檔：{logger.path}（可用 python analyze.py 這個檔名 分析）')
    return code


if __name__ == '__main__':
    sys.exit(main())
