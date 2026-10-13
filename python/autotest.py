# -*- coding: utf-8 -*-
"""
autotest.py — 自動重複測試：每次按住的時間、休息時間完全一樣，不靠手按

為什麼要有它：手按 W / X 每次長短不同（昨天 1.4 秒～16 秒都有），
同一個指令 V 會落在 -1 ～ -16，比參數之間的差異還大，結果就分不出參數好壞。
這支程式用 drive.py 同一個油門模型，照固定劇本自動送指令。

  python autotest.py --label base                       用 drive.py 的預設值跑 3 輪（W 5 秒 → 停 3 秒 → X 5 秒 → 停 3 秒）
  python autotest.py --vmax 18 --vback 18 --label a   換參數就改這裡，一次只改一個
  python autotest.py --kp 2.3 --label kp23              自動把 Kp_speed 設成 2.3（韌體要 ENABLE_TUNING 1；每次開序列埠韌體會重開機，所以每次都從 2.8 開始）
  python autotest.py --kd 50 --label kd50               自動把直立環 Kd 設成 50（從 40 開始，每次 ±10）
  python autotest.py --steer 0.5 --label turn           前進 / 後退時同時打方向 0.5
  python autotest.py --order W --cycles 1 -i --label base   互動模式：先測一次 W，然後等你輸入下一個測試（Enter = 重複上一個）
  python autotest.py --report                           把累積的所有測試依參數分組比較
  python autotest.py --dry-run                          不連線，只印出劇本與指令

流程：開啟序列埠（韌體會重開機）→ 把車子扶正放開 → 站穩 3 秒後自動開始 → 倒地就自動中止。
任何時候按 S / 空白 / Q / Esc 都會立刻停車並結束。結果追加到 logs/autotest_results.csv。
"""

import argparse
import copy
import csv
import os
import shlex
import statistics as st
import sys
import threading
import time
from datetime import datetime

from drive import DEFAULTS, DriveModel, Inputs, format_cmd, start_keyboard

V_TO_CMS = 1.0 / (36.2 * 0.0405)   # 遙測 V（ticks / 40.5 ms）換算成 cm/s
KP0 = 2.8                          # 韌體開機的 Kp_speed
KD0 = 40.0                         # 韌體開機的 Kd_balance_normal
SETTLE = 1.5                       # 每段開頭跳過幾秒（油門爬升 + 速度環反應），之後才算「穩態」
SAT_DEG = 3.0                      # |SA| 達到這個值視為速度環飽和（上限 3.5）
RESULT_FIELDS = ['time', 'label', 'vmax', 'vback', 'vmin', 'vmin_back', 'kp', 'kd', 'steer', 'hold',
                 'cycle', 'phase', 'n', 'speed_mean_cms', 'speed_sd_cms', 'speed_peak_cms',
                 'sa_sat_pct', 'pwm_mean', 'overshoot', 'emerg', 'fell', 'log',
                 'yaw_cmd', 'yaw_mean', 'yaw_sd', 'yaw_ratio']


# ---------------------------------------------------------------------------
# 劇本與分析（純計算，不碰硬體）
# ---------------------------------------------------------------------------
class Phase:
    def __init__(self, kind, dur, cycle=0):
        self.kind = kind        # 'W' 前進 / 'X' 後退 / 'R' 休息
        self.dur = dur
        self.cycle = cycle
        self.vcmd = None        # 這段送出的目標速度（W = vmax、X = vback），用來算超速倍數
        self.wcmd = None        # 這段最後送出的角速度 rad/s（steer != 0 時用來算轉向達成率）
        self.t0 = None          # 實際開始時間（session 秒數）
        self.t1 = None

    def __repr__(self):
        return f'{self.kind}{self.dur:g}s'


def build_schedule(hold, rest, cycles, order='WX'):
    sched = []
    for c in range(1, cycles + 1):
        for k in order:
            sched.append(Phase(k, hold, c))
            sched.append(Phase('R', rest, c))
    return sched


def analyze(recs, sched, settle=SETTLE, tail=7.0):
    """recs：[(t, state, ang, sa, v, pwm, yr)]；回傳每個 W / X 段的統計。
    倒地 / 緊急模式 / 超速倍數都看「這段開始 → 放開後 tail 秒（或下一個 W / X 段開始）」，
    因為失控常常是放開油門之後才發生。超速倍數 = 峰值速度 ÷ 目標速度（1 = 剛好；3 = 衝到 3 倍）。"""
    out = []
    moves = [p for p in sched if p.kind in 'WX' and p.t0 is not None]
    for ph in moves:
        t1 = ph.t1 if ph.t1 is not None else ph.t0 + ph.dur
        nxt = min([p.t0 for p in moves if p.t0 > ph.t0] or [float('inf')])
        t_end = min(t1 + tail, nxt)
        win = [r for r in recs if ph.t0 + settle <= r[0] <= t1]
        span = [r for r in recs if ph.t0 <= r[0] <= t_end]
        fell = any(r[1] == 'FALLEN' for r in span)
        emerg = any(r[1] in ('EMERGENCY', 'FALLEN') for r in span)
        sign = -1.0 if ph.kind == 'W' else 1.0       # 前進時 V 為負、後退時 V 為正；統一成「沿指令方向」為正
        sp = [sign * r[4] * V_TO_CMS for r in win if r[1] != 'FALLEN']
        over = ''
        if ph.vcmd:
            peak_v = max([sign * r[4] for r in span if r[1] == 'NORMAL'] or [0.0])
            over = peak_v / ph.vcmd
        row = dict(cycle=ph.cycle, phase=ph.kind, n=len(sp), fell=int(fell), emerg=int(emerg), overshoot=over)
        if len(sp) >= 3:
            row.update(speed_mean_cms=st.mean(sp), speed_sd_cms=st.pstdev(sp), speed_peak_cms=max(sp),
                       sa_sat_pct=100.0 * sum(abs(r[3]) >= SAT_DEG for r in win) / len(win),
                       pwm_mean=st.mean(abs(r[5]) for r in win))
        else:
            row.update(speed_mean_cms='', speed_sd_cms='', speed_peak_cms='', sa_sat_pct='', pwm_mean='')
        row.update(yaw_cmd='', yaw_mean='', yaw_sd='', yaw_ratio='')
        if ph.wcmd is not None and abs(ph.wcmd) >= 0.05:
            # YR 的正負和指令角速度同向（右轉為正，前進 / 後退都一樣），乘上指令符號後「轉對方向」為正
            sg = 1.0 if ph.wcmd > 0 else -1.0
            yr = [sg * r[6] for r in win if r[1] == 'NORMAL' and len(r) > 6]
            if len(yr) >= 3:
                row.update(yaw_cmd=abs(ph.wcmd), yaw_mean=st.mean(yr), yaw_sd=st.pstdev(yr),
                           yaw_ratio=st.mean(yr) / abs(ph.wcmd))
        out.append(row)
    return out


def fmt(x, nd=1):
    return f'{x:.{nd}f}' if isinstance(x, (int, float)) else '-'


def print_phase_table(rows):
    print('\n  輪  段   n  平均速度  標準差  峰值  超速倍數  SA飽和   PWM   倒地   (速度 cm/s，沿指令方向為正；超速倍數 = 峰值速度 / 目標速度，含放開後約 7 秒)')
    for r in rows:
        note = ''
        if isinstance(r['speed_mean_cms'], float) and r['speed_mean_cms'] < 2.0:
            note = '  <- 幾乎沒動（卡住）'
        if isinstance(r['overshoot'], float) and r['overshoot'] >= 2.0:
            note += '  <- 衝過頭'
        if r['fell']:
            note += '  <- 倒地'
        elif r['emerg']:
            note += '  <- 進入緊急模式'
        yaw = ''
        if isinstance(r.get('yaw_ratio'), float):
            yaw = f"  轉向 指令{r['yaw_cmd']:.2f} 實際{r['yaw_mean']:.2f} rad/s 達成{100 * r['yaw_ratio']:.0f}% 晃動SD{r['yaw_sd']:.2f}"
            if r['yaw_ratio'] < 0.5:
                note += '  <- 轉不過去'
        print(f"  {r['cycle']:>2}  {r['phase']}  {r['n']:>3}  {fmt(r['speed_mean_cms']):>7}  {fmt(r['speed_sd_cms']):>6}  "
              f"{fmt(r['speed_peak_cms']):>5}  {fmt(r['overshoot']):>7}x  {fmt(r['sa_sat_pct'], 0):>5}%  {fmt(r['pwm_mean'], 0):>4}  {'是' if r['fell'] else '否':>3}{yaw}{note}")


def report(path, label=None):
    """把 autotest_results.csv 依參數分組：每組幾輪、平均、輪與輪之間的標準差（越小代表越可重現）"""
    if not os.path.exists(path):
        print('還沒有結果檔：', path)
        return
    groups = {}
    with open(path, newline='', encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            if label and label not in r['label']:
                continue
            key = (r['label'], r['vmax'], r['vback'], r['vmin'], r['vmin_back'], r['kp'], r['kd'], r['steer'], r['phase'])
            groups.setdefault(key, []).append(r)
    print('\n標籤        vmax vback vmin vminB  Kp   Kd  steer 段 輪數  平均速度 輪間SD 超速倍數  SA飽和  倒地  轉向達成率')
    for key in sorted(groups):
        rs = groups[key]
        sp = [float(r['speed_mean_cms']) for r in rs if r['speed_mean_cms'] != '']
        sat = [float(r['sa_sat_pct']) for r in rs if r['sa_sat_pct'] != '']
        falls = sum(int(r['fell']) for r in rs)
        ov = [float(r['overshoot']) for r in rs if r.get('overshoot') not in (None, '')]
        mean = st.mean(sp) if sp else None
        sd = st.stdev(sp) if len(sp) >= 2 else None
        lab, vmax, vback, vmin, vminb, kp, kd, steer, ph = key
        yr = [float(r['yaw_ratio']) for r in rs if r.get('yaw_ratio') not in (None, '')]
        yaw_txt = f'{100 * st.mean(yr):.0f}%' if yr else '-'
        print(f'{lab[:10]:<10} {vmax:>5} {vback:>5} {vmin:>4} {vminb:>5} {kp:>4} {kd:>4} {steer:>5}  {ph}  {len(rs):>3}  '
              f'{fmt(mean):>7}  {fmt(sd):>6}  {fmt(st.mean(ov) if ov else None):>6}x  {fmt(st.mean(sat) if sat else None, 0):>5}%  {falls:>3}  {yaw_txt:>8}')
    print('\n看法：「輪間SD」比兩組平均速度的差距還大，代表那個差距只是雜訊，不能據此選參數。')
    print('超速倍數接近 1 才是好的；超過 2 代表速度環衝過頭（控制不住），倒地欄是整組的倒地次數。')


def list_results(path):
    """列出結果檔每一筆（編號從 1 開始），給 --drop 用"""
    if not os.path.exists(path):
        print('還沒有結果檔：', path)
        return
    with open(path, newline='', encoding='utf-8-sig') as f:
        for i, r in enumerate(csv.DictReader(f), 1):
            try:
                mean = f"{float(r['speed_mean_cms']):.1f}"
            except (ValueError, KeyError):
                mean = '-'
            try:
                yaw = f"  steer {r['steer']} 轉向達成 {100 * float(r['yaw_ratio']):.0f}%"
            except (ValueError, KeyError, TypeError):
                yaw = ''
            print(f"{i:>3}  {r['time'][11:]}  {r['label']:<8} {r['phase']}  vmax {r['vmax']} vback {r['vback']} vminB {r['vmin_back']}  平均 {mean} cm/s  倒地 {r['fell']}{yaw}")


def _count_rows(path):
    if not os.path.exists(path):
        return 0
    with open(path, newline='', encoding='utf-8-sig') as f:
        return sum(1 for _ in csv.DictReader(f))


def _upgrade_results(path):
    """結果檔尾端新增欄位時，把舊資料補上空白欄（不改名、不遺失）"""
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=RESULT_FIELDS, restval='')
        w.writeheader()
        w.writerows(rows)


def drop_results(path, numbers):
    """刪掉指定編號的結果（例如撞牆、被碰到的那次），其餘保留。回傳實際刪掉幾筆"""
    if not os.path.exists(path):
        return 0
    with open(path, newline='', encoding='utf-8-sig') as f:
        rd = csv.DictReader(f)
        fields = rd.fieldnames
        rows = list(rd)
    drop = {n for n in numbers if 1 <= n <= len(rows)}
    keep = [r for i, r in enumerate(rows, 1) if i not in drop]
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(keep)
    return len(drop)


# ---------------------------------------------------------------------------
# 實車執行
# ---------------------------------------------------------------------------
def make_parser():
    ap = argparse.ArgumentParser(description='自動重複測試（需要韌體 v32 以上的 #D 指令）')
    ap.add_argument('--port', default=None)
    ap.add_argument('--baud', type=int, default=None)
    ap.add_argument('--label', default='test', help='這組測試的名稱（報表用）')
    ap.add_argument('--vmax', type=float, default=DEFAULTS['vmax'])
    ap.add_argument('--vback', type=float, default=DEFAULTS['vback'])
    ap.add_argument('--vmin', type=float, default=DEFAULTS['vmin'])
    ap.add_argument('--vmin-back', type=float, default=DEFAULTS['vmin_back'])
    ap.add_argument('--steer', type=float, default=0.0, help='前進 / 後退時的方向值 -1～1（預設 0 = 直走）')
    ap.add_argument('--hold', type=float, default=5.0, help='每次按住幾秒（預設 5）')
    ap.add_argument('--rest', type=float, default=3.0, help='每次放開休息幾秒（預設 3）')
    ap.add_argument('--cycles', type=int, default=3, help='重複幾輪（預設 3）')
    ap.add_argument('--order', default='WX', help='每輪的順序：WX（預設）、XW、只測前進 W、只測後退 X')
    ap.add_argument('--kp', type=float, default=None, help='把 Kp_speed 設成這個值（韌體 ENABLE_TUNING 1；每次 ±0.5）')
    ap.add_argument('--kd', type=float, default=None, help='把直立環 Kd 設成這個值（每次 ±10）')
    ap.add_argument('--hz', type=float, default=DEFAULTS['hz'])
    ap.add_argument('-i', '--interactive', action='store_true', help='測完一個等你輸入下一個（連線不中斷，Kp / Kd 設定會保留）')
    ap.add_argument('--report', action='store_true', help='不測試，只印出累積結果的分組比較')
    ap.add_argument('--list', action='store_true', help='不測試，列出結果檔每一筆（附編號）')
    ap.add_argument('--drop', type=int, nargs='+', metavar='N', help='不測試，把結果檔裡編號 N 的那幾筆刪掉（編號用 --list 看；例如撞牆那次）')
    ap.add_argument('--dry-run', action='store_true', help='不連線，只印出劇本與指令')
    return ap


def check_args(a):
    if not (0.5 <= a.vmax <= 20.0) or not (0.5 <= a.vback <= 20.0) or not (-1.0 <= a.steer <= 1.0):
        raise ValueError('數值超出範圍：--vmax 0.5～20、--vback 0.5～20、--steer -1～1')
    if a.hold < 3.0:
        raise ValueError('--hold 至少 3 秒（前 1.5 秒是爬升期不計入，太短量不到穩態）')
    if not a.order or set(a.order.upper()) - set('WX'):
        raise ValueError('--order 只能用 W 和 X（例如 WX、XW、W、X）')
    if a.cycles < 1:
        raise ValueError('--cycles 至少 1')


class Ctx:
    """連線與共用狀態（互動模式下整個流程只連線一次）"""

    def __init__(self, control, dry_run):
        self.control = control
        self.dry_run = dry_run
        self.link = self.logger = self.session = None
        self.recs = []
        self.tel = {}
        self.inputs = Inputs()
        self.kp = KP0
        self.kd = KD0
        self.last_rows = 0     # 最近一次測試寫進結果檔的筆數（互動模式的 d 指令用）

    def connect(self, port, baud):
        control = self.control
        try:
            self.link = control.RobotLink(port, baud)
        except Exception as e:
            raise SystemExit(f'無法開啟序列埠 {port}：{e}')
        self.logger = control.TelemetryLogger()
        self.session = control.Session(self.link, self.logger)
        ctx = self

        def reader():
            while ctx.session.running:
                try:
                    line = ctx.link.read_line()
                except Exception:
                    break
                if not line:
                    time.sleep(0.01)
                    continue
                state, f = control.parse_telemetry(line)
                if state is not None:
                    ctx.tel.clear()
                    ctx.tel.update(f)
                    ctx.tel['state'] = state
                    try:
                        ctx.recs.append((ctx.session.elapsed(), state, float(f.get('ANG', 0)), float(f.get('SA', 0)),
                                         float(f.get('V', 0)), float(f.get('PWM_L', 0)), float(f.get('YR', 0))))
                    except ValueError:
                        pass
                elif line.startswith('>>'):
                    print('\n' + line)
                try:
                    ctx.logger.log(ctx.session.elapsed(), ctx.session.active_cmd(), line)
                except ValueError:
                    break

        threading.Thread(target=reader, daemon=True).start()
        print(f'記錄檔：{self.logger.path}')

    def aborted(self):
        return any(ev in ('stop', 'quit') for ev in self.inputs.pop_events())

    def stop_robot(self):
        if self.link:
            for _ in range(3):
                try:
                    self.link.send('#D 0 0\n')
                    time.sleep(0.05)
                except Exception:
                    break

    def close(self):
        if self.link:
            self.stop_robot()
            try:
                self.link.send('s')
            except Exception:
                pass
            time.sleep(0.5)
            self.session.running = False
            time.sleep(0.2)
            self.link.close()
            self.logger.close()

    def tune(self, a):
        """依目標值設定 Kp_speed / Kd（相對於目前值，所以互動模式下不會重複累加）"""
        for name, target, cur, step, up, down in (('Kp_speed', a.kp, self.kp, 0.5, 'l', 'j'),
                                                  ('Kd', a.kd, self.kd, 10.0, 'h', 'g')):
            if target is None:
                continue
            n = int(round((target - cur) / step))
            if n == 0:
                continue
            print(f'設定 {name}：{cur:g} -> {cur + n * step:g}（按 {up if n > 0 else down} {abs(n)} 次）')
            for _ in range(abs(n)):
                self.link.send(up if n > 0 else down)
                time.sleep(0.25)
            if name == 'Kp_speed':
                self.kp = cur + n * step
            else:
                self.kd = cur + n * step

    def wait_stable(self):
        print('\n>> 請把車子扶正、放到起點、放開手，站穩 3 秒後會自動開始。（S / 空白 / Q 隨時停止）')
        t_ok = None
        t_wait = time.time()
        while True:
            if self.aborted():
                raise KeyboardInterrupt
            tel = self.tel
            if tel.get('state') == 'NORMAL' and abs(float(tel.get('V', 99))) < 10.0 and abs(float(tel.get('ANG', 99))) < 6.0:   # 靜止時 V 會在 ±7 內抖動
                t_ok = t_ok or time.time()
                if time.time() - t_ok >= 3.0:
                    return
            else:
                t_ok = None
            if time.time() - t_wait > 120:
                raise TimeoutError('120 秒內車子沒有站穩')
            time.sleep(0.1)


def run_test(a, ctx, results_path):
    """跑一次劇本。回傳 'ok' / 'fell' / 'aborted' / 'timeout'"""
    check_args(a)
    model = DriveModel(vmax=a.vmax, vback=a.vback, vmin=a.vmin, vmin_back=a.vmin_back)
    sched = build_schedule(a.hold, a.rest, a.cycles, a.order.upper())
    total = sum(p.dur for p in sched)
    link, session, tel = ctx.link, ctx.session, ctx.tel
    print(f'\n劇本：{" ".join(map(repr, sched))}  共 {total:.0f} 秒')
    print(f'參數：vmax {a.vmax} vback {a.vback} vmin {a.vmin} vmin_back {a.vmin_back} steer {a.steer}'
          f' Kp {a.kp if a.kp is not None else ctx.kp:g} Kd {a.kd if a.kd is not None else ctx.kd:g}  標籤 {a.label}')
    status = 'ok'
    ctx.last_rows = 0
    for p in sched:
        p.vcmd = a.vmax if p.kind == 'W' else (a.vback if p.kind == 'X' else None)
    try:
        if link:
            ctx.inputs.pop_events()          # 清掉在提示字元打字時誤觸的 S / Q
            ctx.tune(a)
            ctx.wait_stable()
            print('>> 開始。')
        dt = 1.0 / a.hz
        nxt = time.time()
        steer_dir = 0 if a.steer == 0 else (1 if a.steer > 0 else -1)
        for ph in sched:
            ph.t0 = session.elapsed() if session else 0.0
            thr_dir = {'W': 1, 'X': -1, 'R': 0}[ph.kind]
            for i in range(int(round(ph.dur * a.hz))):
                if ctx.aborted():
                    raise KeyboardInterrupt
                if link and tel.get('state') == 'FALLEN':
                    status = 'fell'
                    break
                model.update_keys(dt, thr_dir, steer_dir if thr_dir else 0)
                if abs(a.steer) < 1.0 and abs(model.steer) > abs(a.steer):
                    model.steer = a.steer
                v, w = model.output()
                if thr_dir:
                    ph.wcmd = w
                cmd = format_cmd(v, w)
                if link:
                    link.send(cmd)
                    session.active_key = f'D{v:+.1f}/{w:+.2f}' if (v or w) else None
                    sys.stdout.write(f'\r第 {ph.cycle}/{a.cycles} 輪  {ph.kind}  {i * (1 / a.hz):3.1f}/{ph.dur:g}s  v {v:+.1f} w {w:+.2f}  '
                                     f"{tel.get('state', '-')} V {tel.get('V', '-')}".ljust(80))
                    sys.stdout.flush()
                    nxt += dt
                    time.sleep(max(0.0, nxt - time.time()))
                else:
                    print(f'{ph.kind} {i * dt:4.1f}s  {cmd.strip()}')
            ph.t1 = session.elapsed() if session else ph.dur
            if status == 'fell':
                break
        if link and status == 'ok':
            # 劇本結束後再觀察 4 秒：失控常常是放開油門之後才發生，不能太早收工
            print('\n>> 劇本結束，再觀察 4 秒（不要碰車子）...')
            ctx.stop_robot()
            t_end = time.time() + 4.0
            while time.time() < t_end:
                if ctx.aborted():
                    raise KeyboardInterrupt
                time.sleep(0.1)
            if tel.get('state') == 'FALLEN':
                status = 'fell'
        print()
    except KeyboardInterrupt:
        print('\n>> 使用者中止')
        status = 'aborted'
    except TimeoutError as e:
        print(f'\n>> {e}，取消這次測試。')
        status = 'timeout'
    finally:
        ctx.stop_robot()
    if status == 'fell':
        print('\n!! 倒地，測試中止。這之前的資料仍有記錄。')
    if link:
        done = [p for p in sched if p.t1 is not None]
        rows = analyze(ctx.recs, done)
        if rows:
            print_phase_table(rows)
            os.makedirs(ctx.control.LOG_ROOT, exist_ok=True)
            if os.path.exists(results_path):
                with open(results_path, newline='', encoding='utf-8-sig') as f:
                    old_header = next(csv.reader(f), [])
                if old_header != RESULT_FIELDS:
                    if old_header == RESULT_FIELDS[:len(old_header)]:
                        _upgrade_results(results_path)    # 只是尾端多了新欄位：原檔補空白欄，舊資料全部保留
                    else:                                  # 欄位改版：舊檔改名保留，避免欄位錯位
                        os.replace(results_path, results_path.replace('.csv', f'_old_{datetime.now():%H%M%S}.csv'))
            new = not os.path.exists(results_path)
            with open(results_path, 'a', newline='', encoding='utf-8-sig') as f:
                w = csv.DictWriter(f, fieldnames=RESULT_FIELDS)
                if new:
                    w.writeheader()
                for r in rows:
                    r.update(time=datetime.now().strftime('%Y-%m-%d %H:%M:%S'), label=a.label, vmax=a.vmax, vback=a.vback,
                             vmin=a.vmin, vmin_back=a.vmin_back, kp=ctx.kp, kd=ctx.kd, steer=a.steer, hold=a.hold,
                             log=os.path.basename(ctx.logger.path))
                    w.writerow(r)
            ctx.last_rows = len(rows)
            print(f'\n結果已追加到 {results_path}')
    return status


def main():
    ap = make_parser()
    a = ap.parse_args()
    import control
    results_path = os.path.join(control.LOG_ROOT, 'autotest_results.csv')
    if a.list:
        list_results(results_path)
        return
    if a.drop:
        n = drop_results(results_path, a.drop)
        print(f'已刪除 {n} 筆。目前的結果：')
        list_results(results_path)
        return
    if a.report:
        report(results_path)
        return
    try:
        check_args(a)
    except ValueError as e:
        raise SystemExit(str(e))

    ctx = Ctx(control, a.dry_run)
    if not a.dry_run:
        start_keyboard(ctx.inputs)
        ctx.connect(a.port or control.COM_PORT, a.baud or control.BAUD_RATE)

    try:
        run_test(a, ctx, results_path)
        if a.interactive and not a.dry_run:
            last = a
            while True:
                try:
                    line = input('\n下一個測試？ Enter = 重複上一個 | 例：--order X --vback 4.0 --label vb40 | r = 報表 | d = 丟掉上一次（撞牆等）| q = 結束\n> ').strip()
                except EOFError:
                    break
                if line.lower() in ('q', 'quit', 'exit'):
                    break
                if line.lower() == 'r':
                    report(results_path)
                    continue
                if line.lower() == 'd':
                    n = drop_results(results_path, range(_count_rows(results_path) - ctx.last_rows + 1, _count_rows(results_path) + 1)) if ctx.last_rows else 0
                    print(f'已丟掉上一次測試的 {n} 筆結果。' if n else '沒有可丟的結果。')
                    ctx.last_rows = 0
                    continue
                if line:
                    try:
                        # 沒寫的選項沿用上一次的值；Kp / Kd 沿用「目前韌體裡的值」
                        new = ap.parse_args(shlex.split(line), namespace=copy.copy(last))
                    except SystemExit:
                        continue
                    last = new
                else:
                    new = last
                try:
                    run_test(new, ctx, results_path)
                except ValueError as e:
                    print(e)
        elif not a.dry_run:
            print('同一組參數請至少跑 2～3 次再比較：python autotest.py --report')
    except KeyboardInterrupt:
        print('\n>> 結束')
    finally:
        ctx.close()


if __name__ == '__main__':
    main()
