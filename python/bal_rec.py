# -*- coding: utf-8 -*-
"""
平衡震動診斷錄影（搭配韌體 bal_v28）

用法（先關掉 control.py / nav.py，同一時間只能有一個程式用序列埠）：
  python bal_rec.py                 車子站著不動，錄 3 次（每次 0.65 秒、每 10 ms 一筆）
  python bal_rec.py -n 5            錄 5 次
  python bal_rec.py --port COM5     指定序列埠
  python bal_rec.py --move w        移動模式：錄 2.6 秒（40 ms 一筆），腳本自動長按 W 1.2 秒再放開（--move x = 後退）
  python bal_rec.py --move w -n 3 --hold 1.5   錄 3 次、每次長按 1.5 秒（車子會往前跑十幾公分，前面要留空間）
  python bal_rec.py --keys hh       錄影前先送按鍵（h = Kd +10、g = Kd -10），可和其他選項合用
  python bal_rec.py --file xxx.csv  不連線，只分析以前錄好的檔案

輸出：logs/日期/rec_時間.csv（原始資料）和 .png（角度 / PWM / 編碼器曲線），並印出分析結果
"""
import argparse
import csv
import math
import os
import statistics as st
import sys
import time
from datetime import datetime

DT = 0.01013  # 控制週期 (s)，與 control.py / 韌體一致


def dominant_freq(x, dt=DT, fmin=0.8):
    """純 Python DFT：回傳 (主頻 Hz, 該頻率的振幅)。100 點 → 頻率解析度約 1 Hz"""
    n = len(x)
    m = sum(x) / n
    y = [v - m for v in x]
    best_f, best_a = 0.0, 0.0
    for k in range(1, n // 2 + 1):
        f = k / (n * dt)
        if f < fmin:
            continue
        re = sum(y[i] * math.cos(2 * math.pi * k * i / n) for i in range(n))
        im = sum(y[i] * math.sin(2 * math.pi * k * i / n) for i in range(n))
        a = 2.0 * math.hypot(re, im) / n
        if a > best_a:
            best_f, best_a = f, a
    return best_f, best_a


def analyze(rec):
    """rec: [(ang_deg, gyro, sp, bal, pwm, dl), ...] → 分析結果 dict（舊檔沒有的欄位是 None）"""
    ang = [r[0] for r in rec]
    pwm = [r[4] for r in rec]
    dl = [r[5] for r in rec]
    n = len(rec)
    secs = n * DT
    flips = sum(1 for a, b in zip(pwm, pwm[1:]) if a * b < 0)
    fa, aa = dominant_freq(ang)
    fp, ap = dominant_freq(pwm)
    # 「有推力但輪子沒動」：|PWM| >= 10 卻連續 0 tick
    push = [i for i in range(n) if abs(pwm[i]) >= 10]
    stuck = sum(1 for i in push if dl[i] == 0)
    if rec[0][1] is not None:
        push, stuck = [], 0
    # 輪子動作的「段數」：tick 從 0 變成非 0 的次數
    bursts = sum(1 for a, b in zip(dl, dl[1:]) if a == 0 and b != 0)
    if rec[0][1] is not None:
        bursts = 0
    g = gyro_check(rec)
    g2 = gyro_check(rec, 5)
    new = rec[0][1] is not None  # 新版站立檔：最後一欄是暫存器陀螺儀，不是編碼器
    return dict(
        gyro=g, gyro_reg=g2,
        ang_sd=st.pstdev(ang), ang_min=min(ang), ang_max=max(ang),
        pwm_sd=st.pstdev(pwm), pwm_peak=max(abs(p) for p in pwm),
        flips_per_s=flips / secs, f_ang=fa, a_ang=aa, f_pwm=fp, a_pwm=ap,
        push_n=len(push), stuck_n=stuck, zero_frac=(0.0 if new else sum(1 for v in dl if v == 0) / n),
        bursts_per_s=bursts / secs, tick_sd=st.pstdev(dl))


KP, KD = 1275.0, 80.0  # 韌體 Kp_balance_normal / Kd_balance_normal


def gyro_check(rec, col=1):
    """檢查陀螺儀有沒有真的在提供阻尼。回傳 dict 或 None（舊檔沒有陀螺儀欄位）
    - slope：陀螺儀 ≈ slope × (角度對時間的微分)；正常應接近 +1
    - kd_eff / kp_eff：用 balance = kp·(目標角 − 角度) + kd·陀螺儀 回歸出的實際增益
      韌體設定 Kd=80 → 回歸出來的 kd 應約 -80（rad 制）、kp 約 +1275"""
    if rec[0][1] is None:
        return None
    n = len(rec)
    ang = [math.radians(r[0]) for r in rec]
    gy = [math.radians(r[col]) for r in rec]
    dang = [(ang[min(i + 1, n - 1)] - ang[max(i - 1, 0)]) / (DT * (min(i + 1, n - 1) - max(i - 1, 0))) for i in range(n)]
    sxx = sum(d * d for d in dang)
    sxy = sum(d * g for d, g in zip(dang, gy))
    syy = sum(g * g for g in gy)
    slope = sxy / sxx if sxx else 0.0
    corr = sxy / math.sqrt(sxx * syy) if sxx and syy else 0.0
    # balance = kp * e + kd * gyro，e = 目標角 - 角度（rad），最小平方法（2 參數）
    e = [math.radians(r[2] / 20.0) - a for r, a in zip(rec, ang)]
    bal = [r[3] for r in rec]
    a11 = sum(v * v for v in e)
    a12 = sum(u * v for u, v in zip(e, gy))
    a22 = syy
    b1 = sum(u * v for u, v in zip(e, bal))
    b2 = sum(u * v for u, v in zip(gy, bal))
    det = a11 * a22 - a12 * a12
    kp_eff = (b1 * a22 - b2 * a12) / det if det else 0.0
    kd_eff = (a11 * b2 - a12 * b1) / det if det else 0.0
    pred = [KP * u - KD * v for u, v in zip(e, gy)]
    err = st.pstdev([p - b for p, b in zip(pred, bal)])
    return dict(slope=slope, corr=corr, kp_eff=kp_eff, kd_eff=kd_eff, pred_err=err, gyro_sd=st.pstdev([r[col] for r in rec]))


def verdict(a):
    lines = []
    g = a.get('gyro')
    if g:
        if abs(g['corr']) < 0.5:
            lines.append('陀螺儀和角度變化幾乎不相關（相關係數 %.2f）：D 項吃到的是雜訊或錯的軸' % g['corr'])
        elif g['slope'] < 0:
            lines.append('陀螺儀方向和角度變化「相反」（斜率 %.2f）：D 項變成正回授，會越推越晃' % g['slope'])
        elif g['slope'] < 0.5 or g['slope'] > 2.0:
            lines.append('陀螺儀量值和角度變化差很多（斜率 %.2f，正常約 1）：D 項實際強度不是 Kd=80 的效果' % g['slope'])
    f = a['f_pwm']
    if a['pwm_sd'] < 8 and a['ang_sd'] < 0.3:
        lines.append('站得很穩（PWM 起伏小、角度晃動小）')
    if f >= 6:
        lines.append('主要是「快速抖動」（%.1f Hz）：可能是角速度雜訊被 Kd 放大，或馬達 / 齒輪間隙來回' % f)
    elif f >= 1.5:
        lines.append('主要是 %.1f Hz 的「週期性來回」：典型的死區 / 齒輪間隙 / 靜摩擦造成的極限環' % f)
    elif a['pwm_sd'] >= 8:
        lines.append('主要是慢速（%.1f Hz）搖晃：偏向速度外環或重心 / 設定角度造成' % f)
    if a['push_n'] > 0 and a['stuck_n'] / a['push_n'] > 0.35:
        lines.append('有 %.0f%% 的推力時間輪子沒轉（卡住）：靜摩擦 / 死區補償不夠，推力要累積到夠大才一次衝出去 → 頓挫感' %
                     (100.0 * a['stuck_n'] / a['push_n']))
    if a['bursts_per_s'] > 4:
        lines.append('輪子每秒起停 %.1f 次：走走停停' % a['bursts_per_s'])
    return lines or ['沒有明顯特徵，請多錄幾次']


def report(rec, tag=''):
    a = analyze(rec)
    print('--- %s分析 ---' % tag)
    print('角度：標準差 %.2f°，範圍 %.2f ~ %.2f°，主頻 %.1f Hz（振幅 %.2f°）' %
          (a['ang_sd'], a['ang_min'], a['ang_max'], a['f_ang'], a['a_ang']))
    print('PWM：標準差 %.1f，峰值 %d，每秒正負換向 %.1f 次，主頻 %.1f Hz（振幅 %.1f）' %
          (a['pwm_sd'], a['pwm_peak'], a['flips_per_s'], a['f_pwm'], a['a_pwm']))
    if a['gyro'] is None:
        print('輪子：%.0f%% 的時間完全沒動，每秒起停 %.1f 次；有推力(|PWM|≥10)共 %d 筆，其中沒轉 %d 筆' %
              (100 * a['zero_frac'], a['bursts_per_s'], a['push_n'], a['stuck_n']))
    g = a['gyro']
    if g:
        print('陀螺儀：標準差 %.1f°/s；陀螺儀 ≈ %.2f × 角度微分（相關 %.2f）；回歸增益 kp=%.0f（設定 %.0f）、kd=%.0f（設定 %.0f）；公式預測誤差 %.1f' %
              (g['gyro_sd'], g['slope'], g['corr'], g['kp_eff'], KP, -g['kd_eff'], KD, g['pred_err']))
    g2 = a['gyro_reg']
    if g2:
        print('暫存器陀螺儀（直接讀 GYRO_XOUT）：≈ %.2f × 角度微分（相關 %.2f），標準差 %.1f°/s' %
              (g2['slope'], g2['corr'], g2['gyro_sd']))
        if g and g['gyro_sd'] > 0:
            print('DMP 陀螺儀 / 暫存器陀螺儀 的量值比 = %.3f（正常應為 1）' % (g['gyro_sd'] / g2['gyro_sd'] if g2['gyro_sd'] else 0))
    for s in verdict(a):
        print('  ★', s)
    return a


def decode_move(rec):
    """移動模式的欄位換算：回傳 dict of list（V、TV 單位 ticks/40ms；SA、BS 單位 °）"""
    return dict(ang=[r[0] for r in rec], v=[r[1] / 4.0 for r in rec], tv=[r[2] / 4.0 for r in rec],
                sa=[r[3] / 20.0 for r in rec], pwm=[r[4] for r in rec], bs=[r[5] / 50.0 for r in rec])


def analyze_move(rec):
    d = decode_move(rec)
    n = len(rec)
    dt = 4 * DT
    tv, v, ang, sa, pwm = d['tv'], d['v'], d['ang'], d['sa'], d['pwm']
    ipk = max(range(n), key=lambda i: abs(tv[i]))
    tv_peak = abs(tv[ipk])
    if tv_peak < 1.0:
        return None  # 沒有收到移動指令
    sgn = 1.0 if tv[ipk] > 0 else -1.0
    # 放開時間點：目標速度峰值之後第一次掉到 97% 以下
    irel = next((i for i in range(ipk, n) if abs(tv[i]) < 0.97 * tv_peak), n - 1)
    istart = next((i for i in range(n) if abs(tv[i]) > 0.3), 0)
    icruise = next((i for i in range(n) if abs(tv[i]) >= 0.9 * tv_peak), ipk)
    vdir = [sgn * x for x in v]  # 指令方向為正
    v_peak = max(vdir)
    rev = max(0.0, -min(vdir[irel:])) if irel < n else 0.0  # 放開後往反方向的最大速度
    cr = slice(icruise, max(irel, icruise + 1))
    po = slice(irel, n)
    flips = lambda x: sum(1 for a, b in zip(x, x[1:]) if a * b < 0)  # noqa: E731
    # 放開後多久速度降到 |V|<2 並維持
    t_settle = None
    for i in range(irel, n):
        if all(abs(x) < 2.0 for x in v[i:min(i + 4, n)]):
            t_settle = (i - irel) * dt
            break
    return dict(
        tv_peak=tv_peak, v_peak=v_peak, ratio=v_peak / tv_peak, t_rise=(icruise - istart) * dt,
        t_cruise=(max(irel, icruise + 1) - icruise) * dt,
        cr_ang_sd=st.pstdev(ang[cr]) if len(ang[cr]) > 1 else 0.0,
        cr_sa_rng=(max(sa[cr]) - min(sa[cr])) if len(sa[cr]) > 1 else 0.0,
        cr_pwm_flips=flips(pwm[cr]) / max(1e-6, (max(irel, icruise + 1) - icruise) * dt),
        po_rev_v=rev, po_ang_min=min(ang[po]), po_ang_max=max(ang[po]),
        po_sa_rng=max(sa[po]) - min(sa[po]), po_sa_max=max(abs(x) for x in sa[po]),
        po_pwm_peak=max(abs(x) for x in pwm[po]),
        t_settle=t_settle, ang_pre=st.mean(ang[:max(1, istart)]), sgn=sgn)


def report_move(rec, tag=''):
    a = analyze_move(rec)
    print('--- %s移動分析 ---' % tag)
    if not a:
        print('沒有看到移動指令（目標速度一直接近 0）：確認 W/X 有被收到、車子是站著開始的')
        return None
    print('起步到巡航：%.2f s；目標速度峰值 %.1f，實際速度峰值 %.1f（%.1f 倍）' %
          (a['t_rise'], a['tv_peak'], a['v_peak'], a['ratio']))
    print('巡航中（%.2f s）：角度標準差 %.2f°，補償角(SA)擺動 %.1f°，PWM 每秒換向 %.1f 次' %
          (a['t_cruise'], a['cr_ang_sd'], a['cr_sa_rng'], a['cr_pwm_flips']))
    print('放開之後：反向最大速度 %.1f；角度範圍 %.1f ~ %.1f°（站立約 -1.5°）；補償角最大 %.1f°，擺動 %.1f°；PWM 峰值 %d；速度降到 |V|<2 花 %s' %
          (a['po_rev_v'], a['po_ang_min'], a['po_ang_max'], a['po_sa_max'], a['po_sa_rng'], a['po_pwm_peak'],
           ('%.2f s' % a['t_settle']) if a['t_settle'] is not None else '超過錄影範圍'))
    if a['ratio'] > 1.8:
        print('  ★ 實際速度衝到目標的 %.1f 倍：速度外環壓不住（累積的傾角補償太大 / 積分過衝）' % a['ratio'])
    if a['po_rev_v'] > 3.0:
        print('  ★ 放開後車子往反方向溜 %.1f：煞車過頭 → 停下瞬間的來回晃' % a['po_rev_v'])
    if a['po_sa_rng'] > 4.0:
        print('  ★ 放開後補償角擺動 %.1f°：煞車時傾角被推得很大' % a['po_sa_rng'])
    if a['cr_pwm_flips'] > 6:
        print('  ★ 巡航中 PWM 每秒換向 %.1f 次：移動中就在抖' % a['cr_pwm_flips'])
    return a


def parse_rec_line(line):
    """'REC:12,-34,3,-20,5,15,2' → (角度°, 陀螺儀, 目標角編碼, 直立環輸出, pwm, dl)；不是資料行回 None"""
    if not line.startswith('REC:') or line.startswith('REC:END'):
        return None
    try:
        i, ang, gyro, sp, bal, pwm, dl = [int(v) for v in line[4:].split(',')]
    except ValueError:
        return None
    return (ang / 100.0, gyro, sp, bal, pwm, dl)


def save(recs, path):
    """recs: [(mode, rows), ...]；mode 0 = 站立，1 = 移動（欄位意義見 decode_move）"""
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['run', 'i', 't_s', 'ang_deg', 'gyro_dps', 'sp_x20', 'bal_pwm', 'pwm_L', 'dL_ticks', 'mode'])
        for r, (mode, rec) in enumerate(recs):
            step = DT * (4 if mode else 1)
            for i, row in enumerate(rec):
                w.writerow([r, i, round(i * step, 4)] + list(row) + [mode])


def plot(recs, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    move = recs[0][0] == 1
    if move:
        fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
        for r, (mode, rec) in enumerate(recs):
            d = decode_move(rec)
            t = [i * 4 * DT for i in range(len(rec))]
            c = 'C%d' % r
            ax[0].plot(t, d['ang'], color=c, label='#%d' % (r + 1))
            ax[1].plot(t, d['v'], color=c)
            ax[1].plot(t, d['tv'], color=c, ls='--', alpha=0.6)
            ax[2].plot(t, d['sa'], color=c)
            ax[2].plot(t, d['bs'], color=c, ls=':', alpha=0.6)
            ax[3].plot(t, d['pwm'], color=c)
        labels = ['角度 (°)', '速度 V（實線）/ 目標 TV（虛線）', '補償角 SA（實線）/ 起步推力 BS（點線）(°)', 'PWM_L']
        ax[-1].set_xlabel('時間 (s)')
    else:
        fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        for r, (mode, rec) in enumerate(recs):
            t = [i * DT for i in range(len(rec))]
            ax[0].plot(t, [x[0] for x in rec], label='#%d' % (r + 1))
            ax[1].plot(t, [x[4] for x in rec])
            ax[2].plot(t, [x[5] for x in rec])
        labels = ['角度 (°)', 'PWM_L', '左輪 tick / 10 ms']
        ax[-1].set_xlabel('時間 (s)')
    for a, lb in zip(ax, labels):
        a.set_ylabel(lb, fontsize=8)
        a.grid(alpha=0.3)
    ax[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


def load(path):
    """回傳 [(mode, rows), ...]"""
    runs = {}
    modes = {}
    with open(path, encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            if 'gyro_dps' in row:
                t = (float(row['ang_deg']), int(row['gyro_dps']), int(row['sp_x20']), int(row['bal_pwm']),
                     int(row['pwm_L']), int(row['dL_ticks']))
            else:  # 舊版（沒有陀螺儀欄位）
                t = (float(row['ang_deg']), None, None, None, int(row['pwm_L']), int(row['dL_ticks']))
            runs.setdefault(int(row['run']), []).append(t)
            modes[int(row['run'])] = int(row.get('mode') or 0)
    return [(modes[k], runs[k]) for k in sorted(runs)]


def report_any(mode, rec, tag=''):
    return report_move(rec, tag) if mode == 1 else report(rec, tag)


def capture(port, baud, n_runs, move='', hold=1.2, keys=''):
    """move：'' = 站立錄影；'w' / 'x' = 每次自動長按該鍵；'wx' = 交替前進 / 後退（車子不會越跑越遠）"""
    import serial
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()
    recs = []
    buf = bytearray()
    for k in keys:  # 錄影前先送的調參按鍵（例如 h = Kd +10），印出車子的回應
        ser.write(k.encode())
        time.sleep(0.4)
    if keys:
        time.sleep(0.6)
        for line in ser.read(ser.in_waiting).decode('utf-8', errors='ignore').splitlines():
            if line.startswith('>>'):
                print(line.strip())
    ser.reset_input_buffer()
    for run in range(n_runs):
        key = move[run % len(move)] if move else ''
        if key:
            print('第 %d/%d 次：錄影中，自動長按 %s %.1f 秒（車子會移動，前後留空間）…' % (run + 1, n_runs, key.upper(), hold))
            ser.write(b'm')
        else:
            print('第 %d/%d 次：開始錄影（請讓車子保持站著不動）…' % (run + 1, n_runs))
            ser.write(b'r')
        rec, mode = [], 0
        t0 = time.time()
        last_send, released = 0.0, False
        done = False
        while time.time() - t0 < 25 and not done:
            now = time.time() - t0
            if key and not released:
                if 0.3 <= now < 0.3 + hold and now - last_send >= 0.1:  # 長按：每 0.1 s 重送（與 control.py 相同）
                    ser.write(key.encode())
                    last_send = now
                elif now >= 0.3 + hold:
                    ser.write(b's')  # 放開 → control.py 會送 S 煞停
                    released = True
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
            else:
                time.sleep(0.01)
            while True:
                i = buf.find(b'\n')
                if i < 0:
                    break
                line = bytes(buf[:i]).decode('utf-8', errors='ignore').strip()
                del buf[:i + 1]
                if line.startswith('REC:END'):
                    done = True
                    break
                if line.startswith('REC:MODE,'):
                    mode = 1 if line.endswith('1') else 0
                    continue
                if line.startswith('REC:KD,'):
                    print('  （目前直立環 Kd = %s）' % line[7:])
                    continue
                p = parse_rec_line(line)
                if p:
                    rec.append(p)
        if key and not released:
            ser.write(b's')
        if len(rec) < 60:
            print('  只收到 %d 筆資料（應為 64），這次作廢；可能車子倒了或序列埠忙碌' % len(rec))
        else:
            recs.append((mode, rec))
            report_any(mode, rec, '第 %d 次' % (run + 1))
        time.sleep(3.0 if key else 1.5)
    ser.close()
    return recs


def main():
    ap = argparse.ArgumentParser(description='平衡震動診斷錄影')
    ap.add_argument('-n', type=int, default=3, help='錄幾次（預設 3）')
    ap.add_argument('--port', default='COM3')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--move', default='', choices=['', 'w', 'x', 'wx'],
                    help='移動模式：w = 前進、x = 後退、wx = 交替（預設不移動，錄站立）')
    ap.add_argument('--hold', type=float, default=1.2, help='移動模式每次長按幾秒（預設 1.2，最多 1.8）')
    ap.add_argument('--keys', default='', help='錄影前先送出的按鍵，例如 hh = Kd +20、g = Kd -10')
    ap.add_argument('--file', help='只分析這個 CSV，不連線')
    args = ap.parse_args()
    args.hold = min(args.hold, 1.8)

    if args.file:
        for i, (mode, rec) in enumerate(load(args.file)):
            report_any(mode, rec, '第 %d 次' % (i + 1))
        return
    try:
        recs = capture(args.port, args.baud, args.n, args.move, args.hold, args.keys)
    except Exception as e:  # noqa: BLE001
        print('連線失敗：%s（control.py / nav.py 是否還開著？）' % e)
        sys.exit(1)
    if not recs:
        print('沒有收到有效資料。確認已上傳 bal_v28，且車子站著。')
        sys.exit(1)
    here = os.path.dirname(os.path.abspath(__file__))
    d = os.path.join(here, 'logs', datetime.now().strftime('%Y%m%d'))
    os.makedirs(d, exist_ok=True)
    base = os.path.join(d, ('recm_' if args.move else 'rec_') + datetime.now().strftime('%H%M%S'))
    save(recs, base + '.csv')
    print('\n原始資料：' + base + '.csv')
    if plot(recs, base + '.png'):
        print('曲線圖：' + base + '.png')


if __name__ == '__main__':
    main()
