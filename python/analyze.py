# -*- coding: utf-8 -*-
"""
雙輪平衡車 遙測 CSV 自動分析工具

用法（在 controller 資料夾執行）：
  python analyze.py                       分析 logs 裡「最新」的一份 CSV
  python analyze.py logs/20261010/xxx.csv 分析指定檔案（可一次給多個）
  python analyze.py logs/20261010         分析整個資料夾：每份一個報告 + 一份總表
  python analyze.py --no-plot ...         不畫圖（沒裝 matplotlib 時會自動略過）

輸出（放在 CSV 旁邊）：
  <檔名>_report.txt   文字報告（重點摘要 + 各項統計）
  <檔名>_report.png   時間序列圖（需要 matplotlib：pip install matplotlib）
  資料夾模式另外產生 summary.txt（每份 CSV 一行，方便比較版本）

會分析的項目：
  1. 站立：平衡點估計（建議 setpoint）、晃動大小、溜車 / 爆衝次數
  2. W / X：起步延遲、巡航速度、滑行距離、停下後回退、晃動
  3. A / D：原地轉成功率（判斷「漂移 / 單輪轉」）、放開後是否爆衝
  4. 航向：停車 / 靜止時車頭偏了多少
  5. 編碼器方向測試：倒地 / 斷電時的 EL、ER（v25 起）
  6. 虛擬座標：PX / PY 軌跡圖、每次 W/X 走了幾公分（quad_v26 起）
  7. 座標導航：每個轉向 / 直走花多久、到達位置與目標的誤差（nav_v27 + nav.py）
支援三種舊格式：最早的「角度: xx° | PWM」、只有 Raw_Message 的 ANG: 格式、現在分欄位的格式。
"""

import csv
import glob
import math
import os
import re
import statistics as st
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.join(BASE_DIR, 'logs')

# ---------------------------------------------------------------------------
# 判斷門檻（數值單位與遙測相同：角度 deg、速度 ticks/40ms）
# ---------------------------------------------------------------------------
REST_AFTER_CMD_S = 3.0      # 放開按鍵 / 站起來後多久才算「靜止站立」
DASH_SPEED = 15.0           # 靜止時 |V| 超過這個值 = 溜車 / 爆衝
MOVE_START_DX = 30          # W/X 按下後位移超過 30 ticks = 真正開始移動
STOP_SPEED = 3.0            # |V| 低於這個值 = 停下
SPIN_STEADY_T = 20.0        # |T| 超過這個值 = 穩定自轉中
SPIN_BAD_B = 15.0           # 自轉中平均 |B| 超過 → 漂移（自轉時本來就有約 +10 的偏移，所以門檻放 15）
SPIN_BAD_V = 4.5            # 自轉中平均 |V| 超過 → 漂移
SPIN_WEAK_PWM = 20.0        # 自轉中內側輪 |PWM| 低於這個值 = 進死區（單輪轉）
POST_SPIN_WINDOW_S = 4.0    # 放開 A/D 後觀察多久

STATE_MAP = {'常態雙閉環': 'N', '大角度緊急救援': 'E', '跌倒保護': 'F'}
_PAIR_RE = re.compile(r'([A-Z]+):(-?\d+(?:\.\d+)?)(?:/(-?\d+(?:\.\d+)?))?')
_STATE_RE = re.compile(r'【(.*?)】')
_OLD_RE = re.compile(r'角度:\s*(-?\d+(?:\.\d+)?)°\s*\|\s*PWM:\s*(-?\d+(?:\.\d+)?)')


# =============================================================================
# 讀檔
# =============================================================================
def parse_line(raw):
    """把一行遙測解析成 dict；不是遙測行回傳 None"""
    m = _STATE_RE.search(raw)
    state = STATE_MAP.get(m.group(1)) if m else None
    if raw.startswith('ANG:'):
        f = {}
        for key, v1, v2 in _PAIR_RE.findall(raw):
            if key == 'PWM':
                f['PWM_L'] = float(v1)
                f['PWM_R'] = float(v2) if v2 else float(v1)
            elif key not in f:
                f[key] = float(v1)
        if 'ANG' not in f or state is None:
            return None
        f['state'] = state
        return f
    m2 = _OLD_RE.search(raw)
    if m2 and state is not None:
        return {'ANG': float(m2.group(1)), 'PWM_L': float(m2.group(2)),
                'PWM_R': float(m2.group(2)), 'state': state}
    return None


def load(path):
    samples = []
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for row in csv.DictReader(fh):
            raw = (row.get('Raw_Message') or '').strip()
            d = parse_line(raw)
            if d is None:
                continue
            try:
                d['t'] = float(row.get('Timestamp (s)', 'nan'))
            except ValueError:
                continue
            d['cmd'] = (row.get('Active_Cmd') or 'IDLE').strip()
            samples.append(d)
    return samples


_NAV_POSE_RE = re.compile(r'PX:(-?\d+(?:\.\d+)?) PY:(-?\d+(?:\.\d+)?)(?: H:(-?\d+(?:\.\d+)?))?')


def load_nav_events(path):
    """讀出 NAV: 開頭的事件行（韌體回報 + nav.py 的 NAV:PLAN）"""
    ev = []
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for row in csv.DictReader(fh):
            raw = (row.get('Raw_Message') or '').strip()
            if not raw.startswith('NAV:'):
                continue
            try:
                t = float(row.get('Timestamp (s)', 'nan'))
            except ValueError:
                continue
            ev.append((t, raw))
    return ev


def has(samples, key):
    return any(key in d for d in samples)


def g(d, key, default=0.0):
    v = d.get(key)
    return default if v is None else v


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def mean(xs, default=float('nan')):
    xs = list(xs)
    return st.mean(xs) if xs else default


def sd(xs, default=float('nan')):
    xs = list(xs)
    return st.pstdev(xs) if len(xs) > 1 else default


def fmt(x, nd=1):
    return '-' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{nd}f}'


# =============================================================================
# 切段
# =============================================================================
def runs_of(samples, keys):
    """連續按著同一個鍵的區段 → [(key, i0, i1)]"""
    out = []
    cur = None
    for i, d in enumerate(samples):
        k = d['cmd']
        if k in keys:
            if cur and cur[0] == k and cur[2] == i - 1:
                cur[2] = i
            else:
                cur = [k, i, i]
                out.append(cur)
        else:
            cur = None
    return [tuple(c) for c in out]


def after_release(samples, i1, window_s):
    """放開後到下一次按鍵前（最多 window_s 秒）的樣本"""
    t0 = samples[i1]['t']
    seg = []
    for d in samples[i1 + 1:]:
        if d['cmd'] != 'IDLE' or d['t'] - t0 > window_s:
            break
        seg.append(d)
    return seg


# =============================================================================
# 分析
# =============================================================================
def analyze_overview(S):
    dur = S[-1]['t'] - S[0]['t'] if S else 0
    cnt = {k: sum(1 for d in S if d['state'] == k) for k in 'NEF'}
    # 倒地次數：倒地後還有再站起來的才算（最後一次通常是測試結束時把車拿起來）
    last_n = max((i for i, d in enumerate(S) if d['state'] == 'N'), default=-1)
    falls = sum(1 for i in range(1, len(S))
                if S[i - 1]['state'] in 'NE' and S[i]['state'] == 'F' and i < last_n)
    emerg = sum(1 for a, b in zip(S, S[1:]) if a['state'] == 'N' and b['state'] == 'E')
    sps = sorted({round(d['SP'], 2) for d in S if 'SP' in d})
    keys = {k: sum(1 for d in S if d['cmd'] == k) for k in 'wxadsc'}
    return dict(dur=dur, n=len(S), cnt=cnt, falls=falls, emerg=emerg, sps=sps, keys=keys)


def rest_samples(S):
    """靜止站立：沒按鍵、目標速度 0、站立中，且距離上次按鍵 / 站起來 > REST_AFTER_CMD_S"""
    last = -1e9
    out = []
    for d in S:
        if d['state'] != 'N' or d['cmd'] != 'IDLE' or abs(g(d, 'TV')) > 1e-6:
            last = d['t']
            continue
        if d['t'] - last > REST_AFTER_CMD_S:
            out.append(d)
    return out


def analyze_rest(S):
    R = rest_samples(S)
    if len(R) < 10:
        return None
    dur = len(R) * mean((b['t'] - a['t'] for a, b in zip(S, S[1:])), 0.2)
    res = dict(n=len(R), dur=dur,
               ang_mean=mean(d['ANG'] for d in R), ang_sd=sd(d['ANG'] for d in R),
               b_sd=sd(g(d, 'B') for d in R))
    if has(R, 'SA'):
        res['sa_mean'] = mean(g(d, 'SA') for d in R)
        res['sa_sd'] = sd(g(d, 'SA') for d in R)
    if has(R, 'SP'):
        res['sp'] = mean(g(d, 'SP') for d in R)
    if has(R, 'V'):
        res['v_sd'] = sd(g(d, 'V') for d in R)
        # 溜車 / 爆衝事件（1 秒內算同一次）
        ev = []
        last_spin = -1e9
        for d in S:
            if d['cmd'] in ('a', 'd'):
                last_spin = d['t']
            d['_last_spin'] = last_spin
        for d in R:
            if abs(g(d, 'V')) > DASH_SPEED and (not ev or d['t'] - ev[-1]['t'] > 1.0):
                ev.append(dict(t=d['t'], v=g(d, 'V'), since_spin=d['t'] - d['_last_spin']))
        res['dash'] = ev
        res['dash_per_min'] = len(ev) / dur * 60 if dur > 0 else 0
    return res


def analyze_moves(S):
    if not has(S, 'V') or not has(S, 'DX'):
        return []
    out = []
    for k, i0, i1 in runs_of(S, ('w', 'x')):
        seg = S[i0:i1 + 1]
        sg = -1 if k == 'w' else 1          # W = 負方向
        t0, tr = seg[0]['t'], seg[-1]['t']
        # 韌體在按下 W/X 時把 DX 歸零；第一筆可能還是歸零前的值 → 從 |DX| 接近 0 的那筆開始找
        j0 = next((j for j, d in enumerate(seg) if abs(d['DX']) <= 60), 0)
        lag = next((d['t'] - t0 for d in seg[j0:] if sg * d['DX'] > MOVE_START_DX), None)
        last1s = [sg * d['V'] for d in seg if tr - d['t'] <= 1.0]
        after = after_release(S, i1, 6.0)
        r = dict(key=k, t0=t0, dur=tr - t0, lag=lag,
                 v_end=mean(last1s), v_max=max((sg * d['V'] for d in seg), default=0),
                 ang_sd=sd(d['ANG'] for d in seg),
                 b_max=max((abs(g(d, 'B')) for d in seg + after), default=0),
                 emergency=any(d['state'] != 'N' for d in seg + after))
        if after:
            dx_rel = S[i1]['DX']
            peak = max(after, key=lambda d: sg * d['DX'])
            r['coast'] = sg * (peak['DX'] - dx_rel) + 0.0
            r['rollback'] = sg * (peak['DX'] - after[-1]['DX']) + 0.0   # + 0.0：避免印出 -0
            stop = next((d['t'] - tr for d in after if abs(d['V']) < STOP_SPEED), None)
            r['t_stop'] = stop
            r['gap'] = after[-1]['t'] - tr
            base = after[0].get('SP', -1.5)
            r['stop_dev'] = max((abs(d['ANG'] - base) for d in after[:8]), default=0)
        out.append(r)
    return out


def analyze_spins(S):
    if not has(S, 'T'):
        return []
    out = []
    for k, i0, i1 in runs_of(S, ('a', 'd')):
        seg = S[i0:i1 + 1]
        steady = [d for d in seg if abs(g(d, 'T')) > SPIN_STEADY_T]
        yaw = sum(wrap_pi(g(b, 'Y') - g(a, 'Y')) for a, b in zip(seg, seg[1:]))
        r = dict(key=k, t0=seg[0]['t'], dur=seg[-1]['t'] - seg[0]['t'],
                 yaw_deg=math.degrees(yaw), steady_n=len(steady),
                 emergency=any(d['state'] != 'N' for d in seg))
        if steady:
            r['yr'] = mean(abs(g(d, 'YR')) for d in steady)
            r['b_mean'] = mean(g(d, 'B') for d in steady)
            r['v_mean'] = mean(g(d, 'V') for d in steady) if has(steady, 'V') else float('nan')
            # 內側輪：A（左旋）時左輪往後、D 時右輪往後 → 取兩輪 |PWM| 較小者的最小值
            r['weak_pwm'] = min(min(abs(g(d, 'PWM_L')), abs(g(d, 'PWM_R'))) for d in steady)
            r['weak_pwm_mean'] = mean(min(abs(g(d, 'PWM_L')), abs(g(d, 'PWM_R'))) for d in steady)
            v_bad = (not math.isnan(r['v_mean'])) and abs(r['v_mean']) > SPIN_BAD_V
            r['bad'] = abs(r['b_mean']) > SPIN_BAD_B or v_bad or r['weak_pwm_mean'] < SPIN_WEAK_PWM
        else:
            r['bad'] = None   # 太短，沒進入穩定自轉
        after = after_release(S, i1, POST_SPIN_WINDOW_S)
        if after and has(after, 'V'):
            r['post_vmax'] = max(abs(g(d, 'V')) for d in after)
            r['post_dip'] = min(d['ANG'] - d.get('SP', -1.5) for d in after[:8])
            r['post_dash'] = r['post_vmax'] > DASH_SPEED
        out.append(r)
    return out


def analyze_heading(S):
    if not has(S, 'Y'):
        return None
    acc = {'cmd': 0.0, 'brake': 0.0, 'rest': 0.0}
    last_turn = -1e9
    for d in S:
        if d['cmd'] in ('a', 'd', 'c'):
            last_turn = d['t']
        d['_last_turn'] = last_turn
    N = [d for d in S if d['state'] == 'N']
    for a, b in zip(N, N[1:]):
        if b['t'] - a['t'] > 1.0:
            continue
        dy = math.degrees(wrap_pi(g(b, 'Y') - g(a, 'Y')))
        # 自轉本身、放開後的慣性旋轉（1.5 s 內或角速度仍 > 0.3 rad/s）都不算偏移
        if a['cmd'] in ('a', 'd', 'c', 'NAV') or a['t'] - a['_last_turn'] < 1.5 or abs(g(a, 'YR')) > 0.3:
            continue
        if a['cmd'] in ('w', 'x'):
            acc['cmd'] += dy
        elif abs(g(a, 'TV')) > 1e-6:
            acc['brake'] += dy
        else:
            acc['rest'] += dy
    return acc


def analyze_odometry(S):
    P = [d for d in S if 'PX' in d and 'PY' in d and d['state'] == 'N']
    if len(P) < 5:
        return None
    path = sum(math.hypot(b['PX'] - a['PX'], b['PY'] - a['PY']) for a, b in zip(P, P[1:])
               if b['t'] - a['t'] < 1.0)
    runs = []
    for k, i0, i1 in runs_of(S, ('w', 'x')):
        seg = [d for d in S[i0:i1 + 1] if 'PX' in d]
        after = [d for d in after_release(S, i1, 6.0) if 'PX' in d]
        if seg and after:
            a, b = seg[0], after[-1]
            runs.append(dict(key=k, t0=seg[0]['t'],
                             dist=math.hypot(b['PX'] - a['PX'], b['PY'] - a['PY'])))
    return dict(final=(P[-1]['PX'], P[-1]['PY']), path=path, runs=runs,
                xmin=min(d['PX'] for d in P), xmax=max(d['PX'] for d in P),
                ymin=min(d['PY'] for d in P), ymax=max(d['PY'] for d in P))


def analyze_nav(events):
    if not events:
        return None
    steps, plans = [], []
    start = None
    for t, msg in events:
        parts = msg.split()
        tag = parts[0]
        if tag == 'NAV:PLAN' and len(parts) >= 4:
            plans.append(dict(t=t, idx=int(parts[1]), x=float(parts[2]), y=float(parts[3])))
        elif tag == 'NAV:START':
            start = (t, ' '.join(parts[1:]))
        elif tag in ('NAV:DONE', 'NAV:TIMEOUT', 'NAV:ABORT', 'NAV:REJECT'):
            m = _NAV_POSE_RE.search(msg)
            pose = (float(m.group(1)), float(m.group(2))) if m else None
            head = float(m.group(3)) if (m and m.group(3)) else None
            kind = parts[1] if len(parts) > 1 and parts[1] in ('T', 'M') else ''
            steps.append(dict(t=t, result=tag[4:], kind=kind, cmd=start[1] if start else '',
                              dur=(t - start[0]) if start else None, pose=pose, head=head))
            start = None
    # 每個目標點的到達誤差：PLAN 之後的第一個 MOVE 完成
    for p in plans:
        done = next((s for s in steps if s['t'] > p['t'] and s['kind'] == 'M' and s['pose']), None)
        if done:
            p['reached'] = done['pose']
            p['err'] = math.hypot(done['pose'][0] - p['x'], done['pose'][1] - p['y'])
    return dict(steps=steps, plans=plans)


def analyze_encoder_test(S):
    E = [d for d in S if 'EL' in d and 'ER' in d]
    if not E:
        return None
    last = E[-1]
    return dict(el=last['EL'], er=last['ER'], n=len(E),
                el_max=max(abs(d['EL']) for d in E), er_max=max(abs(d['ER']) for d in E))


# =============================================================================
# 報告
# =============================================================================
def build_report(path, S, nav_events=None):
    ov = analyze_overview(S)
    rest = analyze_rest(S)
    moves = analyze_moves(S)
    spins = analyze_spins(S)
    head = analyze_heading(S)
    enc = analyze_encoder_test(S)
    odo = analyze_odometry(S)
    navr = analyze_nav(nav_events)

    L = []
    name = os.path.basename(path)
    L.append(f'=== 遙測分析報告：{name} ===')
    L.append(f'長度 {ov["dur"]:.0f} s，{ov["n"]} 筆；站立 {ov["cnt"]["N"]} / 救援 {ov["cnt"]["E"]} / 倒地 {ov["cnt"]["F"]} 筆')
    L.append(f'倒地 {ov["falls"]} 次（不含結束時拿起來）、進入緊急救援 {ov["emerg"]} 次；使用的 setpoint：{ov["sps"] or "（舊格式無 SP）"}')
    L.append('按鍵筆數：' + '  '.join(f'{k.upper()}={v}' for k, v in ov['keys'].items() if v))

    # ---------- 重點 ----------
    hl = []
    if rest:
        est = rest['ang_mean']
        cur = f'目前 setpoint {rest["sp"]:.2f}°' if 'sp' in rest else '舊格式沒有 setpoint 欄位'
        hl.append(f'站立平衡點約 {est:.2f}°（{cur}），晃動 ANG sd {rest["ang_sd"]:.2f}°')
        if 'dash' in rest:
            hl.append(f'靜止溜車 / 爆衝：{len(rest["dash"])} 次（每分鐘 {rest["dash_per_min"]:.1f} 次）')
    if moves:
        lags = [m['lag'] for m in moves if m['lag'] is not None]
        nmove = sum(1 for m in moves if m['lag'] is None)
        hl.append(f'W/X {len(moves)} 次：起步延遲平均 {fmt(mean(lags), 2)} s'
                  + (f'，{nmove} 次沒有真正移動' if nmove else ''))
    if spins:
        valid = [s for s in spins if s['bad'] is not None]
        bad = [s for s in valid if s['bad']]
        if valid:
            hl.append(f'A/D {len(valid)} 次：原地轉 {len(valid) - len(bad)} 次、漂移/單輪轉 {len(bad)} 次'
                      f'（成功率 {100 * (len(valid) - len(bad)) / len(valid):.0f}%）')
        pd = [s for s in spins if s.get('post_dash')]
        if pd:
            hl.append(f'放開 A/D 後爆衝 {len(pd)} 次')
    if navr and navr['plans']:
        errs = [p['err'] for p in navr['plans'] if 'err' in p]
        fails = [s for s in navr['steps'] if s['result'] != 'DONE']
        hl.append(f'座標導航：{len(navr["plans"])} 個目標點，到達 {len(errs)} 個'
                  + (f'，最大誤差 {max(errs):.1f} cm（里程計估計）' if errs else '')
                  + (f'，{len(fails)} 個動作失敗 / 中止' if fails else ''))
    if enc:
        hl.append(f'編碼器方向測試：EL={enc["el"]:.0f}、ER={enc["er"]:.0f}（往前推時兩個都應該是負的）')
    L.append('')
    L.append('【重點】')
    L += ['  - ' + h for h in hl] if hl else ['  （資料不足）']

    # ---------- 站立 ----------
    L.append('')
    L.append('【1. 靜止站立】（放開按鍵 / 站起來 3 秒後才算）')
    if not rest:
        L.append('  靜止資料不足')
    else:
        L.append(f'  時間 {rest["dur"]:.0f} s')
        L.append(f'  平均傾角 {rest["ang_mean"]:.2f}°（= 真正的平衡點估計），ANG sd {rest["ang_sd"]:.2f}°，B sd {rest["b_sd"]:.1f}')
        if 'sa_mean' in rest:
            L.append(f'  速度外環補償 SA 平均 {rest["sa_mean"]:+.2f}°、sd {rest["sa_sd"]:.2f}°'
                     + ('  → setpoint 偏了，建議改成約 %.2f°' % rest['ang_mean']
                        if abs(rest['sa_mean']) > 0.15 else '  → setpoint 合適'))
        if 'v_sd' in rest:
            L.append(f'  速度晃動 V sd {rest["v_sd"]:.1f}')
        if rest.get('dash'):
            L.append(f'  溜車 / 爆衝事件（|V|>{DASH_SPEED:.0f}）：')
            for e in rest['dash']:
                note = f'，放開 A/D 後 {e["since_spin"]:.1f} s' if e['since_spin'] < 6 else ''
                L.append(f'    {e["t"]:7.1f} s  V={e["v"]:+.0f}{note}')

    # ---------- W/X ----------
    L.append('')
    L.append('【2. 前後移動 W / X】（速度單位 ticks/40ms，正值 = 往指令方向）')
    if not moves:
        L.append('  沒有 W/X 資料（或舊格式沒有 V/DX）')
    else:
        L.append('  鍵   開始(s) 按住(s) 延遲(s) 放開前速度 最高速 ANG sd  B max  滑行 回退 停下(s) 停車偏離°')
        for m in moves:
            L.append('  %s  %7.1f %6.1f %7s %9s %6.1f %6.2f %6.0f %5s %4s %6s %8s%s' % (
                m['key'].upper(), m['t0'], m['dur'], fmt(m['lag'], 1), fmt(m['v_end'], 1), m['v_max'],
                m['ang_sd'], m['b_max'], fmt(m.get('coast'), 0), fmt(m.get('rollback'), 0),
                fmt(m.get('t_stop'), 1), fmt(m.get('stop_dev'), 1), '  ⚠救援/倒地' if m['emergency'] else ''))
        for k in ('w', 'x'):
            M = [m for m in moves if m['key'] == k]
            if not M:
                continue
            L.append('  %s 平均：延遲 %s s、放開前速度 %s、滑行 %s、回退 %s、停下 %s s' % (
                k.upper(), fmt(mean(m['lag'] for m in M if m['lag'] is not None), 2),
                fmt(mean(m['v_end'] for m in M if not math.isnan(m['v_end'])), 1),
                fmt(mean(m['coast'] for m in M if 'coast' in m), 0),
                fmt(mean(m['rollback'] for m in M if 'rollback' in m and m['gap'] > 2.5), 0),
                fmt(mean(m['t_stop'] for m in M if m.get('t_stop') is not None), 2)))
        L.append('  （回退只統計放開後等超過 2.5 s 才按下一個的那幾次）')

    # ---------- A/D ----------
    L.append('')
    L.append('【3. 原地自轉 A / D】')
    if not spins:
        L.append('  沒有 A/D 資料')
    else:
        L.append('  鍵   開始(s) 按住(s) 轉了(°) 角速度  平均B  平移V 內側輪PWM  結果      放開後最大V')
        for s in spins:
            if s['bad'] is None:
                res = '太短'
            else:
                res = '漂移/單輪轉' if s['bad'] else '原地'
            L.append('  %s  %7.1f %6.1f %7.0f %6s %6s %6s %8s   %-8s %s%s' % (
                s['key'].upper(), s['t0'], s['dur'], s['yaw_deg'], fmt(s.get('yr'), 2),
                fmt(s.get('b_mean'), 0), fmt(s.get('v_mean'), 1), fmt(s.get('weak_pwm_mean'), 0), res,
                fmt(s.get('post_vmax'), 0), '  ⚠放開後爆衝' if s.get('post_dash') else ''))
        for k in ('a', 'd'):
            V = [s for s in spins if s['key'] == k and s['bad'] is not None]
            if V:
                L.append(f'  {k.upper()}：{len(V)} 次，原地 {sum(1 for s in V if not s["bad"])} 次')
        L.append(f'  判斷標準：穩定自轉時 |平均B|>{SPIN_BAD_B:.0f}、|平移V|>{SPIN_BAD_V} 或 內側輪平均|PWM|<{SPIN_WEAK_PWM:.0f} → 漂移/單輪轉')

    # ---------- 航向 ----------
    L.append('')
    L.append('【4. 航向偏移】（不含 A/D/C 自轉本身）')
    if head is None:
        L.append('  沒有航向資料')
    else:
        L.append(f'  按住 W/X 時 {head["cmd"]:+.1f}°、煞車滑行時 {head["brake"]:+.1f}°、靜止時 {head["rest"]:+.1f}°'
                 f'，合計 {head["cmd"] + head["brake"] + head["rest"]:+.1f}°')

    # ---------- 編碼器 ----------
    if enc:
        L.append('')
        L.append('【5. 編碼器方向測試】（倒地 / 斷電時的累計 tick）')
        L.append(f'  最後一筆 EL={enc["el"]:.0f}、ER={enc["er"]:.0f}（最大 |EL|={enc["el_max"]:.0f}、|ER|={enc["er_max"]:.0f}）')
        L.append('  扶著往前（W 方向）推：兩個都要是負數、大小差不多；哪一輪是正的就把 ENC_x_SIGN 改成 (-1)')
        if enc['el_max'] < 20 or enc['er_max'] < 20:
            L.append('  ⚠ 有一輪幾乎沒有計數：檢查 A/B 相接線')

    # ---------- 虛擬座標 ----------
    if odo:
        L.append('')
        L.append('【6. 虛擬座標】（PX 往前為正、PY 往右為正，單位 cm；每次站起來 / 按 z 歸零）')
        L.append(f'  最後位置 ({odo["final"][0]:.1f}, {odo["final"][1]:.1f}) cm，總路徑 {odo["path"]:.0f} cm')
        L.append(f'  活動範圍 PX {odo["xmin"]:.0f} ~ {odo["xmax"]:.0f}、PY {odo["ymin"]:.0f} ~ {odo["ymax"]:.0f} cm')
        for r in odo['runs']:
            L.append(f'  {r["key"].upper()} {r["t0"]:6.1f} s：按下到停下共移動 {r["dist"]:.1f} cm')

    # ---------- 座標導航 ----------
    if navr:
        L.append('')
        L.append('【7. 座標導航】（nav.py；位置是車上里程計的估計，實際位置請用捲尺量）')
        L.append('  時間(s)  動作               結果     花費(s)  完成時位置(cm)     航向(°)')
        for s_ in navr['steps']:
            pose = f'({s_["pose"][0]:6.1f},{s_["pose"][1]:6.1f})' if s_['pose'] else '-'
            L.append('  %7.1f  %-17s  %-7s  %6s  %-17s  %s' % (
                s_['t'], s_['cmd'], s_['result'], fmt(s_['dur'], 1), pose, fmt(s_['head'], 1)))
        if navr['plans']:
            L.append('  目標點：')
            for p in navr['plans']:
                if 'err' in p:
                    L.append(f'    點 {p["idx"]}：目標 ({p["x"]:.1f}, {p["y"]:.1f}) → 到達 ({p["reached"][0]:.1f}, {p["reached"][1]:.1f})，誤差 {p["err"]:.1f} cm')
                else:
                    L.append(f'    點 {p["idx"]}：目標 ({p["x"]:.1f}, {p["y"]:.1f}) → 沒有完成')

    return '\n'.join(L), dict(ov=ov, rest=rest, moves=moves, spins=spins, head=head, enc=enc, odo=odo, nav=navr)


# =============================================================================
# 畫圖
# =============================================================================
CMD_COLOR = {'w': '#4C78A8', 'x': '#F58518', 'a': '#54A24B', 'd': '#B279A2', 'c': '#9D755D', 'NAV': '#E45756'}


def plot(path, S, out_png, nav_events=None):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    t = [d['t'] for d in S]
    dur = t[-1] - t[0] if t else 1
    rows = [('ANG', 'ANG / target (deg)')]
    if has(S, 'V'):
        rows.append(('V', 'V / TV (ticks/40ms)'))
    rows.append(('B', 'B / T (PWM)'))
    rows.append(('PWM', 'PWM L / R'))
    if has(S, 'Y'):
        rows.append(('Y', 'Yaw (deg, unwrapped)'))
    fig, axes = plt.subplots(len(rows), 1, sharex=True,
                             figsize=(min(40, max(12, dur / 5)), 2.2 * len(rows)))
    if len(rows) == 1:
        axes = [axes]

    # 背景：按鍵與倒地
    def shade(ax):
        for k, i0, i1 in runs_of(S, ('w', 'x', 'a', 'd', 'c')):
            ax.axvspan(S[i0]['t'], S[i1]['t'], color=CMD_COLOR[k], alpha=0.15, lw=0)
        i = 0
        while i < len(S):                        # 導航中（遙測有 NV 欄位）
            if 'NV' in S[i]:
                j = i
                while j + 1 < len(S) and 'NV' in S[j + 1]:
                    j += 1
                ax.axvspan(S[i]['t'], S[j]['t'], color=CMD_COLOR['NAV'], alpha=0.12, lw=0)
                i = j + 1
            else:
                i += 1
        i = 0
        while i < len(S):
            if S[i]['state'] == 'F':
                j = i
                while j + 1 < len(S) and S[j + 1]['state'] == 'F':
                    j += 1
                ax.axvspan(S[i]['t'], S[j]['t'], color='0.5', alpha=0.25, lw=0)
                i = j + 1
            else:
                i += 1

    nan = float('nan')
    for ax, (key, label) in zip(axes, rows):
        shade(ax)
        if key == 'ANG':
            ang = [d['ANG'] if d['state'] != 'F' else nan for d in S]
            ax.plot(t, ang, lw=1, label='ANG')
            if has(S, 'SP'):
                tgt = [g(d, 'SP') + g(d, 'SA') if d['state'] == 'N' else nan for d in S]
                ax.plot(t, tgt, lw=1, label='SP+SA (target)')
            lo = [a for a in ang if not math.isnan(a)]
            if lo:
                ax.set_ylim(max(min(lo), -10) - 0.5, min(max(lo), 8) + 0.5)
        elif key == 'V':
            ax.plot(t, [g(d, 'V') if d['state'] != 'F' else nan for d in S], lw=1, label='V')
            ax.plot(t, [g(d, 'TV') if d['state'] != 'F' else nan for d in S], lw=1, label='TV')
            ax.axhline(0, color='0.6', lw=0.5)
        elif key == 'B':
            ax.plot(t, [g(d, 'B') if d['state'] != 'F' else nan for d in S], lw=1, label='B')
            if has(S, 'T'):
                ax.plot(t, [g(d, 'T') if d['state'] != 'F' else nan for d in S], lw=1, label='T')
            ax.axhline(0, color='0.6', lw=0.5)
        elif key == 'PWM':
            ax.plot(t, [g(d, 'PWM_L') if d['state'] != 'F' else nan for d in S], lw=1, label='PWM L')
            ax.plot(t, [g(d, 'PWM_R') if d['state'] != 'F' else nan for d in S], lw=1, label='PWM R')
            ax.axhline(0, color='0.6', lw=0.5)
        elif key == 'Y':
            acc, prev, y = 0.0, None, []
            for d in S:
                cur = g(d, 'Y')
                if prev is not None:
                    acc += wrap_pi(cur - prev)
                prev = cur
                y.append(math.degrees(acc))
            ax.plot(t, y, lw=1, label='Yaw')
        ax.set_ylabel(label, fontsize=8)
        ax.legend(loc='upper right', fontsize=7, ncol=3)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel('time (s)   shading: W blue / X orange / A green / D purple / NAV red / fallen gray')
    fig.suptitle(os.path.basename(path), fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)

    # 虛擬座標軌跡（quad_v26 起）
    P = [d for d in S if 'PX' in d and 'PY' in d and d['state'] == 'N']
    if len(P) > 5:
        fig, ax = plt.subplots(figsize=(7, 7))
        seg_x, seg_y, seg_c = [], [], None
        for d in P:
            if 'NV' in d:                       # 導航：轉向綠、直走紅
                c = CMD_COLOR['a'] if d['NV'] == 1 else CMD_COLOR['NAV']
            else:
                c = CMD_COLOR.get(d['cmd'], '0.4') if d['cmd'] != 'NAV' else '0.4'
            if c != seg_c and seg_x:
                ax.plot(seg_y, seg_x, color=seg_c, lw=1.5)
                seg_x, seg_y = [seg_x[-1]], [seg_y[-1]]
            seg_c = c
            seg_x.append(d['PX'])
            seg_y.append(d['PY'])
        if seg_x:
            ax.plot(seg_y, seg_x, color=seg_c, lw=1.5)
        navr = analyze_nav(nav_events)
        if navr and navr['plans']:
            for p in navr['plans']:
                ax.plot([p['y']], [p['x']], 'kx', ms=10, mew=2)
                ax.annotate(str(p['idx']), (p['y'], p['x']), textcoords='offset points', xytext=(6, 6), fontsize=9)
            ax.plot([], [], 'kx', label='target')
        ax.plot([P[0]['PY']], [P[0]['PX']], 'go', label='start')
        ax.plot([P[-1]['PY']], [P[-1]['PX']], 'rs', label='end')
        ax.set_xlabel('PY  right (cm)')
        ax.set_ylabel('PX  forward (cm)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title('odometry path  (W blue / X orange / A, D, NAV-turn green / NAV-move red / idle gray)', fontsize=9)
        fig.tight_layout()
        fig.savefig(out_png.replace('_report.png', '_path.png'), dpi=90)
        plt.close(fig)
    return True


# =============================================================================
# 主程式
# =============================================================================
def summary_line(path, R):
    ov, rest, moves, spins = R['ov'], R['rest'], R['moves'], R['spins']
    valid = [s for s in spins if s['bad'] is not None]
    ok = sum(1 for s in valid if not s['bad'])
    lags = [m['lag'] for m in moves if m['lag'] is not None]
    return '%-32s %5.0fs 倒地%2d | 平衡點 %6s sd %4s 溜車/分 %4s | W/X %2d 延遲 %4s 滑行 %4s | A/D %2d 原地 %2s' % (
        os.path.basename(path), ov['dur'], ov['falls'],
        fmt(rest['ang_mean'], 2) if rest else '-', fmt(rest['ang_sd'], 2) if rest else '-',
        fmt(rest.get('dash_per_min'), 1) if rest else '-',
        len(moves), fmt(mean(lags), 2), fmt(mean(m['coast'] for m in moves if 'coast' in m), 0),
        len(valid), ok)


def analyze_file(path, do_plot=True, quiet=False):
    S = load(path)
    if len([d for d in S if d['state'] == 'N']) < 5:
        if not quiet:
            print(f'[略過] {os.path.basename(path)}：幾乎沒有站立資料')
        return None
    nav_events = load_nav_events(path)
    text, R = build_report(path, S, nav_events)
    stem = os.path.splitext(path)[0]
    with open(stem + '_report.txt', 'w', encoding='utf-8') as fh:
        fh.write(text + '\n')
    png = None
    if do_plot and plot(path, S, stem + '_report.png', nav_events):
        png = stem + '_report.png'
    if not quiet:
        print(text)
        print(f'\n>> 報告：{stem}_report.txt' + (f'\n>> 圖：{png}' if png else '\n>> （沒有畫圖：未安裝 matplotlib 或加了 --no-plot）'))
    return R


def newest_csv():
    files = glob.glob(os.path.join(LOG_ROOT, '**', 'telemetry_*.csv'), recursive=True)
    files += glob.glob(os.path.join(BASE_DIR, 'telemetry_*.csv'))
    return max(files, key=os.path.getmtime) if files else None


def main(argv):
    do_plot = '--no-plot' not in argv
    targets = [a for a in argv if not a.startswith('--')]
    if not targets:
        f = newest_csv()
        if not f:
            print('找不到任何 telemetry_*.csv')
            return
        targets = [f]
    for tgt in targets:
        if os.path.isdir(tgt):
            files = sorted(glob.glob(os.path.join(tgt, 'telemetry_*.csv')))
            lines = []
            for f in files:
                R = analyze_file(f, do_plot, quiet=True)
                if R:
                    lines.append(summary_line(f, R))
                    print('  完成', os.path.basename(f))
            out = os.path.join(tgt, 'summary.txt')
            with open(out, 'w', encoding='utf-8') as fh:
                fh.write('\n'.join(lines) + '\n')
            print('\n'.join(lines))
            print(f'\n>> 總表：{out}')
        else:
            analyze_file(tgt, do_plot)


if __name__ == '__main__':
    main(sys.argv[1:])
