# -*- coding: utf-8 -*-
"""
cruise_stats.py — 看遙控（drive.py）的 CSV：直走時速度有沒有忽快忽慢、補償角有沒有頂到上限、哪次倒地

用法（在 controller 資料夾）：
  python cruise_stats.py                 分析最新一份 logs\\日期\\telemetry_*.csv
  python cruise_stats.py 檔名.csv        分析指定檔案
  python cruise_stats.py 檔1.csv 檔2.csv 並排比較（調 Kp_speed 前後）
需要 pandas（python -m pip install pandas）。
"""
import glob
import os
import sys

import pandas as pd


def load(path):
    d = pd.read_csv(path, encoding='utf-8-sig')
    return d.rename(columns={'Timestamp (s)': 't'})


def latest():
    fs = sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs', '*', 'telemetry_*.csv')))
    if not fs:
        raise SystemExit('找不到 logs\\*\\telemetry_*.csv')
    return fs[-1]


def report(path):
    d = load(path)
    n = d[d.State == 'NORMAL']
    print('=' * 70)
    print(os.path.basename(path), f'  共 {d.t.max():.0f} s，常態 {len(n)} 筆')
    # 前進直走（W）：連續按住時 V 應該穩定在目標附近
    w = n[n.Active_Cmd.astype(str).str.match(r'^D\+[1-9]') & (n.TV <= -4.5)]
    if len(w) > 5:
        print(f'[前進 TV≤-4.5] {len(w)} 筆：V 中位數 {w.V.median():.1f}，'
              f'5%～95% 範圍 {w.V.quantile(.05):.1f} ～ {w.V.quantile(.95):.1f}，最大速度 {w.V.min():.1f}')
        print(f'   速度擺動幅度（95%-5%）= {w.V.quantile(.95) - w.V.quantile(.05):.1f}（越小越穩；舊版 W 鍵約 8）')
    x = n[n.Active_Cmd.astype(str).str.match(r'^D-') & (n.TV >= 2.0)]
    if len(x) > 5:
        print(f'[後退 TV≥2] {len(x)} 筆：V 中位數 {x.V.median():.1f}，最大速度 {x.V.max():.1f}（目標中位數 {x.TV.median():.1f}）')
    sat = ((n.SA.abs() >= 2.9).mean()) * 100
    print(f'[補償角 SA] 頂到 ±3° 以上的時間占 {sat:.0f}%（越高代表速度外環一直在最大力氣煞車 / 追速度）')
    print(f'[速度] |V| 最大 {n.V.abs().max():.0f}；|V|>30 的筆數 {(n.V.abs() > 30).sum()}')
    # 倒地
    s = d.State.values
    for i in range(1, len(d)):
        if s[i] == 'FALLEN' and s[i - 1] != 'FALLEN' and i > 5:
            pre = d.iloc[max(0, i - 15):i]
            pre = pre[pre.State != 'EVENT']
            print(f'[倒地] t={d.t[i]:.1f} s：前 3 秒 |V| 最大 {pre.V.abs().max():.0f}，SA 範圍 {pre.SA.min():.1f}～{pre.SA.max():.1f}，'
                  f'倒地前 ANG {pre.ANG.iloc[-1]:.1f}°，指令 {d.Active_Cmd[i - 1]}')


if __name__ == '__main__':
    for p in (sys.argv[1:] or [latest()]):
        report(p)
