# -*- coding: utf-8 -*-
"""
把 controller 資料夾裡散落的 telemetry_YYYYMMDD_HHMMSS.csv
依日期搬進 logs/YYYYMMDD/ 資料夾。

用法：
  單獨執行：python organize_logs.py
  control.py 每次啟動時也會自動執行一次
已存在同名檔案時不覆蓋（跳過並提示）。
"""

import os
import re
import shutil

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.join(BASE_DIR, 'logs')
_NAME_RE = re.compile(r'^telemetry_(\d{8})_\d{6}\.csv$')


def organize_logs(verbose=True):
    moved, skipped = [], []
    for name in sorted(os.listdir(BASE_DIR)):
        m = _NAME_RE.match(name)
        if not m:
            continue
        src = os.path.join(BASE_DIR, name)
        if not os.path.isfile(src):
            continue
        day_dir = os.path.join(LOG_ROOT, m.group(1))
        os.makedirs(day_dir, exist_ok=True)
        dst = os.path.join(day_dir, name)
        if os.path.exists(dst):
            skipped.append(name)
            continue
        shutil.move(src, dst)
        moved.append((name, m.group(1)))

    if verbose:
        if moved:
            print(f">> 已整理 {len(moved)} 個舊記錄檔到 logs/日期/：")
            for name, day in moved:
                print(f"   {name}  ->  logs/{day}/")
        for name in skipped:
            print(f"   [跳過] logs 裡已有同名檔案：{name}")
    return moved, skipped


if __name__ == '__main__':
    moved, skipped = organize_logs()
    if not moved and not skipped:
        print(">> 沒有需要整理的記錄檔")
