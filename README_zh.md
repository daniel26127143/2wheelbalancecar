# 全向輪自平衡機器人設計

一台由共線麥克拉姆輪驅動的欠驅動兩輪倒單擺機器人。以 ATmega328P（Arduino Uno）實作，使用硬體計時器的多速率串級控制，並預留未來加裝第三顆輪子做橫移的設計。

[![YouTube](https://img.shields.io/badge/YouTube-Video%20Demo-red?logo=youtube)](https://www.youtube.com/shorts/1qH6NnuWVuc)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20Uno-blue.svg)](https://www.arduino.cc/)

[English](README.md) | [繁體中文](README_zh.md)

## 專案概述

| 項目 | 規格 |
|---|---|
| 主控 | ATmega328P，16 MHz（Arduino Uno） |
| 姿態感測 | MPU-6050（DMP 取得傾角與航向角，直接讀陀螺儀暫存器取得傾角速度） |
| 馬達驅動 | TB6612FNG，兩顆附霍爾編碼器的直流減速馬達 |
| 輪子 | 兩顆 65 mm 麥克拉姆輪，裝在同一根軸上 |
| 尺寸與重量 | 寬 19 cm、高 17 cm、1.039 kg |
| 無線連線 | HC-06 藍牙，9600 baud |
| 電源 | 2S 18650 電池（7.4 V）供馬達；5 V 行動電源供晶片與感測器 |

目前狀態：**兩輪機器人已完成並測試**。**三輪版本（中間加一顆 90° 全向輪做橫移）與安卓手機遙控還在規劃，尚未製作。**

## 實測結果

數據來自 2026 年 10/11 至 10/13 的 log。數據是車上自己的感測器量到的，記錄頻率 5 Hz，樣本數不多，且各次測試之間韌體有修改。

| 測試 | 結果 |
|---|---|
| 站立不動 | 傾角維持在平均值 ±1.2° 以內（最大偏差 1.1°，σ = 0.42°；10/12 共 35 秒站立 log） |
| 最長站立時間 | 超過 90 秒（示範影片） |
| 直線行駛（韌體 v34，10/13 自動測試） | 前進平均 16.5 cm/s（指令 18，4 次）；後退平均 17.7 cm/s（指令 20，3 次）；這些測試沒有倒下 |
| 移動指令 `#M`（10/11，舊版韌體） | 31 次完成 26 次；平均誤差 1.1 cm，最差 2.6 cm |
| 轉向指令 `#T`（10/11，舊版韌體） | 32 次；平均誤差 1.3°，最差 2.4° |

10/13 的自動測試中，43 次有 1 次倒下。距離使用編碼器比例 36.2 ticks/cm，是推車 50 cm 校正出來的。

## 控制設計

Timer2（CTC 模式）中斷每 **10.13 ms（98.7 Hz）** 執行一次平衡環，每 4 次執行一次速度環（**40.5 ms**）。中斷內有防重入保護，並重新開啟中斷，讓編碼器計數不會漏掉。

1. **平衡環（PD）：** `ub = Kp·(θref − θ) − Kd·q`，Kp = 1275、Kd = 40。傾角速度 `q` 直接讀陀螺儀暫存器。高速時加上反電動勢前饋（速度超過 8 之後，每單位加 1.8 PWM，上限 50）。
2. **速度環（PI）：** 輪速先濾波（時間常數約 95 ms）。PI 輸出（Kp = 2.8、Ki = 0.015）是一個小的傾角偏移 θoff，限制在 ±3.5°（煞車時 ±3.0°，超速煞車時 ±4.0°）。速度比目標快超過 8 個單位時，P 增益乘以 1.6。
3. **航向環：** `ut = Kff·ωref + Kpy·(ψref − ψ) + Kdy·(ωref − ω)`，Kff = 30、Kpy = 38、Kdy = 7。輸出是左右輪的扭矩差：`uL = ub + ut`、`uR = ub − ut`。
4. **馬達輸出：** 死區補償（5 PWM）、左輪增益校正 1.037，最後 PWM 限制在 ±255。

安全狀態（每個平衡週期最先判斷）：

| 條件 | 動作 |
|---|---|
| 傾角誤差 > 35° | `FALLEN`：關閉馬達、重置狀態。傾角誤差低於 3° 並維持 20 個週期後重新啟動 |
| 傾角誤差 > 7°（低於 4.5° 退出） | `RESCUE`：只用強力 PD（Kp 1400、Kd 48），不轉向 |
| 300 ms 沒收到指令 | 停止移動指令 |
| 導航中 1 秒沒收到任何序列字元 | 中止導航 |

機器人模型與座標軸定義依照 Watson 等人的論文 [1]。

## 指令

指令是透過藍牙序列埠傳送的文字行。

| 指令 | 說明 |
|---|---|
| `#D v ω` | 連續遙控：速度 `v` 為 −20 到 20，轉速 `ω` 移動時最大 ±0.6 rad/s（原地 ±1.0） |
| `#M 距離 [角度]` | 直走 `距離` 公分（負值為後退），可選擇鎖定航向 |
| `#T 角度` | 轉到指定方向（度；0 = 起點車頭方向，往右為正） |
| `#C 弧長 半徑` | 走弧線（半徑為正 = 右轉） |
| `#S` 或 `#A` | 停止或中止 |

導航回報：`NAV:START`、`NAV:DONE`、`NAV:TIMEOUT`、`NAV:ABORT`、`NAV:REJECT`。遙測每 200 ms 傳一次（傾角、速度、狀態、PWM、里程計 X/Y）。

## 接線

| 功能 | Arduino 腳位 |
|---|---|
| 馬達 A：IN1、IN2、PWM | D7、D6、D9 |
| 馬達 B：IN3、IN4、PWM | D13、D12、D10 |
| TB6612FNG STBY | D8 |
| 左輪編碼器 A / B | D2（INT0）/ A0 |
| 右輪編碼器 A / B | A1（腳位變化中斷）/ D4 |
| MPU-6050 | I²C（A4 = SDA、A5 = SCL） |
| HC-06 | 硬體序列埠（D0 / D1），9600 baud |

編碼器：每圈 660 ticks（11 線霍爾 × 1:30 減速比 × 2 邊緣）。

## 資料夾結構

| 路徑 | 內容 |
|---|---|
| `repo.ino`、`nav.h`、`nav_cmd.h`、`rec.h`、`tuning.h` | 目前的韌體（v34）。這五個檔案要放在同一個資料夾 |
| `python/` | 電腦端工具：`drive.py`（遙控）、`nav.py`（路線）、`autotest.py`、`analyze.py` 等，詳見裡面的 README |
| `latest/`、`stable/` | 較早的韌體版本 |
| `libraries/` | 韌體使用的 Arduino 函式庫 |
| `Experiments/` | 編碼器測試程式 |
| `flow/` | 流程圖 |
| `historyrepo/` | 舊版文件與歷史版本 |

## 編譯與執行

1. 安裝 `libraries/` 裡的 Arduino 函式庫：I2Cdev、MPU6050（含 DMP MotionApps 2.0 標頭檔）和 PinChangeInterrupt。
2. 用 Arduino IDE 開啟根目錄的 `repo.ino`。`nav.h`、`nav_cmd.h`、`rec.h`、`tuning.h` 要放在同一個資料夾，少一個就編譯不過。
3. 選擇 **Arduino Uno** 並上傳。
4. 開機時讓車子直立並保持不動，等 DMP 啟動。序列埠會印出啟動訊息和 `v34 EMF/OVS/FAST=111`，代表新版韌體已燒錄成功。`setup()` 裡的 MPU-6050 偏移值是這台車專用，換一台車要重新校正。
5. 電腦配對 HC-06，執行 `python/` 裡的工具：
   ```
   cd python
   python -m pip install pyserial
   python drive.py --port COM5
   ```

備註：Flash 幾乎用滿（約 32.2 KB / 32.3 KB），所以 `ENABLE_TUNING` 預設為 0。在 `repo.ino` 把 `ENABLE_EMF_FF`、`ENABLE_OVERSPEED_BRAKE`、`ENABLE_FAST_START` 設為 0，就會回到 v33 的行為。

## 未來規劃（規劃中，尚未製作）

- **三輪機器人：** 左右兩顆麥克拉姆輪，加上中間一顆 90° 全向輪做橫移，平衡環保持不變。指令從 `#D v ω` 擴充為 `#D vx vy ω`。
- **安卓手機遙控：** 使用現成的藍牙終端機或搖桿 App 傳送 `#D`，並可執行固定路線。

## 參考文獻

[1] M. T. Watson, D. T. Gladwin, T. J. Prescott and S. O. Conran, "Dual-mode model predictive control of an omnidirectional wheeled inverted pendulum," *IEEE/ASME Trans. Mechatronics*, vol. 24, no. 6, pp. 2964–2975, Dec. 2019, doi: 10.1109/TMECH.2019.2943708.
