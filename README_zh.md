# 全向輪自平衡機器人設計

以兩顆同軸的麥克拉姆輪驅動的欠驅動兩輪倒單擺機器人。使用 ATmega328P（Arduino Uno），以硬體計時、多速率的串級控制運作，並預留之後加裝第三顆輪子做側向移動的設計。

[![YouTube](https://img.shields.io/badge/YouTube-Video%20Demo-red?logo=youtube)](https://www.youtube.com/shorts/1qH6NnuWVuc)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20Uno-blue.svg)](https://www.arduino.cc/)

[English](README.md) | [繁體中文](README_zh.md)

## 專案概述

| 項目 | 內容 |
|---|---|
| 主控 | ATmega328P，16 MHz（Arduino Uno） |
| 姿態感測 | MPU-6050（DMP 取俯仰角與航向角，傾角速度直接讀陀螺儀暫存器） |
| 馬達驅動 | TB6612FNG，兩顆附霍爾編碼器的直流減速馬達 |
| 輪子 | 兩顆 65 mm 麥克拉姆輪，裝在同一軸線上 |
| 尺寸與重量 | 寬 19 cm、高 17 cm、1.039 kg |
| 無線 | HC-06 藍牙，9600 baud |
| 電源 | 馬達用 2S 18650 電池（7.4 V）；晶片與感測器用 5 V 行動電源 |
| 韌體 | v37g（根目錄的 `repo.ino`） |
| 電腦端工具 | C++（`cpp/`）：`drive.exe` 遙控、`nav.exe` 路線 |

進度：**兩輪版已完成並實測**。**三輪版（中間加一顆轉 90° 的全向輪做側移）還在規劃中，尚未製作。**

## 實測結果

所有數字都來自機器人自己的感測器（每 200 ms 一筆的狀態記錄），沒有外部量測；樣本數少，而且各次測試之間韌體有更新。

| 測試 | 結果 | 依據 |
|---|---|---|
| 靜止站立 | 傾角在平均值 ±1.2° 內（σ = 0.42°） | 35 秒，10/12 記錄 |
| 最長平衡時間 | 約 850 秒沒有倒，期間來回行駛 | 10/20 記錄，韌體 v37e |
| 直線行駛 | 前進 16.5 cm/s、後退 17.7 cm/s | 10/13 自動測試，韌體 v34 |
| 直走指令 `#M` | 31 次完成 26 次；平均誤差 1.1 cm，最大 2.6 cm | 10/11 記錄，里程計 |
| 轉向指令 `#T` | 32 次；平均誤差 1.3°，最大 2.4° | 10/11 記錄，航向感測 |
| 手動遙控（W/X/A/D） | 按鍵與小車軌跡，畫在簡報第 16 頁 | `log/drive.csv`，10/27，v37g |
| 8 字形（三段 `#C` 圓弧，半徑 20 cm） | 終點離起點約 2 cm（里程計） | `log/8pattern.csv`，10/27，v37g |

說明：
- 31 次直走中有 5 次沒完成（3 次逾時、2 次緊急停止）。10/13 的 43 次自動測試中有 1 次倒地。
- 10/20 以前的記錄用的距離比例是 36.2 ticks/cm（推車 50 cm 校正）。10/20 用捲尺重新量測得到 33.6 ticks/cm（v37f 起使用），所以較早的距離與速度大約偏低 8%。

## 控制設計

Timer2 中斷（CTC 模式，除頻 64，OCR2A = 148）每 **10.13 ms（98.7 Hz）** 執行一次直立環，每 4 次執行一次速度環（**40.5 ms**）。中斷內有防重入保護，並重新開啟中斷，讓編碼器計數不中斷。

1. **直立環（PD）**：`ub = Kp·(θref − θ) − Kd·q`，Kp = 1275、Kd = 40。傾角速度 `q` 直接讀陀螺儀暫存器（DMP 給的陀螺儀值幾乎都是 0）。高速時加上反電動勢前饋（速度超過 8 的部分每單位 1.8 PWM，最多 50）。
2. **速度環（PI）**：輪速先濾波（`Vf = 0.7·Vf + 0.3·V`），PI 輸出（Kp = 2.8、Ki = 0.015）是一個小的補償傾角 θoff，限制在 ±3.5°（煞車時 ±3.0°，超速煞車時 ±4.0°）。速度比目標快 8 以上時，P 增益乘 1.6。
3. **航向環**：`ut = Kff·ωref + Kpy·(ψref − ψ) + Kdy·(ωref − ω)`，Kff = 30、Kpy = 38、Kdy = 7，限制在 ±45 PWM，傾角誤差大時會縮小（平衡優先）。v36 / v37 起：直行時加航向積分；靜止時可保持航向（按 `Y` 設定航向，偏超過 4° 就轉回來，回到 1.5° 內停止）。
4. **馬達輸出**：`uL = ub + ut`、`uR = ub − ut`，再加死區補償（5 PWM）與左輪增益校正 1.037，限制在 ±255。

每個直立環週期最先檢查的安全狀態：

| 條件 | 動作 |
|---|---|
| 傾角誤差 > 35° | `FALLEN`：馬達斷電、狀態歸零。傾角誤差連續 20 個週期小於 3° 後重新啟動 |
| 傾角誤差 > 7°（< 4.5° 退出） | `RESCUE`：只用較強的 PD（Kp 1400、Kd 48），不轉向 |
| 300 ms 沒收到指令 | 停止移動指令 |
| 導航中 1 秒沒收到任何字元 | 中止導航 |

機器人模型與座標軸定義參考 Watson 等人 [1]。

## 指令

指令是透過藍牙序列埠傳送的文字行。

| 指令 | 意義 |
|---|---|
| `#D v ω` | 連續遙控：速度 `v` 從 −20 到 20，角速度 `ω` 行進中最多 ±0.6 rad/s（原地 ±1.0）。需要每秒約 10 次持續重送 |
| `#M 距離 [角度]` | 直走指定公分（負數 = 後退），可指定鎖定的方向 |
| `#T 角度` | 轉到指定方向（度，0 = 起始方向，正 = 往右） |
| `#C 弧長 半徑 [速度]` | 走圓弧，弧長單位公分（半徑正 = 右轉）。圓弧進行中收到新的 `#C` 會直接串接，不停車 |
| `#F` | 結束目前的圓弧並減速 |
| `#S` 或 `#A` | 停止 / 中止 |

導航回報：`NAV:START`、`NAV:DONE`、`NAV:TIMEOUT`、`NAV:ABORT`、`NAV:REJECT`。每 200 ms 回傳一行遙測（傾角、速度、狀態、PWM、里程計 X/Y），欄位說明見 `log/README.md`。

## 接線

| 功能 | Arduino 腳位 |
|---|---|
| 馬達 A：IN1、IN2、PWM | D7、D6、D9 |
| 馬達 B：IN3、IN4、PWM | D13、D12、D10 |
| TB6612FNG STBY | D8 |
| 左編碼器 A / B | D2（INT0）/ A0 |
| 右編碼器 A / B | A1（pin change）/ D4 |
| MPU-6050 | I²C（A4 = SDA、A5 = SCL） |
| HC-06 | 硬體序列埠（D0 / D1），9600 baud |

編碼器：輪子每圈 660 tick（11 線霍爾 × 1:30 減速 × 計 2 個邊緣）。距離比例：33.6 ticks/cm（10/20 捲尺量測）。

## 資料夾結構

| 路徑 | 內容 |
|---|---|
| `repo.ino`、`nav.h`、`nav_cmd.h`、`rec.h`、`tuning.h` | **目前的韌體（v37g）**，五個檔案要放在同一個資料夾 |
| `cpp/` | **電腦端 C++ 工具**：`drive.cpp`（鍵盤遙控）、`nav.cpp`（路線）、`link.h`（序列埠與 CSV 記錄）、`sim.h`（模擬車）。說明見 `cpp/README.md` |
| `log/` | 簡報第 16 頁兩張圖的原始記錄：`drive.csv`（手動遙控）、`8pattern.csv`（8 字形） |
| `python/` | 早期的 Python 工具；自動測試 `autotest.py` 與記錄分析 `analyze.py` 仍在使用 |
| `stable/` | 保存的韌體版本（`v0.3` = v37g） |
| `latest/` | 較舊的韌體版本，保留作為紀錄（目前版本在根目錄） |
| `libraries/` | 韌體使用的 Arduino 函式庫 |
| `Experiments/` | 編碼器與感測器測試程式 |
| `flow/` | 流程圖 |
| `historyrepo/` | 較早的文件與版本 |

## 編譯與執行

**韌體**
1. 從 `libraries/`（或程式庫管理員）安裝 I2Cdev、MPU6050（含 DMP MotionApps 2.0 標頭檔）與 PinChangeInterrupt。
2. 用 Arduino IDE 開啟根目錄的 `repo.ino`；`nav.h`、`nav_cmd.h`、`rec.h`、`tuning.h` 要在同一個資料夾，否則無法編譯。
3. 選擇 **Arduino Uno** 並上傳。
4. 開機時讓車子直立不動，等 DMP 啟動。序列埠會印出開機訊息與 `v37g 11`，代表韌體已更新。`setup()` 裡的 MPU-6050 偏移值屬於這一顆感測器，換感測器要重新校正。

**電腦端工具（Windows，C++）**
1. 安裝 g++（MinGW-w64），在 `cpp/` 執行 `build.bat`，會產生 `drive.exe` 與 `nav.exe`（`g++ -std=c++17 -O2 -static`）。
2. 藍牙配對 HC-06 後，用鍵盤遙控：
   ```
   drive.exe --port COM9
   ```
   按鍵：`W`/`X` 前進 / 後退、`A`/`D` 左 / 右、`S` 停、`C` 方向回中、`Z` 座標歸零、`Y` 設定航向、`U`/`I` 微調航向、`Q` 結束。
3. 執行路線，例如簡報第 16 頁的 8 字形：
   ```
   nav.exe "C62.8@-20@15 C125.7@20@15 C62.8@-20@15" --steps --port COM9
   ```
   建議先用 `--dry-run`（預覽）或 `--sim`（模擬車）。`nav.exe --help` 會列出所有選項。
4. 兩個工具每次執行都會把 CSV 記錄存到 `controller/logs/<日期>/`。

備註：Flash 幾乎用滿，所以 `ENABLE_TUNING` 預設為 0。`repo.ino` 裡的功能開關（`ENABLE_EMF_FF`、`ENABLE_OVERSPEED_BRAKE`、`ENABLE_FAST_START`、`ENABLE_YAW_HOLD`、`ENABLE_YAW_I`）設成 0 即可個別關閉。

## 未來規劃（規劃中，尚未製作）

- **三輪版**：左右兩顆麥克拉姆輪，加上中間一顆轉 90° 的全向輪做側移，直立環不變。只有兩顆輪子時，側移和轉向一定同時發生；第三顆輪子補上缺少的約束。指令會從 `#D v ω` 擴充為 `#D vx vy ω`。
- **Android 手機遙控**：用現成的藍牙搖桿 App 傳送 `#D`，另外提供固定路線。

## 參考文獻

[1] M. T. Watson, D. T. Gladwin, T. J. Prescott and S. O. Conran, "Dual-mode model predictive control of an omnidirectional wheeled inverted pendulum," *IEEE/ASME Trans. Mechatronics*, vol. 24, no. 6, pp. 2964–2975, Dec. 2019, doi: 10.1109/TMECH.2019.2943708.
