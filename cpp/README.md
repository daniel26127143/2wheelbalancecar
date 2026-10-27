# 小車操控軟體（C++ 版）

把 `controller` 資料夾裡的 Python 操控軟體改寫成 C++，和 Arduino 韌體同一種語言。
通訊協定、指令、預設值、CSV 記錄格式都和 Python 版相同，所以 `analyze.py` 可以直接分析 C++ 版存的記錄檔。

| C++ | 對應的 Python | 做什麼 |
|---|---|---|
| `drive.cpp` | `drive.py` | 鍵盤遙控（W/X 油門、A/D 方向、S 停、Z 座標歸零…） |
| `nav.cpp` | `nav.py` + `run_nav.py` | 座標導航；沒有參數 = 選單，有參數 = 命令列（和 `nav.py` 的選項相同） |
| `link.h` | `control.py` | 序列埠（Win32 API）、遙測解析、CSV 記錄 |
| `sim.h` | `nav.py` 的 `NavCore` / `SimRobot` | 模擬車（`--sim`，不用實車也能測流程） |

沒有移植的：USB 手把（`drive.py --pad`）、`autotest.py`、`analyze.py`（這三個繼續用 Python，記錄檔格式相容）。

## 第一次使用

1. 在命令列輸入 `g++ --version`，確認有版本號，而且是 **GCC 9 以上**（要 C++17 的 `<filesystem>`）。
2. 用 VS Code 開啟這個 `cpp` 資料夾（File → Open Folder）。
3. **一次性設定**：在 `cpp` 資料夾裡建立 `.vscode` 資料夾，把 `vscode_tasks.json` 複製進去並改名為 `tasks.json`
   （程式沒有權限替你建立 `.vscode`）。
4. 打開 `nav.cpp` 或 `drive.cpp`，按 **Ctrl+Shift+B**：自動編譯並在下方 Terminal 執行（等同 Python 的 Run）。
   - 不想設定 `tasks.json`：直接雙擊 `build_run.bat`，或在這個資料夾打 `build_run.bat nav` / `build_run.bat drive`。
   - 只編譯不執行：`build.bat`（產生 `drive.exe`、`nav.exe`）。

編譯指令本身只有一行（`-static` 讓執行檔不需要額外的 DLL）：

```
g++ -std=c++17 -O2 -Wall -static nav.cpp -o nav.exe
```

## 怎麼用

**導航（`nav.exe`）**：沒有參數 → 選單（和 `run_nav.py` 一樣：選形狀 → 選模式 → 倍率 → 開始）。第一次請依序用「預覽」→「模擬車」→「實車」。
命令列也可以：

```
nav.exe "0,0 3,0 3,3 0,3 0,0" --dry-run     預覽計畫
nav.exe "0,0 3,0 3,3 0,3 0,0" --sim         模擬車
nav.exe "0,0 3,0 3,3 0,3 0,0"               實車
nav.exe --help                              全部選項
```

**遙控（`drive.exe`）**：`W/↑` 前進、`X/↓` 後退、`A/←` 左、`D/→` 右、`S/空白` 停、`C` 方向回中、`Z` 座標歸零、`Y` 設定航向、`Q/Esc` 結束。
**`Y`（韌體 v37 以上）**：把按下當下的車頭方向記成航向 setpoint；車子停著（停下超過 1 秒）時偏離超過約 4° 就自己轉回來。沒按過 `Y` 就不做靜止航向保持；按 `A/D/C` 或導航後設定作廢，要再按一次 `Y`；`W/X/S` 不影響。按下後車子會回 `>> 航向設定 xx.x`（度），遙測的 `TY` 就是設定角度。
`drive.exe --dry-run` 不連線，只印出每個畫面會送出的指令。選項（`--port`、`--vmax`、`--vback`、`--vmin`、`--vmin-back`、`--min-radius`、`--wmax`、`--hz`）和 `drive.py` 相同。

序列埠預設 `COM3`、9600；改埠號用 `--port COM5`，或改 `link.h` 最上面的 `COM_PORT`。

## 安全

- 導航中按 **Ctrl+C**：送 `#S` 和 `s` 中止。
- 程式被關掉或當掉時，韌體自己會煞車（導航 1 秒沒收到心跳、遙控 300 ms 沒收到指令）。
- `drive.exe` 讀鍵盤的方式和 Python 的 pynput 一樣是「全域」的：不管哪個視窗在前景，按 W/X/A/D 都會影響車子。開車時不要切去別的視窗打字。
- 實車前其他程式（Arduino IDE 序列埠監控視窗、Python 版）要先關掉，否則序列埠被占用。

## 這次有測、沒測的

有測（在沒有車的環境）：
- `DriveModel`（油門 / 方向 → 速度 / 角速度）和 `drive.py` 用 446 步隨機輸入逐筆比對，輸出完全相同。
- 導航的預覽（路線、`--smooth`、`--steps`、`--circle`、路線檔）輸出和 `nav.py` 逐字相同。
- 模擬車跑完整流程：路線、`--steps`、`--smooth`、`--circle --laps 0`、選單、Ctrl+C 中止；誤差大小和 Python 版模擬同一個等級。
- 序列埠收發、半行重組、CSV（UTF-8 BOM、CRLF、欄位順序）：用假的序列埠（pseudo-terminal）測過，`analyze.py` 能讀。
- 用 MinGW-w64 交叉編譯 Windows 版，沒有警告。

**沒測（需要你在 Windows 和實車上確認）**：
- Windows 的序列埠程式碼（`CreateFile` / `DCB` / `ReadFile`）：只編譯過，沒有真的開 COM 埠。
- `drive.exe` 的鍵盤（`GetAsyncKeyState`）：Windows 才有，這裡只用腳本模擬按鍵。
- VS Code 的 `tasks.json`、中文輸出在你的終端機是否正常。
- 實車導航：目前只確認「流程和 Python 版相同」，不代表實車表現。

建議第一次實車先用 `drive.exe --dry-run` 確認按鍵，再用 `drive.exe` 低速試開，最後才跑導航。
