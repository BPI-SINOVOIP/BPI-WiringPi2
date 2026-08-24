# BPI-M4 Berry 板型資訊崩潰修正紀錄

## 問題

在 BPI-M4 Berry 上以最佳化參數編譯後，`gpio -v` 會發生段錯誤。GDB
確認崩潰位於硬體資訊輸出，`maker` 與 `warranty` 尚未初始化。

## 原因

第一次偵測已從 Device Tree 得到 BPI-M4 Berry 型號 79，但
`bpi_piBoardId()` 隨後呼叫共用的 `piGpioLayout()`。此時
`RaspberryPiModel` 尚未由 `bpi_piBoardId()` 寫入，因此共用函式回傳 `-1`；
查表失敗後函式提早返回，輸出參數保留未定值。

## 修正

`bpi_piBoardId()` 改為直接呼叫 `bpi_piGpioLayout()`，取得同一次偵測已快取
的 BPI 型號，再完成所有輸出參數與後端狀態初始化。來源碼回歸測試
`tools/test-bpi-board-id.py` 會防止呼叫關係退回有問題的共用路徑。

## 驗證要求

1. 執行 `python3 tools/test-bpi-board-id.py -v`。
2. 以 `./build debian` 產生 arm64 套件並安裝到 BPI-M4 Berry。
3. `gpio -v` 必須正常結束，板型必須顯示 `Banana Pi M4 Berry[H618]`。
4. `sudo gpio readall` 必須正常顯示 40-pin 對照；未接負載時不得執行輸出測試。
