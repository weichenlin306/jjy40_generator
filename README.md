# 電波錶JJY40對時電波產生器

http://RadioClockWave.mp4

### 簡介
- 本JJY40電波產生器可模擬日本發射的JJY40低頻對時電波，於每天午夜零時起連續六個整點發射電波，每次持續五分鐘。

### 硬體需求
- ESP32系列，具內建WiFi
- 使用DAC GPIO25作為電波輸出埠
- GPIO23作為啟動狀態LED指示燈輸出接腳
- 實測DAC GPIO25若僅接220Ω電阻與LED燈，在裝置10公分半徑範圍內可收到對時電波訊號。

### 程式編譯環境
- Eclips IDE 2022-09
- Espressif ESP-IDF

### License
- 無
- 部分程式段取自樂鑫範例程式
