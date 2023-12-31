# 電波錶JJY40對時電波產生器

https://github.com/weichenlin306/jjy40_generator/assets/133075659/c4bba9bd-c1b7-4e7d-8b46-566906e790c2

### 簡介
- 本JJY40電波產生器可模擬日本發射的JJY40低頻對時電波，於每天午夜零時起連續六個整點發射電波，每次持續五分鐘

### 硬體需求
- ESP32系列，具內建WiFi
- 使用DAC channel 0 (GPIO25)作為電波輸出埠
- 實測GPIO25若僅接220Ω電阻與LED燈(演示用)，在裝置10公分半徑範圍內可收到對時電波訊號
- GPIO23作為啟動狀態LED指示燈輸出接腳

### 程式編譯環境
- Eclipse IDE 2022-09
- Espressif ESP-IDF (stable v5.1.1)
- 使用時須把

    #define MY_ESP_WIFI_SSID "MY_SSID"

    #define MY_ESP_WIFI_PASS "MY_PASSWORD"

  字串(雙引號內文字)改為實際WiFi帳密
- 發射電波以外時間，均令其進入深度睡眠。由於深度睡眠使用之內部時鐘頻率低，且易受環境溫度、濕度影響計時準確度，最後一次發射完電波進入沈睡後，無法準時在午夜零時前醒來(有數分鐘誤差)，故提前在11:30pm左右先讓它醒來一次再令其沈睡。較短區間沈睡可以在較準確時間醒來。

### License
- 無
- 部分程式段取自樂鑫範例程式
