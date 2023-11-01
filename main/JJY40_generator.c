#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "rtc_wdt.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_sntp.h"
#include "time.h"
#include "sys/time.h"
#include "esp_timer.h"
#include "driver/dac_continuous.h"
#include "driver/gpio.h"

#define byte uint8_t
#define TICK_VALUE 998242LL	// 每秒振盪器內相當數值
#define WAVE_POINT_NUM 60

static time_t now;
static struct tm timeinfo;

// 代表full power全波形與reduced power (< 10%)的衰減波形
static byte  fullWav[WAVE_POINT_NUM];
static byte  reducedWav[WAVE_POINT_NUM];

static byte	 JJY[60];
static uint64_t lastSec = 0;
static short lastMin = -1;
static bool  gotTime = false;
static bool  secAdv = true;		//  second advanced? Act as run_once for the first time
static bool  jjy40Sending = false;

static uint64_t elapsed_time_since_boot;
static int64_t  deep_sleep_time = 0;

// periodic timer callback every 0.1s
static void periodic_timer_callback(void* arg)
{
	secAdv = false;
	static time_t currSecs;

	if ( gotTime ) {
		time(&currSecs);
		if ( currSecs != lastSec ) {
			lastSec = currSecs;
			secAdv = true;
		}
	}
}

void time_sync_notification_cb(struct timeval *tv)
{
	static const char* TAG = "sntp";
	ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
	static const char* TAG = "sntp";
	ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    esp_sntp_init();
}

static void obtain_time(void)
{
    static const char* TAG = "sntp";

    /**
     * NTP server address could be aquired via DHCP,
     * see LWIP_DHCP_GET_NTP_SRV menuconfig option
     */
#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1);
#endif

    initialize_sntp();

    // wait for time to be set
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
    	ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    gotTime = ( retry < retry_count );
}


#define MY_ESP_WIFI_SSID      "MY_SSID"
#define MY_ESP_WIFI_PASS      "MY_PASSWORD"
#define MY_ESP_MAXIMUM_RETRY  10

#define WIFI_CONNECTED_BIT	BIT0
#define WIFI_FAIL_BIT		BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	static const char* TAG = "WiFi station";

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MY_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    static const char* TAG = "WiFi station";
	s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
        	.ssid = MY_ESP_WIFI_SSID,
            .password = MY_ESP_WIFI_PASS,
	    	.threshold.authmode = WIFI_AUTH_WPA2_PSK,
	    	.sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
    	ESP_LOGI(TAG, "connected to ap SSID: %s", MY_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
    	ESP_LOGI(TAG, "Failed to connect to SSID: %s", MY_ESP_WIFI_SSID);
    } else {
    	ESP_LOGI(TAG, "UNEXPECTED EVENT");
    }

    // Contact NTP server to retrieve current date/time
    obtain_time();

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void sync_time(void)
{
	// Establish WiFi Internet connection & query NTP server for date/time
    static const char* TAG = "sync_time";
    char strftime_buf[64];

	wifi_init_sta();
    setenv("TZ", "JST-9", 1);
    tzset();
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c (%j)", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Tokyo is: %s", strftime_buf);
}

static dac_continuous_handle_t chan0_handle;
static dac_continuous_config_t cont0_cfg = {
	.chan_mask = DAC_CHANNEL_MASK_ALL,
	.desc_num = 8,
	.buf_size = WAVE_POINT_NUM,
	.freq_hz = 40000 * WAVE_POINT_NUM,
	.offset = 0,
	.clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
	.chan_mode = DAC_CHANNEL_MODE_SIMUL,
};

// 產生鋸齒波陣列
void generateSawWave(void)	// 鋸齒波形
{
	int numToRidge = 8;
	float voltage;

	// 波峰前
	for (int i = 0; i < numToRidge; i++) {
		voltage = 255.0 * i / numToRidge;
		// 全波形
		fullWav[i] = voltage;
		// 減振波形
		reducedWav[i] = voltage / 16.0;
	}

	// 波峰後
	for (int i = numToRidge; i < WAVE_POINT_NUM; i++) {
		voltage = 255.0 * (WAVE_POINT_NUM - i) / (WAVE_POINT_NUM - numToRidge);
		// 全波形
		fullWav[i] = voltage;
		// 減振波形
		reducedWav[i] = voltage / 16.0;
	}
}

void sendBit(byte bit)
{
	ESP_ERROR_CHECK(dac_continuous_write_cyclically(chan0_handle, (byte*)fullWav, WAVE_POINT_NUM, NULL));

	switch (bit) {
		case 'M':
			vTaskDelay(pdMS_TO_TICKS(200));
			break;
		case 0:
			vTaskDelay(pdMS_TO_TICKS(800));
			break;
		case 1:
			vTaskDelay(pdMS_TO_TICKS(500));
			break;
	}

	ESP_ERROR_CHECK(dac_continuous_write_cyclically(chan0_handle, (byte*)reducedWav, WAVE_POINT_NUM, NULL));
}

void app_main(void)
{
	// Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

     // Initialize array
	for (int i=0; i < 60; i++) {
		if ( i == 0 || i % 10 == 9 ) {
			JJY[i] = 'M';
		} else {
			JJY[i] = 0;
		}
	}

	// 產生鋸齒波陣列
	generateSawWave();

	// Setup & enable timer
    const esp_timer_create_args_t periodic_timer_args = {
		.callback = &periodic_timer_callback,
		.name = "periodic"	// name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    // Start the timers, callback every 0.1s
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TICK_VALUE/10));

    // Setup cosine wave channel with frequency of 40kHz
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont0_cfg, &chan0_handle));
    ESP_ERROR_CHECK(dac_continuous_enable(chan0_handle));

    // Light up LED to indicate active status
    gpio_reset_pin(GPIO_NUM_23);
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_23, 1);

	// Disable RTC watchdog
	rtc_wdt_disable();

	while ( true ) {
		if ( secAdv ) {
			secAdv = false;
			time(&now);
			localtime_r(&now, &timeinfo);

			if ( gotTime ) {
				if ( timeinfo.tm_sec == 0 ) {
					elapsed_time_since_boot = esp_timer_get_time();
					if ( elapsed_time_since_boot > 5 * 60 * TICK_VALUE ) {	// 若已運行5分鐘，則進入沈睡模式
						if ( timeinfo.tm_hour > 1 ) {	// 沈睡至 23:30:00am 再重開機
							deep_sleep_time = (((23 - timeinfo.tm_hour) * 60 + (30 - timeinfo.tm_min)) * 60 ) * TICK_VALUE;
							if ( deep_sleep_time < 0 ) {
								deep_sleep_time = (((24 - timeinfo.tm_hour) * 60 + (60 - timeinfo.tm_min)) * 60 - 20 ) * TICK_VALUE;
							}
						} else {						// 沈睡至 1:59:40am 再重開機
							deep_sleep_time = ((60 - timeinfo.tm_min) * 60 - 20) * TICK_VALUE;
						}
						esp_deep_sleep(deep_sleep_time);
					} else if ( ESP_SLEEP_WAKEUP_TIMER == esp_sleep_get_wakeup_cause() && timeinfo.tm_hour == 23 &&
						elapsed_time_since_boot > 20 * TICK_VALUE ) {		// 如由沈睡中甦醒超過20秒且是日本時間23:xx，則繼續沈睡
						deep_sleep_time = (((24 - timeinfo.tm_hour) * 60 + (60 - timeinfo.tm_min)) * 60 - 20 ) * TICK_VALUE;
						esp_deep_sleep(deep_sleep_time);
					}
				}
				jjy40Sending = true;
			} else {
				// 開機或從沈睡中甦醒後網路對時一次
				jjy40Sending = false;	// 對時期間暫時關閉JJY40訊號送出
				sync_time();
				// 若未對時成功(WiFi failed?)則沈睡5分鐘後再開機對時一次
				if ( !gotTime ) {
					esp_deep_sleep(5 * 60 * TICK_VALUE);
				}
			}

			// 每分鐘起始點設定JJY40欲傳遞時間(Tokyo)
			byte bcd;
			int val;
			if ( timeinfo.tm_min != lastMin ) {
				lastMin = timeinfo.tm_min;
				char buf[36];
				strftime(buf, 36, "%c (%j)", &timeinfo);
				printf("%s\n", buf);
				// Initialize even parity of minutes bit
				JJY[37] = 0;
				// Set minute
				val = timeinfo.tm_min;
				bcd = val/10 * 16 + val%10;
				JJY[8] = bcd & 0x01;  JJY[37] ^= JJY[8];  bcd >>= 1;
				JJY[7] = bcd & 0x01;  JJY[37] ^= JJY[7];  bcd >>= 1;
				JJY[6] = bcd & 0x01;  JJY[37] ^= JJY[6];  bcd >>= 1;
				JJY[5] = bcd & 0x01;  JJY[37] ^= JJY[5];  bcd >>= 1;
				JJY[3] = bcd & 0x01;  JJY[37] ^= JJY[3];  bcd >>= 1;
				JJY[2] = bcd & 0x01;  JJY[37] ^= JJY[2];  bcd >>= 1;
				JJY[1] = bcd & 0x01;  JJY[37] ^= JJY[1];
				// Initialize even parity of hours bit
				JJY[36] = 0;
				// Set hour
				val = timeinfo.tm_hour;
				bcd = val/10 * 16 + val%10;
				JJY[18] = bcd & 0x01;  JJY[36] ^= JJY[18];  bcd >>= 1;
				JJY[17] = bcd & 0x01;  JJY[36] ^= JJY[17];  bcd >>= 1;
				JJY[16] = bcd & 0x01;  JJY[36] ^= JJY[16];  bcd >>= 1;
				JJY[15] = bcd & 0x01;  JJY[36] ^= JJY[15];  bcd >>= 1;
				JJY[13] = bcd & 0x01;  JJY[36] ^= JJY[13];  bcd >>= 1;
				JJY[12] = bcd & 0x01;  JJY[36] ^= JJY[12];
				// Set day of year
				val = timeinfo.tm_yday + 1; // 1=Jan 1
				bcd = val/100;  val = val - bcd*100;
				JJY[23] = bcd & 0x01;  bcd >>= 1;
				JJY[22] = bcd & 0x01;
				bcd = val/10 * 16 + val%10;
				JJY[33] = bcd & 0x01;  bcd >>= 1;
				JJY[32] = bcd & 0x01;  bcd >>= 1;
				JJY[31] = bcd & 0x01;  bcd >>= 1;
				JJY[30] = bcd & 0x01;  bcd >>= 1;
				JJY[28] = bcd & 0x01;  bcd >>= 1;
				JJY[27] = bcd & 0x01;  bcd >>= 1;
				JJY[26] = bcd & 0x01;  bcd >>= 1;
				JJY[25] = bcd & 0x01;
				// Set year
				val = timeinfo.tm_year%100;
				bcd = val/10 * 16 + val%10;
				JJY[48] = bcd & 0x01;  bcd >>= 1;
				JJY[47] = bcd & 0x01;  bcd >>= 1;
				JJY[46] = bcd & 0x01;  bcd >>= 1;
				JJY[45] = bcd & 0x01;  bcd >>= 1;
				JJY[44] = bcd & 0x01;  bcd >>= 1;
				JJY[43] = bcd & 0x01;  bcd >>= 1;
				JJY[42] = bcd & 0x01;  bcd >>= 1;
				JJY[41] = bcd & 0x01;
				// Set day of week
				bcd = timeinfo.tm_wday;
				JJY[52] = bcd & 0x01;  bcd >>= 1;
				JJY[51] = bcd & 0x01;  bcd >>= 1;
				JJY[50] = bcd & 0x01;

				for (int i=0; i < 60; i++) {
					if ( JJY[i] == 'M' ) {
						printf(" M ");
					} else {
						printf("%d", JJY[i]);
					}
				}
				printf("%s\n", jjy40Sending?"(sending)":"(not sent)");
			}

			// Send signal
			if ( jjy40Sending ) {
				sendBit(JJY[timeinfo.tm_sec]);
			}
		}
		vTaskDelay(1);
	}
}
