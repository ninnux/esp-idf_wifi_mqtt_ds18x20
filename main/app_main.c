#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/i2c.h"

#include "esp_sleep.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"


#include "ds18x20.h"

#include "ninux_esp32_ota.h"


#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define TAG_BME280 "BME280"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

//static const char *TAG = "MQTT_EXAMPLE";
//const char *TAG = "MQTT_EXAMPLE";

//static EventGroupHandle_t wifi_event_group;
EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;


static RTC_DATA_ATTR struct timeval sleep_enter_time;

uint8_t msgData[32];

SemaphoreHandle_t xSemaphore = NULL;


static const gpio_num_t SENSOR_GPIO = 17;
static const uint32_t LOOP_DELAY_MS = 250;
static const int MAX_SENSORS = 8;
static const int RESCAN_INTERVAL = 8;

void ds18x20_test(void *pvParameter)
{
   if( xSemaphore != NULL )
   {
       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
       {
         ds18x20_addr_t addrs[MAX_SENSORS];
         float temps[MAX_SENSORS];
         int sensor_count;

         // There is no special initialization required before using the ds18x20
         // routines.  However, we make sure that the internal pull-up resistor is
         // enabled on the GPIO pin so that one can connect up a sensor without
         // needing an external pull-up (Note: The internal (~47k) pull-ups of the
         // ESP8266 do appear to work, at least for simple setups (one or two sensors
         // connected with short leads), but do not technically meet the pull-up
         // requirements from the ds18x20 datasheet and may not always be reliable.
         // For a real application, a proper 4.7k external pull-up resistor is
         // recommended instead!)

         // Every RESCAN_INTERVAL samples, check to see if the sensors connected
         // to our bus have changed.
         sensor_count = ds18x20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS);

         if (sensor_count < 1)
             printf("No sensors detected!\n");
         else
         {
             printf("%d sensors detected:\n", sensor_count);
             // If there were more sensors found than we have space to handle,
             // just report the first MAX_SENSORS..
             if (sensor_count > MAX_SENSORS)
                 sensor_count = MAX_SENSORS;

             // Do a number of temperature samples, and print the results.
             for (int i = 0; i < RESCAN_INTERVAL; i++)
             {
                 ds18x20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count, temps);
                 for (int j = 0; j < sensor_count; j++)
                 {
                     // The ds18x20 address is a 64-bit integer, but newlib-nano
                     // printf does not support printing 64-bit values, so we
                     // split it up into two 32-bit integers and print them
                     // back-to-back to make it look like one big hex number.
                     uint32_t addr0 = addrs[j] >> 32;
                     uint32_t addr1 = addrs[j];
                     float temp_c = temps[j];
                     float temp_f = (temp_c * 1.8) + 32;
                     printf("  Sensor %08x%08x reports %f deg C (%f deg F)\n", addr0, addr1, temp_c, temp_f);
	                 sprintf((char*)msgData,"{\"temp\":%d}",(int) temp_c*100);
                 }
                 printf("\n");

                 // Wait for a little bit between each sample (note that the
                 // ds18x20_measure_and_read_multi operation already takes at
                 // least 750ms to run, so this is on top of that delay).
                 vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);
             }
         }
	     xSemaphoreGive( xSemaphore );
	     vTaskDelete(NULL);
        }
    }

}


void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    const int ext_wakeup_pin_1 = 25;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = 26;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char mqtt_topic[128];
    bzero(mqtt_topic,sizeof(mqtt_topic));
    sprintf(mqtt_topic,"ambiente/%s/jsondata",CONFIG_MQTT_NODE_NAME);
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, mqtt_topic,(const char *) msgData, 0, 1, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static int s_retry_num = 0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < 3) {
            	esp_wifi_connect();
            	xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
                s_retry_num++;
	    }else{
		printf("restart!!\n");
		esp_restart();	
	    } 
            break;
	}
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };


   while (1) {
   	if( xSemaphore != NULL )
   	{


#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */


    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) { 
      printf("semaforo libero");
      esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
      esp_mqtt_client_start(client);
      
      //vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
      sleeppa(300);

    }else{
      //printf("semaforo occupato");
    }
    }
  }
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);




    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    wifi_init();
    esp_ota_mark_app_valid_cancel_rollback(); 
    ninux_esp32_ota();


    vSemaphoreCreateBinary( xSemaphore );
    vTaskDelay( 1000 / portTICK_RATE_MS );
    //xTaskCreate( &DHT_task, "DHT_task", 2048, NULL, 5, NULL );
    //i2c_master_init();
    //xTaskCreate(&task_bme280_normal_mode, "bme280_normal_mode",  2048, NULL, 6, NULL);
    xTaskCreate(ds18x20_test, "ds18x20_test", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    vTaskDelay( 3000 / portTICK_RATE_MS );

    mqtt_app_start();
}
