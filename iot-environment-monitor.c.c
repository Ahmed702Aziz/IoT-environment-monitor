#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_tls.h"
#include "esp_netif.h"
#include "lwip/apps/sntp.h"
#include "cJSON.h"

#define DHT_PIN            GPIO_NUM_2
#define DHT_POWER_PIN      GPIO_NUM_4
#define LED_PIN            GPIO_NUM_5
#define ANALOG_PIN         ADC1_CHANNEL_0
#define ACS712_POWER_PIN   GPIO_NUM_18

#define MAX_TEMP           40.0
#define MIN_TEMP           14.0
#define MAX_HUM            70.0
#define MIN_HUM            20.0
#define MAX_CURRENT        25.0
#define MIN_CURRENT        0.1
#define SENSITIVITY        0.066
#define VCC                1.0
#define SLEEP_TIME_US      60000000ULL

static const char* TAG = "MAIN";

#define MQTT_URI "mqtts://ahmedaziz:Aziz1234567@94fbaebfe12e4662be30e6222d1af83d.s1.eu.hivemq.cloud:8883"
#define TEMP_TOPIC "youssef/temp"
#define HUM_TOPIC "youssef/hum"
#define ALERT_TOPIC "youssef/alert"
#define CURRENT_TOPIC "datacenter/current"
#define CURRENT_ALERT_TOPIC "datacenter/alerts"

static esp_mqtt_client_handle_t mqtt_client = NULL;
static float vref = 0.5;

void configure_gpio() {
    gpio_set_direction(DHT_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ACS712_POWER_PIN, GPIO_MODE_OUTPUT);
}

void calibrate_acs712() {
    ESP_LOGI(TAG, "Calibrating ACS712...");
    int sum = 0;
    for (int i = 0; i < 500; ++i) {
        int val = adc1_get_raw(ANALOG_PIN);
        sum += val;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    vref = ((float)sum / 500.0) / 4096.0 * VCC;
    ESP_LOGI(TAG, "Vref: %.4f", vref);
}

float get_ac_rms_current() {
    double sum_sq = 0;
    for (int i = 0; i < 1000; ++i) {
        float volt = ((float)adc1_get_raw(ANALOG_PIN) / 4096.0 * VCC) - vref;
        sum_sq += volt * volt;
        ets_delay_us(800);
    }
    float rms = sqrt(sum_sq / 1000.0);
    return rms / SENSITIVITY;
}

void mqtt_publish_json(const char* topic, float value) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "value", value);
    char *msg = cJSON_PrintUnformatted(json);
    esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 1, 0);
    cJSON_Delete(json);
    free(msg);
}

void send_alert(const char* topic, const char* message, float temp, float hum) {
    cJSON *alert = cJSON_CreateObject();
    cJSON_AddStringToObject(alert, "device", "ESP32-DHT22");
    cJSON_AddNumberToObject(alert, "temperature", temp);
    cJSON_AddNumberToObject(alert, "humidity", hum);
    cJSON_AddStringToObject(alert, "message", message);

    char *msg = cJSON_PrintUnformatted(alert);
    esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 1, 0);
    cJSON_Delete(alert);
    free(msg);
}

void initialize_sntp() {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    time_t now = 0;
    while (now < 100000) {
        time(&now);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Time synced: %ld", now);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected");
    }
}

void app_main() {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    configure_gpio();

    gpio_set_level(ACS712_POWER_PIN, 1);
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ANALOG_PIN, ADC_ATTEN_DB_11);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_URI,
        .cert_pem = NULL
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    initialize_sntp();
    calibrate_acs712();

    // NOTE: DHT22 read would require DHT component (or adapt one from PlatformIO)
    float temperature = 25.0; // Dummy for now
    float humidity = 50.0;    // Dummy for now

    float current = get_ac_rms_current();

    mqtt_publish_json(TEMP_TOPIC, temperature);
    mqtt_publish_json(HUM_TOPIC, humidity);

    time_t now;
    time(&now);
    cJSON *cur = cJSON_CreateObject();
    cJSON_AddNumberToObject(cur, "timestamp", now);
    cJSON_AddNumberToObject(cur, "current_A", current);
    cJSON_AddNumberToObject(cur, "vref", vref);
    char *cur_msg = cJSON_PrintUnformatted(cur);
    esp_mqtt_client_publish(mqtt_client, CURRENT_TOPIC, cur_msg, 0, 1, 0);
    cJSON_Delete(cur);
    free(cur_msg);

    if (temperature > MAX_TEMP || temperature < MIN_TEMP || humidity > MAX_HUM || humidity < MIN_HUM) {
        send_alert(ALERT_TOPIC, "Temp/Humidity Alert", temperature, humidity);
    }
    if (current > MAX_CURRENT || current < MIN_CURRENT) {
        const char* msg = current > MAX_CURRENT ? "OVERLOAD" : "POWER LOSS";
        send_alert(CURRENT_ALERT_TOPIC, msg, temperature, humidity);
    }

    esp_mqtt_client_stop(mqtt_client);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(ACS712_POWER_PIN, 0);
    esp_deep_sleep(SLEEP_TIME_US);
}
