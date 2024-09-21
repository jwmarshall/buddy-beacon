/*
 * Buddy Beacon Receiver
 *
 * This code runs on an ESP32-S2 and listens for a beacon message "BuddyBeacon"
 * from a transmitter using ESPNOW. When the message is received, it turns on
 * a PWM-controlled LED connected to GPIO 33 for a short duration.
 */

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_pm.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <string.h>
#include <esp_log.h>
#include <esp_mac.h>

/* Tag used for ESP_LOG */
static const char *TAG = "BuddyBeacon_Receiver";

/* Definitions for GPIO pins */
#define LED_GPIO GPIO_NUM_33  // GPIO pin connected to the LED

/* LEDC (LED Controller) Configuration */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Resolution of PWM duty
#define LEDC_FREQUENCY          5000              // Frequency in Hertz

/* Primary Master Key (PMK) for ESPNOW */
#define ESPNOW_PMK "1234567890"

/* Local Master Key (LMK) for encryption with the transmitter */
#define ESPNOW_LMK "1234567890"

/* Expected message */
static const char *expected_message = "BuddyBeacon";

/* Function Prototypes */
void init_gpio(void);
void init_led_pwm(void);
void init_espnow(void);
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len);
void turn_on_led(void);
void print_mac_address(void);

/**
 * @brief Main application entry point
 */
extern "C" void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Set Wi-Fi mode to station */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Initialize GPIO and PWM */
    init_gpio();
    init_led_pwm();

    /* Print the device's MAC address */
    print_mac_address();

    /* Initialize ESPNOW */
    init_espnow();
}

/**
 * @brief Initialize the GPIO for the LED output
 */
void init_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;    // Disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;          // Set as output mode
    io_conf.pin_bit_mask = (1ULL << LED_GPIO); // Bit mask of the pins to set
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;  // Disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "GPIO initialized for LED output");
}

/**
 * @brief Initialize the LED PWM controller
 */
void init_led_pwm(void)
{
    /* Prepare and set configuration of timer */
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_FREQUENCY;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /* Prepare and set configuration of channel */
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = LEDC_CHANNEL;
    ledc_channel.timer_sel      = LEDC_TIMER;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = LED_GPIO;
    ledc_channel.duty           = 0; // Set duty to 0%
    ledc_channel.hpoint         = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "LEDC initialized for PWM output on GPIO %d", LED_GPIO);
}

/**
 * @brief Initialize ESPNOW and register the receive callback
 */
void init_espnow(void)
{
    /* Initialize ESPNOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_LOGI(TAG, "ESP-NOW initialized");

    /* Set Primary Master Key (PMK) */
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));
    ESP_LOGI(TAG, "Primary Master Key (PMK) set");

    /* Register the receive callback function */
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Set up peer information */
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0; // Channel number (auto)
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = true; // Enable encryption

    /* Since we don't know the transmitter's MAC address in advance, set it to all zeros */
    memset(peerInfo.peer_addr, 0, ESP_NOW_ETH_ALEN);

    /* Set Local Master Key (LMK) */
    memcpy(peerInfo.lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);

    /* Add the peer */
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add peer");
    }
    else
    {
        ESP_LOGI(TAG, "Peer added successfully");
    }
}

/**
 * @brief Receive callback function executed when data is received
 *
 * @param recv_info Information about the received data
 * @param data      Pointer to the received data buffer
 * @param data_len  Length of the received data
 */
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    char mac_str[18];

    /* Convert sender's MAC address to string */
    snprintf(mac_str, sizeof(mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    /* Null-terminate the received data */
    char received_message[256] = {0};
    memcpy(received_message, data, data_len < 255 ? data_len : 255);

    ESP_LOGI(TAG, "Received data from %s: %s", mac_str, received_message);

    /* Check if the received message matches the expected message */
    if (strcmp(received_message, expected_message) == 0)
    {
        ESP_LOGI(TAG, "Expected message received. Turning on LED.");
        turn_on_led();
    }
    else
    {
        ESP_LOGW(TAG, "Unexpected message received.");
    }
}

/**
 * @brief Turn on the LED for a short duration using PWM
 */
void turn_on_led(void)
{
    /* Set LED duty cycle to 50% */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (1 << LEDC_DUTY_RES) / 2));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    /* Keep the LED on for 2 seconds */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Turn off the LED */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    ESP_LOGI(TAG, "LED turned off");
}

/**
 * @brief Print the device's MAC address to the console
 */
void print_mac_address(void)
{
    uint8_t mac_addr[6];
    /* Get the MAC address for the Wi-Fi station interface */
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac_addr));

    /* Convert MAC address to string */
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    ESP_LOGI(TAG, "Device MAC Address (STA): %s", mac_str);
}

