/*
 * Buddy Beacon Transmitter
 *
 * This code runs on an ESP32-S2 and sends a beacon message "BuddyBeacon"
 * to a specified receiver using ESPNOW when a button connected to GPIO 32 is pressed.
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
#include <string.h>
#include <esp_log.h>
#include <esp_mac.h>

/* Tag used for ESP_LOG */
static const char *TAG = "BuddyBeacon_Transmitter";

/* Definitions for GPIO pins */
#define BUTTON_GPIO GPIO_NUM_32  // GPIO pin connected to the button

/* Receiver's MAC Address */  // 4C:11:AE:65:76:C4
static uint8_t receiver_mac[ESP_NOW_ETH_ALEN] = {0x4C, 0x11, 0xAE, 0x65, 0x76, 0xC4};

/* Primary Master Key (PMK) for ESPNOW */
#define ESPNOW_PMK "1234567890"

/* Local Master Key (LMK) for encryption with the receiver */
#define ESPNOW_LMK "1234567890"

/* Message to send */
static const char *message = "BuddyBeacon";

/* Function Prototypes */
void init_gpio(void);
void init_espnow(void);
void send_buddy_beacon(void);
void button_task(void *pvParameter);
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
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

    /* Initialize GPIO and ESPNOW */
    init_gpio();
    init_espnow();

    /* Create a FreeRTOS task to handle button presses */
    xTaskCreate(&button_task, "button_task", 2048, NULL, 5, NULL);

    /* Print the device's MAC address */
    print_mac_address();
}

/**
 * @brief Initialize the GPIO for the button input
 */
void init_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;    // Disable interrupt
    io_conf.mode = GPIO_MODE_INPUT;           // Set as input mode
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO); // Bit mask of the pins to set
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up mode (button default state is HIGH)
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "GPIO initialized for button input");
}

/**
 * @brief Initialize ESPNOW and register the peer
 */
void init_espnow(void)
{
    /* Initialize ESPNOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_LOGI(TAG, "ESP-NOW initialized");

    /* Set Primary Master Key (PMK) */
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));
    ESP_LOGI(TAG, "Primary Master Key (PMK) set");

    /* Register the sending callback function */
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    /* Register peer with MAC address and encryption key */
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiver_mac, ESP_NOW_ETH_ALEN);
    peerInfo.channel = 0; // Channel number (auto)
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = true; // Enable encryption

    /* Set Local Master Key (LMK) for this peer */
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
 * @brief Task to monitor the button and send the beacon message
 *
 * @param pvParameter Task parameters (unused)
 */
void button_task(void *pvParameter)
{
    bool button_pressed = false;
    while (true)
    {
        /* Read the button state */
        int button_state = gpio_get_level(BUTTON_GPIO);

        if (button_state == 0 && !button_pressed)
        {
            /* Button is pressed */
            button_pressed = true;
            ESP_LOGI(TAG, "Button pressed");
            send_buddy_beacon();
        }
        else if (button_state == 1 && button_pressed)
        {
            /* Button is released */
            button_pressed = false;
            ESP_LOGI(TAG, "Button released");
        }

        /* Delay to debounce the button */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Send the beacon message to the receiver
 */
void send_buddy_beacon(void)
{
    /* Prepare the data to send */
    const uint8_t *peer_addr = receiver_mac;
    const uint8_t *data = (const uint8_t *)message;
    size_t len = strlen(message) + 1; // Include null terminator

    /* Send the data */
    esp_err_t result = esp_now_send(peer_addr, data, len);
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Sent message: %s", message);
    }
    else
    {
        ESP_LOGE(TAG, "Error sending message");
    }
}

/**
 * @brief Callback function executed when data is sent
 *
 * @param mac_addr MAC address of the receiver
 * @param status   Status of the transmission
 */
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char mac_str[18];
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send callback: NULL MAC address");
        return;
    }

    /* Convert MAC address to string */
    snprintf(mac_str, sizeof(mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "Data sent successfully to %s", mac_str);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send data to %s", mac_str);
    }
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

