#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include <sys/queue.h>

#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_check.h"


#include "esp_netif.h"
#include "led_indicator.h"
#include "led_indicator_blink_default.h"
#include "usbh_modem_board.h"
#include "usbh_modem_wifi.h"

#ifdef CONFIG_MODEM_ENABLE_WEB_ROUTER
#include "modem_http_config.h"
#endif

#ifdef CONFIG_MODEM_PING_NETWORK
#include "ping/ping_sock.h"
#endif

#include "user_4g_modem.h"

static const char *TAG = "4g_main";

#define LED_RED_SYSTEM_GPIO                 CONFIG_MODEM_LED_RED_SYSTEM_GPIO
#define LED_BLUE_WIFI_GPIO                  CONFIG_MODEM_LED_BLUE_WIFI_GPIO
#define LED_GREEN_4GMODEM_GPIO              CONFIG_MODEM_LED_GREEN_4GMODEM_GPIO
#define LED_ACTIVE_LEVEL                    1

static modem_wifi_config_t s_modem_wifi_config = MODEM_WIFI_DEFAULT_CONFIG();

static led_indicator_handle_t s_led_system_handle = NULL;
static led_indicator_handle_t s_led_wifi_handle = NULL;
static led_indicator_handle_t s_led_4g_handle = NULL;

static void _led_indicator_init()
{
    led_indicator_gpio_config_t led_indicator_gpio_config = {
        .is_active_level_high = LED_ACTIVE_LEVEL,
    };

    led_indicator_config_t led_config = {
        .led_indicator_gpio_config = &led_indicator_gpio_config,
        .mode = LED_GPIO_MODE,
    };

    if (LED_RED_SYSTEM_GPIO) {
        led_indicator_gpio_config.gpio_num = LED_RED_SYSTEM_GPIO;
        s_led_system_handle = led_indicator_create(&led_config);
        assert(s_led_system_handle != NULL);
    }
    if (LED_BLUE_WIFI_GPIO) {
        led_indicator_gpio_config.gpio_num = LED_BLUE_WIFI_GPIO;
        s_led_wifi_handle = led_indicator_create(&led_config);
        assert(s_led_wifi_handle != NULL);
        led_indicator_stop(s_led_wifi_handle, BLINK_CONNECTED);
        led_indicator_start(s_led_wifi_handle, BLINK_CONNECTING);
    }
    if (LED_GREEN_4GMODEM_GPIO) {
        led_indicator_gpio_config.gpio_num = LED_GREEN_4GMODEM_GPIO;
        s_led_4g_handle = led_indicator_create(&led_config);
        assert(s_led_4g_handle != NULL);
        led_indicator_stop(s_led_4g_handle, BLINK_CONNECTED);
        led_indicator_start(s_led_4g_handle, BLINK_CONNECTING);
    }
}

static void on_modem_event(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == MODEM_BOARD_EVENT) {
        if (event_id == MODEM_EVENT_SIMCARD_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: SIM Card disconnected");
            led_indicator_start(s_led_system_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_SIMCARD_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: SIM Card Connected");
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_DTE_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: USB disconnected");
            led_indicator_start(s_led_system_handle, BLINK_CONNECTING);
        } else if (event_id == MODEM_EVENT_DTE_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: USB connected");
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTED);
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTING);
        } else if (event_id == MODEM_EVENT_DTE_RESTART) {
            ESP_LOGW(TAG, "Modem Board Event: Hardware restart");
            led_indicator_start(s_led_system_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_DTE_RESTART_DONE) {
            ESP_LOGI(TAG, "Modem Board Event: Hardware restart done");
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_NET_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: Network connected");
            led_indicator_start(s_led_4g_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_NET_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: Network disconnected");
            led_indicator_stop(s_led_4g_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_WIFI_STA_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: Station connected");
            led_indicator_start(s_led_wifi_handle, BLINK_CONNECTED);
        } else if (event_id == MODEM_EVENT_WIFI_STA_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: All stations disconnected");
            led_indicator_stop(s_led_wifi_handle, BLINK_CONNECTED);
        }
    }
}

#ifdef CONFIG_MODEM_PING_NETWORK
static void on_ping_success(esp_ping_handle_t hdl, void *args)
{
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGI(TAG, "%"PRIu32" bytes from %s icmp_seq=%u ttl=%u time=%"PRIu32" ms\n", recv_len, ipaddr_ntoa(&target_addr), seqno, ttl, elapsed_time);
}

static void on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    ESP_LOGW(TAG, "From %s icmp_seq=%u timeout\n", ipaddr_ntoa(&target_addr), seqno);
    // Users can add logic to handle ping timeout
    // Add Wait or Reset logic
}
#endif

int user_4g_modem_init()
{
    /* Initialize led indicator */
    _led_indicator_init();

    /* Initialize default TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Waiting for modem powerup */
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "     ESP 4G Cat.1 Wi-Fi Router");
    ESP_LOGI(TAG, "====================================");

    /* Initialize modem board. Dial-up internet */
    modem_config_t modem_config = MODEM_DEFAULT_CONFIG();
    /* Modem init flag, used to control init process */
#ifndef CONFIG_MODEM_ENTER_PPP_DURING_INIT
    /* if Not enter ppp, modem will enter command mode after init */
    modem_config.flags |= MODEM_FLAGS_INIT_NOT_ENTER_PPP;
    /* if Not waiting for modem ready, just return after modem init */
    modem_config.flags |= MODEM_FLAGS_INIT_NOT_BLOCK;
#endif
    modem_config.handler = on_modem_event;
    modem_board_init(&modem_config);

#ifdef CONFIG_MODEM_ENABLE_WEB_ROUTER
    modem_http_get_nvs_wifi_config(&s_modem_wifi_config);
    modem_http_init(&s_modem_wifi_config);
#endif
    esp_netif_t *ap_netif = modem_wifi_ap_init();
    assert(ap_netif != NULL);
    ESP_ERROR_CHECK(modem_wifi_set(&s_modem_wifi_config));

#ifdef CONFIG_MODEM_PING_NETWORK
    ip_addr_t target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    char *ping_addr_s = NULL;
#ifdef CONFIG_MODEM_PING_MANUAL_ADDR
    // Ping users defined address
    ping_addr_s = CONFIG_MODEM_PING_MANUAL_ADDR;
#else
    // otherwise Ping DNS server
    esp_netif_dns_info_t dns2;
    modem_board_get_dns_info(ESP_NETIF_DNS_MAIN, &dns2);
    ping_addr_s = ip4addr_ntoa((ip4_addr_t *)(&dns2.ip.u_addr.ip4));
#endif
    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ipaddr_aton(ping_addr_s, &target_addr);
    ping_config.target_addr = target_addr;
    ping_config.timeout_ms = CONFIG_MODEM_PING_TIMEOUT;
    ping_config.task_stack_size = 4096;
    ping_config.count = 1;

    /* set callback functions */
    esp_ping_callbacks_t cbs = {
        .on_ping_success = on_ping_success,
        .on_ping_timeout = on_ping_timeout,
        .on_ping_end = NULL,
        .cb_args = NULL,
    };
    esp_ping_handle_t ping;
    esp_ping_new_session(&ping_config, &cbs, &ping);
#endif

    uint32_t ap_dns_addr = 0;
    while (1) {

#if !defined(CONFIG_MODEM_ENTER_PPP_DURING_INIT) || defined(CONFIG_MODEM_SUPPORT_SECONDARY_AT_PORT)
        // if you want to send AT command during ppp network working, make sure the modem support secondary AT port,
        // otherwise, the modem interface must switch to command mode before send command
        int rssi = 0, ber = 0;
        modem_board_get_signal_quality(&rssi, &ber);
        ESP_LOGI(TAG, "rssi=%d, ber=%d", rssi, ber);
#endif

        // If manual DNS not defined, set DNS when got address, user better to add a queue to handle this
#ifdef CONFIG_MODEM_AUTO_UPDATE_DNS
        esp_netif_dns_info_t dns;
        modem_board_get_dns_info(ESP_NETIF_DNS_MAIN, &dns);
        uint32_t _ap_dns_addr = dns.ip.u_addr.ip4.addr;
        if (_ap_dns_addr != ap_dns_addr) {
            modem_wifi_set_dns(ap_netif, _ap_dns_addr);
            ap_dns_addr = _ap_dns_addr;
            ESP_LOGI(TAG, "changed: ap dns addr (auto): %s", inet_ntoa(ap_dns_addr));
        }
#endif

#ifdef CONFIG_MODEM_PING_NETWORK
        ESP_LOGI(TAG, "Ping addr %s Restart..", ping_addr_s);
        esp_ping_start(ping);
#endif

#ifdef CONFIG_DUMP_SYSTEM_STATUS
        _system_dump();
#endif
        vTaskDelay(pdMS_TO_TICKS(10000));
    }	

	return 0;
}