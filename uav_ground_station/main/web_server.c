/**
 * @file web_server.c
 * @brief WiFi SoftAP and HTTP Server implementation for Ground Station
 */

#include "web_server.h"
#include "web_dashboard.h"
#include "telemetry_protocol.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "WEB_SERVER";

#define WIFI_AP_SSID      "UAV-GroundStation"
#define WIFI_AP_PASS      "uavtelemetry"
#define WIFI_AP_CHANNEL   6
#define MAX_STA_CONN      4

/* Thong tin WiFi Hotspot tu dien thoai cua ban de Laptop co Internet xem Map */
#define WIFI_STA_SSID     "riprip"
#define WIFI_STA_PASS     "4444333333"

static httpd_handle_t server = NULL;
static SemaphoreHandle_t state_mutex = NULL;
static GroundStationState gs_state;

static char s_sta_ip[32] = "0.0.0.0";
static bool s_sta_connected = false;

void ground_station_update_state(const GroundStationState *new_state)
{
    if (state_mutex != NULL && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        gs_state = *new_state;
        xSemaphoreGive(state_mutex);
    }
}

void ground_station_get_state(GroundStationState *out_state)
{
    if (state_mutex != NULL && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *out_state = gs_state;
        xSemaphoreGive(state_mutex);
    }
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    const size_t chunk_size = 4096;
    size_t total_len = strlen(DASHBOARD_HTML);
    size_t offset = 0;
    while (offset < total_len) {
        size_t send_len = (total_len - offset) > chunk_size ? chunk_size : (total_len - offset);
        esp_err_t err = httpd_resp_send_chunk(req, DASHBOARD_HTML + offset, send_len);
        if (err != ESP_OK) {
            return err;
        }
        offset += send_len;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t api_telemetry_get_handler(httpd_req_t *req)
{
    GroundStationState state;
    ground_station_get_state(&state);

    char json[1400];
    snprintf(json, sizeof(json),
        "{\"link_ok\":%s,\"pkts\":%lu,\"loss\":%lu,\"hz\":%.1f,\"link_pct\":%.0f,\"uptime_s\":%lu,"
        "\"armed\":%s,\"failsafe\":%s,\"fs_mode\":%u,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"hdg\":%.2f,\"mag_hdg\":%.2f,"
        "\"alt\":%.2f,\"raw_alt\":%.2f,\"vz\":%.2f,\"press\":%.1f,\"baro_temp\":%.1f,\"baro_ok\":%s,"
        "\"gps_lat\":%.7f,\"gps_lon\":%.7f,\"gps_alt\":%.1f,\"gps_spd\":%.1f,\"gps_crs\":%.1f,"
        "\"gps_hdop\":%.2f,\"gps_sats\":%u,\"gps_fix\":%u,\"gps_flags\":%u,\"gps_dist\":%.0f,\"gps_brg\":%.1f,\"gps_ok\":%s,"
        "\"m1\":%u,\"m2\":%u,\"m3\":%u,\"m4\":%u,"
        "\"pid_r\":%.1f,\"pid_p\":%.1f,\"pid_y\":%.1f,"
        "\"roll_stick\":%.2f,\"pitch_stick\":%.2f,\"yaw_stick\":%.2f,\"thr\":%.2f,\"servo\":%u,"
        "\"rc_roll\":%u,\"rc_pitch\":%u,\"rc_yaw\":%u,\"rc_thr\":%u,\"rc_ch5\":%u,"
        "\"loop_dt\":%u,\"sta_ok\":%s,\"sta_ip\":\"%s\"}",
        state.link_ok ? "true" : "false",
        (unsigned long)state.rx_packet_count,
        (unsigned long)state.packet_loss_count,
        state.packet_rate_hz,
        state.link_quality_pct,
        (unsigned long)(state.uptime_ms / 1000),
        state.armed ? "true" : "false",
        state.failsafe ? "true" : "false",
        state.failsafe_mode,
        state.roll_deg,
        state.pitch_deg,
        state.heading_deg,
        state.mag_heading_deg,
        state.altitude_m,
        state.altitude_raw_m,
        state.climb_rate_ms,
        state.pressure_hpa,
        state.baro_temp_c,
        state.baro_ready ? "true" : "false",
        (double)state.gps_lat / 1e7,
        (double)state.gps_lon / 1e7,
        state.gps_alt_m,
        state.gps_speed_kmh,
        state.gps_course_deg,
        state.gps_hdop,
        state.gps_satellites,
        state.gps_fix_type,
        state.gps_flags,
        state.gps_home_dist_m,
        state.gps_home_bearing_deg,
        (state.gps_flags & TEL_GPS_FLAG_PRESENT) ? "true" : "false",
        state.motor_us[0],
        state.motor_us[1],
        state.motor_us[2],
        state.motor_us[3],
        state.pid_out[0],
        state.pid_out[1],
        state.pid_out[2],
        state.roll_stick,
        state.pitch_stick,
        state.yaw_stick,
        state.throttle_norm,
        state.servo_us,
        state.rc_raw[0] ? state.rc_raw[0] : (uint16_t)(1500 + state.roll_stick * 450),
        state.rc_raw[1] ? state.rc_raw[1] : (uint16_t)(1500 + state.pitch_stick * 450),
        state.rc_raw[2] ? state.rc_raw[2] : (uint16_t)(1500 + state.yaw_stick * 450),
        state.rc_raw[3] ? state.rc_raw[3] : (uint16_t)(1000 + state.throttle_norm * 1000),
        state.rc_ch5 ? state.rc_ch5 : 1500,
        state.loop_dt_avg_us,
        s_sta_connected ? "true" : "false",
        s_sta_ip
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_api_telemetry = {
    .uri       = "/api/telemetry",
    .method    = HTTP_GET,
    .handler   = api_telemetry_get_handler,
    .user_ctx  = NULL
};

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting HTTP Server on port %d...", config.server_port);
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_api_telemetry);
        ESP_LOGI(TAG, "Web server started! Open http://192.168.4.1 in your browser.");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP Server: %s", esp_err_to_name(ret));
    return ret;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
            ESP_LOGI(TAG, "Thiet bi ket noi vao SoftAP ESP32: MAC="MACSTR" AID=%d",
                     MAC2STR(event->mac), event->aid);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
            ESP_LOGI(TAG, "Thiet bi ngat ket noi SoftAP: MAC="MACSTR" AID=%d",
                     MAC2STR(event->mac), event->aid);
        } else if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "STA bat dau -> Dang ket noi toi Hotspot '%s'...", WIFI_STA_SSID);
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            s_sta_connected = false;
            ESP_LOGW(TAG, "Chua ket noi duoc voi Hotspot '%s' (hoac ngoai pham vi). Se thu lai...", WIFI_STA_SSID);
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            esp_ip4addr_ntoa(&event->ip_info.ip, s_sta_ip, sizeof(s_sta_ip));
            s_sta_connected = true;
            ESP_LOGI(TAG, "==========================================================");
            ESP_LOGI(TAG, " [HOTSPOT] DA KET NOI VOI HOTSPOT '%s'!", WIFI_STA_SSID);
            ESP_LOGI(TAG, " [GCS URL] MO TRINH DUYET TREN LAPTOP: http://" IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, " Laptop vua co Mang 4G xem Ban Do Ve Tinh, vua nhan Telemetry!");
            ESP_LOGI(TAG, "==========================================================");
        }
    }
}

esp_err_t wifi_init_softap(void)
{
    state_mutex = xSemaphoreCreateMutex();
    memset(&gs_state, 0, sizeof(gs_state));
    gs_state.motor_us[0] = 1000;
    gs_state.motor_us[1] = 1000;
    gs_state.motor_us[2] = 1000;
    gs_state.motor_us[3] = 1000;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 1. Khoi tao AP noi bo de du phong */
    esp_netif_create_default_wifi_ap();

    /* 2. Khoi tao Station ket noi vao Hotspot dien thoai */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif) {
        esp_netif_set_hostname(sta_netif, "uav-gcs");
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(WIFI_AP_PASS) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    /* Kich hoat dong thoi ca 2 che do AP + STA */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP+STA khoi tao thanh cong:");
    ESP_LOGI(TAG, "  [1. AP ESP32]  SSID: %s | Pass: %s | IP: 192.168.4.1", WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "  [2. STA Hotspot] Dang ket noi toi: '%s' | Pass: '%s'", WIFI_STA_SSID, WIFI_STA_PASS);

    return ESP_OK;
}
