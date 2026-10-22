/**
 * @file web_server.h
 * @brief WiFi SoftAP and HTTP Server for Ground Station Web Dashboard
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cấu trúc dữ liệu Telemetry tổng hợp hiển thị lên Web Dashboard */
typedef struct {
    /* Link Stats */
    bool     link_ok;
    uint32_t rx_packet_count;
    uint32_t packet_loss_count;
    float    packet_rate_hz;
    float    link_quality_pct;
    uint32_t last_packet_time_ms;

    /* FC State */
    uint32_t uptime_ms;
    uint16_t flags;
    bool     armed;
    bool     failsafe;
    uint8_t  failsafe_mode;

    /* Attitude */
    float roll_deg;
    float pitch_deg;
    float heading_deg;
    float mag_heading_deg;
    float gyro_dps[3];
    float pid_out[3];

    /* Motors (1000 - 2000 us) */
    uint16_t motor_us[4];

    /* RC Inputs */
    float roll_stick;
    float pitch_stick;
    float yaw_stick;
    float throttle_norm;
    uint16_t servo_us;
    uint16_t rc_raw[4]; /* Roll, Pitch, Yaw, Throttle in us */
    uint16_t rc_ch5;    /* Mode AUX switch in us */

    /* Altitude / Barometer (BMP388) */
    float    altitude_m;
    float    altitude_raw_m;
    float    climb_rate_ms;
    float    pressure_hpa;
    float    baro_temp_c;
    bool     baro_ready;

    /* GPS (BZGNSS BZ-251 / USART3) */
    int32_t  gps_lat;              /* deg * 1e7 */
    int32_t  gps_lon;              /* deg * 1e7 */
    float    gps_alt_m;            /* meters */
    float    gps_speed_kmh;        /* km/h */
    float    gps_course_deg;       /* degrees */
    float    gps_hdop;             /* HDOP */
    uint8_t  gps_satellites;       /* count */
    uint8_t  gps_fix_type;         /* 0=None, 2=2D, 3=3D, 4=DGPS */
    uint8_t  gps_flags;            /* TEL_GPS_FLAG_* */
    float    gps_home_dist_m;      /* meters */
    float    gps_home_bearing_deg; /* degrees */
    uint32_t gps_last_update_ms;

    /* Stats */
    uint16_t loop_dt_avg_us;
} GroundStationState;

/* Khởi tạo WiFi SoftAP (SSID: UAV-GroundStation / Pass: uavtelemetry) */
esp_err_t wifi_init_softap(void);

/* Khởi động HTTP Server phục vụ Web Dashboard & API */
esp_err_t web_server_start(void);

/* Cập nhật trạng thái telemetry an toàn (Thread-safe) */
void ground_station_update_state(const GroundStationState *new_state);

/* Lấy bản sao trạng thái telemetry an toàn */
void ground_station_get_state(GroundStationState *out_state);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
