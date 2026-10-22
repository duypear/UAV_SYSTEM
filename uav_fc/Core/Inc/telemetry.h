/**
  ******************************************************************************
  * @file    telemetry.h
  * @brief   Telemetry nRF24L01 (SPI1 remap) cho Blue Pill FC.
  *
  *          Thiet ke de KHONG anh huong vong bay:
  *            - Khong dung ngat, khong HAL_Delay, khong cho tren duong bay.
  *            - telemetry_poll() chi duoc goi trong thoi gian ranh cua vong
  *              250 Hz, va chi lam viec nang khi con du "slack" truoc nhip
  *              dieu khien ke tiep.
  *            - Moi buoc nang (dong goi / ghi SPI) < ~100 us.
  *            - nRF24 loi/khong gan -> tu tat telemetry, tu thu lai, bay
  *              khong bi anh huong.
  *
  *          Chan:
  *            CE=PB5  CSN=PB12  SCK=PB13  MISO=PB14  MOSI=PB15  IRQ=khong dung
  *            (SPI2 phan cung goc; CE dung GPIO PB5 thuan)
  ******************************************************************************
  */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "telemetry_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Cau hinh co the chinh -------------------------------------------- */
#define TEL_SLOT_MS               10U   /* 1 goi / 10 ms = 100 goi/s        */
#define TEL_MIN_SLACK_US          250U  /* chi lam viec nang neu con >= us  */
#define TEL_MIN_SLACK_STARVED_US  130U  /* noi long neu bi doi > 150 ms     */
#define TEL_STARVE_MS             150U
#define TEL_TX_TIMEOUT_MS         3U
#define TEL_FAIL_LIMIT            8U    /* so loi lien tiep truoc khi reinit */
#define TEL_HEALTH_PERIOD_MS      1000U
#define TEL_RETRY_PERIOD_MS       500U
#define TEL_USE_IRQ_PIN           0     /* 0: doc STATUS qua SPI thay IRQ (SPI2 khong dung chan IRQ) */

/* RF_SETUP: 1 Mbps, -18 dBm (0x00) cho module PA/LNA test ban khong sut nguon/bao hoa.
 * 0 dBm = 0x06, 2 Mbps = 0x0E, 250 kbps (chi nRF24L01+) = 0x20 */
#define TEL_RF_SETUP_VALUE        0x00U

/* ---- Du lieu FC cung cap cho telemetry --------------------------------- */
typedef struct {
  uint32_t uptime_ms;
  uint16_t flags;                 /* TEL_FLAG_* */
  uint8_t  failsafe_mode;
  uint8_t  imu_fail_streak;
  uint8_t  mag_fail_streak;

  float roll_deg, pitch_deg;
  float heading_deg, heading_ref_deg, heading_error_deg, mag_heading_deg;
  float gyro_dps[3];
  float pid_out[3];               /* roll, pitch, yaw */
  uint16_t motor_us[4];

  uint16_t rc_raw[4];
  uint16_t ch5_us, servo_us;
  float roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd;
  float desired_yaw_rate, failsafe_throttle, tilt_boost;

  float accel_norm, accel_trust;
  float mag_field, mag_raw[3];
  float yaw_dynamic, yaw_throttle_ff, yaw_ff, yaw_maneuver_bias, yaw_learn_bias;
  float mixer_yaw_scale, mixer_rp_scale, yaw_maneuver_scale, takeoff_assist;
  float throttle_delta, throttle_rate;

  /* Barometer / Altitude Telemetry */
  float altitude_m;
  float altitude_raw_m;
  float climb_rate_ms;
  float pressure_hpa;
  float baro_temp_c;
  bool  baro_ready;

  /* GPS Telemetry (BZGNSS BZ-251 / USART3) */
  int32_t  gps_lat;
  int32_t  gps_lon;
  int32_t  gps_alt_cm;
  uint16_t gps_speed_cms;
  uint16_t gps_course_cdeg;
  uint16_t gps_hdop;
  uint8_t  gps_satellites;
  uint8_t  gps_fix_type;
  uint8_t  gps_flags;
  uint16_t gps_home_dist_m;
  uint16_t gps_home_bearing_cdeg;

  uint16_t rc_center[3];          /* roll, pitch, yaw (us) */
  float pid_gain[3][3];           /* [axis][kp,ki,kd] */
} TelemetryData;

/* Callback do main.c cung cap: copy trang thai bay hien tai vao `out`.
 * Duoc goi trong ngu canh main (cung ngu canh voi vong bay) nen khong can khoa. */
typedef void (*TelemetryFillFn)(TelemetryData *out, const void *ctx);

/* Goi 1 lan luc khoi dong (sau >=100 ms tu luc cap nguon nRF).
 * Tra ve false neu khong thay nRF24 (telemetry se tu thu lai ve sau). */
bool telemetry_init(TelemetryFillFn fill);

/* Goi moi chu ky dieu khien voi so cycle DWT giua 2 chu ky (de do suc khoe vong lap). */
void telemetry_note_loop_cycles(uint32_t cycles);

/* Goi trong thoi gian ranh cua vong lap chinh.
 * slack_cycles = (int32_t)(next_loop_cycles - DWT->CYCCNT). */
void telemetry_poll(uint32_t now_ms, int32_t slack_cycles, const void *ctx);

/* Ghi vao buf 1 dong chan doan (gia tri thanh ghi that doc tu nRF24). */
void telemetry_diag(char *buf, uint32_t len);

/* true neu radio dang hoat dong (da cau hinh, chua loi). */
bool telemetry_link_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
