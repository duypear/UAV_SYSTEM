/**
  ******************************************************************************
  * @file    telemetry_protocol.h
  * @brief   Dinh nghia goi tin telemetry FC (STM32) -> Ground Station (ESP32-C3).
  *
  *          File nay KHONG phu thuoc HAL/CMSIS, dung chung cho ca 2 ben:
  *            - STM32F103 (Core/Inc)
  *            - ESP32-C3  (uav_ground_station/main)
  *
  *          Quy uoc:
  *            - Moi goi = 32 byte co dinh (payload nRF24L01 static 32 byte).
  *            - Little-endian.
  *            - Byte 0  : loai goi (0xA1..0xA6, dong thoi la byte dong bo).
  *            - Byte 1  : so thu tu goi (seq, tang 1 cho moi goi, tran 8 bit)
  *                        -> ben nhan dem mat goi bang (seq - last_seq - 1).
  *            - Byte 31 : CRC-8 (poly 0x07, init 0x00) cua byte 0..30.
  *            - nRF24 da co CRC 16-bit phan cung, CRC-8 nay chi la lop bao ve
  *              them (nhieu kenh/dia chi trung).
  *
  *          Layout chi tiet (offset: kich thuoc, don vi):
  *
  *   0xA1 ATT    (50 Hz)
  *     2:u16 ts_ms   4:i16 roll(0.01deg)  6:i16 pitch(0.01deg)  8:u16 heading(0.01deg)
  *    10:i16 gyroX  12:i16 gyroY  14:i16 gyroZ   (0.1 deg/s)
  *    16:i16 pidRoll 18:i16 pidPitch 20:i16 pidYaw (0.1)
  *    22:u16 motor1_us 24:u16 motor2_us 26:u16 motor3_us 28:u16 motor4_us
  *    30:u8  flags_lo
  *
  *   0xA2 RC     (25 Hz)
  *     2:u16 ts_ms   4:u16 raw[0..3] (us)  12:u16 ch5_us  14:u16 servo_us
  *    16:i16 roll_cmd 18:i16 pitch_cmd 20:i16 yaw_cmd 22:i16 throttle (x1000)
  *    24:i16 desired_yaw_rate(0.1 dps) 26:i16 failsafe_throttle(x1000)
  *    28:i16 tilt_boost(x1000)  30:u8 flags_hi
  *
  *   0xA3 STATUS (10 Hz)
  *     2:u32 uptime_ms  6:u16 flags  8:u8 failsafe_mode  9:u8 imu_fail_streak
  *    10:u8 mag_fail_streak  11:u8 link_reinit_count
  *    12:u16 accel_norm(x1000) 14:u16 accel_trust(x1000)
  *    16:u16 loop_dt_avg_us 18:u16 loop_dt_max_us
  *    20:u16 tel_sent 22:u16 tel_fail  24:u8 tel_cpu_max_us 25:u8 tel_late
  *    26:i16 throttle_rate(x100) 28:i16 throttle_delta(x1000) 30:u8 proto_ver
  *
  *   0xA4 TUNE / NAV (5 Hz)
  *     2:u16 ts_ms  4:i16 magX 6:i16 magY 8:i16 magZ (x1000 gauss)
  *    10:u16 mag_field(x1000) 12:u16 mag_heading(0.01deg)
  *    14:i16 alt_cm(0.01m)  16:i16 vz_cms(0.01m/s)  18:u16 press(0.1hPa)  20:i16 temp(0.1C)
  *    22:i16 raw_alt_cm(0.01m)  24:u8 baro_flags  25:u8 takeoff_assist(x100)
  *    26:u8 yaw_maneuver(x100) 27:u8 rp_scale(x100)  28:i16 heading_error(0.1deg)
  *    30:u8 flags_tune
  *
  *   0xA5 CFG    (5 Hz)
  *     2:u16 ts_ms  4:u16 rc_center_roll 6:u16 rc_center_pitch 8:u16 rc_center_yaw
  *    10..27: u16 x9 = Kp,Ki,Kd (x1000) cua Roll, Pitch, Yaw
  *    28:u16 heading_ref(0.01deg)
  *
  *   0xA6 GPS    (5 Hz)
  *     2:u16 ts_ms
  *     4:i32 lat(deg * 1e7)  8:i32 lon(deg * 1e7)
  *    12:i32 alt_gps_cm(cm)
  *    16:u16 speed_cms(cm/s) 18:u16 course_cdeg(0.01deg)
  *    20:u16 hdop_x100(x100) 22:u8  satellites  23:u8 fix_type
  *    24:u8  gps_flags       25:u16 home_dist_m 27:u16 home_bearing_cdeg
  *    29:u8  reserved0       30:u8  reserved1
  ******************************************************************************
  */
#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TEL_PROTOCOL_VERSION   1U
#define TEL_PACKET_SIZE        32U

/* Radio: kenh 80 = 2480 MHz, nam tren kenh WiFi 11 -> it bi ESP32 WiFi
 * (AP dat kenh 1..11) gay nhieu. Doi 2 ben cung gia tri neu muon doi. */
#define TEL_RF_CHANNEL         80U
#define TEL_ADDR_WIDTH         5U
#define TEL_ADDRESS_INIT       { 0x6BU, 0xE3U, 0x1CU, 0x95U, 0xA2U }

#define TEL_FRAME_ATT          0xA1U
#define TEL_FRAME_RC           0xA2U
#define TEL_FRAME_STATUS       0xA3U
#define TEL_FRAME_TUNE         0xA4U
#define TEL_FRAME_CFG          0xA5U
#define TEL_FRAME_GPS          0xA6U

/* Bit trong truong flags (u16). ATT mang 8 bit thap, RC mang 8 bit cao,
 * STATUS mang du 16 bit. */
#define TEL_FLAG_ARMED              (1U << 0)
#define TEL_FLAG_FAILSAFE           (1U << 1)
#define TEL_FLAG_IMU_READY          (1U << 2)
#define TEL_FLAG_MAG_READY          (1U << 3)
#define TEL_FLAG_MAG_USABLE         (1U << 4)
#define TEL_FLAG_MAG_CORR_ALLOWED   (1U << 5)
#define TEL_FLAG_MAG_CALIBRATED     (1U << 6)
#define TEL_FLAG_HEADING_HOLD       (1U << 7)
#define TEL_FLAG_HEADING_HOLD_OK    (1U << 8)
#define TEL_FLAG_MIXER_SATURATED    (1U << 9)
#define TEL_FLAG_SERVO_RELEASED     (1U << 10)
#define TEL_FLAG_RF_LOSS            (1U << 11)
#define TEL_FLAG_THROTTLE_LOW       (1U << 12)
#define TEL_FLAG_RP_DYNAMIC         (1U << 13)
#define TEL_FLAG_MAG_HDG_VALID      (1U << 14)
#define TEL_FLAG_RC_VALID           (1U << 15)

/* Bit trong baro_flags */
#define TEL_BARO_FLAG_READY         (1U << 0)
#define TEL_BARO_FLAG_REF_OK        (1U << 1)

/* Bit trong gps_flags */
#define TEL_GPS_FLAG_PRESENT        (1U << 0)
#define TEL_GPS_FLAG_FIX_3D         (1U << 1)
#define TEL_GPS_FLAG_HOME_SET       (1U << 2)
#define TEL_GPS_FLAG_TIMEOUT        (1U << 3)

/* CRC-8, poly 0x07, init 0x00 (dung chung 2 ben). */
static inline uint8_t tel_crc8(const uint8_t *data, uint32_t len)
{
  uint8_t crc = 0U;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0U; i < 8U; i++) {
      crc = (uint8_t)((crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1));
    }
  }
  return crc;
}

/* Kiem tra 1 goi nhan duoc (dung o ground station). */
static inline int tel_packet_valid(const uint8_t *p)
{
  return (p[0] >= TEL_FRAME_ATT) && (p[0] <= TEL_FRAME_GPS) &&
         (tel_crc8(p, TEL_PACKET_SIZE - 1U) == p[TEL_PACKET_SIZE - 1U]);
}

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PROTOCOL_H */
