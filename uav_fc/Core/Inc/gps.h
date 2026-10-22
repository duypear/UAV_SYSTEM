/**
  ******************************************************************************
  * @file    gps.h
  * @brief   Driver va bo giai ma NMEA GPS (BZGNSS BZ-251 / USART3) cho UAV FC.
  *
  *          Ket noi phan cung STM32 Blue Pill:
  *            - PB10: USART3_TX (ket noi RX cua GPS)
  *            - PB11: USART3_RX (ket noi TX cua GPS)
  *            - VCC:  3.3V hoac 5V (theo module)
  *            - GND:  GND
  *
  *          Thiet ke phi chan (Non-blocking):
  *            - USART3 nhan du lieu qua ngat RXNE, luu vao ring buffer.
  *            - gps_poll() xu ly tung dong NMEA trong thoi gian ranh, khong block CPU,
  *              khong gay delay cho vong lap dieu khien bay 500 Hz.
  *            - Khi chua gan GPS: PB11 keo tro pull-up cao, khong co ngat,
  *              ton 0% CPU, he thong hoat dong binh thuong.
  ******************************************************************************
  */
#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t  lat;               /* Vi do (deg * 1e7, VD: 10.7626220 -> 107626220) */
  int32_t  lon;               /* Kinh do (deg * 1e7, VD: 106.6601720 -> 1066601720) */
  int32_t  alt_cm;            /* Do cao GPS so voi mat bien (cm) */
  uint16_t speed_cms;         /* Van toc mat dat (cm/s) */
  uint16_t course_cdeg;       /* Huong bay thuc (0.01 deg: 0..35999) */
  uint16_t hdop_x100;         /* Do suy giam do chinh xac (HDOP * 100) */
  uint8_t  satellites;        /* So ve tinh dang su dung */
  uint8_t  fix_type;          /* 0=Chua fix, 2=2D fix, 3=3D fix, 4=DGPS */
  uint8_t  flags;             /* TEL_GPS_FLAG_* */
  uint16_t home_dist_m;       /* Khoang cach toi diem Home (m) */
  uint16_t home_bearing_cdeg; /* Huong quay ve diem Home (0.01 deg) */
  uint32_t last_fix_ms;       /* Thoi diem fix hop le gan nhat (HAL_GetTick) */
  uint32_t last_packet_ms;    /* Thoi diem nhan byte gan nhat */
  bool     present;           /* Da tung phat hien co goi tin GPS gui ve */
} GpsData;

/* Khoi tao phan cung USART3 (PB10=TX, PB11=RX, 9600 baud, ngat RXNE) */
void gps_init(void);

/* Goi xu ly NMEA trong vong lap chinh (non-blocking) */
void gps_poll(uint32_t now_ms);

/* Lay con tro du lieu GPS hien tai de dong goi Telemetry */
const GpsData *gps_get_data(void);

/* Dat lai diem Home ve toa do hien tai */
void gps_set_home(void);

/* Trinh phuc vu ngat USART3 (goi tu stm32f1xx_it.c) */
void gps_uart_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
