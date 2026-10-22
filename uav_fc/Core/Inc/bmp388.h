/**
  ******************************************************************************
  * @file    bmp388.h
  * @brief   Driver cam bien do do cao / ap suat BMP388 / BMP390 qua I2C.
  *          Chuyen dung doc du lieu kiem tra phan cung (chua can thiep logic bay).
  ******************************************************************************
  */
#ifndef BMP388_H
#define BMP388_H

#include <stdbool.h>
#include <stdint.h>

#define BMP388_I2C_ADDR_PRIM   0x76U  /* SDO noi GND (8-bit: 0xEC) */
#define BMP388_I2C_ADDR_SEC    0x77U  /* SDO noi 3.3V (8-bit: 0xEE) */

#define BMP388_CHIP_ID         0x50U  /* Chip ID cua BMP388 */
#define BMP390_CHIP_ID         0x60U  /* Chip ID cua BMP390 (tuong thich) */

typedef struct {
  float temperature_c;    /* Nhiet do (do C) */
  float pressure_pa;      /* Ap suat tuyet doi (Pa) */
  float pressure_hpa;     /* Ap suat tuyet doi (hPa / mbar) */
  float altitude_m;       /* Do cao da qua bo loc Kalman (met) */
  float altitude_raw_m;   /* Do cao tho chua qua loc (met) */
  float altitude_abs_m;   /* Do cao tuyet doi so voi muc nuoc bien (met) */
  float climb_rate_ms;    /* Van toc leo / ha (m/s) uoc luong tu Kalman */
} Bmp388Data;

/* Quet toan bo bus I2C1 va in danh sach dia chi phan hoi ra UART */
void i2c_scan(void);

/* Khoi tao cam bien BMP388 (tu do tim dia chi 0x76 hoac 0x77) */
bool bmp388_init(void);

/* Doc du lieu ap suat, nhiet do va tinh do cao */
bool bmp388_read(Bmp388Data *data);

/* Thiet lap lai ap suat chuan mat dat P0 (moc 0m) */
void bmp388_set_reference(void);

/* Thiet lap tuc thi ap suat chuan mat dat P0 (moc 0.0m) phi-chan (non-blocking) luc ARM */
void bmp388_zero_altitude(void);

/* Kiem tra trang thai cam bien BMP388 */
bool bmp388_is_ready(void);

/* Lay dia chi I2C cua BMP388 tim thay (7-bit) */
uint8_t bmp388_get_addr(void);

#endif /* BMP388_H */
