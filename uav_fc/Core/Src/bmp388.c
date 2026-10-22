/**
  ******************************************************************************
  * @file    bmp388.c
  * @brief   Driver cam bien do do cao / ap suat BMP388 / BMP390 qua I2C.
  ******************************************************************************
  */

#include "main.h"
#include "bmp388.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Thanh ghi BMP388 */
#define BMP388_REG_CHIP_ID       0x00U
#define BMP388_REG_ERR_REG       0x02U
#define BMP388_REG_STATUS        0x03U
#define BMP388_REG_DATA_0        0x04U  /* 6 bytes: 3 bytes press + 3 bytes temp */
#define BMP388_REG_PWR_CTRL      0x1BU
#define BMP388_REG_OSR           0x1CU
#define BMP388_REG_ODR           0x1DU
#define BMP388_REG_CONFIG        0x1FU
#define BMP388_REG_CALIB_DATA    0x31U  /* 21 bytes */
#define BMP388_REG_CMD           0x7EU

/* Cau truc luu cac he so calib da quy doi float */
typedef struct {
  float par_t1;
  float par_t2;
  float par_t3;
  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;
  float t_lin;
} Bmp388CalibParams;

static Bmp388CalibParams calib_params;
static uint8_t bmp388_detected_addr = 0U; /* 8-bit address (0xEC hoac 0xEE) */
static bool bmp388_ready = false;
static float ground_pressure_p0_pa = 101325.0f;

/* Cau truc bo loc Kalman 2 trang thai: [do cao z (m), van toc leo v (m/s)] */
typedef struct {
  float z;       /* Do cao uoc luong (m) */
  float v;       /* Van toc leo / ha (m/s) */
  float p00;     /* Ma tran hiep phuong sai P */
  float p01;
  float p10;
  float p11;
  uint32_t last_update_ms;
  bool initialized;
} AltitudeKalman;

static AltitudeKalman alt_kf = {
  .z = 0.0f,
  .v = 0.0f,
  .p00 = 1.0f,
  .p01 = 0.0f,
  .p10 = 0.0f,
  .p11 = 1.0f,
  .last_update_ms = 0U,
  .initialized = false
};

/* Tham so nhieu:
 * - ALT_KF_ACCEL_VARIANCE: Phuong sai gia toc tien trinh (m/s^2)^2 (0.5 - 1.5).
 * - ALT_KF_BARO_VARIANCE: Phuong sai nhieu do cua BMP388 (m^2) (~0.06m^2).
 */
#define ALT_KF_ACCEL_VARIANCE    0.80f
#define ALT_KF_BARO_VARIANCE     0.06f

static void altitude_kalman_reset(float initial_z)
{
  alt_kf.z = initial_z;
  alt_kf.v = 0.0f;
  alt_kf.p00 = 0.5f;
  alt_kf.p01 = 0.0f;
  alt_kf.p10 = 0.0f;
  alt_kf.p11 = 1.0f;
  alt_kf.last_update_ms = HAL_GetTick();
  alt_kf.initialized = true;
}

static void altitude_kalman_update(float raw_alt, uint32_t now_ms, float *out_z, float *out_v)
{
  if (!alt_kf.initialized) {
    altitude_kalman_reset(raw_alt);
    if (out_z != NULL) *out_z = alt_kf.z;
    if (out_v != NULL) *out_v = 0.0f;
    return;
  }

  float dt = (float)(now_ms - alt_kf.last_update_ms) * 0.001f;
  alt_kf.last_update_ms = now_ms;

  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.04f; /* Mac dinh ~25Hz (40ms) neu khoang tre bi gian doan */
  }

  /* 1. Buoc du doan (Prediction) */
  const float z_pred = alt_kf.z + alt_kf.v * dt;
  const float v_pred = alt_kf.v;

  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;
  const float dt4 = dt2 * dt2;
  const float q00 = 0.25f * dt4 * ALT_KF_ACCEL_VARIANCE;
  const float q01 = 0.50f * dt3 * ALT_KF_ACCEL_VARIANCE;
  const float q11 = dt2 * ALT_KF_ACCEL_VARIANCE;

  const float p00_pred = alt_kf.p00 + dt * (alt_kf.p01 + alt_kf.p10) + dt2 * alt_kf.p11 + q00;
  const float p01_pred = alt_kf.p01 + dt * alt_kf.p11 + q01;
  const float p10_pred = p01_pred;
  const float p11_pred = alt_kf.p11 + q11;

  /* 2. Buoc hieu chinh (Measurement Update) */
  const float y = raw_alt - z_pred;
  const float s = p00_pred + ALT_KF_BARO_VARIANCE;
  const float inv_s = 1.0f / s;

  const float k0 = p00_pred * inv_s;
  const float k1 = p10_pred * inv_s;

  alt_kf.z = z_pred + k0 * y;
  alt_kf.v = v_pred + k1 * y;

  alt_kf.p00 = p00_pred - k0 * p00_pred;
  alt_kf.p01 = p01_pred - k0 * p01_pred;
  alt_kf.p10 = alt_kf.p01;
  alt_kf.p11 = p11_pred - k1 * p01_pred;

  if (out_z != NULL) *out_z = alt_kf.z;
  if (out_v != NULL) *out_v = alt_kf.v;
}

static bool bmp388_write_reg(uint8_t reg, uint8_t val)
{
  for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
    if (hw_i2c_write_register(bmp388_detected_addr, reg, val)) {
      return true;
    }
    HAL_Delay(2U);
  }
  return false;
}

static uint8_t bmp388_read_reg(uint8_t reg)
{
  uint8_t val = 0U;
  (void)hw_i2c_read_registers(bmp388_detected_addr, reg, &val, 1U);
  return val;
}

void i2c_scan(void)
{
  char scan_buf[96];
  uint8_t count = 0U;

  uart_log("\r\n[I2C] Scanning I2C1 bus (PB6=SCL, PB7=SDA)...\r\n");

  for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
    if (hw_i2c_ping(addr)) {
      const char *name = "";
      if (addr == 0x68U || addr == 0x69U) name = " -> MPU6050 / IMU";
      else if (addr == 0x1EU) name = " -> HMC5883L Mag";
      else if (addr == 0x0DU) name = " -> QMC5883L Mag";
      else if (addr == 0x76U) name = " -> BMP388 (SDO=GND)";
      else if (addr == 0x77U) name = " -> BMP388 (SDO=3.3V)";

      (void)snprintf(scan_buf, sizeof(scan_buf), "  [I2C] Found 0x%02X (8b: 0x%02X)%s\r\n",
                     addr, (unsigned int)(addr << 1), name);
      uart_log(scan_buf);
      count++;
    }
  }

  (void)snprintf(scan_buf, sizeof(scan_buf), "[I2C] Scan done: %u device(s) detected.\r\n\r\n", count);
  uart_log(scan_buf);
}

bool bmp388_init(void)
{
  uint8_t chip_id = 0U;
  char buf[128];

  bmp388_ready = false;
  bmp388_detected_addr = 0U;

  /* 1. Kiem tra thu dia chi 0x76 truoc (SDO noi GND) */
  if (hw_i2c_read_registers(BMP388_I2C_ADDR_PRIM << 1, BMP388_REG_CHIP_ID, &chip_id, 1U) &&
      (chip_id == BMP388_CHIP_ID || chip_id == BMP390_CHIP_ID)) {
    bmp388_detected_addr = (uint8_t)(BMP388_I2C_ADDR_PRIM << 1);
  }
  /* 2. Neu chua tim thay, thu tiep dia chi 0x77 (SDO noi VDDIO/3.3V) */
  else if (hw_i2c_read_registers(BMP388_I2C_ADDR_SEC << 1, BMP388_REG_CHIP_ID, &chip_id, 1U) &&
           (chip_id == BMP388_CHIP_ID || chip_id == BMP390_CHIP_ID)) {
    bmp388_detected_addr = (uint8_t)(BMP388_I2C_ADDR_SEC << 1);
  }

  if (bmp388_detected_addr == 0U) {
    uart_log("[BARO] BMP388 not detected at 0x76 or 0x77.\r\n");
    return false;
  }

  /* 3. Soft reset cam bien */
  (void)bmp388_write_reg(BMP388_REG_CMD, 0xB6U);
  HAL_Delay(25U);

  /* Cho chip hoan thanh reset va san sang (cmd_rdy bit 4 = 1) */
  uint32_t reset_timeout = HAL_GetTick() + 60U;
  while ((bmp388_read_reg(BMP388_REG_STATUS) & 0x10U) == 0U) {
    if (HAL_GetTick() > reset_timeout) {
      break;
    }
    HAL_Delay(5U);
  }

  /* 4. Doc 21 byte he so hieu chuan NVM tu thanh ghi 0x31 */
  uint8_t calib[21];
  if (!hw_i2c_read_registers(bmp388_detected_addr, BMP388_REG_CALIB_DATA, calib, 21U)) {
    uart_log("[BARO] Failed to read calibration data.\r\n");
    return false;
  }

  /* Giai ma he so nhiet do */
  uint16_t t1 = (uint16_t)(calib[0] | ((uint16_t)calib[1] << 8));
  uint16_t t2 = (uint16_t)(calib[2] | ((uint16_t)calib[3] << 8));
  int8_t   t3 = (int8_t)calib[4];

  /* Giai ma he so ap suat */
  int16_t  p1 = (int16_t)(calib[5] | ((uint16_t)calib[6] << 8));
  int16_t  p2 = (int16_t)(calib[7] | ((uint16_t)calib[8] << 8));
  int8_t   p3 = (int8_t)calib[9];
  int8_t   p4 = (int8_t)calib[10];
  uint16_t p5 = (uint16_t)(calib[11] | ((uint16_t)calib[12] << 8));
  uint16_t p6 = (uint16_t)(calib[13] | ((uint16_t)calib[14] << 8));
  int8_t   p7 = (int8_t)calib[15];
  int8_t   p8 = (int8_t)calib[16];
  int16_t  p9 = (int16_t)(calib[17] | ((uint16_t)calib[18] << 8));
  int8_t   p10 = (int8_t)calib[19];
  int8_t   p11 = (int8_t)calib[20];

  /* Quy doi he so theo cong thuc Bosch Sensortec API */
  calib_params.par_t1 = (float)t1 / 0.00390625f;
  calib_params.par_t2 = (float)t2 / 1073741824.0f;
  calib_params.par_t3 = (float)t3 / 281474976710656.0f;

  calib_params.par_p1 = ((float)p1 - 16384.0f) / 1048576.0f;
  calib_params.par_p2 = ((float)p2 - 16384.0f) / 536870912.0f;
  calib_params.par_p3 = (float)p3 / 4294967296.0f;
  calib_params.par_p4 = (float)p4 / 137438953472.0f;
  calib_params.par_p5 = (float)p5 / 0.125f;
  calib_params.par_p6 = (float)p6 / 64.0f;
  calib_params.par_p7 = (float)p7 / 256.0f;
  calib_params.par_p8 = (float)p8 / 32768.0f;
  calib_params.par_p9 = (float)p9 / 281474976710656.0f;
  calib_params.par_p10 = (float)p10 / 281474976710656.0f;
  calib_params.par_p11 = (float)p11 / 36893488147419103232.0f;
  calib_params.t_lin = 0.0f;

  /* 5. Trinh tu cau hinh theo Bosch BMP388 Datasheet (Muc 3.4.1):
   * a. Dua ve Sleep Mode (PWR_CTRL = 0x00) de cho phep ghi OSR/ODR khong bi conf_err.
   * b. OSR (0x1C): osr_p = x8 (0x03), osr_t = x1 (0x00) -> 0x03 (T_conv ~18.9ms)
   * c. ODR (0x1D): 25 Hz (0x03) (chu ky 40ms >> 18.9ms, tuyet doi khong vi pham timing)
   * d. CONFIG (0x1F): IIR filter coef 3 (0x02 << 1 = 0x04) loc nhieu gio va song ap
   * e. PWR_CTRL (0x1B): press_en=1 (bit 0), temp_en=1 (bit 1), normal mode (0x30) -> 0x33
   */
  (void)bmp388_write_reg(BMP388_REG_PWR_CTRL, 0x00U);
  HAL_Delay(10U);

  (void)bmp388_write_reg(BMP388_REG_OSR, 0x03U);
  HAL_Delay(2U);

  (void)bmp388_write_reg(BMP388_REG_ODR, 0x03U);
  HAL_Delay(2U);

  (void)bmp388_write_reg(BMP388_REG_CONFIG, 0x04U);
  HAL_Delay(2U);

  (void)bmp388_write_reg(BMP388_REG_PWR_CTRL, 0x33U);
  HAL_Delay(50U);

  /* Kiem tra lai cac thanh ghi trang thai */
  uint8_t pwr_ctrl = bmp388_read_reg(BMP388_REG_PWR_CTRL);
  uint8_t err_reg = bmp388_read_reg(BMP388_REG_ERR_REG);
  uint8_t status_reg = bmp388_read_reg(BMP388_REG_STATUS);

  if (err_reg != 0U) {
    (void)snprintf(buf, sizeof(buf), "[BARO] Warning: ERR_REG=0x%02X (conf_err=%u)\r\n",
                   err_reg, (unsigned int)((err_reg >> 2) & 0x01U));
    uart_log(buf);
  }

  /* 6. Thiet lap moc ap suat mat dat P0 qua trung binh cac mau hop le */
  Bmp388Data sample;
  float sum_p = 0.0f;
  uint8_t samples_read = 0U;

  for (uint8_t i = 0U; i < 25U; i++) {
    HAL_Delay(40U);
    if (bmp388_read(&sample)) {
      sum_p += sample.pressure_pa;
      samples_read++;
      if (samples_read >= 10U) {
        break;
      }
    }
  }

  if (samples_read > 0U) {
    ground_pressure_p0_pa = sum_p / (float)samples_read;
    bmp388_ready = true;
  } else {
    ground_pressure_p0_pa = 101325.0f;
    bmp388_ready = false;
  }

  (void)snprintf(buf, sizeof(buf),
                 "[BARO] BMP388 %s (addr=0x%02X, ID=0x%02X, P0=%.2f hPa, PWR=0x%02X ERR=0x%02X STS=0x%02X n=%u).\r\n",
                 bmp388_ready ? "ready" : "warn",
                 (unsigned int)(bmp388_detected_addr >> 1), chip_id,
                 ground_pressure_p0_pa / 100.0f,
                 pwr_ctrl, err_reg, status_reg, samples_read);
  uart_log(buf);

  return bmp388_ready;
}

bool bmp388_read(Bmp388Data *data)
{
  if (data == NULL || bmp388_detected_addr == 0U) {
    return false;
  }

  /* Giam sat bao ve: neu cam bien vo tinh bi reset hoac roi khoi Normal mode, tai kich hoat */
  static uint32_t last_mode_check_ms = 0U;
  const uint32_t now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - last_mode_check_ms) >= 1000U) {
    last_mode_check_ms = now_ms;
    const uint8_t cur_pwr = bmp388_read_reg(BMP388_REG_PWR_CTRL);
    if ((cur_pwr & 0x30U) != 0x30U) {
      (void)bmp388_write_reg(BMP388_REG_PWR_CTRL, 0x33U);
    }
  }

  uint8_t raw[6];
  if (!hw_i2c_read_registers(bmp388_detected_addr, BMP388_REG_DATA_0, raw, 6U)) {
    return false;
  }

  const uint32_t uncomp_press = (uint32_t)(raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16));
  const uint32_t uncomp_temp  = (uint32_t)(raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16));

  /* Loc gia tri uninitialized 0x800000 (reset default) hoac 0 */
  if (uncomp_press == 0x800000U || uncomp_press == 0U ||
      uncomp_temp == 0x800000U || uncomp_temp == 0U) {
    return false;
  }

  /* Tinh nhiet do bu */
  const float partial_data1 = (float)(uncomp_temp - calib_params.par_t1);
  const float partial_data2 = partial_data1 * calib_params.par_t2;
  calib_params.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_params.par_t3;
  data->temperature_c = calib_params.t_lin;

  /* Tinh ap suat bu */
  float p_part1, p_part2, p_part3, p_part4;
  float p_out1, p_out2;

  p_part1 = calib_params.par_p6 * calib_params.t_lin;
  p_part2 = calib_params.par_p7 * (calib_params.t_lin * calib_params.t_lin);
  p_part3 = calib_params.par_p8 * (calib_params.t_lin * calib_params.t_lin * calib_params.t_lin);
  p_out1  = calib_params.par_p5 + p_part1 + p_part2 + p_part3;

  p_part1 = calib_params.par_p2 * calib_params.t_lin;
  p_part2 = calib_params.par_p3 * (calib_params.t_lin * calib_params.t_lin);
  p_part3 = calib_params.par_p4 * (calib_params.t_lin * calib_params.t_lin * calib_params.t_lin);
  p_out2  = (float)uncomp_press * (calib_params.par_p1 + p_part1 + p_part2 + p_part3);

  p_part1 = (float)uncomp_press * (float)uncomp_press;
  p_part2 = calib_params.par_p9 + calib_params.par_p10 * calib_params.t_lin;
  p_part3 = p_part1 * p_part2;
  p_part4 = p_part3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_params.par_p11;

  const float comp_press = p_out1 + p_out2 + p_part4;
  data->pressure_pa = comp_press;
  data->pressure_hpa = comp_press / 100.0f;

  /* Tinh do cao tuong doi so voi mat dat luc khoi dong */
  float raw_alt = 0.0f;
  if (ground_pressure_p0_pa > 1000.0f) {
    raw_alt = 44330.0f * (1.0f - powf(comp_press / ground_pressure_p0_pa, 0.190295f));
  }
  data->altitude_raw_m = raw_alt;

  /* Bo loc Kalman 2 trang thai: uoc luong do cao min (m) va van toc leo/ha vz (m/s) */
  altitude_kalman_update(raw_alt, now_ms, &data->altitude_m, &data->climb_rate_ms);

  /* Do cao tuyet doi so voi muc nuoc bien (1013.25 hPa) */
  data->altitude_abs_m = 44330.0f * (1.0f - powf(comp_press / 101325.0f, 0.190295f));

  return true;
}

void bmp388_set_reference(void)
{
  Bmp388Data sample;
  float sum_p = 0.0f;
  uint8_t count = 0U;

  for (uint8_t i = 0U; i < 8U; i++) {
    HAL_Delay(40U);
    if (bmp388_read(&sample)) {
      sum_p += sample.pressure_pa;
      count++;
    }
  }

  if (count > 0U) {
    ground_pressure_p0_pa = sum_p / (float)count;
    altitude_kalman_reset(0.0f);
  }
}

void bmp388_zero_altitude(void)
{
  Bmp388Data sample;
  if (bmp388_read(&sample) && (sample.pressure_pa > 1000.0f)) {
    ground_pressure_p0_pa = sample.pressure_pa;
  }
  altitude_kalman_reset(0.0f);
}

bool bmp388_is_ready(void)
{
  return bmp388_ready;
}

uint8_t bmp388_get_addr(void)
{
  return (uint8_t)(bmp388_detected_addr >> 1);
}
