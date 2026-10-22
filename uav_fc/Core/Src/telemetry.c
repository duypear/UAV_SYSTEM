/**
  ******************************************************************************
  * @file    telemetry.c
  * @brief   nRF24L01 telemetry cho FC: driver SPI1 (remap) + lap lich khong chan.
  *
  *          - SPI1 duoc dieu khien truc tiep bang thanh ghi (khong can bat
  *            module HAL_SPI trong stm32f1xx_hal_conf.h).
  *          - Che do: PTX, tat auto-ack, payload tinh 32 byte, CRC 16-bit.
  *            (Telemetry mot chieu: goi cu khong gui lai, goi moi quan trong
  *             hon; ben nhan dem mat goi qua seq.)
  *          - Khong dung ngat. IRQ (PB14) chi duoc doc muc logic.
  ******************************************************************************
  */

#include "main.h"
#include "telemetry.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------------ */
/* Chan: Dung SPI2 phan cung goc (PB12..PB15) + PB5 lam CE                   */
/* ------------------------------------------------------------------------ */
#define NRF_CE_PORT      GPIOB
#define NRF_CE_PIN       GPIO_PIN_5     /* PB5 lam CE */
#define NRF_CSN_PORT     GPIOB
#define NRF_CSN_PIN      GPIO_PIN_12    /* PB12 lam CSN (SPI2_NSS) */
#define NRF_SCK_PIN      GPIO_PIN_13    /* PB13 lam SCK (SPI2_SCK) */
#define NRF_MISO_PIN     GPIO_PIN_14    /* PB14 lam MISO (SPI2_MISO) */
#define NRF_MOSI_PIN     GPIO_PIN_15    /* PB15 lam MOSI (SPI2_MOSI) */

#define CE_HIGH()   (NRF_CE_PORT->BSRR = (uint32_t)NRF_CE_PIN)
#define CE_LOW()    (NRF_CE_PORT->BSRR = ((uint32_t)NRF_CE_PIN << 16))
#define CSN_HIGH()  (NRF_CSN_PORT->BSRR = (uint32_t)NRF_CSN_PIN)
#define CSN_LOW()   (NRF_CSN_PORT->BSRR = ((uint32_t)NRF_CSN_PIN << 16))

/* ------------------------------------------------------------------------ */
/* nRF24L01 lenh / thanh ghi                                                 */
/* ------------------------------------------------------------------------ */
#define CMD_R_REGISTER    0x00U
#define CMD_W_REGISTER    0x20U
#define CMD_W_TX_PAYLOAD  0xA0U
#define CMD_FLUSH_TX      0xE1U
#define CMD_FLUSH_RX      0xE2U
#define CMD_NOP           0xFFU

#define REG_CONFIG        0x00U
#define REG_EN_AA         0x01U
#define REG_EN_RXADDR     0x02U
#define REG_SETUP_AW      0x03U
#define REG_SETUP_RETR    0x04U
#define REG_RF_CH         0x05U
#define REG_RF_SETUP      0x06U
#define REG_STATUS        0x07U
#define REG_RX_ADDR_P0    0x0AU
#define REG_TX_ADDR       0x10U
#define REG_RX_PW_P0      0x11U
#define REG_DYNPD         0x1CU
#define REG_FEATURE       0x1DU

#define STATUS_RX_DR      0x40U
#define STATUS_TX_DS      0x20U
#define STATUS_MAX_RT     0x10U
#define STATUS_CLEAR_ALL  0x70U

/* MASK_RX_DR=1, EN_CRC=1, CRCO=1 (16-bit). */
#define CONFIG_PWR_DOWN   0x4CU
/* + PWR_UP=1, PRIM_RX=0 (PTX). */
#define CONFIG_PTX_UP     0x4EU

#define SPI_GUARD         4000U
#define POWER_UP_WAIT_MS  5U

/* ------------------------------------------------------------------------ */
/* Trang thai                                                                */
/* ------------------------------------------------------------------------ */
typedef enum {
  TEL_ST_OFF = 0,     /* chua co/loi radio, cho thu lai            */
  TEL_ST_WAIT_PWR,    /* vua cau hinh, doi oscillator on dinh      */
  TEL_ST_IDLE,        /* san sang, cho den slot ke tiep            */
  TEL_ST_BUILT,       /* da dong goi, cho ghi vao TX FIFO          */
  TEL_ST_TX           /* da phat, cho TX_DS                        */
} TelState;

static const uint8_t  nrf_address[TEL_ADDR_WIDTH] = TEL_ADDRESS_INIT;

/* Lich phat 20 slot (200 ms): ATT 50 Hz, RC 25 Hz, STATUS 10 Hz, TUNE 5 Hz, GPS 5 Hz, CFG 5 Hz */
#define SCHED_LEN 20U
static const uint8_t schedule[SCHED_LEN] = {
  TEL_FRAME_ATT, TEL_FRAME_RC,     TEL_FRAME_ATT, TEL_FRAME_STATUS,
  TEL_FRAME_ATT, TEL_FRAME_RC,     TEL_FRAME_ATT, TEL_FRAME_TUNE,
  TEL_FRAME_ATT, TEL_FRAME_RC,     TEL_FRAME_ATT, TEL_FRAME_STATUS,
  TEL_FRAME_ATT, TEL_FRAME_RC,     TEL_FRAME_ATT, TEL_FRAME_GPS,
  TEL_FRAME_ATT, TEL_FRAME_RC,     TEL_FRAME_ATT, TEL_FRAME_CFG
};

static TelemetryFillFn fill_cb = NULL;
static TelemetryData   tdata;
static uint8_t         frame[TEL_PACKET_SIZE];

static TelState g_state = TEL_ST_OFF;
static bool     spi_fault = false;
static uint32_t cycles_per_us = 72U;

static uint8_t  sched_idx = 0U;
static uint8_t  seq = 0U;
static uint32_t next_slot_ms = 0U;
static uint32_t next_health_ms = 0U;
static uint32_t retry_ms = 0U;
static uint32_t state_ms = 0U;
static uint32_t tx_start_ms = 0U;
static uint8_t  consecutive_fail = 0U;

/* Chan doan: gia tri that doc duoc tu nRF24 luc khoi dong */
static uint8_t  diag_status = 0U, diag_aw = 0U, diag_ch = 0U, diag_addr0 = 0U, diag_cfg = 0U;
static bool     diag_cfg_valid = false;

static bool     starving = false;
static uint32_t starve_start_ms = 0U;

/* Thong ke */
static uint16_t stat_sent = 0U;
static uint16_t stat_fail = 0U;
static uint8_t  stat_reinit = 0U;
static uint8_t  stat_late = 0U;
static uint32_t stat_cpu_max_us = 0U;
static uint32_t loop_sum_us = 0U;
static uint32_t loop_cnt = 0U;
static uint32_t loop_max_us = 0U;

/* ------------------------------------------------------------------------ */
/* Tien ich thoi gian                                                        */
/* ------------------------------------------------------------------------ */
static inline uint32_t cyc(void)
{
  return DWT->CYCCNT;
}

static void delay_us(uint32_t us)
{
  const uint32_t start = cyc();
  const uint32_t span = us * cycles_per_us;
  while ((uint32_t)(cyc() - start) < span) {
  }
}

/* ------------------------------------------------------------------------ */
/* SPI2 muc thanh ghi                                                        */
/* ------------------------------------------------------------------------ */
static uint8_t spi_xfer(uint8_t tx)
{
  uint32_t guard = SPI_GUARD;

  while (((SPI2->SR & SPI_SR_TXE) == 0U) && (guard > 0U)) {
    guard--;
  }
  SPI2->DR = tx;

  guard = SPI_GUARD;
  while (((SPI2->SR & SPI_SR_RXNE) == 0U) && (guard > 0U)) {
    guard--;
  }
  if (guard == 0U) {
    spi_fault = true;
  }
  return (uint8_t)SPI2->DR;
}

static inline void nrf_begin(void)
{
  CSN_LOW();
  delay_us(2U);
}

static void nrf_end(void)
{
  uint32_t guard = SPI_GUARD;
  while (((SPI2->SR & SPI_SR_BSY) != 0U) && (guard > 0U)) {
    guard--;
  }
  CSN_HIGH();
  delay_us(3U);
}

static uint8_t nrf_cmd(uint8_t cmd)
{
  uint8_t status;
  nrf_begin();
  status = spi_xfer(cmd);
  nrf_end();
  return status;
}

static uint8_t nrf_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t status;
  nrf_begin();
  status = spi_xfer((uint8_t)(CMD_W_REGISTER | (reg & 0x1FU)));
  (void)spi_xfer(value);
  nrf_end();
  return status; /* STATUS truoc khi ghi */
}

static uint8_t nrf_read_reg(uint8_t reg)
{
  uint8_t value;
  nrf_begin();
  (void)spi_xfer((uint8_t)(CMD_R_REGISTER | (reg & 0x1FU)));
  value = spi_xfer(CMD_NOP);
  nrf_end();
  return value;
}

static void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
  nrf_begin();
  (void)spi_xfer((uint8_t)(CMD_W_REGISTER | (reg & 0x1FU)));
  for (uint8_t i = 0U; i < len; i++) {
    (void)spi_xfer(buf[i]);
  }
  nrf_end();
}

static void nrf_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
  nrf_begin();
  (void)spi_xfer((uint8_t)(CMD_R_REGISTER | (reg & 0x1FU)));
  for (uint8_t i = 0U; i < len; i++) {
    buf[i] = spi_xfer(CMD_NOP);
  }
  nrf_end();
}

/* Cau hinh toan bo thanh ghi + kiem tra doc lai. Khong co delay.
 * Cuoi cung bat PWR_UP; caller phai doi >= POWER_UP_WAIT_MS. */
static bool nrf_configure(void)
{
  uint8_t verify[TEL_ADDR_WIDTH];

  spi_fault = false;
  CE_LOW();
  CSN_HIGH();

  nrf_write_reg(REG_CONFIG, CONFIG_PWR_DOWN);
  nrf_write_reg(REG_EN_AA, 0x00U);          /* tat auto-ack            */
  nrf_write_reg(REG_EN_RXADDR, 0x01U);
  nrf_write_reg(REG_SETUP_AW, 0x03U);       /* dia chi 5 byte          */
  nrf_write_reg(REG_SETUP_RETR, 0x00U);     /* khong retry             */
  nrf_write_reg(REG_RF_CH, (uint8_t)TEL_RF_CHANNEL);
  nrf_write_reg(REG_RF_SETUP, (uint8_t)TEL_RF_SETUP_VALUE);
  nrf_write_buf(REG_TX_ADDR, nrf_address, TEL_ADDR_WIDTH);
  nrf_write_buf(REG_RX_ADDR_P0, nrf_address, TEL_ADDR_WIDTH);
  nrf_write_reg(REG_RX_PW_P0, (uint8_t)TEL_PACKET_SIZE);
  nrf_write_reg(REG_DYNPD, 0x00U);
  nrf_write_reg(REG_FEATURE, 0x00U);
  (void)nrf_cmd(CMD_FLUSH_TX);
  (void)nrf_cmd(CMD_FLUSH_RX);
  nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);

  /* Doc lai de chac chan module ton tai va SPI dung (luu lai de chan doan). */
  diag_status = nrf_cmd(CMD_NOP);
  diag_aw = nrf_read_reg(REG_SETUP_AW);
  diag_ch = nrf_read_reg(REG_RF_CH);
  nrf_read_buf(REG_TX_ADDR, verify, TEL_ADDR_WIDTH);
  diag_addr0 = verify[0];

  if (spi_fault) {
    return false;
  }
  if (diag_aw != 0x03U) {
    return false;
  }
  if (diag_ch != (uint8_t)TEL_RF_CHANNEL) {
    return false;
  }
  if (memcmp(verify, nrf_address, TEL_ADDR_WIDTH) != 0) {
    return false;
  }

  nrf_write_reg(REG_CONFIG, CONFIG_PTX_UP);
  return !spi_fault;
}

/* ------------------------------------------------------------------------ */
/* Dong goi                                                                  */
/* ------------------------------------------------------------------------ */
static inline void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static inline void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
  p[2] = (uint8_t)((v >> 16) & 0xFFU);
  p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* Float -> int (cat ngan, khong lam tron: toi thieu chi phi soft-float),
 * roi chan bien bang so nguyen. */
static inline void put_q16(uint8_t *p, float v, float scale)
{
  int32_t i = (int32_t)(v * scale);
  if (i > 32767) {
    i = 32767;
  } else if (i < -32768) {
    i = -32768;
  }
  put_u16(p, (uint16_t)(int16_t)i);
}

static inline void put_qu16(uint8_t *p, float v, float scale)
{
  int32_t i = (int32_t)(v * scale);
  if (i > 65535) {
    i = 65535;
  } else if (i < 0) {
    i = 0;
  }
  put_u16(p, (uint16_t)i);
}

static inline uint8_t q_u8(float v, float scale)
{
  int32_t i = (int32_t)(v * scale);
  if (i > 255) {
    i = 255;
  } else if (i < 0) {
    i = 0;
  }
  return (uint8_t)i;
}

static void build_att(uint8_t *f, const TelemetryData *d)
{
  f[0] = TEL_FRAME_ATT;
  put_u16(f + 2, (uint16_t)d->uptime_ms);
  put_q16(f + 4, d->roll_deg, 100.0f);
  put_q16(f + 6, d->pitch_deg, 100.0f);
  put_qu16(f + 8, d->heading_deg, 100.0f);
  for (uint8_t i = 0U; i < 3U; i++) {
    put_q16(f + 10 + (2 * i), d->gyro_dps[i], 10.0f);
    put_q16(f + 16 + (2 * i), d->pid_out[i], 10.0f);
  }
  for (uint8_t i = 0U; i < 4U; i++) {
    put_u16(f + 22 + (2 * i), d->motor_us[i]);
  }
  f[30] = (uint8_t)(d->flags & 0xFFU);
}

static void build_rc(uint8_t *f, const TelemetryData *d)
{
  f[0] = TEL_FRAME_RC;
  put_u16(f + 2, (uint16_t)d->uptime_ms);
  for (uint8_t i = 0U; i < 4U; i++) {
    put_u16(f + 4 + (2 * i), d->rc_raw[i]);
  }
  put_u16(f + 12, d->ch5_us);
  put_u16(f + 14, d->servo_us);
  put_q16(f + 16, d->roll_cmd, 1000.0f);
  put_q16(f + 18, d->pitch_cmd, 1000.0f);
  put_q16(f + 20, d->yaw_cmd, 1000.0f);
  put_q16(f + 22, d->throttle_cmd, 1000.0f);
  put_q16(f + 24, d->desired_yaw_rate, 10.0f);
  put_q16(f + 26, d->failsafe_throttle, 1000.0f);
  put_q16(f + 28, d->tilt_boost, 1000.0f);
  f[30] = (uint8_t)(d->flags >> 8);
}

static void build_status(uint8_t *f, const TelemetryData *d)
{
  const uint32_t avg_us = (loop_cnt > 0U) ? (loop_sum_us / loop_cnt) : 0U;
  const uint32_t cpu_us = (stat_cpu_max_us > 255U) ? 255U : stat_cpu_max_us;

  f[0] = TEL_FRAME_STATUS;
  put_u32(f + 2, d->uptime_ms);
  put_u16(f + 6, d->flags);
  f[8] = d->failsafe_mode;
  f[9] = d->imu_fail_streak;
  f[10] = d->mag_fail_streak;
  f[11] = stat_reinit;
  put_qu16(f + 12, d->accel_norm, 1000.0f);
  put_qu16(f + 14, d->accel_trust, 1000.0f);
  put_u16(f + 16, (uint16_t)((avg_us > 65535U) ? 65535U : avg_us));
  put_u16(f + 18, (uint16_t)((loop_max_us > 65535U) ? 65535U : loop_max_us));
  put_u16(f + 20, stat_sent);
  put_u16(f + 22, stat_fail);
  f[24] = (uint8_t)cpu_us;
  f[25] = stat_late;
  put_q16(f + 26, d->throttle_rate, 100.0f);
  put_q16(f + 28, d->throttle_delta, 1000.0f);
  f[30] = (uint8_t)TEL_PROTOCOL_VERSION;

  /* cua so thong ke bat dau lai sau moi STATUS */
  loop_sum_us = 0U;
  loop_cnt = 0U;
  loop_max_us = 0U;
  stat_cpu_max_us = 0U;
}

static void build_tune(uint8_t *f, const TelemetryData *d)
{
  f[0] = TEL_FRAME_TUNE;
  put_u16(f + 2, (uint16_t)d->uptime_ms);
  put_q16(f + 4, d->mag_raw[0], 1000.0f);
  put_q16(f + 6, d->mag_raw[1], 1000.0f);
  put_q16(f + 8, d->mag_raw[2], 1000.0f);
  put_qu16(f + 10, d->mag_field, 1000.0f);
  put_qu16(f + 12, d->mag_heading_deg, 100.0f);

  /* Do cao va khi ap (Baro telemetry) */
  put_q16(f + 14, d->altitude_m, 100.0f);      /* cm (0.01m) */
  put_q16(f + 16, d->climb_rate_ms, 100.0f);   /* cm/s (0.01m/s) */
  put_qu16(f + 18, d->pressure_hpa, 10.0f);    /* 0.1 hPa */
  put_q16(f + 20, d->baro_temp_c, 10.0f);      /* 0.1 deg C */
  put_q16(f + 22, d->altitude_raw_m, 100.0f);  /* cm (0.01m) */

  f[24] = d->baro_ready ? (uint8_t)TEL_BARO_FLAG_READY : 0U;
  f[25] = q_u8(d->takeoff_assist, 100.0f);
  f[26] = q_u8(d->yaw_maneuver_scale, 100.0f);
  f[27] = q_u8(d->mixer_rp_scale, 100.0f);
  put_q16(f + 28, d->heading_error_deg, 10.0f);
  f[30] = 0U;
}

static void build_cfg(uint8_t *f, const TelemetryData *d)
{
  f[0] = TEL_FRAME_CFG;
  put_u16(f + 2, (uint16_t)d->uptime_ms);
  put_u16(f + 4, d->rc_center[0]);
  put_u16(f + 6, d->rc_center[1]);
  put_u16(f + 8, d->rc_center[2]);
  for (uint8_t axis = 0U; axis < 3U; axis++) {
    for (uint8_t k = 0U; k < 3U; k++) {
      put_qu16(f + 10 + (2 * ((axis * 3U) + k)), d->pid_gain[axis][k], 1000.0f);
    }
  }
  put_qu16(f + 28, d->heading_ref_deg, 100.0f);
}

static void build_gps(uint8_t *f, const TelemetryData *d)
{
  f[0] = TEL_FRAME_GPS;
  put_u16(f + 2, (uint16_t)d->uptime_ms);
  put_u32(f + 4, (uint32_t)d->gps_lat);
  put_u32(f + 8, (uint32_t)d->gps_lon);
  put_u32(f + 12, (uint32_t)d->gps_alt_cm);
  put_u16(f + 16, d->gps_speed_cms);
  put_u16(f + 18, d->gps_course_cdeg);
  put_u16(f + 20, d->gps_hdop);
  f[22] = d->gps_satellites;
  f[23] = d->gps_fix_type;
  f[24] = d->gps_flags;
  put_u16(f + 25, d->gps_home_dist_m);
  put_u16(f + 27, d->gps_home_bearing_cdeg);
  f[29] = 0U;
  f[30] = 0U;
}

static void build_next_frame(uint32_t now_ms, const void *ctx)
{
  const uint8_t type = schedule[sched_idx];

  sched_idx++;
  if (sched_idx >= SCHED_LEN) {
    sched_idx = 0U;
  }

  fill_cb(&tdata, ctx);
  tdata.uptime_ms = now_ms;

  memset(frame, 0, sizeof(frame));
  switch (type) {
    case TEL_FRAME_ATT:    build_att(frame, &tdata);    break;
    case TEL_FRAME_RC:     build_rc(frame, &tdata);     break;
    case TEL_FRAME_STATUS: build_status(frame, &tdata); break;
    case TEL_FRAME_TUNE:   build_tune(frame, &tdata);   break;
    case TEL_FRAME_GPS:    build_gps(frame, &tdata);    break;
    default:               build_cfg(frame, &tdata);    break;
  }
  frame[1] = seq++;
  frame[TEL_PACKET_SIZE - 1U] = tel_crc8(frame, TEL_PACKET_SIZE - 1U);

  /* Lich slot: khong don no goi bu, chi nhac "tre" de thong ke. */
  if ((int32_t)(now_ms - next_slot_ms) > (int32_t)TEL_SLOT_MS) {
    if (stat_late < 255U) {
      stat_late++;
    }
    next_slot_ms = now_ms + TEL_SLOT_MS;
  } else {
    next_slot_ms += TEL_SLOT_MS;
  }

  g_state = TEL_ST_BUILT;
}

/* ------------------------------------------------------------------------ */
/* Phat / kiem tra / phuc hoi                                                */
/* ------------------------------------------------------------------------ */
static void enter_off(uint32_t now_ms, uint32_t delay_ms)
{
  CE_LOW();
  g_state = TEL_ST_OFF;
  retry_ms = now_ms + delay_ms;
  consecutive_fail = 0U;
  if (stat_reinit < 255U) {
    stat_reinit++;
  }
}

static void send_frame(uint32_t now_ms)
{
  spi_fault = false;

  (void)nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);

  nrf_begin();
  (void)spi_xfer(CMD_W_TX_PAYLOAD);
  for (uint8_t i = 0U; i < TEL_PACKET_SIZE; i++) {
    (void)spi_xfer(frame[i]);
  }
  nrf_end();

  if (spi_fault) {
    (void)nrf_cmd(CMD_FLUSH_TX);
    stat_fail++;
    consecutive_fail++;
    g_state = TEL_ST_IDLE;
    if (consecutive_fail >= TEL_FAIL_LIMIT) {
      enter_off(now_ms, 50U);
    }
    return;
  }

  /* Xung CE >= 10 us kich hoat phat dung 1 goi. */
  CE_HIGH();
  delay_us(12U);
  CE_LOW();

  tx_start_ms = now_ms;
  g_state = TEL_ST_TX;
}

static void tx_check(uint32_t now_ms)
{
  bool done;
  uint8_t status = 0U;

#if TEL_USE_IRQ_PIN
  done = ((NRF_IRQ_PORT->IDR & NRF_IRQ_PIN) == 0U);
#else
  status = nrf_cmd(CMD_NOP);
  done = ((status & (STATUS_TX_DS | STATUS_MAX_RT)) != 0U);
#endif

  if (done) {
    status = nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);
    if ((status & STATUS_TX_DS) != 0U) {
      stat_sent++;
      consecutive_fail = 0U;
      g_state = TEL_ST_IDLE;
      return;
    }
  } else if ((uint32_t)(now_ms - tx_start_ms) < TEL_TX_TIMEOUT_MS) {
    return;
  }

  /* MAX_RT hoac qua thoi gian: don FIFO, dem loi. */
  (void)nrf_cmd(CMD_FLUSH_TX);
  (void)nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);
  stat_fail++;
  consecutive_fail++;
  g_state = TEL_ST_IDLE;
  if (consecutive_fail >= TEL_FAIL_LIMIT) {
    enter_off(now_ms, 50U);
  }
}

static void health_check(uint32_t now_ms)
{
  uint8_t cfg;
  uint8_t ch;

  next_health_ms = now_ms + TEL_HEALTH_PERIOD_MS;
  spi_fault = false;
  cfg = nrf_read_reg(REG_CONFIG);
  ch = nrf_read_reg(REG_RF_CH);
  if (spi_fault || (cfg != CONFIG_PTX_UP) || (ch != (uint8_t)TEL_RF_CHANNEL)) {
    /* nRF bi reset (sut ap/nhieu) -> cau hinh lai. */
    enter_off(now_ms, 0U);
  }
}

static void try_recover(uint32_t now_ms)
{
  if (nrf_configure()) {
    g_state = TEL_ST_WAIT_PWR;
    state_ms = now_ms;
  } else {
    CE_LOW();
    retry_ms = now_ms + TEL_RETRY_PERIOD_MS;
  }
}

static bool step_due(uint32_t now_ms)
{
  switch (g_state) {
    case TEL_ST_OFF:
      return ((int32_t)(now_ms - retry_ms) >= 0);
    case TEL_ST_IDLE:
      return ((int32_t)(now_ms - next_slot_ms) >= 0) ||
             ((int32_t)(now_ms - next_health_ms) >= 0);
    case TEL_ST_BUILT:
      return true;
    default:
      return false;
  }
}

/* ------------------------------------------------------------------------ */
/* API                                                                       */
/* ------------------------------------------------------------------------ */
bool telemetry_init(TelemetryFillFn fill)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t now_ms;

  fill_cb = fill;
  cycles_per_us = SystemCoreClock / 1000000U;
  if (cycles_per_us == 0U) {
    cycles_per_us = 1U;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  /* Tat JTAG, giu SWD (ST-LINK) */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  CSN_HIGH();
  CE_LOW();

  gpio.Pin = NRF_CE_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_CE_PORT, &gpio);

  gpio.Pin = NRF_CSN_PIN;
  HAL_GPIO_Init(NRF_CSN_PORT, &gpio);

  gpio.Pin = NRF_SCK_PIN | NRF_MOSI_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = NRF_MISO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Master, mode 0, 8 bit, MSB truoc, NSS mem. PCLK1 = 36 MHz, /16 = 2.25 MHz (on dinh cho module PA/LNA). */
  SPI2->CR1 = 0U;
  SPI2->CR2 = 0U;
  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (3U << 3);
  SPI2->CR1 |= SPI_CR1_SPE;

  now_ms = HAL_GetTick();
  g_state = TEL_ST_OFF;
  retry_ms = now_ms + TEL_RETRY_PERIOD_MS;

  if (!nrf_configure()) {
    return false;
  }

  HAL_Delay(POWER_UP_WAIT_MS);   /* chi luc khoi dong, chua bay */
  now_ms = HAL_GetTick();

  diag_cfg = nrf_read_reg(REG_CONFIG);
  diag_cfg_valid = true;
  if (diag_cfg != CONFIG_PTX_UP) {
    return false;
  }

  g_state = TEL_ST_IDLE;
  next_slot_ms = now_ms;
  next_health_ms = now_ms + TEL_HEALTH_PERIOD_MS;
  return true;
}

void telemetry_diag(char *buf, uint32_t len)
{
#if TEL_USE_IRQ_PIN
  const unsigned int irq = ((NRF_IRQ_PORT->IDR & NRF_IRQ_PIN) != 0U) ? 1U : 0U;
#else
  const unsigned int irq = 1U;
#endif

  if (diag_cfg_valid) {
    (void)snprintf(buf, len,
                   "[TEL] diag: ST=%02X AW=%02X CH=%02X A0=%02X CFG=%02X SPIERR=%u IRQ=%u (can: 0E 03 50 6B 4E 0 1)\r\n",
                   diag_status, diag_aw, diag_ch, diag_addr0, diag_cfg,
                   spi_fault ? 1U : 0U, irq);
  } else {
    (void)snprintf(buf, len,
                   "[TEL] diag: ST=%02X AW=%02X CH=%02X A0=%02X CFG=-- SPIERR=%u IRQ=%u (can: 0E 03 50 6B 4E 0 1)\r\n",
                   diag_status, diag_aw, diag_ch, diag_addr0,
                   spi_fault ? 1U : 0U, irq);
  }
}

void telemetry_note_loop_cycles(uint32_t cycles)
{
  const uint32_t us = cycles / cycles_per_us;

  loop_sum_us += us;
  loop_cnt++;
  if (us > loop_max_us) {
    loop_max_us = us;
  }
}

bool telemetry_link_ok(void)
{
  return (g_state != TEL_ST_OFF);
}

void telemetry_poll(uint32_t now_ms, int32_t slack_cycles, const void *ctx)
{
  uint32_t required_us;
  uint32_t t0;
  uint32_t used_us;

  if (fill_cb == NULL) {
    return;
  }

  /* Buoc nhe (khong SPI nang), khong can slack. */
  if (g_state == TEL_ST_TX) {
    tx_check(now_ms);
    return;
  }
  if (g_state == TEL_ST_WAIT_PWR) {
    if ((uint32_t)(now_ms - state_ms) >= POWER_UP_WAIT_MS) {
      g_state = TEL_ST_IDLE;
      consecutive_fail = 0U;
      next_slot_ms = now_ms;
      next_health_ms = now_ms + TEL_HEALTH_PERIOD_MS;
    }
    return;
  }

  if (!step_due(now_ms)) {
    starving = false;
    return;
  }

  /* Buoc nang: chi chay khi con du thoi gian truoc nhip dieu khien ke tiep. */
  required_us = TEL_MIN_SLACK_US;
  if (starving && ((uint32_t)(now_ms - starve_start_ms) > TEL_STARVE_MS)) {
    required_us = TEL_MIN_SLACK_STARVED_US;
  }
  if (slack_cycles < (int32_t)(required_us * cycles_per_us)) {
    if (!starving) {
      starving = true;
      starve_start_ms = now_ms;
    }
    return;
  }
  starving = false;

  t0 = cyc();

  switch (g_state) {
    case TEL_ST_OFF:
      try_recover(now_ms);
      break;

    case TEL_ST_IDLE:
      if ((int32_t)(now_ms - next_slot_ms) >= 0) {
        build_next_frame(now_ms, ctx);
      } else {
        health_check(now_ms);
      }
      break;

    case TEL_ST_BUILT:
      send_frame(now_ms);
      break;

    default:
      break;
  }

  used_us = (uint32_t)(cyc() - t0) / cycles_per_us;
  if (used_us > stat_cpu_max_us) {
    stat_cpu_max_us = used_us;
  }
}
