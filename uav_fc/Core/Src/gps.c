/**
  ******************************************************************************
  * @file    gps.c
  * @brief   Driver va bo giai ma NMEA GPS (BZGNSS BZ-251 / USART3) cho UAV FC.
  ******************************************************************************
  */

#include "gps.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define GPS_RX_BUF_SIZE       256U
#define GPS_NMEA_MAX_LEN      128U
#define GPS_TIMEOUT_MS        3000U

/* Danh sach baud rate tu dong do: BZ-251 mac dinh thuong la 38400 hoac 115200 */
static const uint32_t baud_rates[] = {38400U, 115200U, 9600U, 57600U};
#define BAUD_RATES_COUNT (sizeof(baud_rates) / sizeof(baud_rates[0]))
static uint8_t  baud_idx = 0U;
static bool     baud_locked = false;
static uint32_t last_baud_switch_ms = 0U;

/* Ring buffer cho USART3 RX */
static volatile uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];
static volatile uint16_t gps_rx_head = 0U;
static volatile uint16_t gps_rx_tail = 0U;

/* Bo dem chuoi NMEA dang nhan */
static char    nmea_line[GPS_NMEA_MAX_LEN];
static uint8_t nmea_len = 0U;
static bool    nmea_capturing = false;

/* Du lieu GPS toan cuc */
static GpsData g_gps = {0};

/* Toa do diem Home */
static bool    home_is_set = false;
static int32_t home_lat = 0;
static int32_t home_lon = 0;
static int32_t home_alt_cm = 0;

/* ------------------------------------------------------------------------ */
/* USART3 RX Interrupt Handler                                              */
/* ------------------------------------------------------------------------ */
void gps_uart_irq_handler(void)
{
  if ((USART3->SR & USART_SR_RXNE) != 0U) {
    uint8_t c = (uint8_t)USART3->DR;
    uint16_t next = (uint16_t)((gps_rx_head + 1U) % GPS_RX_BUF_SIZE);
    if (next != gps_rx_tail) {
      gps_rx_buf[gps_rx_head] = c;
      gps_rx_head = next;
    }
  }
  if ((USART3->SR & USART_SR_ORE) != 0U) {
    (void)USART3->DR; /* Xoa co tran */
  }
}

/* ------------------------------------------------------------------------ */
/* Chuyen doi toc do Baud truc tiep tren USART3                            */
/* ------------------------------------------------------------------------ */
static void gps_set_baudrate(uint32_t baud)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t div = (pclk1 + (baud / 2U)) / baud;

  USART3->CR1 &= ~USART_CR1_UE;
  USART3->BRR = (uint16_t)(((div / 16U) << 4) | (div % 16U));
  USART3->CR1 |= USART_CR1_UE;

  __disable_irq();
  gps_rx_head = 0U;
  gps_rx_tail = 0U;
  nmea_len = 0U;
  nmea_capturing = false;
  __enable_irq();
}

/* ------------------------------------------------------------------------ */
/* Khoi tao phan cung USART3 (PB10=TX, PB11=RX, Auto-Baud scanner)          */
/* ------------------------------------------------------------------------ */
void gps_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  memset(&g_gps, 0, sizeof(g_gps));
  gps_rx_head = 0U;
  gps_rx_tail = 0U;
  nmea_len = 0U;
  nmea_capturing = false;
  baud_idx = 0U;
  baud_locked = false;
  last_baud_switch_ms = 0U;

  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PB10 = USART3_TX */
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* PB11 = USART3_RX (Keo tro pull-up cao: an toan tuyet doi khi chua gan day) */
  gpio.Pin = GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Bat dau bang baud rate mac dinh pho bien nhat cua BZ-251: 38400 bps */
  gps_set_baudrate(baud_rates[0]);

  USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  USART3->CR2 = 0U;
  USART3->CR3 = 0U;

  HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* ------------------------------------------------------------------------ */
/* Parser tien ich: NMEA -> So nguyen khong bi mat do chinh xac             */
/* ------------------------------------------------------------------------ */

/* Chuyen ddmm.mmmmm -> deg * 1e7 */
static int32_t parse_coord(const char *s, char dir)
{
  if (!s || *s == '\0') {
    return 0;
  }
  const char *dot = strchr(s, '.');
  if (!dot) {
    return 0;
  }

  int dot_idx = (int)(dot - s);
  if (dot_idx < 2) {
    return 0;
  }

  int min_int = (s[dot_idx - 2] - '0') * 10 + (s[dot_idx - 1] - '0');

  int deg_int = 0;
  for (int i = 0; i < dot_idx - 2; i++) {
    if (s[i] >= '0' && s[i] <= '9') {
      deg_int = deg_int * 10 + (s[i] - '0');
    }
  }

  /* Phan thap phan cua phut toi da 5 chu so */
  int32_t min_frac = 0;
  int32_t mult = 10000;
  const char *p = dot + 1;
  while (*p >= '0' && *p <= '9' && mult > 0) {
    min_frac += (*p - '0') * mult;
    mult /= 10;
    p++;
  }

  int64_t total_min_1e5 = (int64_t)min_int * 100000LL + min_frac;
  int32_t deg_from_min = (int32_t)((total_min_1e5 * 100LL) / 60LL);

  int32_t result = deg_int * 10000000 + deg_from_min;
  if (dir == 'S' || dir == 'W') {
    result = -result;
  }
  return result;
}

/* Doc chuoi so thuc thanh so nguyen nhan voi scale (VD: "12.3" * 100 = 1230) */
static int32_t parse_float_scale(const char *s, int scale)
{
  if (!s || *s == '\0') {
    return 0;
  }
  float f = (float)atof(s);
  return (int32_t)(f * (float)scale);
}

/* ------------------------------------------------------------------------ */
/* Tinh khoang cach va huong quay ve diem Home (Equirectangular)             */
/* ------------------------------------------------------------------------ */
static void gps_update_home(GpsData *g)
{
  if (!home_is_set) {
    if (g->fix_type >= 3 && g->satellites >= 6 && g->hdop_x100 <= 300) {
      home_lat = g->lat;
      home_lon = g->lon;
      home_alt_cm = g->alt_cm;
      home_is_set = true;
      g->flags |= TEL_GPS_FLAG_HOME_SET;
    } else {
      g->home_dist_m = 0U;
      g->home_bearing_cdeg = 0U;
      return;
    }
  }

  int32_t dlat = g->lat - home_lat;
  int32_t dlon = g->lon - home_lon;

  /* 1e-7 deg approx 0.0111139 m */
  float lat_rad = ((float)home_lat * 1e-7f) * 0.0174532925f;
  float cos_lat = cosf(lat_rad);
  if (cos_lat < 0.1f) {
    cos_lat = 0.1f;
  }

  float dy_m = (float)dlat * 0.0111139f;
  float dx_m = (float)dlon * 0.0111139f * cos_lat;

  float dist_m = sqrtf(dx_m * dx_m + dy_m * dy_m);
  if (dist_m > 65535.0f) {
    dist_m = 65535.0f;
  }
  g->home_dist_m = (uint16_t)dist_m;

  float bearing_deg = atan2f(dx_m, dy_m) * 57.2957795f; /* 0 = Bac, 90 = Dong */
  if (bearing_deg < 0.0f) {
    bearing_deg += 360.0f;
  }
  g->home_bearing_cdeg = (uint16_t)(bearing_deg * 100.0f);
}

/* ------------------------------------------------------------------------ */
/* Giai ma mot cau NMEA hop le                                              */
/* ------------------------------------------------------------------------ */
static void parse_nmea_sentence(char *s, uint32_t now_ms)
{
  /* Tach thanh cac token ngan cach bang dau phay */
  char *tokens[24] = {0};
  uint8_t token_cnt = 0U;

  char *p = s;
  tokens[token_cnt++] = p;
  while (*p && token_cnt < 24U) {
    if (*p == ',') {
      *p = '\0';
      tokens[token_cnt++] = p + 1;
    }
    p++;
  }

  if (token_cnt < 2) {
    return;
  }

  /* 1. Giai ma GGA: $--GGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,... */
  if ((strstr(tokens[0], "GGA") != NULL) && (token_cnt >= 10)) {
    int fix_q = atoi(tokens[6]);
    if (fix_q > 0) {
      g_gps.fix_type = (fix_q >= 2) ? 4U : 3U; /* 4=DGPS, 3=3D Fix */
      g_gps.flags |= TEL_GPS_FLAG_FIX_3D;
      g_gps.last_fix_ms = now_ms;
    } else {
      g_gps.fix_type = 0U;
      g_gps.flags &= ~TEL_GPS_FLAG_FIX_3D;
    }

    if (tokens[2][0] != '\0' && tokens[4][0] != '\0') {
      g_gps.lat = parse_coord(tokens[2], tokens[3][0]);
      g_gps.lon = parse_coord(tokens[4], tokens[5][0]);
    }
    g_gps.satellites = (uint8_t)atoi(tokens[7]);
    g_gps.hdop_x100  = (uint16_t)parse_float_scale(tokens[8], 100);
    g_gps.alt_cm     = parse_float_scale(tokens[9], 100); /* met -> cm */

    gps_update_home(&g_gps);
  }

  /* 2. Giai ma RMC: $--RMC,time,status,lat,N,lon,E,spd_knots,course_deg,date,... */
  else if ((strstr(tokens[0], "RMC") != NULL) && (token_cnt >= 9)) {
    char status = tokens[2][0];
    if (status == 'A') { /* Active / Valid */
      if (g_gps.fix_type == 0U) {
        g_gps.fix_type = 2U; /* It nhat co 2D Fix */
      }
      if (g_gps.lat == 0 && tokens[3][0] != '\0') {
        g_gps.lat = parse_coord(tokens[3], tokens[4][0]);
        g_gps.lon = parse_coord(tokens[5], tokens[6][0]);
      }
      /* Knots -> cm/s (1 knot = 51.4444 cm/s) */
      float knots = (float)atof(tokens[7]);
      g_gps.speed_cms = (uint16_t)(knots * 51.4444f);
      g_gps.course_cdeg = (uint16_t)parse_float_scale(tokens[8], 100);
    }
  }
}

/* ------------------------------------------------------------------------ */
/* Non-blocking Poll: goi trong main loop                                   */
/* ------------------------------------------------------------------------ */
void gps_poll(uint32_t now_ms)
{
  /* Auto-Baud scanner: Neu chua lock duoc baud hop le thi tu dong doi toc do sau moi 1.2 giay */
  if (!baud_locked) {
    if (last_baud_switch_ms == 0U) {
      last_baud_switch_ms = now_ms;
    } else if ((uint32_t)(now_ms - last_baud_switch_ms) >= 1200U) {
      last_baud_switch_ms = now_ms;
      baud_idx = (uint8_t)((baud_idx + 1U) % BAUD_RATES_COUNT);
      gps_set_baudrate(baud_rates[baud_idx]);
      char bmsg[64];
      (void)snprintf(bmsg, sizeof(bmsg), "[GPS] Auto-baud: scanning %lu bps...\r\n", (unsigned long)baud_rates[baud_idx]);
      uart_log(bmsg);
    }
  }

  /* Doc cac byte co san trong ring buffer */
  while (gps_rx_head != gps_rx_tail) {
    uint8_t c = gps_rx_buf[gps_rx_tail];
    gps_rx_tail = (uint8_t)((gps_rx_tail + 1U) % GPS_RX_BUF_SIZE);

    if (c == '$') {
      nmea_len = 0U;
      nmea_capturing = true;
      continue;
    }

    if (!nmea_capturing) {
      continue;
    }

    if (c == '\r' || c == '\n') {
      nmea_line[nmea_len] = '\0';
      nmea_capturing = false;

      if (nmea_len >= 9U) {
        /* Kiem tra Checksum NMEA (*hh o cuoi chuoi) */
        char *ast = strchr(nmea_line, '*');
        if (ast && (ast - nmea_line) < (nmea_len - 1U)) {
          uint8_t expected_cs = (uint8_t)strtol(ast + 1, NULL, 16);
          uint8_t calc_cs = 0U;
          for (char *q = nmea_line; q < ast; q++) {
            calc_cs ^= (uint8_t)(*q);
          }
          if (calc_cs == expected_cs) {
            *ast = '\0'; /* Cat bo phan checksum */

            if (!baud_locked) {
              baud_locked = true;
              char bmsg[64];
              (void)snprintf(bmsg, sizeof(bmsg), "[GPS] Baud LOCKED at %lu bps!\r\n", (unsigned long)baud_rates[baud_idx]);
              uart_log(bmsg);
            }

            g_gps.last_packet_ms = now_ms;
            g_gps.present = true;
            g_gps.flags |= TEL_GPS_FLAG_PRESENT;
            g_gps.flags &= ~TEL_GPS_FLAG_TIMEOUT;

            parse_nmea_sentence(nmea_line, now_ms);
          }
        }
      }
      nmea_len = 0U;
    } else {
      if (nmea_len < (GPS_NMEA_MAX_LEN - 1U)) {
        nmea_line[nmea_len++] = (char)c;
      } else {
        nmea_capturing = false; /* Qua dai -> bo */
      }
    }
  }

  /* Watchdog: mat ket noi GPS qua 3s */
  if (g_gps.present && ((uint32_t)(now_ms - g_gps.last_packet_ms) > GPS_TIMEOUT_MS)) {
    g_gps.flags |= TEL_GPS_FLAG_TIMEOUT;
    g_gps.fix_type = 0U;
    baud_locked = false; /* Cho phep scan lai neu mat tin hieu lau */
  }
}

const GpsData *gps_get_data(void)
{
  return &g_gps;
}

void gps_set_home(void)
{
  if (g_gps.fix_type >= 3) {
    home_lat = g_gps.lat;
    home_lon = g_gps.lon;
    home_alt_cm = g_gps.alt_cm;
    home_is_set = true;
    g_gps.flags |= TEL_GPS_FLAG_HOME_SET;
  }
}
