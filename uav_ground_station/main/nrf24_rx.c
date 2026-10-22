/**
 * @file nrf24_rx.c
 * @brief nRF24L01 Receiver Driver for ESP32-C3 (ESP-IDF)
 */

#include "nrf24_rx.h"
#include "telemetry_protocol.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "NRF24_RX";

/* Lenh nRF24L01 */
#define CMD_R_REGISTER    0x00U
#define CMD_W_REGISTER    0x20U
#define CMD_R_RX_PAYLOAD  0x61U
#define CMD_FLUSH_TX      0xE1U
#define CMD_FLUSH_RX      0xE2U
#define CMD_NOP           0xFFU

/* Thanh ghi nRF24L01 */
#define REG_CONFIG        0x00U
#define REG_EN_AA         0x01U
#define REG_EN_RXADDR     0x02U
#define REG_SETUP_AW      0x03U
#define REG_SETUP_RETR    0x04U
#define REG_RF_CH         0x05U
#define REG_RF_SETUP      0x06U
#define REG_STATUS        0x07U
#define REG_RX_ADDR_P0    0x0AU
#define REG_RX_PW_P0      0x11U
#define REG_FIFO_STATUS   0x17U
#define REG_DYNPD         0x1CU
#define REG_FEATURE       0x1DU

#define STATUS_RX_DR      0x40U
#define STATUS_TX_DS      0x20U
#define STATUS_MAX_RT     0x10U
#define STATUS_CLEAR_ALL  0x70U

/* CONFIG: MASK_MAX_RT=1, MASK_TX_DS=1, EN_CRC=1, CRCO=1 (16-bit), PWR_UP=1, PRIM_RX=1 */
#define CONFIG_PRX_UP     0x3FU

static spi_device_handle_t spi_dev = NULL;
static const uint8_t rx_address[TEL_ADDR_WIDTH] = TEL_ADDRESS_INIT;

static inline void csn_low(void)
{
    gpio_set_level((gpio_num_t)NRF_PIN_CSN, 0);
    esp_rom_delay_us(2);
}

static inline void csn_high(void)
{
    gpio_set_level((gpio_num_t)NRF_PIN_CSN, 1);
    esp_rom_delay_us(3);
}

static inline void ce_high(void)
{
    gpio_set_level((gpio_num_t)NRF_PIN_CE, 1);
    esp_rom_delay_us(10);
}

static inline void ce_low(void)
{
    gpio_set_level((gpio_num_t)NRF_PIN_CE, 0);
    esp_rom_delay_us(10);
}

static uint8_t spi_xfer_byte(uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = data;

    if (spi_device_polling_transmit(spi_dev, &t) != ESP_OK) {
        return 0;
    }
    return t.rx_data[0];
}

static uint8_t nrf_cmd(uint8_t cmd)
{
    csn_low();
    uint8_t status = spi_xfer_byte(cmd);
    csn_high();
    return status;
}

static uint8_t nrf_write_reg(uint8_t reg, uint8_t val)
{
    csn_low();
    uint8_t status = spi_xfer_byte(CMD_W_REGISTER | (reg & 0x1FU));
    spi_xfer_byte(val);
    csn_high();
    return status;
}

static uint8_t nrf_read_reg(uint8_t reg)
{
    csn_low();
    spi_xfer_byte(CMD_R_REGISTER | (reg & 0x1FU));
    uint8_t val = spi_xfer_byte(CMD_NOP);
    csn_high();
    return val;
}

static void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    csn_low();
    spi_xfer_byte(CMD_W_REGISTER | (reg & 0x1FU));
    for (uint8_t i = 0; i < len; i++) {
        spi_xfer_byte(buf[i]);
    }
    csn_high();
}

static void nrf_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    csn_low();
    spi_xfer_byte(CMD_R_REGISTER | (reg & 0x1FU));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_xfer_byte(CMD_NOP);
    }
    csn_high();
}

esp_err_t nrf24_rx_init(void)
{
    ESP_LOGI(TAG, "Initializing SPI bus and GPIOs...");

    /* Cau hinh GPIO CE va CSN */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NRF_PIN_CE) | (1ULL << NRF_PIN_CSN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    csn_high();
    ce_low();

    /* Cau hinh SPI Bus */
    spi_bus_config_t buscfg = {
        .miso_io_num = NRF_PIN_MISO,
        .mosi_io_num = NRF_PIN_MOSI,
        .sclk_io_num = NRF_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Cau hinh SPI Device (Mode 0, 2 MHz de on dinh voi module anten ngoai) */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, /* 2 MHz */
        .mode = 0,                         /* CPOL=0, CPHA=0 */
        .spics_io_num = -1,                /* CSN dieu khien tay bang GPIO */
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Cau hinh thanh ghi nRF24L01 o che do PRX */
    ce_low();
    csn_high();
    vTaskDelay(pdMS_TO_TICKS(10));

    nrf_write_reg(REG_CONFIG, 0x00);          /* Power down tam thoi */
    nrf_write_reg(REG_EN_AA, 0x00);           /* Tat Auto-ACK */
    nrf_write_reg(REG_EN_RXADDR, 0x01);       /* Bat Pipe 0 */
    nrf_write_reg(REG_SETUP_AW, 0x03);        /* Dia chi 5 byte */
    nrf_write_reg(REG_SETUP_RETR, 0x00);      /* Khong retry */
    nrf_write_reg(REG_RF_CH, TEL_RF_CHANNEL); /* Kenh 80 (2480 MHz) */
    nrf_write_reg(REG_RF_SETUP, 0x06);        /* 1 Mbps, 0 dBm LNA */
    nrf_write_buf(REG_RX_ADDR_P0, rx_address, TEL_ADDR_WIDTH);
    nrf_write_reg(REG_RX_PW_P0, TEL_PACKET_SIZE); /* 32 bytes */
    nrf_write_reg(REG_DYNPD, 0x00);
    nrf_write_reg(REG_FEATURE, 0x00);

    nrf_cmd(CMD_FLUSH_RX);
    nrf_cmd(CMD_FLUSH_TX);
    nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);

    /* Xac thuc thanh ghi xem SPI va chip co chay dung khong */
    uint8_t aw = nrf_read_reg(REG_SETUP_AW);
    uint8_t ch = nrf_read_reg(REG_RF_CH);
    uint8_t pw = nrf_read_reg(REG_RX_PW_P0);
    uint8_t addr_buf[5] = {0};
    nrf_read_buf(REG_RX_ADDR_P0, addr_buf, 5);

    ESP_LOGI(TAG, "Xac thuc nRF24: AW=0x%02X, CH=%d, PW=%d, Addr0=0x%02X", aw, ch, pw, addr_buf[0]);

    if (aw != 0x03 || ch != TEL_RF_CHANNEL || pw != TEL_PACKET_SIZE || addr_buf[0] != rx_address[0]) {
        ESP_LOGE(TAG, "nRF24L01 NOT FOUND hoac loi giao tiep SPI! Kiem tra lai day noi.");
        return ESP_FAIL;
    }

    /* Bat nguon che do PRX (Receiver) */
    nrf_write_reg(REG_CONFIG, CONFIG_PRX_UP);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Kéo CE len HIGH de bat dau nhan song */
    ce_high();

    ESP_LOGI(TAG, "nRF24L01 Receiver ready on Channel %d (1Mbps, Pipe0: 32 bytes)", TEL_RF_CHANNEL);
    return ESP_OK;
}

bool nrf24_rx_packet_available(void)
{
    uint8_t fifo = nrf_read_reg(REG_FIFO_STATUS);
    uint8_t status = nrf_cmd(CMD_NOP);
    return ((fifo & 0x01) == 0) || ((status & STATUS_RX_DR) != 0);
}

bool nrf24_rx_read_packet(uint8_t *buf)
{
    uint8_t fifo_status = nrf_read_reg(REG_FIFO_STATUS);
    uint8_t status = nrf_cmd(CMD_NOP);

    /* Kiem tra co goi tin trong RX FIFO khong (Bit 0 cua FIFO_STATUS la RX_EMPTY) */
    bool has_packet = ((fifo_status & 0x01) == 0) || ((status & STATUS_RX_DR) != 0);
    if (!has_packet) {
        return false;
    }

    /* Doc 32 bytes payload tu FIFO */
    csn_low();
    spi_xfer_byte(CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < TEL_PACKET_SIZE; i++) {
        buf[i] = spi_xfer_byte(CMD_NOP);
    }
    csn_high();

    /* Xoa co RX_DR va cac co ngat de khong bo sot goi sau */
    nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);
    return true;
}

void nrf24_print_details(void)
{
    uint8_t st = nrf_cmd(CMD_NOP);
    uint8_t cfg = nrf_read_reg(REG_CONFIG);
    uint8_t ch = nrf_read_reg(REG_RF_CH);
    uint8_t fifo = nrf_read_reg(REG_FIFO_STATUS);
    ESP_LOGI(TAG, "STATUS=0x%02X, CONFIG=0x%02X, CH=%d, FIFO_STATUS=0x%02X", st, cfg, ch, fifo);
}

void nrf24_rx_recover_if_needed(void)
{
    uint8_t cfg = nrf_read_reg(REG_CONFIG);
    uint8_t ch = nrf_read_reg(REG_RF_CH);
    uint8_t fifo = nrf_read_reg(REG_FIFO_STATUS);

    /* Neu chip bi mat cau hinh do sut ap (CONFIG != 0x3F hoac kenh lech) */
    if (cfg != CONFIG_PRX_UP || ch != TEL_RF_CHANNEL) {
        ESP_LOGW(TAG, "nRF24 bi sut nguon/reset (CONFIG=0x%02X, CH=%d) -> Tu dong khoi tao lai!", cfg, ch);
        nrf24_rx_init();
        return;
    }

    /* Neu FIFO bi day kẹt (Bit 1 cua FIFO_STATUS la RX_FULL) */
    if ((fifo & 0x02) != 0) {
        ESP_LOGW(TAG, "nRF24 FIFO bi day -> Xoa bo dem RX de don goi moi...");
        nrf_cmd(CMD_FLUSH_RX);
        nrf_write_reg(REG_STATUS, STATUS_CLEAR_ALL);
        ce_high();
    }
}
