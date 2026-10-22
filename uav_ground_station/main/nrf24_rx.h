/**
 * @file nrf24_rx.h
 * @brief nRF24L01 Receiver Driver for ESP32-C3 (ESP-IDF)
 */

#ifndef NRF24_RX_H
#define NRF24_RX_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Cấu hình chân ESP32-C3 nối nRF24L01 ------------------------------- */
#define NRF_PIN_CE       3   /* GPIO 3  -> Chân 3 (CE) nRF24               */
#define NRF_PIN_CSN      7   /* GPIO 7  -> Chân 4 (CSN) nRF24              */
#define NRF_PIN_SCK      4   /* GPIO 4  -> Chân 5 (SCK) nRF24              */
#define NRF_PIN_MISO     5   /* GPIO 5  -> Chân 7 (MISO) nRF24             */
#define NRF_PIN_MOSI     6   /* GPIO 6  -> Chân 6 (MOSI) nRF24             */

/* Khởi tạo SPI và cấu hình nRF24L01 ở chế độ PRX (Receiver) */
esp_err_t nrf24_rx_init(void);

/* Kiểm tra xem có gói tin trong RX FIFO không */
bool nrf24_rx_packet_available(void);

/* Đọc gói tin 32 bytes từ RX FIFO */
bool nrf24_rx_read_packet(uint8_t *buf);

/* In chi tiết các thanh ghi ra UART để chẩn đoán */
void nrf24_print_details(void);

/* Tự động phục hồi nRF24 nếu bị sụt nguồn hoặc kẹt FIFO */
void nrf24_rx_recover_if_needed(void);

#ifdef __cplusplus
}
#endif

#endif /* NRF24_RX_H */
