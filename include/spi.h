#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PIN_NUM_MISO 41
#define PIN_NUM_MOSI 40
#define PIN_NUM_CLK  42
#define PIN_NUM_CS   39
#define PIN_NUM_RESET 45

void init_spi(void);
void spi_transaction(uint8_t *tx_data, uint8_t *rx_data, size_t len);
