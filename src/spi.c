#include "spi.h"

//static const char *TAG = "SPI_DEB";
static esp_err_t ret;
static spi_device_handle_t spi;

void init_spi(void)
{
    // Configuración del bus SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, // No usado
        .quadhd_io_num = -1, // No usado
        .max_transfer_sz = 32
        };

    // Inicializa el bus SPI
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    // Configuración del dispositivo esclavo
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  // 8 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,         // Pin CS
        .queue_size = 1,
        };


    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void spi_transaction(uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    spi_transaction_t trans = {
        .length = 8 * len, // longitud en bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    ret = spi_device_transmit(spi, &trans);
    ESP_ERROR_CHECK(ret);

    //ESP_LOGI(TAG, "Datos recibidos:");
    //for (int i = 0; i < sizeof(rx_data); i++) {
    //    ESP_LOGI(TAG, "Byte %d: 0x%02X", i, rx_data[i]);
    //}
}