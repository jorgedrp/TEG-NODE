#include "lora.h"
#include "spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

uint8_t _packetIndex = 0;
uint16_t lora_time_out = 4000;

void init_lora(sx1278_config_t lora_config)
{
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RESET, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_level(PIN_NUM_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    gpio_set_level(PIN_NUM_RESET, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RESET, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    init_spi();

    tsleep();

    setFrequency(lora_config.frequency);
    setSignalBandwidth(lora_config.bw);
    setSpreadingFactor(lora_config.sf);
    setTxPower(lora_config.txpow, PA_OUTPUT_PA_BOOST_PIN);

    enableCrc();
    setCodingRate(8);
    setPreambleLength(16);

    writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    writeRegister(REG_SYNC_WORD, 0x33);
    writeRegister(REG_MODEM_CONFIG_3, 0x00);
    writeRegister(REG_LNA, 0x20);

    idle();
}

void config_lora(sx1278_config_t* lora_config, int mode)
{
    switch (mode)
    {
    case 1:
        lora_config->sf = 7;
        lora_config->bw = 125e3;
        lora_config->frequency = 433775000;
        lora_config->txpow = 17;
        lora_time_out = 1000;
        break;
    case 2:
        lora_config->sf = 7;
        lora_config->bw = 250e3;
        lora_config->frequency = 433775000;
        lora_config->txpow = 17;
        lora_time_out = 1000;
        break;
    case 3:
        lora_config->sf = 7;
        lora_config->bw = 500e3;
        lora_config->frequency = 433775000;
        lora_config->txpow = 17;
        lora_time_out = 1000;
        break;
    case 4:
        lora_config->sf = 8;
        lora_config->bw = 125e3;
        lora_config->frequency = 433775000;
        lora_config->txpow = 17;
        lora_time_out = 1000;
        break;
    case 5:
        lora_config->sf = 9;
        lora_config->bw = 125e3;
        lora_config->frequency = 433775000;
        lora_config->txpow = 17;
        lora_time_out = 2000;
        break;
    case 6:
        lora_config->sf = 10;
        lora_config->bw = 125e3;
        lora_config->frequency = 433175000;
        lora_config->txpow = 20;
        lora_time_out = 4000;
        break;
    case 7:
        lora_config->sf = 12;
        lora_config->bw = 125e3;
        lora_config->frequency = 433175000;
        lora_config->txpow = 20;
        lora_time_out = 10000;
        break;
    }

    idle();

    if (mode == 3)
    {
        writeRegister(0x36, 0x02);
        writeRegister(0x3A, 0x64);
        setCodingRate(5);
        setPreambleLength(8);
    }
    else if (mode >= 6)
    {
        writeRegister(0x36, 0x03);
        writeRegister(0x3A, 0x65);
        setCodingRate(8);
        setPreambleLength(16);
    }
    else
    {
        writeRegister(0x36, 0x03);
        writeRegister(0x3A, 0x65);
        setCodingRate(5);
        setPreambleLength(8);
    }

    setSignalBandwidth(lora_config->bw);
    setSpreadingFactor(lora_config->sf);
    setFrequency(lora_config->frequency);
    setTxPower(lora_config->txpow, PA_OUTPUT_PA_BOOST_PIN);
}

int8_t packetRssi(sx1278_config_t lora_config)
{
    return (readRegister(REG_PKT_RSSI_VALUE) - (lora_config.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float packetSnr(void)
{
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

int8_t rssi(sx1278_config_t lora_config)
{
    return (readRegister(REG_RSSI_VALUE) - (lora_config.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

void idle(void)
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void tsleep(void)
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void setTxPower(uint8_t level, uint8_t outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin)
    {
        // RFO
        if (level > 14)
        {
            level = 14;
        }
        writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else
    {
        // PA BOOST
        if (level > 17)
        {
            if (level > 20)
            {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        }
        else
        {
            if (level < 2)
            {
                level = 2;
            }
            //Default value PA_HF/LF or +17dBm
            writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }

        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void setFrequency(uint32_t frequency)
{
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void setSpreadingFactor(uint8_t sf)
{
    if (sf < 6)
    {
        sf = 6;
    }
    else if (sf > 12)
    {
        sf = 12;
    }

    if (sf == 6)
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    //setLdoFlag();
}

void setSignalBandwidth(uint32_t sbw)
{
    int bw;

    if (sbw <= 7.8E3)
    {
        bw = 0;
    }
    else if (sbw <= 10.4E3)
    {
        bw = 1;
    }
    else if (sbw <= 15.6E3)
    {
        bw = 2;
    }
    else if (sbw <= 20.8E3)
    {
        bw = 3;
    }
    else if (sbw <= 31.25E3)
    {
        bw = 4;
    }
    else if (sbw <= 41.7E3)
    {
        bw = 5;
    }
    else if (sbw <= 62.5E3)
    {
        bw = 6;
    }
    else if (sbw <= 125E3)
    {
        bw = 7;
    }
    else if (sbw <= 250E3)
    {
        bw = 8;
    }
    else /*if (sbw >= 250E3)*/
    {
        bw = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    //setLdoFlag();
}

int32_t getSignalBandwidth(void)
{
    uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (bw)
    {
        case 0: return 7.8E3;
        case 1: return 10.4E3;
        case 2: return 15.6E3;
        case 3: return 20.8E3;
        case 4: return 31.25E3;
        case 5: return 41.7E3;
        case 6: return 62.5E3;
        case 7: return 125E3;
        case 8: return 250E3;
        case 9: return 500E3;
    }

    return -1;
}

uint8_t getSpreadingFactor(void)
{
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

// void setLdoFlag(void)
// {
//     // Section 4.1.1.5
//     long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

//     // Section 4.1.1.6
//     boolean ldoOn = symbolDuration > 16;

//     uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
//     bitWrite(config3, 3, ldoOn);
//     writeRegister(REG_MODEM_CONFIG_3, config3);
// }

void setCodingRate(uint8_t denominator)
{
    if (denominator < 5)
    {
        denominator = 5;
    }
    else if (denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void setPreambleLength(uint16_t length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void enableCrc(void)
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void disableCrc(void)
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120)
    {
        ocpTrim = (mA - 45) / 5;
    }
    else if (mA <=240)
    {
        ocpTrim = (mA + 30) / 10;
    }

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

uint8_t readRegister(uint8_t address)
{
    uint8_t out[2] = {address & 0x7f, 0x00};
    uint8_t in[2];
    spi_transaction(out, in, sizeof(out));
    return in[1];
}

void writeRegister(uint8_t address, uint8_t value)
{
    uint8_t out[2] = {0x80 | address, value};
    uint8_t in[2];
    spi_transaction(out, in, sizeof(out));
}

void lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", readRegister(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}

int single_send_packet(uint8_t *data_buffer, size_t size)
{
    EventGroupHandle_t task_event_group = get_lora_event_group();

    writeRegister(REG_PAYLOAD_LENGTH, size);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 = TxDone

    for(size_t i = 0 ; i < size ; i++)
    {
        writeRegister(REG_FIFO, data_buffer[i]);
    }

    writeRegister(REG_OP_MODE, 0x8B);

    EventBits_t txBits = xEventGroupWaitBits(task_event_group, LORA_IRQ_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(lora_time_out));

    writeRegister(REG_IRQ_FLAGS, 0xFF);

    if ((txBits & LORA_IRQ_BIT) != 0)
    {
        return 1;
    }
    else
    {
        writeRegister(REG_OP_MODE, 0x89);
    }
    return 0;
}

int single_receive_packet(uint8_t command, uint16_t wait_time)
{
    EventGroupHandle_t task_event_group = get_lora_event_group();

    writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_RX_LENGTH);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
    writeRegister(REG_OP_MODE, 0x8D);

    EventBits_t uxBits = xEventGroupWaitBits(task_event_group, LORA_IRQ_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(wait_time));

    if ((uxBits & LORA_IRQ_BIT) != 0)
    {
        uint8_t rx_irq_flags = readRegister(REG_IRQ_FLAGS);
        writeRegister(REG_IRQ_FLAGS, rx_irq_flags);

        if((rx_irq_flags & IRQ_RX_DONE_MASK) != 0 && (rx_irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
        {
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

            uint8_t dev_id = readRegister(REG_FIFO);
            uint8_t cmd = readRegister(REG_FIFO);

            if(dev_id == DEV_ID && cmd == command)
            {
                return 1;
            }
        }
    }
    else
    {
        writeRegister(REG_OP_MODE, 0x89);
    }
    return 0;
}
