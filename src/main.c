#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "lora.h"
#include "spi.h"
#include "i2c.h"
#include "mpu6050.h"
#include "bme280.h"
#include "clock.h"
#include "esp_timer.h"
#include "inttypes.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "nvs.h"

#define BLINK_GPIO            GPIO_NUM_48
#define LED_STRIP_LED_NUM     1
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define MPU_BUFFER_SIZE       510000    //15 * 510000 = 7,47MB
#define BME_BUFFER_SIZE       3600      //15 * 3600 = 52,7kB

sx1278_config_t lora_config = {
    .sf = 10,
    .bw = 125e3,
    .txpow = 20,
    .implicit = 0,
    .frequency = 433175000,
};

volatile uint8_t data_channel = 3;

static TaskHandle_t mpu_task_handle = NULL;
static TaskHandle_t bme_task_handle = NULL;
static EventGroupHandle_t task_event_group = NULL;

SemaphoreHandle_t i2c_mutex;

static mpu6050_data_t *mpu_buffer = NULL;
//static mpu6050_offset_t offset_data_1;
static mpu6050_umbral_t umbral_data;

static bme280_data_t *bme_buffer = NULL;

static led_strip_handle_t led_strip;

const int MODO_TIEMPO = BIT0;
const int MODO_EVENTO = BIT1;
const int MODO_STANDBY = BIT2;
const int CONFIG = BIT3;
const int CLOCK = BIT4;
const int DATA = BIT5;
const int CLOCK_DONE = BIT6;
const int TRANSMIT_INIT = BIT7;
const int TRANSMIT_COMPLETE = BIT8;
const int LORA_IRQ = LORA_IRQ_BIT;

void init_time_mode(void);
void init_event_mode(void);
void init_standby_mode(void);
void init_time_sync(void *p);

static volatile uint16_t tiempo = 10;
static volatile uint8_t num_sensores = 1;
static volatile int eventos = 0;
static volatile int event_now = 0;
static volatile int send_event_now = 0;
static volatile int rate = 5;
static volatile int retransmit = 0;
static volatile uint16_t bme_data_count = 0;
static volatile size_t indice_mpu_buffer = 0;
static volatile size_t indice_mpu_buffer_inicial = 0;
static volatile size_t indice_mpu_buffer_final = 0;
static volatile size_t indice_mpu_buffer_transmit = 0;
static volatile size_t indice_mpu_trigger = 0;
static struct timeval start_time;
volatile uint64_t t4_hardware_us = 0;


void init_led_strip(void)
{
    led_strip_config_t strip_config = {
    .strip_gpio_num = BLINK_GPIO,
    .max_leds = LED_STRIP_LED_NUM,
    .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
    .led_model = LED_MODEL_WS2812,
    .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
    .resolution_hz = LED_STRIP_RMT_RES_HZ,
    .flags.with_dma = false,
    };

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    led_strip_clear(led_strip);
}

static void IRAM_ATTR lora_dio0_isr_handler(void* arg)
{
    t4_hardware_us = esp_timer_get_time();
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(task_event_group, LORA_IRQ_BIT, &xHigherPriorityTaskWoken);
}

EventGroupHandle_t get_lora_event_group(void)
{
    return task_event_group;
}

void lora_interrupt_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_DIO0_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO0_PIN, lora_dio0_isr_handler, NULL);
}

void mpu_buffer_init(void)
{
    mpu_buffer = (mpu6050_data_t *)heap_caps_malloc(MPU_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
}

void bme_buffer_init(void)
{
    bme_buffer = (bme280_data_t *)heap_caps_malloc(BME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
}

void task_tx(void *p)
{
    uint8_t data[PACKET_SIZE];
    uint8_t new_mpu_packet = 1;
    uint8_t new_bme_packet = 1;
    uint16_t mpu_packet_num = 0;
    uint16_t bme_packet_num = 0;
    size_t data_index = 0;
    uint64_t timestamp;
    int retry = 0;

    uint32_t pre_buffer = (tiempo * 200) / rate;

    mpu6050_data_t item;
    bme280_data_t item2;
    data_available_t signal;

    timestamp = (uint64_t)start_time.tv_sec * 1000000L + start_time.tv_usec;

//    time_t segundos = (time_t)(timestamp / 1000000L);
//    struct tm *tm_local = localtime(&segundos);
//    char buffer_fecha_hora[100];
//    strftime(buffer_fecha_hora, sizeof(buffer_fecha_hora), "%Y-%m-%d %H:%M:%S", tm_local);
//    printf("Fecha de evento: %s\n", buffer_fecha_hora);

    signal.dev_id = DEV_ID;
    signal.transmit_data = DATA_AVAILABLE;
    signal.timestamp_1 = (uint8_t)((timestamp >> 56) & 0xFF);
    signal.timestamp_2 = (uint8_t)((timestamp >> 48) & 0xFF);
    signal.timestamp_3 = (uint8_t)((timestamp >> 40) & 0xFF);
    signal.timestamp_4 = (uint8_t)((timestamp >> 32) & 0xFF);
    signal.timestamp_5 = (uint8_t)((timestamp >> 24) & 0xFF);
    signal.timestamp_6 = (uint8_t)((timestamp >> 16) & 0xFF);
    signal.timestamp_7 = (uint8_t)((timestamp >> 8) & 0xFF);
    signal.timestamp_8 = (uint8_t)((timestamp) & 0xFF);

    while(1)
    {
        single_send_packet((uint8_t*)&signal, sizeof(data_available_t));

        if(single_receive_packet(DOWNLOAD_DATA, 2000))
        {
            printf("Solicitud de datos del maestro recibida.\n");
            break;
        }

        retry++;

        if(retry == 5)
        {
            printf("Error, no se enviaron los datos al maestro.\n");
            xEventGroupSetBits(task_event_group, TRANSMIT_INIT);
            vTaskDelete(NULL);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    if(eventos && (indice_mpu_buffer_inicial >= pre_buffer))
    {
        indice_mpu_buffer_inicial -= pre_buffer;
    }
    else if(eventos && (indice_mpu_buffer_inicial < pre_buffer))
    {
        indice_mpu_buffer_inicial = indice_mpu_buffer_inicial + MPU_BUFFER_SIZE - pre_buffer;
        indice_mpu_trigger += MPU_BUFFER_SIZE;
    }

    indice_mpu_buffer_transmit = indice_mpu_buffer_inicial;

//    printf("Final: %i. Transmit: %i.\n", indice_mpu_buffer_final, indice_mpu_buffer_transmit);

    config_lora(&lora_config, data_channel);

    while(1)
    {
        if(indice_mpu_buffer_transmit <= indice_mpu_buffer_final)
        {
            if(new_mpu_packet)
            {
                data[0] = DEV_ID | MPU_ID;
                data[1] = (uint8_t)(mpu_packet_num & 0xFF);
                data[2] = (uint8_t)((mpu_packet_num >> 8) & 0xFF);
                mpu_packet_num++;
                new_mpu_packet = 0; 
                data_index = 3;
            }

            size_t indice_lectura;
            if(indice_mpu_buffer_transmit < MPU_BUFFER_SIZE)
            {
                indice_lectura = indice_mpu_buffer_transmit;
            }
            else
            {
                indice_lectura = indice_mpu_buffer_transmit - MPU_BUFFER_SIZE;
            }
            while(indice_lectura == indice_mpu_buffer)
            {
                vTaskDelay(pdMS_TO_TICKS(rate)); 
            }
            item = mpu_buffer[indice_lectura];


            if(eventos && (indice_mpu_buffer_transmit <= indice_mpu_trigger))
            {
                int32_t muestra = indice_mpu_buffer_transmit - indice_mpu_trigger - 1;
                item.timestamp_l = (uint8_t)(muestra & 0xFF);
                item.timestamp_m = (uint8_t)((muestra >> 8) & 0xFF);
                item.timestamp_h = (uint8_t)((muestra >> 16) & 0xFF);
            }

            if (data_index + sizeof(mpu6050_data_t) <= PACKET_SIZE)
            {
                memcpy(&data[data_index], &item, sizeof(mpu6050_data_t));
                data_index += sizeof(mpu6050_data_t);
                indice_mpu_buffer_transmit++;
            }

            if(data_index == PACKET_SIZE)
            {
                single_send_packet(data, sizeof(data)/sizeof(data[0]));

                new_mpu_packet = 1;

                if(retransmit == 1)
                {
                    printf("Paquete perdido reenviado.\n");
                    indice_mpu_buffer_transmit = indice_mpu_buffer_final + 1;
                    retransmit = 0;
                }
            }
        }
        else if(bme_data_count > 0)
        {
            for(size_t i = 0 ; i < bme_data_count ; i++)
            {
                if(new_bme_packet)
                {
                    data[0] = DEV_ID | BME_ID;
                    data[1] = (uint8_t)(bme_packet_num & 0xFF);
                    data[2] = (uint8_t)((bme_packet_num >> 8) & 0xFF);
                    bme_packet_num++;
                    new_bme_packet = 0; 
                    data_index = 3;
                }

                item2 = bme_buffer[i];

                if (data_index + sizeof(bme280_data_t) <= PACKET_SIZE)
                {
                    memcpy(&data[data_index], &item2, sizeof(bme280_data_t));
                    data_index += sizeof(bme280_data_t);
                }

                if(data_index == PACKET_SIZE)
                {
                    single_send_packet(data, sizeof(data)/sizeof(data[0]));

                    new_bme_packet = 1;
                }
            }
            bme_data_count = 0;
        }
        else
        {
            memset(data, 0xFA, sizeof(data));
            data[1] = DEV_ID | MPU_ID;
            retry = 0;

            while(1)
            {
                single_send_packet(data, sizeof(data)/sizeof(data[0]));
                
                if(single_receive_packet(DATA_LOSS, 5000))
                {
                    uint8_t status = readRegister(REG_FIFO);
                    uint8_t packet_num_l = readRegister(REG_FIFO);
                    uint8_t packet_num_h = readRegister(REG_FIFO);
                    retry = 0;
                
                    if(status == 1)
                    {
                        uint16_t packet_num = (uint16_t)((uint16_t)packet_num_h << 8 | (uint16_t)packet_num_l);
                        mpu_packet_num = packet_num;
                        printf("Paquete perdido #%u solicitado.\n", packet_num);
                        indice_mpu_buffer_transmit = indice_mpu_buffer_inicial + (packet_num * 16);
                        retransmit = 1;
                        break;
                    }
                    else
                    {
                        uint8_t data[] = {DEV_ID, ACKNOWLEDGEMENT, 0xFF, 0xFF, 0xFF};
                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                    
                        if (eventos == 0)
                        { 
                            printf("Tarea por tiempo finalizada, pasando a modo standby.\n");
                            config_lora(&lora_config, SIGNAL_CHANNEL);
                            xEventGroupClearBits(task_event_group, MODO_TIEMPO);
                            xEventGroupSetBits(task_event_group, MODO_STANDBY);
                            xEventGroupSetBits(task_event_group, TRANSMIT_INIT);
                            xEventGroupSetBits(task_event_group, TRANSMIT_COMPLETE);
                            xEventGroupClearBits(task_event_group, DATA);
                            init_standby_mode();
                            vTaskDelete(NULL);
                        }
                        else
                        {
                            printf("Tarea por tiempo finalizada, pasando a modo eventos.\n");
                            config_lora(&lora_config, SIGNAL_CHANNEL);
                            xEventGroupClearBits(task_event_group, MODO_TIEMPO);
                            xEventGroupSetBits(task_event_group, MODO_EVENTO);
                            xEventGroupSetBits(task_event_group, TRANSMIT_INIT);
                            xEventGroupSetBits(task_event_group, TRANSMIT_COMPLETE);
                            xEventGroupClearBits(task_event_group, DATA);
                            init_event_mode();
                            vTaskDelete(NULL);
                        }
                    }
                }
                else
                {
                    retry++;

                    if(retry == 5)
                    {
                        printf("Error, no se pudo comunicar con el maestro.\n");
                        config_lora(&lora_config, SIGNAL_CHANNEL);
                        xEventGroupClearBits(task_event_group, MODO_TIEMPO);
                        xEventGroupSetBits(task_event_group, MODO_STANDBY);
                        xEventGroupSetBits(task_event_group, TRANSMIT_INIT);
                        xEventGroupSetBits(task_event_group, TRANSMIT_COMPLETE);
                        xEventGroupClearBits(task_event_group, DATA);
                        init_standby_mode();
                        vTaskDelete(NULL);
                    }
                }
            }
        }  
    }
}

void task_rx(void *p)
{
    i2c_master_dev_handle_t dev_handle_mpu = get_i2c_dev_handle_mpu();

    while(1)
    {
        writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_RX_LENGTH);
        writeRegister(REG_FIFO_ADDR_PTR, 0x00);
        writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
        writeRegister(REG_OP_MODE, 0x8D);

        xEventGroupWaitBits(task_event_group, LORA_IRQ, pdTRUE, pdTRUE, portMAX_DELAY);

        if(send_event_now)
        {
            uint8_t data[] = {BROADCAST_ID, EVENT_DETECTED, 0xFF, 0xFF, 0xFF};
            single_send_packet(data, sizeof(data)/sizeof(data[0]));
            send_event_now = 0;
            continue;
        }

        uint8_t rx_irq_flags = readRegister(REG_IRQ_FLAGS);
        writeRegister(REG_IRQ_FLAGS, rx_irq_flags);

        if((rx_irq_flags & IRQ_RX_DONE_MASK) != 0 && (rx_irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
        {
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
            
            uint8_t dev_id = readRegister(REG_FIFO);
            uint8_t cmd1 = readRegister(REG_FIFO);
            uint8_t cmd2 = readRegister(REG_FIFO);
            uint8_t cmd3 = readRegister(REG_FIFO);
            uint8_t cmd4 = readRegister(REG_FIFO);

            if(dev_id == DEV_ID || dev_id == BROADCAST_ID)
            {
                printf("Paquete recibido:\n");
                printf("%02X-", cmd1);
                printf("%02X-", cmd2);
                printf("%02X-", cmd3);
                printf("%02X\n", cmd4);
                printf("RSSI: %d\n", packetRssi(lora_config));

                switch (cmd1)
                {
                    case MODO_TIEMPO:
                    {
                        EventBits_t uxBits = xEventGroupGetBits(task_event_group);
                        if((uxBits & MODO_EVENTO) != 0)
                        {
                            xEventGroupClearBits(task_event_group, MODO_EVENTO);
                            xEventGroupSetBits(task_event_group, MODO_TIEMPO);
                            get_corrected_time(&start_time);
                            init_time_mode();
                        }
                        else if((uxBits & MODO_STANDBY) != 0)
                        {
                            xEventGroupClearBits(task_event_group, MODO_STANDBY);
                            xEventGroupSetBits(task_event_group, MODO_TIEMPO);
                            tiempo = cmd2;
                            get_corrected_time(&start_time);
                            init_time_mode();
                        }

                        uint8_t data[] = {DEV_ID, ACKNOWLEDGEMENT, 0xFF, 0xFF, 0xFF};
                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                        break;
                    }
                    case MODO_EVENTO:
                    {
                        float umbral = (float)cmd2 / 100.0;
                        umbral_data.umbral_x_pos =  umbral;
                        umbral_data.umbral_x_neg =  -umbral;
                        umbral_data.umbral_y_pos =  umbral;
                        umbral_data.umbral_y_neg =  -umbral;
                        umbral_data.umbral_z_pos =  1 + umbral;
                        umbral_data.umbral_z_neg =  1 - umbral;
                        //printf("Umbral [x_pos:%.4f x_neg:%.4f y_pos:%.4f y_neg:%.4f z_pos:%.4f z_neg:%.4f]\n", umbral_data.umbral_x_pos, umbral_data.umbral_x_neg, umbral_data.umbral_y_pos, umbral_data.umbral_y_neg, umbral_data.umbral_z_pos, umbral_data.umbral_z_neg);

                        EventBits_t uxBits = xEventGroupGetBits(task_event_group);
                        if((uxBits & MODO_EVENTO) == 0)
                        {
                            xEventGroupClearBits(task_event_group, MODO_STANDBY);
                            xEventGroupSetBits(task_event_group, MODO_EVENTO);
                            tiempo = cmd3;
                            init_event_mode();
                        }

                        uint8_t data[] = {DEV_ID, ACKNOWLEDGEMENT, 0xFF, 0xFF, 0xFF};
                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                        break;
                    }
                    case MODO_STANDBY:
                    {
                        EventBits_t uxBits = xEventGroupGetBits(task_event_group);
                        if((uxBits & MODO_STANDBY) == 0)
                        {
                            xEventGroupClearBits(task_event_group, MODO_TIEMPO);
                            xEventGroupClearBits(task_event_group, MODO_EVENTO);
                            xEventGroupClearBits(task_event_group, DATA);
                            xEventGroupSetBits(task_event_group, MODO_STANDBY);
                            init_standby_mode();
                        }

                        uint8_t data[] = {DEV_ID, ACKNOWLEDGEMENT, 0xFF, 0xFF, 0xFF};
                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                        break;
                    }
                    case CONFIG:
                    {
                        rate = cmd2;
                        if((rate == 1) || (rate == 2))
                        {
                            mpu6050_write_byte(dev_handle_mpu, 0x1A, 0x00);
                        }
                        else if((rate == 4) || (rate == 5))
                        {
                            mpu6050_write_byte(dev_handle_mpu, 0x1A, 0x02);
                        }
                        else if(rate == 10)
                        {
                            mpu6050_write_byte(dev_handle_mpu, 0x1A, 0x03);
                        }
                        data_channel =  cmd3;
                        init_standby_mode();
                        uint8_t data[] = {DEV_ID, ACKNOWLEDGEMENT, 0xFF, 0xFF, 0xFF};
                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                        break;
                    }
                    case CLOCK:
                    {
                        uint8_t data[] = {DEV_ID, ACKNOWLEDGEMENT, 0xFF, 0xFF, 0xFF};
                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                        
                        xEventGroupSetBits(task_event_group, CLOCK);
                        xTaskCreatePinnedToCore(&init_time_sync, "time_sync", 4096, &cmd2, 5, NULL, 1);
                        xEventGroupWaitBits(task_event_group, CLOCK_DONE, pdTRUE, pdTRUE, portMAX_DELAY);
                        break;
                    }
                    case DATA:
                    {
                        EventBits_t uxBits = xEventGroupGetBits(task_event_group);
                        if((uxBits & DATA) != 0)
                        {
                            xTaskCreatePinnedToCore(&task_tx, "task_tx", 4096, NULL, 5, NULL, 1);
                            xEventGroupWaitBits(task_event_group, TRANSMIT_INIT, pdTRUE, pdTRUE, portMAX_DELAY);
                        }
                        break;
                    }
                    case STATUS_CODE:
                    {
                        EventBits_t uxBits = xEventGroupGetBits(task_event_group);
                        uint8_t data[] = {DEV_ID, STATUS_CODE, MODO_STANDBY, 0xFF, 0xFF};

                        if((uxBits & MODO_TIEMPO) != 0)
                        {
                            data[2] = MODO_TIEMPO;
                        }
                        else if((uxBits & MODO_EVENTO) != 0)
                        {
                            data[2] = MODO_EVENTO;
                        }

                        if((uxBits & DATA) != 0)
                        {
                            data[3] = DATA_AVAILABLE;
                        }

                        single_send_packet(data, sizeof(data)/sizeof(data[0]));
                        break;
                    }
                    case EVENT_DETECTED:
                    {
                        EventBits_t uxBits = xEventGroupGetBits(task_event_group);

                        if((uxBits & MODO_EVENTO) != 0)
                        {
                            event_now = 1;
                        }
                        break;
                    }
                }
            }
        }
    }
}

void task_mpu(void *p)
{
    i2c_master_dev_handle_t dev_handle_mpu = get_i2c_dev_handle_mpu();
    mpu6050_data_t medicion;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    uint32_t iteraciones = (tiempo * 1000) / rate;
    uint32_t med_num = 0;

    while(1)
    {
        EventBits_t uxBits = xEventGroupGetBits(task_event_group);

        if((uxBits & MODO_STANDBY) != 0)
        {
            printf("Tarea de adquisición por evento terminada.\n");
            vTaskDelete(NULL);
        }
        else if((uxBits & MODO_EVENTO) != 0)
        {
            medicion = mpu6050_read_accel_gyro(dev_handle_mpu);
            medicion.ax = medicion.ax + MPU_OFFSET_AX;
            medicion.ay = -medicion.ay + MPU_OFFSET_AY;
            medicion.az = -medicion.az + MPU_OFFSET_AZ;
            medicion.gx = medicion.gx + MPU_OFFSET_GX;
            medicion.gy = -medicion.gy + MPU_OFFSET_GY;
            medicion.gz = -medicion.gz + MPU_OFFSET_GZ;

            mpu_buffer[indice_mpu_buffer] = medicion;  

            float ax = medicion.ax/A_R;
            float ay = medicion.ay/A_R;
            float az = medicion.az/A_R;

            if((ax > umbral_data.umbral_x_pos) || (ay > umbral_data.umbral_y_pos) || (az > umbral_data.umbral_z_pos) || 
            (ax < umbral_data.umbral_x_neg) || (ay < umbral_data.umbral_y_neg) || (az < umbral_data.umbral_z_neg) || event_now)
            {
                //printf("Accel [X:%.4f Y:%.4f Z:%.4f]\n", ax, ay, az);

                if(event_now == 0)
                {
                    send_event_now = 1;
                    xEventGroupSetBits(task_event_group, LORA_IRQ);
                }
                else
                {
                    event_now = 0;
                }

                get_corrected_time(&start_time);

                indice_mpu_trigger = indice_mpu_buffer;

                xEventGroupClearBits(task_event_group, MODO_EVENTO);
                xEventGroupSetBits(task_event_group, MODO_TIEMPO);

                uxBits = xEventGroupGetBits(task_event_group);
                if((uxBits & CLOCK) == 0)
                {
                    init_time_mode();
                }
            }
            
            indice_mpu_buffer = (indice_mpu_buffer + 1) % MPU_BUFFER_SIZE;
        }
        else if((uxBits & MODO_TIEMPO) != 0)
        {
            if(med_num == 0)
            {
                indice_mpu_buffer_inicial = indice_mpu_buffer;
                indice_mpu_buffer_final = indice_mpu_buffer + iteraciones - 1;
            }

            if(med_num < iteraciones)
            {
                medicion = mpu6050_read_accel_gyro(dev_handle_mpu);
                medicion.ax = medicion.ax + MPU_OFFSET_AX;
                medicion.ay = -medicion.ay + MPU_OFFSET_AY;
                medicion.az = -medicion.az + MPU_OFFSET_AZ;
                medicion.gx = medicion.gx + MPU_OFFSET_GX;
                medicion.gy = -medicion.gy + MPU_OFFSET_GY;
                medicion.gz = -medicion.gz + MPU_OFFSET_GZ;

                medicion.timestamp_l = (int8_t)(med_num & 0xFF);
                medicion.timestamp_m = (int8_t)((med_num >> 8) & 0xFF);
                medicion.timestamp_h = (int8_t)((med_num >> 16) & 0xFF);

                med_num++;

                mpu_buffer[indice_mpu_buffer] = medicion;

                indice_mpu_buffer = (indice_mpu_buffer + 1) % MPU_BUFFER_SIZE;   
            }
            else
            {
                med_num = 0;

                if(eventos)
                {
                    xEventGroupWaitBits(task_event_group, TRANSMIT_COMPLETE, pdTRUE, pdTRUE, portMAX_DELAY);
                    mpu6050_write_byte(dev_handle_mpu, 0x6A, 0x04);
                    continue;
                }
                else
                {
                    vTaskDelete(NULL);
                }
            }
        }
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(rate));
    }
}

void task_bme(void *p)
{
    i2c_master_dev_handle_t dev_handle_bme = get_i2c_dev_handle_bme();
    uint16_t iteraciones;
    uint16_t retardo;

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if((tiempo % 16) != 0)
        {
            iteraciones = 16 - (tiempo % 16) + tiempo;
        }
        else
        {
            iteraciones = tiempo;
        }

        bme_data_count = 0;
        retardo = (tiempo * 1000) / iteraciones;

        while(bme_data_count < iteraciones)
        {
            bme280_read_data(dev_handle_bme, &bme_buffer[bme_data_count]);

            bme_buffer[bme_data_count].timestamp_l = (int8_t)((bme_data_count * retardo) & 0xFF);
            bme_buffer[bme_data_count].timestamp_m = (int8_t)(((bme_data_count * retardo) >> 8) & 0xFF);
            bme_buffer[bme_data_count].timestamp_h = (int8_t)(((bme_data_count * retardo) >> 16) & 0xFF);

            //printf("Temp: %.4f | Hum: %.4f | Pre: %.4f\n", bme_buffer[bme_data_count].temperature, bme_buffer[bme_data_count].humidity, bme_buffer[bme_data_count].pressure);

            bme_data_count++;

            vTaskDelay(pdMS_TO_TICKS(retardo));
        }
    }
}

void init_time_sync(void *p)
{
    uint8_t mode = *(uint8_t*)p;

    led_strip_set_pixel(led_strip, 0, 255, 255, 0);
    led_strip_refresh(led_strip);

    if(mode == 0x01)
    {
        synchronize_clock();
    }
    else if(mode == 0x02)
    {
        calibrate_clock_skew();
    }
    else if(mode == 0x03)
    {
        start_clock_test();
    }
    
    xEventGroupSetBits(task_event_group, CLOCK_DONE);
    xEventGroupClearBits(task_event_group, CLOCK);
    EventBits_t uxBits = xEventGroupGetBits(task_event_group);

    if(eventos && ((uxBits & MODO_TIEMPO) == 0))
    {
        led_strip_set_pixel(led_strip, 0, 255, 0, 0);
        led_strip_refresh(led_strip);
    }
    else if(eventos && ((uxBits & MODO_TIEMPO) != 0))
    {
        led_strip_set_pixel(led_strip, 0, 0, 255, 0);
        led_strip_refresh(led_strip);
    }
    else
    {
        led_strip_set_pixel(led_strip, 0, 0, 0, 255);
        led_strip_refresh(led_strip);
    }
    vTaskDelete(NULL);
}

void init_time_mode(void)
{
    led_strip_set_pixel(led_strip, 0, 0, 255, 0);
    led_strip_refresh(led_strip);
    printf("Iniciado el modo por tiempo durante %u segundos.\n", tiempo);
    xEventGroupSetBits(task_event_group, DATA);

    if(mpu_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(&task_mpu, "task_mpu_time", 4096, NULL, 10, &mpu_task_handle, 0);
    }

    xTaskNotifyGive(bme_task_handle);
}

void init_event_mode(void)
{
    led_strip_set_pixel(led_strip, 0, 255, 0, 0);
    led_strip_refresh(led_strip);
    printf("Iniciado el modo por eventos.\n");
    eventos = 1;
    if(mpu_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(&task_mpu, "task_mpu_event", 4096, NULL, 10, &mpu_task_handle, 0);
    }
//    xTaskCreatePinnedToCore(&task_rx, "task_rx", 4096, NULL, 5, NULL, 1);
}

void init_standby_mode(void)
{
    led_strip_set_pixel(led_strip, 0, 0, 0, 255);
    led_strip_refresh(led_strip);
    mpu_task_handle = NULL;
    printf("Iniciado el modo standby.\n");
    eventos = 0;
//    xTaskCreatePinnedToCore(&task_rx, "task_rx", 4096, NULL, 5, NULL, 1);
}

void app_main()
{
    esp_wifi_stop();
    esp_wifi_deinit();

    printf("Iniciando sensor inteligente #%i.\n", DEV_ID);
    i2c_master_init();
    i2c_master_dev_handle_t dev_handle_mpu = get_i2c_dev_handle_mpu();
    i2c_master_dev_handle_t dev_handle_bme = get_i2c_dev_handle_bme();

    mpu6050_init(dev_handle_mpu);
    bme280_init(dev_handle_bme);

    init_lora(lora_config);
    lora_interrupt_init();

    init_led_strip();
    mpu_buffer_init();
    bme_buffer_init();

    nvs_flash_init();
    load_clock_calibration();

    i2c_mutex = xSemaphoreCreateMutex();

    task_event_group = xEventGroupCreate();
    xEventGroupSetBits(task_event_group, MODO_STANDBY);

    init_standby_mode();
    xTaskCreatePinnedToCore(&task_rx, "task_rx", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&task_bme, "task_bme_event", 4096, NULL, 4, &bme_task_handle, 0);
}