#include "clock.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lora.h"
#include <time.h>

static const char *TAG = "CLOCK_SYNC";

static double clock_skew = 1.0;
static int64_t clock_offset_us = 0;
extern volatile uint64_t t4_hardware_us;

// Convierte struct timeval a un solo uint64_t de microsegundos
static inline uint64_t tv_to_us(const struct timeval *tv)
{
    return (uint64_t)tv->tv_sec * 1000000L + tv->tv_usec;
}

// Convierte un uint64_t de microsegundos de vuelta a struct timeval
static inline void us_to_tv(uint64_t us, struct timeval *tv)
{
    tv->tv_sec = us / 1000000L;
    tv->tv_usec = us % 1000000L;
}

static uint64_t read_u64_from_fifo(void)
{
    uint8_t b;
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        b = readRegister(REG_FIFO);
        v = (v << 8) | (uint64_t)b;
    }
    return v;
}

int compare_int64(const void *a, const void *b)
{
    int64_t val_a = *(const int64_t *)a;
    int64_t val_b = *(const int64_t *)b;
    if (val_a < val_b) return -1;
    if (val_a > val_b) return 1;
    return 0;
}

esp_err_t save_clock_calibration(double skew)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("clock_sync", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, "clk_skew", &skew, sizeof(double));
    if (err != ESP_OK) ESP_LOGE(TAG, "Fallo al guardar skew");

    //err = nvs_set_i64(nvs_handle, "clk_offset", offset);
    //if (err != ESP_OK) ESP_LOGE(TAG, "Fallo al guardar offset");

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) ESP_LOGE(TAG, "Fallo en commit");

    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Calibración guardada en NVS -> Skew: %.9f", skew);
    return err;
}

void load_clock_calibration(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("clock_sync", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se encontraron datos de calibración previos.");
        return; // Usa valores por defecto
    }

    size_t required_size = sizeof(double); 
    nvs_get_blob(nvs_handle, "clk_skew", &clock_skew, &required_size);
    //nvs_get_i64(nvs_handle, "clk_offset", &clock_offset_us);

    nvs_close(nvs_handle);
}

int wait_server(void)
{
    EventGroupHandle_t event_group = get_lora_event_group();

    writeRegister(REG_PAYLOAD_LENGTH, 0x02);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
    writeRegister(REG_OP_MODE, 0x8D);

    EventBits_t uxBits = xEventGroupWaitBits(event_group, LORA_IRQ_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(6000));

    if((uxBits & LORA_IRQ_BIT) != 0)
    {
        uint8_t rx_irq_flags = readRegister(REG_IRQ_FLAGS);
        writeRegister(REG_IRQ_FLAGS, 0xFF);

        if((rx_irq_flags & IRQ_RX_DONE_MASK) != 0 && (rx_irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
        {
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

            uint8_t dev_id = readRegister(REG_FIFO);
            uint8_t comm = readRegister(REG_FIFO);

            if ((dev_id == DEV_ID) && (comm == CMD_TIME_SYNC_RESPONSE))
            {
                return 1;
            }
            else
            {
                return 2;
            }
        }
    }

    return 0;
}

static void linear_regression(const TimeSample* samples, int n)
{
    if (n < 2)
    {
        ESP_LOGE(TAG, "No hay suficientes muestras para la regresión lineal (%d)", n);
        return;
    }

    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

    uint64_t x0 = samples[0].local_us;
    uint64_t y0 = samples[0].server_us;

    for (int i = 0; i < n; i++)
    {
        double x = (double)(samples[i].local_us - x0);
        double y = (double)(samples[i].server_us - y0);
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double n_double = (double)n;
    double denominator = (n_double * sum_x2 - sum_x * sum_x);

    if (fabs(denominator) < 1e-9)
    {
        ESP_LOGE(TAG, "Error en regresión: denominador es cero.");
        return;
    }

    double m = (n_double * sum_xy - sum_x * sum_y) / denominator;

    double c_relative = (sum_y - m * sum_x) / n_double;

    clock_skew = m;

    clock_offset_us = (int64_t)c_relative + y0 - (int64_t)(m * x0);

    ESP_LOGI(TAG, "Regresión completada. Skew: %.9f, Offset: %lld us", clock_skew, clock_offset_us);
    save_clock_calibration(clock_skew);
}

uint64_t send_time_request(void)
{
    EventGroupHandle_t event_group = get_lora_event_group();

    writeRegister(REG_PAYLOAD_LENGTH, 0x02);
    writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 = TxDone
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_FIFO, DEV_ID);
    writeRegister(REG_FIFO, CMD_TIME_SYNC_REQUEST);

    writeRegister(REG_OP_MODE, 0x8B);

    xEventGroupWaitBits(event_group, LORA_IRQ_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000));

    uint64_t t1_us = esp_timer_get_time();

    writeRegister(REG_IRQ_FLAGS, 0xFF);

    return t1_us;
}

int receive_time(uint64_t *t2, uint64_t *t3, uint64_t *t4)
{

    EventGroupHandle_t event_group = get_lora_event_group();

    writeRegister(REG_PAYLOAD_LENGTH, 0x12);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
    writeRegister(REG_OP_MODE, 0x8D);

    EventBits_t uxBits = xEventGroupWaitBits(event_group, LORA_IRQ_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(2000));

    if ((uxBits & LORA_IRQ_BIT) != 0)
    {
        uint8_t rx_irq_flags = readRegister(REG_IRQ_FLAGS);
        writeRegister(REG_IRQ_FLAGS, 0xFF);

        if((rx_irq_flags & IRQ_RX_DONE_MASK) != 0)
        {
            if((rx_irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
            {
                *t4 = (uint64_t)t4_hardware_us;

                writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

                uint8_t dev_id = readRegister(REG_FIFO);
                uint8_t comm = readRegister(REG_FIFO);

                if ((dev_id == DEV_ID) && (comm == CMD_TIME_SYNC_RESPONSE)) {

                    *t2 = read_u64_from_fifo();
                    *t3 = read_u64_from_fifo();

                    return 1;
                }
            }
        }
    }
    return 0;
}

void synchronize_clock(void)
{
    ESP_LOGI(TAG, "Iniciando proceso de sincronización de reloj (%d intentos)...", NUM_SYNC_ATTEMPTS);

    SyncSample best_samples[NUM_BEST_SAMPLES_TO_KEEP];

    for (int i = 0; i < NUM_BEST_SAMPLES_TO_KEEP; i++)
    {
        best_samples[i].rtt_us = INT64_MAX;
        best_samples[i].offset_us = 0;
    }

    for (int i = 0; i < NUM_SYNC_ATTEMPTS; i++)
    {
        uint64_t t1_us, t2_us, t3_us, t4_us;
        int wait_result = wait_server();
        
        if(wait_result == 1)
        {
            t1_us = send_time_request();
            
            if(!receive_time(&t2_us, &t3_us, &t4_us))
            {
                ESP_LOGW(TAG, "Intento %d/%d: Timeout o paquete de respuesta inválido.", i + 1, NUM_SYNC_ATTEMPTS);
                continue;
            }
        }
        else if(wait_result == 2)
        {
            i -= 1;
            continue;
        }
        else
        {
            ESP_LOGW(TAG, "Intento %d/%d: Timeout o paquete de respuesta inválido.", i + 1, NUM_SYNC_ATTEMPTS);
            continue;
        }

        int64_t round_trip_delay = (t4_us - t1_us) - (t3_us - t2_us);

        if (round_trip_delay < 0)
        {
             ESP_LOGW(TAG, "Intento %d/%d: Delay negativo detectado. Descartando.", i + 1, NUM_SYNC_ATTEMPTS);
             continue;
        }

        int64_t offset = ((int64_t)t2_us - (int64_t)t1_us + (int64_t)t3_us - (int64_t)t4_us) / 2;

        ESP_LOGI(TAG, "Intento %d/%d: Delay = %lld us, Offset = %lld us", i + 1, NUM_SYNC_ATTEMPTS, round_trip_delay, offset);

        int worst_sample_index = -1;
        int64_t max_rtt_in_best = -1;

        for (int j = 0; j < NUM_BEST_SAMPLES_TO_KEEP; j++)
        {
            if (best_samples[j].rtt_us > max_rtt_in_best) 
            {
                max_rtt_in_best = best_samples[j].rtt_us;
                worst_sample_index = j;
            }
        }
        
        if (round_trip_delay < max_rtt_in_best)
        {
            best_samples[worst_sample_index].rtt_us = round_trip_delay;
            best_samples[worst_sample_index].offset_us = offset;
        }
    }

    int64_t offsets_to_sort[NUM_BEST_SAMPLES_TO_KEEP];
    int samples_to_process = 0;
    
    for (int i = 0; i < NUM_BEST_SAMPLES_TO_KEEP; i++)
    {
        if (best_samples[i].rtt_us != INT64_MAX)
        {
            offsets_to_sort[samples_to_process++] = best_samples[i].offset_us;
        }
    }

    if (samples_to_process > 0)
    {
        qsort(offsets_to_sort, samples_to_process, sizeof(int64_t), compare_int64);

        int64_t median_offset;
        if (samples_to_process % 2 == 0)
        {
            median_offset = (offsets_to_sort[samples_to_process / 2 - 1] + offsets_to_sort[samples_to_process / 2]) / 2;
        }
        else
        {
            median_offset = offsets_to_sort[samples_to_process / 2];
        }

        ESP_LOGI(TAG, "Sincronización finalizada. Se procesaron %d muestras.", samples_to_process);
        ESP_LOGI(TAG, "Mediana de los offsets: %lld us.", median_offset);

        struct timeval current_time_tv;
        gettimeofday(&current_time_tv, NULL);

        uint64_t current_time_us = tv_to_us(&current_time_tv);
        uint64_t server_time_us = current_time_us + median_offset;

        clock_offset_us = (int64_t)server_time_us - (int64_t)((double)current_time_us * clock_skew);

    }
    else
    {
        ESP_LOGE(TAG, "Falló la sincronización del reloj. No se recibieron respuestas válidas del servidor.");
    }
}

void calibrate_clock_skew(void)
{
    ESP_LOGI(TAG, "Iniciando calibración del reloj (%d muestras en %d minutos)...", NUM_SAMPLES,  NUM_SAMPLES * 5 / 60);

    TimeSample* samples = (TimeSample*)malloc(NUM_SAMPLES * sizeof(TimeSample));

    if (samples == NULL)
    {
        ESP_LOGE(TAG, "Error al alocar memoria para las muestras.");
        return;
    }

    int successful_samples = 0;

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        uint64_t t1_us, t2_us, t3_us, t4_us;
        int wait_result = wait_server();
        
        if(wait_result == 1)
        {
            t1_us = send_time_request();
            
            if (!receive_time(&t2_us, &t3_us, &t4_us))
            {
                ESP_LOGW(TAG, "Muestra %d/%d: Timeout.", i + 1, NUM_SAMPLES);
            }
            else
            {
                int64_t round_trip_delay = (t4_us - t1_us) - (t3_us - t2_us);

                if (round_trip_delay >= 0)
                {
                    int64_t latency = round_trip_delay / 2;
                    uint64_t server_time_at_t4 = t3_us + latency;

                    samples[successful_samples].local_us = t4_us;
                    samples[successful_samples].server_us = server_time_at_t4;
                    successful_samples++;
                    ESP_LOGI(TAG, "Muestra %d/%d recolectada. Local,Server,RTT: %lli,%llu,%lli", i + 1, NUM_SAMPLES, t4_us, server_time_at_t4,round_trip_delay);
                }
                else
                {
                    ESP_LOGW(TAG, "Muestra %d/%d: Delay negativo. Descartada.", i + 1, NUM_SAMPLES);
                }
            }
        }
        else if(wait_result == 2)
        {
            i -= 1;
            continue;
        }
        else
        {
            ESP_LOGW(TAG, "Muestra %d/%d: Timeout.", i + 1, NUM_SAMPLES);
        }
    }

    linear_regression(samples, successful_samples);

    free(samples); // Liberar la memoria
}

int get_corrected_time(struct timeval *corrected_tv)
{
    if (corrected_tv == NULL)
    {
        return -1;
    }

    uint64_t local_us = esp_timer_get_time();

    //Aplicar la fórmula de la regresión: y = mx + c
    uint64_t corrected_us = (uint64_t)((double)local_us * clock_skew) + clock_offset_us;

    us_to_tv(corrected_us, corrected_tv);
    
    return 0;
}

void start_clock_test(void)
{
    ESP_LOGI(TAG, "Iniciando prueba del reloj (%d muestras en %d minutos)...", TEST_SAMPLES, TEST_SAMPLES / 12);

    for (int i = 0; i < TEST_SAMPLES; i++)
    {
        uint64_t t1_us, t2_us, t3_us, t4_us;
        int wait_result = wait_server();

        if(wait_result == 1)
        {
            t1_us = send_time_request();
            
            if(!receive_time(&t2_us, &t3_us, &t4_us))
            {
                ESP_LOGW(TAG, "Intento %d/%d: Timeout o paquete de respuesta inválido.", i + 1, TEST_SAMPLES);
                continue;
            }
        }
        else if(wait_result == 2)
        {
            i -= 1;
            continue;
        }
        else
        {
            ESP_LOGW(TAG, "Intento %d/%d: Timeout o paquete de respuesta inválido.", i + 1, TEST_SAMPLES);
            continue;
        }
        
        int64_t round_trip_delay = (t4_us - t1_us) - (t3_us - t2_us);

        if (round_trip_delay >= 0)
        {
            int64_t latency = round_trip_delay / 2;
            uint64_t server_time_at_t4 = t3_us + latency;
            uint64_t corrected_t4 = (uint64_t)((double)t4_us * clock_skew) + clock_offset_us;

            ESP_LOGI(TAG, "Local,Corrected,Server: %llu,%llu,%llu", t4_us, corrected_t4, server_time_at_t4);
        }
    }
}