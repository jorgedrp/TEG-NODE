#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#define NUM_SYNC_ATTEMPTS 20
#define NUM_BEST_SAMPLES_TO_KEEP 10
#define NUM_SAMPLES 120
#define TEST_SAMPLES 720

#define CMD_TIME_SYNC_REQUEST  0xA1 // ESP32 -> RPi
#define CMD_TIME_SYNC_RESPONSE 0xA2 // RPi -> ESP32

typedef struct {
    int64_t rtt_us;
    int64_t offset_us;
} SyncSample;

typedef struct {
    uint64_t local_us;  // Timestamp local del ESP32 (T4)
    uint64_t server_us; // Timestamp calculado del Servidor
} TimeSample;

typedef struct {
    TimeSample sample;
    int64_t rtt_us;
} SampleWithRTT;

void synchronize_clock(void);
uint64_t send_time_request(void);
int receive_time(uint64_t *t2, uint64_t *t3, uint64_t *t4);
void calibrate_clock_skew(void);
int get_corrected_time(struct timeval *corrected_tv);
void start_clock_test(void);
void load_clock_calibration(void);