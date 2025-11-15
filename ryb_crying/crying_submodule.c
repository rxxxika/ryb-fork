#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <adc.h>
#include <display.h>
#include <fontx.h>

#define IIC_INDEX            IIC0
#define CRYING_IIC_ADDRESS   0x40
#define IIC_DATA_REGISTER    0

#define MIC_ADC_CHANNEL      ADC0 //change accordingly if mic is for ex. ADC1 or ADC5!!
#define SAMPLE_RATE_HZ       8000
#define FRAME_MS             20
#define FRAME_SAMPLES        ((SAMPLE_RATE_HZ * FRAME_MS) / 1000)
#define RING_CAPACITY        32

#define INTENSITY_FLOOR_DB   (-50.0f)
#define CRY_INTENSITY_THR    90

typedef struct {
    int16_t data[RING_CAPACITY][FRAME_SAMPLES];
    size_t head, tail, count;
    pthread_mutex_t mtx;
    pthread_cond_t not_empty, not_full;
} ring_t;

static ring_t g_ring;

#define NUM_I2C_REGS  32
static volatile uint32_t g_regs[NUM_I2C_REGS] = {0};

static volatile int keep_running = 1;
static uint8_t g_last_intensity = 0;
static pthread_mutex_t g_feat_mtx = PTHREAD_MUTEX_INITIALIZER;

static inline int16_t adc_raw_to_s16(uint32_t raw) {
    int centered = (int)raw - 2048;
    return (int16_t)(centered << 4);
}

static float rms_dbfs_int16(const int16_t *x, int n) {
    if (n <= 0) return -120.0f;
    double acc = 0;
    for (int i = 0; i < n; i++) {
        double s = x[i] / 32768.0;
        acc += s * s;
    }
    double rms = sqrt(acc / n);
    if (rms <= 1e-12) return -120.0f;
    return 20.0f * log10(rms);
}

static uint8_t map_to_byte(float x, float xmin, float xmax) {
    if (x < xmin) x = xmin;
    if (x > xmax) x = xmax;
    float t = (x - xmin) / (xmax - xmin);
    int v = (int)(t * 255.f + 0.5f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

static void ring_init(ring_t *rb) {
    rb->head = rb->tail = rb->count = 0;
    pthread_mutex_init(&rb->mtx, NULL);
    pthread_cond_init(&rb->not_empty, NULL);
    pthread_cond_init(&rb->not_full, NULL);
}

static void ring_push(ring_t *rb, const int16_t *frame) {
    pthread_mutex_lock(&rb->mtx);
    while (rb->count == RING_CAPACITY)
        pthread_cond_wait(&rb->not_full, &rb->mtx);

    memcpy(rb->data[rb->head], frame, sizeof(int16_t) * FRAME_SAMPLES);
    rb->head = (rb->head + 1) % RING_CAPACITY;
    rb->count++;

    pthread_cond_signal(&rb->not_empty);
    pthread_mutex_unlock(&rb->mtx);
}

static void ring_pop(ring_t *rb, int16_t *out) {
    pthread_mutex_lock(&rb->mtx);
    while (rb->count == 0)
        pthread_cond_wait(&rb->not_empty, &rb->mtx);

    memcpy(out, rb->data[rb->tail], sizeof(int16_t) * FRAME_SAMPLES);
    rb->tail = (rb->tail + 1) % RING_CAPACITY;
    rb->count--;

    pthread_cond_signal(&rb->not_full);
    pthread_mutex_unlock(&rb->mtx);
}

static void* thread_adc_acquire(void *arg) {
    (void)arg;

    adc_init();

    int16_t frame[FRAME_SAMPLES];

    while (keep_running) {
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            uint32_t raw = adc_read_channel_raw(MIC_ADC_CHANNEL);
            frame[i] = adc_raw_to_s16(raw);
            sleep_msec(1);
        }
        ring_push(&g_ring, frame);
    }

    adc_destroy();
    return NULL;
}

static void* thread_process(void *arg) {
    (void)arg;

    int16_t frame[FRAME_SAMPLES];

    while (keep_running) {
        ring_pop(&g_ring, frame);

        float db = rms_dbfs_int16(frame, FRAME_SAMPLES);
        uint8_t cry_level = map_to_byte(db, INTENSITY_FLOOR_DB, 0.0f);

        pthread_mutex_lock(&g_feat_mtx);
        g_last_intensity = cry_level;
        pthread_mutex_unlock(&g_feat_mtx);

        g_regs[IIC_DATA_REGISTER] = cry_level;
    }
    return NULL;
}

static void* thread_i2c(void *arg) {
    (void)arg;
    while (keep_running) {
        iic_slave_mode_handler(IIC_INDEX);
        sleep_msec(10);
    }
    return NULL;
}

static void* thread_display(void *arg) {
    (void)arg;

    display_t disp;
    display_init(&disp);
    displayFillScreen(&disp, RGB_BLACK);

    FontxFile fx[2];
    InitFontx(fx, "/home/student/ryb-fork/fonts/ILGH16XB.FNT", "");

    displayDrawString(&disp, fx, 10, 20, (uint8_t*)"CRYING SUBMODULE", 16);
    displayDrawString(&disp, fx, 10, 50, (uint8_t*)"Cry Level:", 10);

    char buf[32];

    while (keep_running) {
        pthread_mutex_lock(&g_feat_mtx);
        uint8_t level = g_last_intensity;
        pthread_mutex_unlock(&g_feat_mtx);

        displayDrawFillRect(&disp, 130, 40, 230, 60, RGB_BLACK);
        snprintf(buf, sizeof(buf), "%3u", level);
        displayDrawString(&disp, fx, 130, 50, (uint8_t*)buf, strlen(buf));

        sleep_msec(100);
    }
    return NULL;
}

int main(void) {
    pynq_init();

    //I2C setup ->
    switchbox_init();
    switchbox_set_pin(IO_PMODA1, SWB_IIC0_SCL);
    switchbox_set_pin(IO_PMODA2, SWB_IIC0_SDA);

    iic_init(IIC_INDEX);
    iic_reset(IIC_INDEX);

    iic_set_slave_mode(
        IIC_INDEX,
        CRYING_IIC_ADDRESS,
        (uint32_t*)g_regs,
        NUM_I2C_REGS
    );

    ring_init(&g_ring);

    pthread_t t1, t2, t3, t4;
    pthread_create(&t1, NULL, thread_adc_acquire, NULL);
    pthread_create(&t2, NULL, thread_process,      NULL);
    pthread_create(&t3, NULL, thread_i2c,          NULL);
    pthread_create(&t4, NULL, thread_display,      NULL);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    pthread_join(t3, NULL);
    pthread_join(t4, NULL);

    return 0;
}