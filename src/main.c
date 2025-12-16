#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "USB.h"
#include "USB_CDC.h"
#include <stdio.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <stdbool.h>
#define ARM_MATH_CM4
#include "arm_math.h"
#include "dsp/transform_functions.h"
#include "dsp/complex_math_functions.h"
#include "dsp/fast_math_functions.h"
#include "dsp/filtering_functions.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

#include "xensiv_bgt60trxx_mtb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define XENSIV_BGT60TRXX_CONF_IMPL
#include "presence_radar_settings.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define USB_CONFIG_DELAY          (50U) /* In milliseconds */

/* Radar 1 Pins */
#define PIN_RADAR1_CS       Radar_CS_1
#define PIN_RADAR1_IRQ      Radar_IRQ_1

/* Radar 2 Pins */
#define PIN_RADAR2_CS       Radar_CS_2
#define PIN_RADAR2_IRQ      Radar_IRQ_2

/* Shared Pins */
#define PIN_RADAR_SPI_SCLK  Radar_SPI_CLK
#define PIN_RADAR_SPI_MOSI  Radar_SPI_MOSI
#define PIN_RADAR_SPI_MISO  Radar_SPI_MISO
#define PIN_RADAR_RST       Radar_RST

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (12000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

#define LED1 P10_3
#define LED2 P10_2

/*******************************************************************************
* Presence Detection Constants & Configuration
********************************************************************************/
/* Speed of light in m/s */
#define RADAR_C                     299792458.0f

/* Radar frequency parameters */
#define RADAR_BANDWIDTH             (XENSIV_BGT60TRXX_CONF_END_FREQ_HZ - XENSIV_BGT60TRXX_CONF_START_FREQ_HZ)

/* Derived constants */
#define PRESENCE_RANGE_RESOLUTION   (RADAR_C / (2.0f * (float32_t)RADAR_BANDWIDTH))
#define PRESENCE_NUM_SAMPLES        XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP  /* 128 */
#define PRESENCE_NUM_CHIRPS         XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME   /* 32 */
#define PRESENCE_MACRO_FFT_SIZE     (PRESENCE_NUM_SAMPLES / 2)  /* 64 range bins */

/* Presence algorithm configuration */
#define PRESENCE_MIN_RANGE_BIN              2       /* Increased to 2 to avoid near-field noise (0.33m) */
#define PRESENCE_MAX_RANGE_BIN              24      /* ~8m range (24 * 0.326m = 7.8m) */
#define PRESENCE_MAX_RANGE_LIMIT_M          8.0f
#define PRESENCE_MACRO_THRESHOLD            2.0f    /* Macro movement threshold (increased for stability) */
#define PRESENCE_MICRO_THRESHOLD            40.0f   /* Micro movement threshold (increased from 25.0f) */
#define PRESENCE_MACRO_COMPARE_INTERVAL_MS  250     /* Compare interval for macro */
#define PRESENCE_MACRO_VALIDITY_MS          1000    /* Macro detection validity */
#define PRESENCE_MICRO_VALIDITY_MS          2000    /* Micro detection validity (reduced from 4000ms) */
#define PRESENCE_MACRO_CONFIRMATIONS        0       /* Required consecutive hits */
#define PRESENCE_MICRO_FFT_SIZE             128     /* Doppler FFT size */
#define PRESENCE_MICRO_COMPARE_IDX          5       /* Doppler bins to sum */
#define PRESENCE_BANDPASS_ENABLED           true    /* Enable bandpass filter */
#define PRESENCE_BANDPASS_NUMTAPS           65      /* FIR filter taps */
#define PRESENCE_BANDPASS_DELAY_MS          490     /* Bandpass stabilization delay */

/*******************************************************************************
* Presence Detection Types
********************************************************************************/
/* Complex float type */
typedef _Complex float cfloat32_t;

/* Presence detection state */
typedef enum {
    PRESENCE_STATE_ABSENCE,
    PRESENCE_STATE_MACRO_PRESENCE,
    PRESENCE_STATE_MICRO_PRESENCE
} presence_state_t;

/* Presence detection result */
typedef struct {
    presence_state_t state;
    int32_t range_bin;
    float32_t range_m;
    float32_t max_macro_value;
    float32_t max_micro_value;
} presence_result_t;

/* Presence detection context for one radar */
typedef struct {
    /* FFT instances */
    arm_rfft_fast_instance_f32 rfft_instance;
    arm_cfft_instance_f32 doppler_fft_instance;
    
    /* Windowing */
    float32_t hamming_window[PRESENCE_NUM_SAMPLES];
    float32_t range_intensity_window[PRESENCE_MACRO_FFT_SIZE];
    
    /* Macro FFT buffers */
    cfloat32_t macro_fft_buffer[PRESENCE_MACRO_FFT_SIZE];
    cfloat32_t last_macro_compare[PRESENCE_MACRO_FFT_SIZE];
    cfloat32_t bandpass_macro_fft_buffer[PRESENCE_MACRO_FFT_SIZE];
    
    /* Bandpass FIR filter instances and states for each range bin */
    arm_fir_instance_f32 bandpass_fir_re[PRESENCE_MAX_RANGE_BIN + 1];
    arm_fir_instance_f32 bandpass_fir_im[PRESENCE_MAX_RANGE_BIN + 1];
    float32_t bandpass_state_re[(PRESENCE_MAX_RANGE_BIN + 1) * (PRESENCE_BANDPASS_NUMTAPS + 1)];
    float32_t bandpass_state_im[(PRESENCE_MAX_RANGE_BIN + 1) * (PRESENCE_BANDPASS_NUMTAPS + 1)];
    
    /* Micro FFT buffer: [micro_fft_size][max_range_bin+1] */
    cfloat32_t micro_fft_buffer[PRESENCE_MICRO_FFT_SIZE * (PRESENCE_MAX_RANGE_BIN + 1)];
    cfloat32_t micro_fft_col_buffer[PRESENCE_MICRO_FFT_SIZE];
    int32_t micro_fft_write_row_idx;
    int32_t micro_fft_calc_col_idx;
    bool micro_fft_ready;
    bool micro_fft_all_calculated;
    
    /* Detection timestamps */
    uint32_t macro_detect_timestamps[PRESENCE_MACRO_FFT_SIZE];
    uint32_t micro_detect_timestamps[PRESENCE_MACRO_FFT_SIZE];
    
    /* State tracking */
    presence_state_t state;
    uint32_t last_macro_compare_ms;
    uint32_t bandpass_initial_time_ms;
    int32_t macro_movement_hit_count;
    int32_t last_macro_reported_idx;
    int32_t last_micro_reported_idx;
    int32_t last_reported_idx;
    bool macro_last_compare_init;
    
    /* Max values for debugging */
    float32_t max_macro;
    int32_t max_macro_idx;
    float32_t max_micro;
    int32_t max_micro_idx;
    
    /* Working buffer for frame processing */
    float32_t frame_buffer[PRESENCE_NUM_SAMPLES];
    float32_t fft_output[PRESENCE_NUM_SAMPLES * 2];  /* For complex FFT output */
} presence_context_t;

/*******************************************************************************
* Bandpass Filter Coefficients (10-35Hz)
* Generated using MATLAB: fir1(64, [10/100 35/100], 'DC-1')
********************************************************************************/
static const float32_t bandpass_coeffs[PRESENCE_BANDPASS_NUMTAPS] = {
    -0.000672018944688787f, 5.40997750800323e-05f, -0.00170551007050673f, 0.000706931294401583f,
    0.000529718080087782f,  0.00403359866465874f,  0.00102443397277923f, 0.00234848093688213f,
    -0.00194992073010673f,  0.00451365295988384f,  0.00312574092180467f, 0.00888191214923986f,
    -0.00340548841703134f, -0.00434494380465395f, -0.0153910491204704f, -0.00133041100723547f,
    -0.00517641595111685f,  0.00200054539528286f, -0.0241426155178683f, -0.0230852875573157f,
    -0.0293254372480552f,   0.0105956968865953f,   0.0175013648649183f,  0.0306608940135099f,
    -0.00856346834860387f,  0.00160778144085906f,  0.0222545709144638f,  0.112213549580022f,
    0.136465963717548f,     0.110216333677660f,   -0.0448122804532963f, -0.174898778170997f,
    0.740136712192538f,    -0.174898778170997f,   -0.0448122804532963f,  0.110216333677660f,
    0.136465963717548f,     0.112213549580022f,    0.0222545709144638f,  0.00160778144085906f,
    -0.00856346834860387f,  0.0306608940135099f,   0.0175013648649183f,  0.0105956968865953f,
    -0.0293254372480552f,  -0.0230852875573157f,  -0.0241426155178683f,  0.00200054539528286f,
    -0.00517641595111685f, -0.00133041100723547f, -0.0153910491204704f, -0.00434494380465395f,
    -0.00340548841703134f,  0.00888191214923986f,  0.00312574092180467f,  0.00451365295988384f,
    -0.00194992073010673f,  0.00234848093688213f,  0.00102443397277923f,  0.00403359866465874f,
    0.000529718080087782f,  0.000706931294401583f, -0.00170551007050673f, 5.40997750800323e-05f,
    -0.000672018944688787f
};

/*******************************************************************************
* Global variables
********************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;

static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor1;
static xensiv_bgt60trxx_mtb_t sensor2;

/* FreeRTOS Synchronization Objects */
static SemaphoreHandle_t xRadar1Sem;
static SemaphoreHandle_t xRadar2Sem;
static SemaphoreHandle_t xSpiMutex;
static SemaphoreHandle_t xStatusMutex;

/* Shared Status */
typedef struct {
    presence_state_t state;
    float32_t range;
} radar_status_t;

static radar_status_t status_r1;
static radar_status_t status_r2;

// static bool sequence_running = true;

/* Allocate enough memory for the radar data frame */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

/* Float frame buffer for presence detection (normalized samples) */
static float32_t frame[NUM_SAMPLES_PER_FRAME];

/* Averaged chirp buffer for presence algorithm */
static float32_t avg_chirp[PRESENCE_NUM_SAMPLES];

/* Presence detection contexts for both radars */
static presence_context_t presence_ctx_1;
static presence_context_t presence_ctx_2;

/* System time counter (milliseconds) */
// static uint32_t system_time_ms = 0; // Replaced by FreeRTOS tick count

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void status_printf(const char *fmt, ...);
static void usb_add_cdc(void);

/* Task Prototypes */
void SystemTask(void *pvParameters);
void RadarTask(void *pvParameters);
void PrintTask(void *pvParameters);

/* Presence detection functions */
static void presence_init(presence_context_t *ctx);
static void presence_reset(presence_context_t *ctx);
static presence_result_t presence_process_frame(presence_context_t *ctx, 
                                                 float32_t *frame_data, 
                                                 uint32_t time_ms);

/* Radar processing */
static void run_presence_detection(xensiv_bgt60trxx_mtb_t *sensor_obj, 
                                   SemaphoreHandle_t sem, 
                                   presence_context_t *ctx,
                                   radar_status_t *status,
                                   uint32_t time_ms);
static cy_rslt_t init_dual_radars(void);

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar1_irq_handler(void *args, cyhal_gpio_event_t event);
void radar2_irq_handler(void *args, cyhal_gpio_event_t event);
#else
void radar1_irq_handler(void *args, cyhal_gpio_irq_event_t event);
void radar2_irq_handler(void *args, cyhal_gpio_irq_event_t event);
#endif

/*********************************************************************
* Information that are used during enumeration
**********************************************************************/
static const USB_DEVICE_INFO usb_deviceInfo = {
    0x058B,                           /* VendorId    */
    0x027D,                           /* ProductId    */
    "Infineon Technologies",          /* VendorName   */
    "CDC Radar Data",                 /* ProductName  */
    "12345678"                        /* SerialNumber */
};

/* Interrupt handlers */
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar1_irq_handler(void *args, cyhal_gpio_event_t event)
#else
void radar1_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xRadar1Sem != NULL) {
        xSemaphoreGiveFromISR(xRadar1Sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar2_irq_handler(void *args, cyhal_gpio_event_t event)
#else
void radar2_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xRadar2Sem != NULL) {
        xSemaphoreGiveFromISR(xRadar2Sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Presence Detection Implementation
********************************************************************************/

/**
 * @brief Generate Hamming window
 */
static void generate_hamming_window(float32_t *win, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        win[i] = 0.54f - 0.46f * arm_cos_f32(2.0f * PI * (float32_t)i / (float32_t)(len - 1));
    }
}

/**
 * @brief Initialize presence detection context
 */
static void presence_init(presence_context_t *ctx)
{
    /* Initialize FFT instances */
    arm_rfft_fast_init_f32(&ctx->rfft_instance, PRESENCE_NUM_SAMPLES);
    arm_cfft_init_f32(&ctx->doppler_fft_instance, PRESENCE_MICRO_FFT_SIZE);
    
    /* Generate Hamming window */
    generate_hamming_window(ctx->hamming_window, PRESENCE_NUM_SAMPLES);
    
    /* Generate range intensity window: 0.2 * (i + 1) */
    for (int32_t i = 0; i < PRESENCE_MACRO_FFT_SIZE; i++) {
        ctx->range_intensity_window[i] = 0.2f * ((float32_t)i + 1.0f);
    }
    
    /* Initialize bandpass FIR filters for each range bin */
    for (int32_t i = 0; i <= PRESENCE_MAX_RANGE_BIN; i++) {
        int32_t state_offset = i * (PRESENCE_BANDPASS_NUMTAPS + 1);
        
        arm_fir_init_f32(&ctx->bandpass_fir_re[i],
                         PRESENCE_BANDPASS_NUMTAPS,
                         (float32_t*)bandpass_coeffs,
                         &ctx->bandpass_state_re[state_offset],
                         1);  /* Block size = 1 */
        
        arm_fir_init_f32(&ctx->bandpass_fir_im[i],
                         PRESENCE_BANDPASS_NUMTAPS,
                         (float32_t*)bandpass_coeffs,
                         &ctx->bandpass_state_im[state_offset],
                         1);  /* Block size = 1 */
    }
    
    /* Reset state */
    presence_reset(ctx);
}

/**
 * @brief Reset presence detection state
 */
static void presence_reset(presence_context_t *ctx)
{
    /* Clear buffers */
    memset(ctx->macro_fft_buffer, 0, sizeof(ctx->macro_fft_buffer));
    memset(ctx->last_macro_compare, 0, sizeof(ctx->last_macro_compare));
    memset(ctx->bandpass_macro_fft_buffer, 0, sizeof(ctx->bandpass_macro_fft_buffer));
    memset(ctx->micro_fft_buffer, 0, sizeof(ctx->micro_fft_buffer));
    memset(ctx->macro_detect_timestamps, 0, sizeof(ctx->macro_detect_timestamps));
    memset(ctx->micro_detect_timestamps, 0, sizeof(ctx->micro_detect_timestamps));
    
    /* Reset state variables */
    ctx->state = PRESENCE_STATE_ABSENCE;
    ctx->micro_fft_write_row_idx = 0;
    ctx->micro_fft_calc_col_idx = PRESENCE_MIN_RANGE_BIN;
    ctx->micro_fft_ready = false;
    ctx->micro_fft_all_calculated = false;
    ctx->last_macro_compare_ms = 0;
    ctx->bandpass_initial_time_ms = 0;
    ctx->macro_movement_hit_count = 0;
    ctx->last_macro_reported_idx = -1;
    ctx->last_micro_reported_idx = -1;
    ctx->last_reported_idx = -1;
    ctx->macro_last_compare_init = false;
    ctx->max_macro = 0.0f;
    ctx->max_macro_idx = -1;
    ctx->max_micro = 0.0f;
    ctx->max_micro_idx = -1;
    
    /* Reinitialize bandpass FIR states */
    memset(ctx->bandpass_state_re, 0, sizeof(ctx->bandpass_state_re));
    memset(ctx->bandpass_state_im, 0, sizeof(ctx->bandpass_state_im));
    
    for (int32_t i = 0; i <= PRESENCE_MAX_RANGE_BIN; i++) {
        int32_t state_offset = i * (PRESENCE_BANDPASS_NUMTAPS + 1);
        
        arm_fir_init_f32(&ctx->bandpass_fir_re[i],
                         PRESENCE_BANDPASS_NUMTAPS,
                         (float32_t*)bandpass_coeffs,
                         &ctx->bandpass_state_re[state_offset],
                         1);
        
        arm_fir_init_f32(&ctx->bandpass_fir_im[i],
                         PRESENCE_BANDPASS_NUMTAPS,
                         (float32_t*)bandpass_coeffs,
                         &ctx->bandpass_state_im[state_offset],
                         1);
    }
}

/**
 * @brief Process one frame for presence detection
 * @param ctx Presence context
 * @param frame_data Averaged chirp data (NUM_SAMPLES floats)
 * @param time_ms Current timestamp in milliseconds
 * @return Presence detection result
 */
static presence_result_t presence_process_frame(presence_context_t *ctx, 
                                                 float32_t *frame_data, 
                                                 uint32_t time_ms)
{
    presence_result_t result = {
        .state = ctx->state,
        .range_bin = ctx->last_reported_idx >= 0 ? ctx->last_reported_idx : 0,
        .range_m = ctx->last_reported_idx >= 0 ? ctx->last_reported_idx * PRESENCE_RANGE_RESOLUTION : 0.0f,
        .max_macro_value = ctx->max_macro,
        .max_micro_value = ctx->max_micro
    };
    
    /* Initialize bandpass delay time on first call */
    if (ctx->bandpass_initial_time_ms == 0) {
        ctx->bandpass_initial_time_ms = time_ms + PRESENCE_BANDPASS_DELAY_MS;
    }
    
    /* ----------------------------------------------------------------
     * Step 1: Apply window and compute Range FFT
     * ---------------------------------------------------------------- */
    
    /* Copy and apply Hamming window */
    for (int32_t i = 0; i < PRESENCE_NUM_SAMPLES; i++) {
        ctx->frame_buffer[i] = frame_data[i] * ctx->hamming_window[i];
    }
    
    /* Compute real FFT */
    arm_rfft_fast_f32(&ctx->rfft_instance, ctx->frame_buffer, ctx->fft_output, 0);
    
    /* Convert to complex format: rfft output is [DC, re1, im1, re2, im2, ...] */
    /* Store in macro_fft_buffer as complex numbers */
    ctx->macro_fft_buffer[0] = ctx->fft_output[0] + I * 0.0f;  /* DC component */
    for (int32_t i = 1; i < PRESENCE_MACRO_FFT_SIZE; i++) {
        float32_t re = ctx->fft_output[2 * i];
        float32_t im = ctx->fft_output[2 * i + 1];
        ctx->macro_fft_buffer[i] = re + I * im;
    }
    
    /* ----------------------------------------------------------------
     * Step 2: Apply Bandpass Filter (optional)
     * ---------------------------------------------------------------- */
    cfloat32_t *active_fft_buffer = ctx->macro_fft_buffer;
    
    if (PRESENCE_BANDPASS_ENABLED) {
        for (int32_t i = 0; i <= PRESENCE_MAX_RANGE_BIN; i++) {
            float32_t in_re = crealf(ctx->macro_fft_buffer[i]);
            float32_t in_im = cimagf(ctx->macro_fft_buffer[i]);
            float32_t out_re, out_im;
            
            arm_fir_f32(&ctx->bandpass_fir_re[i], &in_re, &out_re, 1);
            arm_fir_f32(&ctx->bandpass_fir_im[i], &in_im, &out_im, 1);
            
            ctx->bandpass_macro_fft_buffer[i] = out_re + I * out_im;
        }
        active_fft_buffer = ctx->bandpass_macro_fft_buffer;
    }
    
    /* Initialize last macro compare on first frame */
    if (!ctx->macro_last_compare_init) {
        memcpy(ctx->last_macro_compare, active_fft_buffer, 
               PRESENCE_MACRO_FFT_SIZE * sizeof(cfloat32_t));
        ctx->macro_last_compare_init = true;
    }
    
    /* ----------------------------------------------------------------
     * Step 3: Macro Movement Detection
     * ---------------------------------------------------------------- */
    bool hit = false;
    
    if ((ctx->last_macro_compare_ms + PRESENCE_MACRO_COMPARE_INTERVAL_MS < time_ms) &&
        (time_ms > ctx->bandpass_initial_time_ms)) {
        
        /* Only process if within reasonable time window */
        if (ctx->last_macro_compare_ms + 2 * PRESENCE_MACRO_COMPARE_INTERVAL_MS > time_ms) {
            
            for (int32_t i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
                /* Calculate difference between current and previous FFT */
                cfloat32_t diff = active_fft_buffer[i] - ctx->last_macro_compare[i];
                
                /* Apply range intensity scaling */
                float32_t macro_val = cabsf(diff) * ctx->range_intensity_window[i];
                
                /* Apply bandpass scaling correction */
                if (PRESENCE_BANDPASS_ENABLED) {
                    macro_val = macro_val * (0.5f / 0.45f);
                }
                
                /* Track maximum */
                if (macro_val >= ctx->max_macro) {
                    ctx->max_macro = macro_val;
                    ctx->max_macro_idx = i;
                }
                
                /* Check threshold */
                if (macro_val >= PRESENCE_MACRO_THRESHOLD) {
                    hit = true;
                    ctx->macro_detect_timestamps[i] = time_ms + PRESENCE_MACRO_VALIDITY_MS;
                }
            }
        }
        
        /* Update hit counter */
        if (hit) {
            ctx->macro_movement_hit_count++;
        } else {
            ctx->macro_movement_hit_count = 0;
        }
        
        /* Update last compare buffer */
        memcpy(ctx->last_macro_compare, active_fft_buffer, 
               PRESENCE_MACRO_FFT_SIZE * sizeof(cfloat32_t));
        ctx->last_macro_compare_ms = time_ms;
        
        /* Determine macro movement index */
        int32_t macro_movement_idx = -1;
        if (ctx->macro_movement_hit_count >= PRESENCE_MACRO_CONFIRMATIONS) {
            /* Find first valid range bin */
            for (int32_t i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
                if (time_ms <= ctx->macro_detect_timestamps[i]) {
                    macro_movement_idx = i;
                    break;
                }
            }
        }
        
        /* State transition based on macro detection */
        if (macro_movement_idx != ctx->last_macro_reported_idx) {
            if (macro_movement_idx >= 0) {
                /* Macro presence detected */
                ctx->state = PRESENCE_STATE_MACRO_PRESENCE;
                ctx->last_reported_idx = macro_movement_idx;
                
                result.state = PRESENCE_STATE_MACRO_PRESENCE;
                result.range_bin = macro_movement_idx;
                result.range_m = macro_movement_idx * PRESENCE_RANGE_RESOLUTION;
            } else {
                /* Macro not detected, check micro */
                ctx->state = PRESENCE_STATE_MICRO_PRESENCE;
                ctx->last_micro_reported_idx = -1;
                
                /* Initialize micro timestamps */
                for (int32_t i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
                    if (i >= ctx->last_macro_reported_idx) {
                        ctx->micro_detect_timestamps[i] = time_ms + PRESENCE_MICRO_VALIDITY_MS;
                    } else {
                        ctx->micro_detect_timestamps[i] = 0;
                    }
                }
                ctx->micro_fft_calc_col_idx = PRESENCE_MIN_RANGE_BIN;
            }
            ctx->last_macro_reported_idx = macro_movement_idx;
        }
    }
    
    /* ----------------------------------------------------------------
     * Step 4: Store data for Micro FFT
     * ---------------------------------------------------------------- */
    /* Store current FFT result in micro buffer */
    int32_t micro_buffer_offset = ctx->micro_fft_write_row_idx * (PRESENCE_MAX_RANGE_BIN + 1);
    for (int32_t i = 0; i <= PRESENCE_MAX_RANGE_BIN; i++) {
        ctx->micro_fft_buffer[micro_buffer_offset + i] = ctx->macro_fft_buffer[i];
    }
    ctx->micro_fft_write_row_idx++;
    
    /* Check if micro buffer is full */
    if (ctx->micro_fft_write_row_idx >= PRESENCE_MICRO_FFT_SIZE) {
        ctx->micro_fft_ready = true;
        ctx->micro_fft_write_row_idx = 0;
        ctx->micro_fft_calc_col_idx = PRESENCE_MIN_RANGE_BIN;
    }
    
    /* Skip micro processing if in macro-only states */
    if (ctx->state == PRESENCE_STATE_ABSENCE || 
        ctx->state == PRESENCE_STATE_MACRO_PRESENCE) {
        result.state = ctx->state;
        result.max_macro_value = ctx->max_macro;
        return result;
    }
    
    /* ----------------------------------------------------------------
     * Step 5: Micro Movement Detection (Doppler FFT)
     * ---------------------------------------------------------------- */
    if (ctx->micro_fft_ready) {
        /* Extract column for current range bin */
        cfloat32_t mean = 0.0f + I * 0.0f;
        
        for (int32_t row = 0; row < PRESENCE_MICRO_FFT_SIZE; row++) {
            int32_t actual_row = (row + ctx->micro_fft_write_row_idx) % PRESENCE_MICRO_FFT_SIZE;
            int32_t idx = actual_row * (PRESENCE_MAX_RANGE_BIN + 1) + ctx->micro_fft_calc_col_idx;
            ctx->micro_fft_col_buffer[row] = ctx->micro_fft_buffer[idx];
            mean += ctx->micro_fft_buffer[idx];
        }
        
        /* Mean removal */
        float32_t mean_re = crealf(mean) / (float32_t)PRESENCE_MICRO_FFT_SIZE;
        float32_t mean_im = cimagf(mean) / (float32_t)PRESENCE_MICRO_FFT_SIZE;
        mean = mean_re + I * mean_im;
        
        for (int32_t i = 0; i < PRESENCE_MICRO_FFT_SIZE; i++) {
            ctx->micro_fft_col_buffer[i] -= mean;
        }
        
        /* Perform Doppler FFT */
        arm_cfft_f32(&ctx->doppler_fft_instance, (float32_t*)ctx->micro_fft_col_buffer, 0, 1);
        
        /* Calculate speed score (sum of magnitudes of low Doppler bins) */
        float32_t speed = 0.0f;
        for (int32_t i = 1; i <= PRESENCE_MICRO_COMPARE_IDX; i++) {
            speed += cabsf(ctx->micro_fft_col_buffer[i]);
        }
        
        /* Track maximum */
        if (speed > ctx->max_micro) {
            ctx->max_micro = speed;
            ctx->max_micro_idx = ctx->micro_fft_calc_col_idx;
        }
        
        /* Check threshold */
        if (speed >= PRESENCE_MICRO_THRESHOLD) {
            ctx->micro_detect_timestamps[ctx->micro_fft_calc_col_idx] = 
                time_ms + PRESENCE_MICRO_VALIDITY_MS;
            ctx->state = PRESENCE_STATE_MICRO_PRESENCE;
        }
        
        /* Move to next column */
        ctx->micro_fft_calc_col_idx++;
        if (ctx->micro_fft_calc_col_idx > PRESENCE_MAX_RANGE_BIN) {
            ctx->micro_fft_calc_col_idx = PRESENCE_MIN_RANGE_BIN;
            ctx->micro_fft_all_calculated = true;
        }
    }
    
    /* Determine micro movement state */
    int32_t micro_movement_idx = -1;
    for (int32_t i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
        if (time_ms <= ctx->micro_detect_timestamps[i]) {
            micro_movement_idx = i;
            break;
        }
    }
    
    /* Report micro movement */
    if (micro_movement_idx != ctx->last_micro_reported_idx) {
        ctx->last_micro_reported_idx = micro_movement_idx;
        if (micro_movement_idx >= 0) {
            ctx->last_reported_idx = micro_movement_idx;
            result.state = PRESENCE_STATE_MICRO_PRESENCE;
            result.range_bin = micro_movement_idx;
            result.range_m = micro_movement_idx * PRESENCE_RANGE_RESOLUTION;
        }
    }
    
    /* Check for absence */
    if (micro_movement_idx == -1 && 
        ctx->state == PRESENCE_STATE_MICRO_PRESENCE &&
        ctx->micro_fft_all_calculated) {
        ctx->state = PRESENCE_STATE_ABSENCE;
        ctx->last_micro_reported_idx = -1;
        ctx->micro_fft_all_calculated = false;
        
        result.state = PRESENCE_STATE_ABSENCE;
        result.range_bin = 0;
        result.range_m = 0.0f;
    }
    
    result.max_macro_value = ctx->max_macro;
    result.max_micro_value = ctx->max_micro;
    
    return result;
}

/*******************************************************************************
* Radar Initialization and Processing
********************************************************************************/

static cy_rslt_t init_dual_radars(void) {
    cy_rslt_t rslt;
    
    /* Initialize GPIOs - Both CS pins HIGH (inactive) */
    cyhal_gpio_init(PIN_RADAR1_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(PIN_RADAR2_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(PIN_RADAR_RST, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);

    /* Hard Reset (Shared) - Pulse Low */
    cyhal_gpio_write(PIN_RADAR_RST, 1);
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(PIN_RADAR_RST, 0);
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(PIN_RADAR_RST, 1);
    cyhal_system_delay_ms(10); /* Wait for sensors to boot */

    status_printf("Initializing Radar 1...\r\n");
    cyhal_system_delay_ms(50);
    
    /* Ensure Radar 2 CS is HIGH (inactive) */
    cyhal_gpio_write(PIN_RADAR2_CS, 1);
    
    /* Setup Sensor 1 */
    sensor1.iface.spi = &cyhal_spi;
    sensor1.iface.selpin = PIN_RADAR1_CS;
    sensor1.iface.rstpin = PIN_RADAR_RST;
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
    sensor1.iface.irqpin.pin = NC;
#else
    sensor1.iface.irqpin = NC;
#endif
    
    /* Initialize Sensor 1 (Skip hardware reset as we did it manually) */
    rslt = xensiv_bgt60trxx_init(&sensor1.dev, &sensor1.iface, false);
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_config(&sensor1.dev, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
    }
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_mtb_interrupt_init(&sensor1, NUM_SAMPLES_PER_FRAME, PIN_RADAR1_IRQ, CYHAL_ISR_PRIORITY_DEFAULT, radar1_irq_handler, NULL);
    }
    
    if (rslt != CY_RSLT_SUCCESS) {
        status_printf("Radar 1 init failed: 0x%08lX\r\n", (unsigned long)rslt);
        return rslt;
    }
    
    status_printf("Radar 1 initialized successfully\r\n");
    cyhal_system_delay_ms(50);
    
    /* Ensure Radar 1 CS is HIGH (inactive) before initializing Radar 2 */
    cyhal_gpio_write(PIN_RADAR1_CS, 1);
    cyhal_system_delay_ms(10);
    
    status_printf("Initializing Radar 2...\r\n");
    cyhal_system_delay_ms(50);

    /* Setup Sensor 2 */
    sensor2.iface.spi = &cyhal_spi;
    sensor2.iface.selpin = PIN_RADAR2_CS;
    sensor2.iface.rstpin = PIN_RADAR_RST;
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
    sensor2.iface.irqpin.pin = NC;
#else
    sensor2.iface.irqpin = NC;
#endif

    /* Initialize Sensor 2 */
    rslt = xensiv_bgt60trxx_init(&sensor2.dev, &sensor2.iface, false);
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_config(&sensor2.dev, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
    }
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_mtb_interrupt_init(&sensor2, NUM_SAMPLES_PER_FRAME, PIN_RADAR2_IRQ, CYHAL_ISR_PRIORITY_DEFAULT, radar2_irq_handler, NULL);
    }
    
    if (rslt != CY_RSLT_SUCCESS) {
        status_printf("Radar 2 init failed: 0x%08lX\r\n", (unsigned long)rslt);
        return rslt;
    }
    
    status_printf("Radar 2 initialized successfully\r\n");
    cyhal_system_delay_ms(50);
    
    /* Ensure both CS pins are HIGH (inactive) */
    cyhal_gpio_write(PIN_RADAR1_CS, 1);
    cyhal_gpio_write(PIN_RADAR2_CS, 1);

    return rslt;
}

/**
 * @brief Get state name string
 */
// static const char* get_presence_state_name(presence_state_t state)
// {
//     switch (state) {
//         case PRESENCE_STATE_ABSENCE: return "ABSENCE";
//         case PRESENCE_STATE_MACRO_PRESENCE: return "MACRO";
//         case PRESENCE_STATE_MICRO_PRESENCE: return "MICRO";
//         default: return "UNKNOWN";
//     }
// }

/**
 * @brief Run presence detection on one radar
 */
static void run_presence_detection(xensiv_bgt60trxx_mtb_t *sensor_obj, 
                                   SemaphoreHandle_t sem, 
                                   presence_context_t *ctx,
                                   radar_status_t *status,
                                   uint32_t time_ms) 
{
    /* Start frame acquisition */
    int32_t res = xensiv_bgt60trxx_start_frame(&sensor_obj->dev, true);
    if (res != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("Start frame failed: %ld\r\n", (long)res);
        return;
    }
    
    /* Wait for interrupt with timeout */
    if (xSemaphoreTake(sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
        /* Timeout */
        status_printf("Timeout waiting for interrupt\r\n");
        xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);
        
        /* Update status to indicate failure/absence */
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        status->state = PRESENCE_STATE_ABSENCE; // Default to absence on error
        status->range = 0.0f;
        xSemaphoreGive(xStatusMutex);
        return;
    }
    
    /* Read FIFO data */
    int32_t read_res = xensiv_bgt60trxx_get_fifo_data(&sensor_obj->dev, samples, NUM_SAMPLES_PER_FRAME);
    
    /* Stop frame acquisition */
    xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);

    if (read_res != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("FIFO read failed: %ld\r\n", (long)read_res);
        
        /* Update status to indicate failure/absence */
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        status->state = PRESENCE_STATE_ABSENCE; // Default to absence on error
        status->range = 0.0f;
        xSemaphoreGive(xStatusMutex);
        return;
    }
    
    /* Data preprocessing - convert raw 12-bit ADC values to normalized floats
     * CRITICAL: Must divide by 4096.0 to normalize values to [0, 1) range
     * This matches the Infineon example exactly */
    uint16_t *bgt60_buffer_ptr = samples;
    float32_t *frame_ptr = &frame[0];
    for (int32_t sample = 0; sample < NUM_SAMPLES_PER_FRAME; ++sample)
    {
        *frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4096.0F);
    }
    
    /* Calculate the average of the chirps using Infineon's exact method
     * Data layout with NUM_RX_ANTENNAS=1 is [chirp0_samples][chirp1_samples]... */
    arm_fill_f32(0, avg_chirp, PRESENCE_NUM_SAMPLES);
    
    for (int chirp = 0; chirp < PRESENCE_NUM_CHIRPS; chirp++)
    {
        arm_add_f32(avg_chirp, &frame[PRESENCE_NUM_SAMPLES * chirp], avg_chirp, PRESENCE_NUM_SAMPLES);
    }
    
    arm_scale_f32(avg_chirp, 1.0f / PRESENCE_NUM_CHIRPS, avg_chirp, PRESENCE_NUM_SAMPLES);
    
    /* Process frame for presence detection
     * Note: Mean removal is done internally by the presence algorithm */
    presence_result_t result = presence_process_frame(ctx, avg_chirp, time_ms);
    
    /* Update shared status */
    xSemaphoreTake(xStatusMutex, portMAX_DELAY);
    status->state = result.state;
    status->range = result.range_m;
    xSemaphoreGive(xStatusMutex);
    
    /* Reset max values after each frame */
    ctx->max_macro = 0.0f;
    ctx->max_macro_idx = -1;
    ctx->max_micro = 0.0f;
    ctx->max_micro_idx = -1;
}

/*******************************************************************************
* Main Function
********************************************************************************/

/* Task Definitions */
void SystemTask(void *pvParameters) {
    (void)pvParameters;
    cy_rslt_t result;

    /* Initialize the User LED */
    cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

    /* Initialize the USB stack */
    USBD_Init();
    usb_add_cdc();
    USBD_SetDeviceInfo(&usb_deviceInfo);
    USBD_Start();

    /* Wait for USB configuration with timeout */
    uint32_t timeout = 0;
    while ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
    {
        vTaskDelay(pdMS_TO_TICKS(USB_CONFIG_DELAY));
        timeout++;
        if (timeout > 200)  /* 10 seconds timeout */
        {
            break;
        }
    }

    status_printf("USB configured.\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    status_printf("Dual Radar Presence Detection\r\n");
    status_printf("Range Resolution: %.3f m\r\n", PRESENCE_RANGE_RESOLUTION);
    status_printf("Max Range Bin: %d (%.2f m)\r\n", PRESENCE_MAX_RANGE_BIN, 
                  PRESENCE_MAX_RANGE_BIN * PRESENCE_RANGE_RESOLUTION);

    /* Initialize SPI */
    status_printf("Initializing SPI...\r\n");
    result = cyhal_spi_init(&cyhal_spi,
                            PIN_RADAR_SPI_MOSI,
                            PIN_RADAR_SPI_MISO,
                            PIN_RADAR_SPI_SCLK,
                            NC,
                            NULL,
                            8,
                            CYHAL_SPI_MODE_00_MSB,
                            false);
    
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("SPI initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
        for(;;) {}
    }
    
    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    result = cyhal_spi_set_frequency(&cyhal_spi, XENSIV_BGT60TRXX_SPI_FREQUENCY);
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("SPI frequency setting failed.\r\n");
        for(;;) {}
    }

    /* Initialize dual radars */
    status_printf("Initializing Dual Radars...\r\n");
    result = init_dual_radars();
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("Radar initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
        for(;;) {}
    }
    
    /* Initialize presence detection for both radars */
    status_printf("Initializing Presence Detection...\r\n");
    presence_init(&presence_ctx_1);
    presence_init(&presence_ctx_2);
    
    status_printf("Dual Radars and Presence Detection Initialized.\r\n");
    status_printf("Starting continuous presence detection...\r\n");
    status_printf("Macro Threshold: %.2f, Micro Threshold: %.2f\r\n",
                  PRESENCE_MACRO_THRESHOLD, PRESENCE_MICRO_THRESHOLD);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Create Tasks */
    xTaskCreate(RadarTask, "Radar", 4096, NULL, 2, NULL);
    xTaskCreate(PrintTask, "Print", 1024, NULL, 1, NULL);
    
    /* Delete self */
    vTaskDelete(NULL);
}

void RadarTask(void *pvParameters) {
    (void)pvParameters;
    
    /* Frame period in ms (from config: ~50ms = 20Hz) */
    uint32_t frame_period_ms = (uint32_t)(XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S * 1000.0f);
    uint32_t system_time_ms = 0;

    for(;;) {
        /* Update system time based on frame period, not wall clock */
        system_time_ms += frame_period_ms;
        
        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        
        /* Toggle LED to show activity */
        cyhal_gpio_write(LED1, 1);
        
        // Radar 1
        run_presence_detection(&sensor1, xRadar1Sem, &presence_ctx_1, &status_r1, system_time_ms);
        
        xSemaphoreGive(xSpiMutex);
        
        /* Small delay between radars */
        vTaskDelay(pdMS_TO_TICKS(50));
        
        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        
        // Radar 2
        run_presence_detection(&sensor2, xRadar2Sem, &presence_ctx_2, &status_r2, system_time_ms);
        
        cyhal_gpio_write(LED1, 0);
        
        xSemaphoreGive(xSpiMutex);
        
        vTaskDelay(pdMS_TO_TICKS(frame_period_ms));
    }
}

void PrintTask(void *pvParameters) {
    (void)pvParameters;
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        radar_status_t r1 = status_r1;
        radar_status_t r2 = status_r2;
        xSemaphoreGive(xStatusMutex);
        
        status_printf("--------------------------------------------------\r\n");
        if (r1.state == PRESENCE_STATE_ABSENCE) {
            status_printf("[R1] No human\r\n");
        } else {
            status_printf("[R1] Human detect, Range: %.2f m\r\n", r1.range);
        }
        
        if (r2.state == PRESENCE_STATE_ABSENCE) {
            status_printf("[R2] No human\r\n");
        } else {
            status_printf("[R2] Human detect, Range: %.2f m\r\n", r2.range);
        }
    }
}

int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(P7_1, P7_0, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Create Synchronization Objects */
    xRadar1Sem = xSemaphoreCreateBinary();
    xRadar2Sem = xSemaphoreCreateBinary();
    xSpiMutex = xSemaphoreCreateMutex();
    xStatusMutex = xSemaphoreCreateMutex();
    
    /* Create System Init Task */
    xTaskCreate(SystemTask, "System", 1024, NULL, 3, NULL);
    
    /* Start Scheduler */
    status_printf("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();
    
    for(;;) {}
}

/*******************************************************************************
* USB CDC Functions
********************************************************************************/

static void status_printf(const char *fmt, ...)
{
    if (fmt == NULL) return;

    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    
    /* Check if scheduler is running to use mutex */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // xSemaphoreTake(xStatusMutex, portMAX_DELAY); // Optional: Protect USB write if needed
    }

    if ((USBD_GetState() & USB_STAT_CONFIGURED) == USB_STAT_CONFIGURED) {
        uint32_t len = strlen(buffer);
        /* Non-blocking write - if it fails, we just drop the message */
        USBD_CDC_Write(usb_cdcHandle, (uint8_t *)buffer, len, 0);
    }
    
    /* if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreGive(xStatusMutex);
    } */
}

void usb_add_cdc(void) {
    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    EPBulkIn.Flags          = 0;
    EPBulkIn.InDir          = USB_DIR_IN;
    EPBulkIn.Interval       = 0;
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;
    InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    EPBulkOut.Flags         = 0;
    EPBulkOut.InDir         = USB_DIR_OUT;
    EPBulkOut.Interval      = 0;
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;
    InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

    EPIntIn.Flags           = 0;
    EPIntIn.InDir           = USB_DIR_IN;
    EPIntIn.Interval        = 64;
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;
    InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

    usb_cdcHandle = USBD_CDC_Add(&InitData);
}
