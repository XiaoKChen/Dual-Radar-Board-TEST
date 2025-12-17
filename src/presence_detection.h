#ifndef PRESENCE_DETECTION_H
#define PRESENCE_DETECTION_H

#include <stdint.h>
#include <stdbool.h>
#include <complex.h>
#include "arm_math.h"
#include "dsp/transform_functions.h"
#include "dsp/complex_math_functions.h"
#include "dsp/fast_math_functions.h"
#include "dsp/filtering_functions.h"

#include "presence_radar_settings.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

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
* Function Prototypes
********************************************************************************/
void presence_init(presence_context_t *ctx);
presence_result_t presence_process_frame(presence_context_t *ctx, 
                                         float32_t *frame_data, 
                                         uint32_t time_ms);

#endif /* PRESENCE_DETECTION_H */
