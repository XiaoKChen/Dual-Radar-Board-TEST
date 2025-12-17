#include "presence_detection.h"
#include <string.h>

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
 * @brief Initialize presence detection context
 */
void presence_init(presence_context_t *ctx)
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
 * @brief Process one frame for presence detection
 * @param ctx Presence context
 * @param frame_data Averaged chirp data (NUM_SAMPLES floats)
 * @param time_ms Current timestamp in milliseconds
 * @return Presence detection result
 */
presence_result_t presence_process_frame(presence_context_t *ctx, 
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
