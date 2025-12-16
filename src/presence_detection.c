/*
 * presence_detection.c
 *
 *  Created on: Dec 15, 2025
 *      Author: AI Assistant
 */

#include "presence_detection.h"
#include <string.h> // for memset, memcpy
#include <math.h>   // for sqrtf

// 10-35Hz Bandpass Coefficients (58 taps)
// Pre-flipped? The Python code flips them in init: self.coeffs = np.flip(coeffs)
// We will store them in the order used for convolution: h[0], h[1], ...
// Standard convolution: y[n] = sum(x[n-k] * h[k])
// So if we store recent inputs x[n], x[n-1]..., we need h[0], h[1]...
// The Python code: state is [x[n-57] ... x[n]]. coeffs is FLIPPED h.
// dot(state, flipped_h) = x[n-57]*h[57] + ... + x[n]*h[0]
// This matches standard convolution.
// So I will list the coefficients as they appear in the Python source (h), 
// and in my FIR implementation I will multiply x[n-k] with h[k].

static const float32_t BPF_COEFFS[PRESENCE_BPF_TAPS] = {
    -0.000672018944688787f, 5.40997750800323e-05f, -0.00170551007050673f, 0.000706931294401583f,
    0.000529718080087782f,  0.00403359866465874f,  0.00102443397277923f, 0.00234848093688213f,
    -0.00194992073010673f,  0.00451365295988384f,  0.00312574092180467f, 0.00888191214923986f,
    -0.00340548841703134f,  -0.00434494380465395f, -0.0153910491204704f, -0.00133041100723547f,
    -0.00517641595111685f,  0.00200054539528286f,  -0.0241426155178683f, -0.0230852875573157f,
    -0.0293254372480552f,   0.0105956968865953f,   0.0175013648649183f,  0.0306608940135099f,
    -0.00856346834860387f,  0.00160778144085906f,  0.0222545709144638f,  0.112213549580022f,
    0.136465963717548f,     0.110216333677660f,    -0.0448122804532963f, -0.174898778170997f,
    0.740136712192538f,     -0.174898778170997f,   -0.0448122804532963f, 0.110216333677660f,
    0.136465963717548f,     0.112213549580022f,    0.0222545709144638f,  0.00160778144085906f,
    -0.00856346834860387f,  0.0306608940135099f,   0.0175013648649183f,  0.0105956968865953f,
    -0.0293254372480552f,   -0.0230852875573157f,  -0.0241426155178683f, 0.00200054539528286f,
    -0.00517641595111685f,  -0.00133041100723547f, -0.0153910491204704f, -0.00434494380465395f,
    -0.00340548841703134f,  0.00888191214923986f,  0.00312574092180467f, 0.00451365295988384f,
    -0.00194992073010673f,  0.00234848093688213f,  0.00102443397277923f, 0.00403359866465874f,
    0.000529718080087782f,  0.000706931294401583f, -0.00170551007050673f, 5.40997750800323e-05f,
    -0.000672018944688787f
};

// --- Helpers ---

static void init_hamming_window(float32_t* win, uint32_t len) {
    for(uint32_t i=0; i<len; i++) {
        // Hamming: 0.54 - 0.46 * cos(2*pi*n / (M-1))
        win[i] = 0.54f - 0.46f * cosf(2.0f * PI * (float)i / (float)(len - 1));
    }
}

static float32_t fir_process(fir_filter_t* fir, float32_t input) {
    // Shift state (naive implementation, optimize with circular buffer if needed)
    // state[0] is oldest, state[N-1] is newest
    // We actually need state to hold x[n], x[n-1]...
    // Let's store newest at index 0?
    // Efficient shift: memmove?
    // Or just loop downwards.
    
    for (int i = PRESENCE_BPF_TAPS - 1; i > 0; i--) {
        fir->state[i] = fir->state[i-1];
    }
    fir->state[0] = input;
    
    float32_t acc = 0.0f;
    for (int i = 0; i < PRESENCE_BPF_TAPS; i++) {
        acc += fir->state[i] * BPF_COEFFS[i];
    }
    return acc;
}

// --- API ---

void presence_init(presence_detector_t* handle) {
    memset(handle, 0, sizeof(presence_detector_t));
    
    // Init FFT
    arm_rfft_fast_init_f32(&handle->S_rfft, PRESENCE_NUM_SAMPLES_PER_CHIRP);
    arm_cfft_init_f32(&handle->S_cfft, PRESENCE_MICRO_FFT_SIZE); // 128
    
    // Init Windows
    init_hamming_window(handle->macro_fft_win, PRESENCE_NUM_SAMPLES_PER_CHIRP);
    
    // Range Intensity Window: 0.2 * (i + 1)
    for (int i = 0; i < PRESENCE_MACRO_FFT_SIZE; i++) {
        handle->range_intensity_win[i] = 0.2f * ((float)i + 1.0f);
    }
    
    handle->first_run = true;
    handle->state = PRESENCE_STATE_ABSENCE;
    handle->last_reported_idx = -1;
}

void presence_process(presence_detector_t* handle, const float32_t* raw_chirp_data, uint32_t time_ms, presence_result_t* result) {
    
    // 1. Apply Window
    arm_mult_f32(raw_chirp_data, handle->macro_fft_win, handle->temp_fft_input, PRESENCE_NUM_SAMPLES_PER_CHIRP);
    
    // 2. RFFT
    // Output layout: [R0, R(N/2), R1, I1, ..., R(N/2-1), I(N/2-1)]
    arm_rfft_fast_f32(&handle->S_rfft, handle->temp_fft_input, handle->temp_fft_output, 0);
    
    // Extract complex bins into temp buffer or process directly
    // We need bins 0 to 63. 
    // Bin 0: R=Out[0], I=0
    // Bin k (1..63): R=Out[2k], I=Out[2k+1]
    
    // 3. Bandpass Filter (if enabled)
    // We filter bins defined by PRESENCE_MAX_RANGE_BINS_TO_PROCESS (16)
    // Python code: for i in range(max_range_idx)...
    
    float32_t current_macro_fft_re[PRESENCE_MAX_RANGE_BINS_TO_PROCESS];
    float32_t current_macro_fft_im[PRESENCE_MAX_RANGE_BINS_TO_PROCESS];
    
    // Handle startup delay for BPF
    if (handle->bandpass_initial_time_ms == 0) {
        handle->bandpass_initial_time_ms = time_ms + 490;
    }
    
    bool bpf_ready = (time_ms > handle->bandpass_initial_time_ms);
    
    for (int i = 0; i < PRESENCE_MAX_RANGE_BINS_TO_PROCESS; i++) {
        float32_t raw_re, raw_im;
        if (i == 0) {
            raw_re = handle->temp_fft_output[0];
            raw_im = 0;
        } else {
            raw_re = handle->temp_fft_output[2*i];
            raw_im = handle->temp_fft_output[2*i+1];
        }
        
#if PRESENCE_BPF_ENABLED
        current_macro_fft_re[i] = fir_process(&handle->bpf_re[i], raw_re);
        current_macro_fft_im[i] = fir_process(&handle->bpf_im[i], raw_im);
#else
        current_macro_fft_re[i] = raw_re;
        current_macro_fft_im[i] = raw_im;
#endif
    }
    
    // 4. Macro Presence Detection
    
    if (handle->first_run) {
        // Init reference
        for (int i=0; i < PRESENCE_MAX_RANGE_BINS_TO_PROCESS; i++) {
             handle->last_macro_compare[2*i] = current_macro_fft_re[i];
             handle->last_macro_compare[2*i+1] = current_macro_fft_im[i];
        }
        handle->first_run = false;
    }
    
    bool hit = false;
    
    if ((time_ms > handle->macro_last_compare_ms + PRESENCE_MACRO_COMPARE_INTERVAL_MS) && bpf_ready) {
        
        for (int i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
            float32_t diff_re = current_macro_fft_re[i] - handle->last_macro_compare[2*i];
            float32_t diff_im = current_macro_fft_im[i] - handle->last_macro_compare[2*i+1];
            
            // Magnitude of diff
            float32_t diff_mag = sqrtf(diff_re*diff_re + diff_im*diff_im);
            
            float32_t macro_val = diff_mag * handle->range_intensity_win[i];
            
#if PRESENCE_BPF_ENABLED
            macro_val = macro_val * (0.5f / 0.45f);
#endif
            
            if (macro_val >= PRESENCE_MACRO_THRESHOLD) {
                hit = true;
                handle->macro_detect_timestamps[i] = time_ms + PRESENCE_MACRO_VALIDITY_MS;
            }
        }
        
        if (hit) {
            handle->macro_movement_hit_count++;
        } else {
            handle->macro_movement_hit_count = 0;
        }
        
        // Update Reference
        for (int i=0; i < PRESENCE_MAX_RANGE_BINS_TO_PROCESS; i++) {
             handle->last_macro_compare[2*i] = current_macro_fft_re[i];
             handle->last_macro_compare[2*i+1] = current_macro_fft_im[i];
        }
        handle->macro_last_compare_ms = time_ms;
    }
    
    // Update Macro State
    int macro_movement_idx = -1;
    if (handle->macro_movement_hit_count >= PRESENCE_MACRO_CONFIRMATIONS) {
        for (int i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
            if (time_ms <= handle->macro_detect_timestamps[i]) {
                macro_movement_idx = i;
                break;
            }
        }
    }
    
    // 5. Micro Presence Detection
    
    // Add to buffer
    uint32_t buf_idx = handle->micro_buffer_idx;
    for (int i = 0; i < PRESENCE_MAX_RANGE_BINS_TO_PROCESS; i++) {
        handle->micro_buffer[buf_idx][2*i]   = current_macro_fft_re[i];
        handle->micro_buffer[buf_idx][2*i+1] = current_macro_fft_im[i];
    }
    
    handle->micro_buffer_idx = (buf_idx + 1) % PRESENCE_MICRO_FFT_SIZE;
    if (handle->micro_buffer_count < PRESENCE_MICRO_FFT_SIZE) {
        handle->micro_buffer_count++;
    }
    
    // Process Micro if buffer full
    if (handle->micro_buffer_count == PRESENCE_MICRO_FFT_SIZE) {
        
        // Iterate relevant bins
        float32_t max_confidence = 0;
        
        for (int col = PRESENCE_MIN_RANGE_BIN; col <= PRESENCE_MAX_RANGE_BIN; col++) {
            
            // Extract column: [Time][Bin]
            // Calculate Mean
            float32_t mean_re = 0;
            float32_t mean_im = 0;
            
            for (int t = 0; t < PRESENCE_MICRO_FFT_SIZE; t++) {
                // Access circular buffer: start from oldest? Order doesn't matter for magnitude/FFT shift (mostly) but standard is oldest to newest.
                // But since we just do FFT and magnitude, circular shift phase doesn't affect speed score (magnitude).
                // Let's just iterate linear through the buffer.
                float32_t val_re = handle->micro_buffer[t][2*col];
                float32_t val_im = handle->micro_buffer[t][2*col+1];
                mean_re += val_re;
                mean_im += val_im;
            }
            mean_re /= PRESENCE_MICRO_FFT_SIZE;
            mean_im /= PRESENCE_MICRO_FFT_SIZE;
            
            // Fill temp buffer for CFFT
            for (int t = 0; t < PRESENCE_MICRO_FFT_SIZE; t++) {
                handle->temp_doppler_input[2*t]   = handle->micro_buffer[t][2*col] - mean_re;
                handle->temp_doppler_input[2*t+1] = handle->micro_buffer[t][2*col+1] - mean_im;
            }
            
            // Doppler FFT
            arm_cfft_f32(&handle->S_cfft, handle->temp_doppler_input, 0, 1);
            
            // Calculate Speed Score: Sum of magnitudes of bins 1 to N
            // Bins: 0 is DC. 1..N.
            float32_t speed_score = 0;
            for (int k = 1; k <= PRESENCE_MICRO_COMPARE_IDX; k++) {
                float32_t re = handle->temp_doppler_input[2*k];
                float32_t im = handle->temp_doppler_input[2*k+1];
                speed_score += sqrtf(re*re + im*im);
            }
            
            float32_t confidence = speed_score - PRESENCE_MICRO_THRESHOLD;
            if (confidence > 0) {
                handle->micro_detect_timestamps[col] = time_ms + PRESENCE_MICRO_VALIDITY_MS;
                if (confidence > max_confidence) {
                    max_confidence = confidence;
                }
            }
        }
    }
    
    // 6. Determine Final State
    
    // If Macro is active, report Macro
    bool macro_active = false;
    for (int i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
        if (time_ms <= handle->macro_detect_timestamps[i]) {
            macro_active = true;
            break;
        }
    }
    
    if (macro_movement_idx != -1) {
        // Confirmed Macro Movement
        handle->state = PRESENCE_STATE_MACRO_PRESENCE;
        handle->last_reported_idx = macro_movement_idx;
    } else if (macro_active) {
         // Macro timer still valid but no new hit confirmed? 
         // Python logic: 
         // if macro_movement_idx != -1 -> MACRO
         // else if macro_active -> Check Micro? No, Python says:
         // "If Macro is active, we report Macro" (in comments)
         // But code says:
         // if macro_movement_idx != -1 -> return MACRO
         // ...
         // if not macro_active -> Check Micro
         
         // So if macro is active (timestamps valid), we assume macro presence?
         // Actually the Python code: 
         // 1. If hit count >= confirm -> find index -> return MACRO.
         // 2. If NOT returned: process Micro.
         // 3. inside Micro block: if NOT macro_active -> return MICRO.
         
         // So if macro_active is TRUE, but macro_movement_idx is -1 (e.g. lost confirmation count but timer valid?), we stick to last state?
         // In Python, if we fall through (return default), it returns self.state.
         // So if self.state was MACRO and timestamp is valid, it stays MACRO?
         // The timestamps are used to determine "macro_active".
         
         // Let's mimic:
         // If `macro_movement_idx` was found this frame, we switch to MACRO.
         // If not, we process Micro.
         // If Micro finds something AND !macro_active -> switch to MICRO.
         // If neither -> if state was MACRO and macro_active -> stay MACRO?
         // If state was MICRO and micro_idx found -> stay MICRO?
         // The Python logic returns immediately on state transition.
         
         // Let's refine the logic here.
         // The Python code updates timestamps whenever a hit occurs.
    }

    // Determine return value
    if (macro_movement_idx != -1) {
        handle->state = PRESENCE_STATE_MACRO_PRESENCE;
        handle->last_reported_idx = macro_movement_idx;
    } else {
        // Check Micro validity
        int micro_idx = -1;
        
        // Find if any micro timestamp is valid
        for (int i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
            if (time_ms <= handle->micro_detect_timestamps[i]) {
                micro_idx = i;
                break;
            }
        }
        
        // Check if ANY macro timestamp is valid
        bool any_macro_valid = false;
        for (int i = PRESENCE_MIN_RANGE_BIN; i <= PRESENCE_MAX_RANGE_BIN; i++) {
            if (time_ms <= handle->macro_detect_timestamps[i]) {
                any_macro_valid = true;
                break;
            }
        }
        
        if (!any_macro_valid && micro_idx != -1) {
            handle->state = PRESENCE_STATE_MICRO_PRESENCE;
            handle->last_reported_idx = micro_idx;
        } else if (!any_macro_valid && micro_idx == -1) {
            handle->state = PRESENCE_STATE_ABSENCE;
            handle->last_reported_idx = 0;
        }
        // Else: Macro is valid but not confirmed this frame -> Keep previous state (likely MACRO)
    }
    
    result->state = handle->state;
    result->range_bin = (handle->state != PRESENCE_STATE_ABSENCE) ? handle->last_reported_idx : 0;
    result->range_m = result->range_bin * PRESENCE_RANGE_RESOLUTION;
    
    // Store in handle for easy access
    handle->range_m = result->range_m;
}

