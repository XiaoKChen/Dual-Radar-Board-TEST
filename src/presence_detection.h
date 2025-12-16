/*
 * presence_detection.h
 *
 *  Created on: Dec 15, 2025
 *      Author: AI Assistant
 */

#ifndef PRESENCE_DETECTION_H_
#define PRESENCE_DETECTION_H_

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

/*******************************************************************************
* Constants & Configuration
********************************************************************************/
#define PRESENCE_NUM_SAMPLES_PER_CHIRP    (128)
#define PRESENCE_MACRO_FFT_SIZE           (PRESENCE_NUM_SAMPLES_PER_CHIRP / 2) // 64
#define PRESENCE_MICRO_FFT_SIZE           (128)
#define PRESENCE_MAX_RANGE_BINS_TO_PROCESS (16) // Processing bins 0-15

// Configuration
#define PRESENCE_MACRO_THRESHOLD          (2.0f)
#define PRESENCE_MICRO_THRESHOLD          (20.0f)
#define PRESENCE_MIN_RANGE_BIN            (2)
#define PRESENCE_MAX_RANGE_BIN            (15)
#define PRESENCE_MACRO_COMPARE_INTERVAL_MS (250)
#define PRESENCE_MACRO_VALIDITY_MS        (1000)
#define PRESENCE_MICRO_VALIDITY_MS        (4000)
#define PRESENCE_MACRO_CONFIRMATIONS      (0)
#define PRESENCE_MICRO_COMPARE_IDX        (5) // Index in Doppler FFT for speed score
#define PRESENCE_BPF_ENABLED              (1)
#define PRESENCE_BPF_TAPS                 (65)

#define PRESENCE_RANGE_RESOLUTION         (299792458.0f / (2.0f * (61479903000.0f - 61020099000.0f))) // ~0.326m

/*******************************************************************************
* Data Structures
********************************************************************************/

typedef enum {
    PRESENCE_STATE_ABSENCE = 0,
    PRESENCE_STATE_MACRO_PRESENCE,
    PRESENCE_STATE_MICRO_PRESENCE
} presence_state_t;

typedef struct {
    presence_state_t state;
    int32_t range_bin;
    float32_t range_m;
} presence_result_t;

typedef struct {
    float32_t state[PRESENCE_BPF_TAPS];
} fir_filter_t;

typedef struct {
    // FFT Instances
    arm_rfft_fast_instance_f32 S_rfft; // 128 pt RFFT
    arm_cfft_instance_f32 S_cfft;      // 128 pt CFFT (for Doppler)

    // Buffers
    float32_t macro_fft_win[PRESENCE_NUM_SAMPLES_PER_CHIRP];
    float32_t range_intensity_win[PRESENCE_MACRO_FFT_SIZE];
    
    // State
    // Storing complex values interleaved [re, im, re, im...]
    float32_t last_macro_compare[PRESENCE_MACRO_FFT_SIZE * 2]; 
    uint32_t macro_last_compare_ms;
    uint32_t bandpass_initial_time_ms;
    
    // Bandpass Filters (Real and Imag parts for each processed bin)
    fir_filter_t bpf_re[PRESENCE_MAX_RANGE_BINS_TO_PROCESS];
    fir_filter_t bpf_im[PRESENCE_MAX_RANGE_BINS_TO_PROCESS];
    
    // Micro Presence Buffer
    // Circular buffer of complex values for relevant bins
    // [Time Index][Range Bin * 2 (Complex)]
    float32_t micro_buffer[PRESENCE_MICRO_FFT_SIZE][PRESENCE_MAX_RANGE_BINS_TO_PROCESS * 2]; 
    uint32_t micro_buffer_idx; // Current write index
    uint32_t micro_buffer_count; // Number of elements filled
    
    // Timestamps for validity
    uint32_t macro_detect_timestamps[PRESENCE_MACRO_FFT_SIZE];
    uint32_t micro_detect_timestamps[PRESENCE_MACRO_FFT_SIZE]; // Only first 16 used essentially
    
    // Current State
    presence_state_t state;
    int32_t last_reported_idx;
    uint32_t macro_movement_hit_count;
    
    // Last calculated range in meters (for display)
    float32_t range_m;

    bool first_run;
    
    // Temp buffers to avoid stack overflow (optional, but safer on embedded)
    float32_t temp_fft_input[PRESENCE_NUM_SAMPLES_PER_CHIRP];
    float32_t temp_fft_output[PRESENCE_NUM_SAMPLES_PER_CHIRP]; 
    float32_t temp_doppler_input[PRESENCE_MICRO_FFT_SIZE * 2];

} presence_detector_t;

/*******************************************************************************
* API
********************************************************************************/

/**
 * @brief Initialize the presence detector structure.
 * 
 * @param handle Pointer to the detector structure.
 */
void presence_init(presence_detector_t* handle);

/**
 * @brief Process a new frame of chirp data.
 * 
 * @param handle Pointer to the detector structure.
 * @param raw_chirp_data Array of 128 float samples (real).
 * @param time_ms Current system time in milliseconds.
 * @param result Pointer to store the result.
 */
void presence_process(presence_detector_t* handle, const float32_t* raw_chirp_data, uint32_t time_ms, presence_result_t* result);

#endif /* PRESENCE_DETECTION_H_ */

