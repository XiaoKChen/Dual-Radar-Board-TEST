import numpy as np
import collections
import time

# ==============================================================================
# Configuration & Constants
# ==============================================================================

# Radar Hardware Config (from presence_radar_settings.h)
START_FREQ_HZ = 61020099000
END_FREQ_HZ = 61479903000
NUM_SAMPLES_PER_CHIRP = 128
NUM_CHIRPS_PER_FRAME = 32
FRAME_PERIOD_SEC = 0.05  # 20Hz (approx from 0.0499562)

# Derived Constants
BANDWIDTH = END_FREQ_HZ - START_FREQ_HZ
RANGE_RESOLUTION = 299792458.0 / (2.0 * BANDWIDTH)
MACRO_FFT_SIZE = NUM_SAMPLES_PER_CHIRP // 2

# Algorithm Config (from presence_settings.h and user edits)
CONFIG = {
    'macro_threshold': 2.0,           # User updated
    'micro_threshold': 20.0,          # User updated
    'min_range_bin': 1,
    'max_range_bin': 15,              # User updated
    'macro_compare_interval_ms': 250,
    'macro_movement_validity_ms': 1000,
    'micro_movement_validity_ms': 4000,
    'macro_movement_confirmations': 0,
    'macro_trigger_range': 1,
    'micro_fft_size': 128,
    'micro_movement_compare_idx': 5,
    'macro_fft_bandpass_filter_enabled': True, # User updated
    'mode': 'micro_if_macro'
}

# Bandpass Filter Coefficients (from xensiv_radar_presence.c)
# 10-35Hz Bandpass
BPF_COEFFS = np.array([
    -0.000672018944688787, 5.40997750800323e-05, -0.00170551007050673, 0.000706931294401583,
    0.000529718080087782,  0.00403359866465874,  0.00102443397277923, 0.00234848093688213,
    -0.00194992073010673,  0.00451365295988384,  0.00312574092180467, 0.00888191214923986,
    -0.00340548841703134,  -0.00434494380465395, -0.0153910491204704, -0.00133041100723547,
    -0.00517641595111685,  0.00200054539528286,  -0.0241426155178683, -0.0230852875573157,
    -0.0293254372480552,   0.0105956968865953,   0.0175013648649183,  0.0306608940135099,
    -0.00856346834860387,  0.00160778144085906,  0.0222545709144638,  0.112213549580022,
    0.136465963717548,     0.110216333677660,    -0.0448122804532963, -0.174898778170997,
    0.740136712192538,     -0.174898778170997,   -0.0448122804532963, 0.110216333677660,
    0.136465963717548,     0.112213549580022,    0.0222545709144638,  0.00160778144085906,
    -0.00856346834860387,  0.0306608940135099,   0.0175013648649183,  0.0105956968865953,
    -0.0293254372480552,   -0.0230852875573157,  -0.0241426155178683, 0.00200054539528286,
    -0.00517641595111685,  -0.00133041100723547, -0.0153910491204704, -0.00434494380465395,
    -0.00340548841703134,  0.00888191214923986,  0.00312574092180467, 0.00451365295988384,
    -0.00194992073010673,  0.00234848093688213,  0.00102443397277923, 0.00403359866465874,
    0.000529718080087782,  0.000706931294401583, -0.00170551007050673, 5.40997750800323e-05,
    -0.000672018944688787
], dtype=np.float32)

# ==============================================================================
# Helper Classes
# ==============================================================================

class FIRFilter:
    """
    Stateful FIR Filter implementation to match arm_fir_f32.
    Maintains state between calls.
    """
    def __init__(self, coeffs, block_size=1):
        self.coeffs = np.flip(coeffs) # Convolution flips kernel
        self.num_taps = len(coeffs)
        self.state = np.zeros(self.num_taps + block_size - 1, dtype=np.complex64)
        
    def process(self, x):
        # x is a scalar or block of samples. The C code uses block_size=1
        # Shift state
        self.state[:-1] = self.state[1:]
        self.state[-1] = x
        
        # Compute dot product (convolution)
        # For a single sample input (block_size=1), it's just dot product of state and coeffs
        # The state buffer in CMSIS DSP is [x[n-(numTaps-1)], ..., x[n]]
        # We need to be careful with indices.
        # Let's use numpy convolve 'valid' mode on the state buffer
        
        # Simplified for block_size=1:
        # Output y[n] = sum(b[k] * x[n-k])
        return np.dot(self.state[:self.num_taps], self.coeffs)

class RadarPresenceDetector:
    def __init__(self):
        # 1. Initialization
        self.macro_fft_win = np.hamming(NUM_SAMPLES_PER_CHIRP).astype(np.float32)
        
        # Range Intensity Window (0.2 * (i + 1))
        self.range_intensity_win = 0.2 * (np.arange(MACRO_FFT_SIZE, dtype=np.float32) + 1.0)
        
        # State Variables
        self.last_macro_compare = np.zeros(MACRO_FFT_SIZE, dtype=np.complex64)
        self.macro_last_compare_ms = 0
        self.bandpass_initial_time_ms = 0
        
        # Bandpass Filters (One per range bin)
        # We need separate filters for Real and Imaginary parts? 
        # The C code has `macro_fft_bandpass_fir_re_instances` and `_im_instances`.
        # Yes, filtering complex signal component-wise.
        self.bpf_re = [FIRFilter(BPF_COEFFS) for _ in range(MACRO_FFT_SIZE)]
        self.bpf_im = [FIRFilter(BPF_COEFFS) for _ in range(MACRO_FFT_SIZE)]
        
        # Micro Presence Buffers
        # Buffer size: [micro_fft_size, max_range_bin + 1] (or MACRO_FFT_SIZE to be safe)
        # C code uses `max_range_limit_idx` which is derived from 5m limit.
        # Let's use MACRO_FFT_SIZE for simplicity, or calculate limit.
        self.max_range_idx = int(np.floor(5.0 / RANGE_RESOLUTION)) # ~15 bins
        self.micro_fft_buffer = collections.deque(maxlen=CONFIG['micro_fft_size'])
        
        # Timestamps & State
        self.macro_detect_timestamps = np.zeros(MACRO_FFT_SIZE, dtype=np.float32)
        self.micro_detect_timestamps = np.zeros(MACRO_FFT_SIZE, dtype=np.float32)
        self.state = "ABSENCE"
        self.last_reported_idx = -1
        self.macro_movement_hit_count = 0
        
        # Initial fill flag
        self.first_run = True

    def process_frame(self, raw_chirp_data, time_ms):
        """
        Process a single frame of raw radar data.
        raw_chirp_data: Array of 128 floats (averaged chirp).
        time_ms: Current timestamp in milliseconds.
        """
        
        # ---------------------------------------------------------
        # 1. Range FFT
        # ---------------------------------------------------------
        # Apply Window
        windowed_data = raw_chirp_data * self.macro_fft_win
        
        # FFT (Real to Complex)
        # numpy.fft.rfft returns N/2 + 1 coeffs. We take first N/2.
        fft_out = np.fft.rfft(windowed_data)
        macro_fft_buffer = fft_out[:MACRO_FFT_SIZE]
        
        # ---------------------------------------------------------
        # 2. Bandpass Filter (Optional)
        # ---------------------------------------------------------
        if CONFIG['macro_fft_bandpass_filter_enabled']:
            filtered_fft = np.zeros_like(macro_fft_buffer)
            
            # Initialize start time on first run
            if self.bandpass_initial_time_ms == 0:
                self.bandpass_initial_time_ms = time_ms + 490 # Delay
                
            for i in range(self.max_range_idx):
                re = self.bpf_re[i].process(macro_fft_buffer[i].real)
                im = self.bpf_im[i].process(macro_fft_buffer[i].imag)
                filtered_fft[i] = re + 1j * im
            
            # Use filtered data for macro detection
            current_macro_fft = filtered_fft
        else:
            current_macro_fft = macro_fft_buffer

        # ---------------------------------------------------------
        # 3. Macro Presence Detection
        # ---------------------------------------------------------
        if self.first_run:
            self.last_macro_compare = current_macro_fft.copy()
            self.first_run = False
            
        hit = False
        
        # Check interval (250ms)
        if (time_ms > self.macro_last_compare_ms + CONFIG['macro_compare_interval_ms']) and \
           (time_ms > self.bandpass_initial_time_ms):
            
            # Loop through range bins
            for i in range(CONFIG['min_range_bin'], CONFIG['max_range_bin'] + 1):
                diff = current_macro_fft[i] - self.last_macro_compare[i]
                
                # Scale window
                macro_val = np.abs(diff) * self.range_intensity_win[i]
                
                # Bandpass scaling correction
                if CONFIG['macro_fft_bandpass_filter_enabled']:
                    macro_val = macro_val * (0.5 / 0.45)
                
                # Threshold Check
                if macro_val >= CONFIG['macro_threshold']:
                    hit = True
                    self.macro_detect_timestamps[i] = time_ms + CONFIG['macro_movement_validity_ms']
            
            # Update Hit Count
            if hit:
                self.macro_movement_hit_count += 1
            else:
                self.macro_movement_hit_count = 0
                
            # Update Reference
            self.last_macro_compare = current_macro_fft.copy()
            self.macro_last_compare_ms = time_ms
            
            # Determine Macro State
            macro_movement_idx = -1
            if self.macro_movement_hit_count >= CONFIG['macro_movement_confirmations']:
                # Find closest bin with valid timestamp
                for i in range(CONFIG['min_range_bin'], CONFIG['max_range_bin'] + 1):
                    if time_ms <= self.macro_detect_timestamps[i]:
                        macro_movement_idx = i
                        break
            
            # State Transition
            if macro_movement_idx != -1:
                self.state = "MACRO_PRESENCE"
                self.last_reported_idx = macro_movement_idx
                return {
                    'state': 'MACRO_PRESENCE',
                    'range_bin': macro_movement_idx,
                    'range_m': macro_movement_idx * RANGE_RESOLUTION
                }

        # ---------------------------------------------------------
        # 4. Micro Presence Detection
        # ---------------------------------------------------------
        # Add current frame to buffer (using raw FFT or filtered? Code uses macro_fft_buffer which is filtered if enabled)
        # Actually code says: 
        # cfloat32_t* macro_fft_buffer = handle->config.macro_fft_bandpass_filter_enabled ? handle->bandpass_macro_fft_buffer : handle->macro_fft_buffer;
        # So yes, it uses the filtered data for micro detection too.
        
        self.micro_fft_buffer.append(current_macro_fft)
        
        # Only process if buffer is full
        if len(self.micro_fft_buffer) == CONFIG['micro_fft_size']:
            
            # We need to process one column (range bin) at a time?
            # The C code processes one column per frame to save CPU.
            # In Python we can process all relevant columns at once or just iterate.
            # Let's iterate all relevant bins.
            
            micro_detected = False
            detected_bin = -1
            max_confidence = 0
            
            # Convert buffer to numpy array for easy slicing: [Time, Range]
            micro_data = np.array(self.micro_fft_buffer)
            
            for col in range(CONFIG['min_range_bin'], CONFIG['max_range_bin'] + 1):
                # Extract column
                col_data = micro_data[:, col]
                
                # Mean Removal
                col_data = col_data - np.mean(col_data)
                
                # Doppler FFT
                doppler_fft = np.fft.fft(col_data)
                
                # Calculate Speed Score (Sum of magnitudes of bins 1 to N)
                # Bin 0 is DC (already removed mean, but good to skip).
                speed_score = np.sum(np.abs(doppler_fft[1 : CONFIG['micro_movement_compare_idx'] + 1]))
                
                # Threshold
                confidence = speed_score - CONFIG['micro_threshold']
                
                if confidence > 0:
                    self.micro_detect_timestamps[col] = time_ms + CONFIG['micro_movement_validity_ms']
                    if confidence > max_confidence:
                        max_confidence = confidence
                        detected_bin = col
                        micro_detected = True

            # Logic to switch to Micro Presence
            # If we are not in Macro Presence (or Macro expired)
            # Check if we have valid micro timestamps
            
            # Simple logic: If Macro is active, we report Macro.
            # If Macro is NOT active, we check Micro.
            
            # Check if any Macro is still valid
            macro_active = False
            for i in range(CONFIG['min_range_bin'], CONFIG['max_range_bin'] + 1):
                if time_ms <= self.macro_detect_timestamps[i]:
                    macro_active = True
                    break
            
            if not macro_active:
                # Check Micro
                micro_idx = -1
                for i in range(CONFIG['min_range_bin'], CONFIG['max_range_bin'] + 1):
                    if time_ms <= self.micro_detect_timestamps[i]:
                        micro_idx = i
                        break
                
                if micro_idx != -1:
                    self.state = "MICRO_PRESENCE"
                    return {
                        'state': 'MICRO_PRESENCE',
                        'range_bin': micro_idx,
                        'range_m': micro_idx * RANGE_RESOLUTION
                    }
                else:
                    self.state = "ABSENCE"
                    return {
                        'state': 'ABSENCE',
                        'range_bin': 0,
                        'range_m': 0.0
                    }

        # Default return if no state change or waiting
        return {
            'state': self.state,
            'range_bin': self.last_reported_idx if self.state != "ABSENCE" else 0,
            'range_m': (self.last_reported_idx * RANGE_RESOLUTION) if self.state != "ABSENCE" else 0.0
        }

    def process_next_frame(self, raw_chirp_data):
        """
        Helper for continuous data with fixed frame rate (e.g. reading from a file).
        Automatically increments time by FRAME_PERIOD_SEC * 1000 (50ms).
        """
        if not hasattr(self, '_auto_time_ms'):
            self._auto_time_ms = 0
            
        # Increment time
        step = int(FRAME_PERIOD_SEC * 1000)
        self._auto_time_ms += step
        
        return self.process_frame(raw_chirp_data, self._auto_time_ms)

# ==============================================================================
# Example Usage
# ==============================================================================
def run_simulation_demo():
    """Simulates data with a moving target"""
    detector = RadarPresenceDetector()
    print("Simulating Radar Data (Synthetic)...")
    for frame_idx in range(200):
        # Fake Chirp: Random noise
        raw_data = np.random.normal(0, 0.1, NUM_SAMPLES_PER_CHIRP).astype(np.float32)
        
        # Inject a "Target" at Bin 5 (approx 1.6m) with some movement (phase shift)
        if frame_idx > 50:
            t = np.arange(NUM_SAMPLES_PER_CHIRP)
            freq = 5
            phase = frame_idx * 0.1 
            signal = 0.5 * np.cos(2 * np.pi * freq * t / NUM_SAMPLES_PER_CHIRP + phase)
            raw_data += signal.astype(np.float32)
            
        # Use the auto-increment helper
        result = detector.process_next_frame(raw_data)
        
        if frame_idx % 10 == 0:
            print(f"Frame: {frame_idx} | Time: {detector._auto_time_ms}ms | State: {result['state']} | Range: {result['range_m']:.2f}m")

def run_realtime_demo():
    """Simulates real-time processing using system clock"""
    detector = RadarPresenceDetector()
    print("\nStarting Real-Time Demo (Press Ctrl+C to stop)...")
    print("This simulates receiving data at 20Hz and using system time.")
    
    try:
        while True:
            # 1. Get Data (Replace this with your serial/socket read)
            # raw_data = my_serial.read_chirp()
            raw_data = np.random.normal(0, 0.1, NUM_SAMPLES_PER_CHIRP).astype(np.float32)
            
            # 2. Get Current Time in MS
            # Using monotonic clock is best for intervals
            current_time_ms = int(time.monotonic() * 1000)
            
            # 3. Process
            result = detector.process_frame(raw_data, current_time_ms)
            
            print(f"Time: {current_time_ms} | State: {result['state']} | Range: {result['range_m']:.2f}m")
            
            # 4. Wait for next frame (Simulating 20Hz hardware rate)
            time.sleep(FRAME_PERIOD_SEC)
            
    except KeyboardInterrupt:
        print("Stopped.")

if __name__ == "__main__":
    # Uncomment the one you want to run:
    run_simulation_demo()
    # run_realtime_demo()
