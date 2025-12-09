import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
import re
import numpy as np

def parse_radar_log(filepath):
    """
    Parses a radar log text file directly into a NumPy array.
    Returns: numpy array of shape (Nchirp, Nsample, Nrx)
    """
    chirps = []
    current_samples = []
    
    with open(filepath, 'r', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line: continue

            # Detect Chirp boundaries
            if 'Chirp' in line and ':' in line:
                if current_samples:
                    chirps.append(current_samples)
                    current_samples = []
                continue

            # Detect Sample data
            if 'Sample' in line:
                # Regex to find data inside brackets [x, y, z]
                match = re.findall(r'\[([^\]]+)\]', line)
                if match:
                    # Take the last bracketed group found on the line
                    val_str = match[-1] 
                    try:
                        # specific to your 3-RX format "val1, val2, val3"
                        vals = [float(v) for v in val_str.split(',')]
                        if len(vals) == 3:
                            current_samples.append(vals)
                    except ValueError:
                        continue
                        
    # Append the final chirp
    if current_samples:
        chirps.append(current_samples)

    if not chirps:
        raise ValueError("No valid chirp data found in file.")

    # Convert list -> numpy array
    # Shape becomes (Nchirp, Nsample, Nrx)
    return np.array(chirps, dtype=float)

# %% ------- MODE & PLOTS -------
mode = 'real'           # 'sim' or 'real'
sim_noise       = True
plot_RangeFFT   = True
plot_DopplerFFT = True
plot_RD_map     = True
plot_DBF        = True    # plot beampattern for first detected target

# %% --------- Radar Parameters ---------
fc_start = 61020099000.0
fc_end = 61479903000.0
fc = 0.5 * (fc_start + fc_end)
c = 299792458.0
B = fc_end - fc_start
Tchirp = 6.99625e-05
S = B / Tchirp
fs = 0.72e6        # ADC sampling rate (Hz)
lam = c / fc
d = lam / 2
fr = 1 / Tchirp

# %% --------- SIM Targets (used if mode='sim') ---------
R0        = np.array([1, 4, 7])         # target ranges (m)
v         = np.array([2.5, -1, 0.5])    # velocities (m/s)
theta_deg = np.array([30, -10, 50])     # AoAs (deg)
alpha     = np.array([1, 0.8, 0.5])     # amplitudes
K         = len(R0)
theta     = np.deg2rad(theta_deg)

# %% --------- Build rx_cube ---------
if mode == 'sim':
    Nsample = 128          # samples per chirp
    Nchirp  = 32           # number of chirps
    Nrx     = 2            # number of RX antennas
    Tr      = Tchirp       # PRI ~= Tchirp
    
    # Create axes for broadcasting
    # Shapes: t_fast (1, Nsample, 1), n_chirp (Nchirp, 1, 1), m_rx (1, 1, Nrx)
    t_fast  = np.arange(Nsample).reshape(1, Nsample, 1) / fs
    n_chirp = np.arange(Nchirp).reshape(Nchirp, 1, 1)
    m_rx    = np.arange(Nrx).reshape(1, 1, Nrx)
    
    rx_cube = np.zeros((Nchirp, Nsample, Nrx), dtype=complex)
    
    for k in range(K):
        fb_k = 2 * S * R0[k] / c
        fD_k = 2 * v[k] / lam
        aoa  = theta[k]
        
        phi_r = 2 * np.pi * fb_k * t_fast
        phi_d = 2 * np.pi * fD_k * (n_chirp * Tr)
        phi_a = 2 * np.pi * (m_rx * d * np.sin(aoa) / lam)
        
        # Python broadcasting sums dimensions automatically
        rx_cube += alpha[k] * np.exp(1j * (phi_r + phi_d + phi_a))
        
    if sim_noise:
        SNR_dB = 30
        noise_power = 10**(-SNR_dB / 10)
        # Complex Gaussian noise
        noise = np.sqrt(noise_power / 2) * (
            np.random.randn(*rx_cube.shape) + 1j * np.random.randn(*rx_cube.shape)
        )
        rx_cube += noise

elif mode == 'real':
    # DIRECTLY LOAD FROM TEXT FILE
    # No need for .mat files
    filename = 'frames.txt'  # Update with your actual filename
    
    print(f"Parsing {filename}...")
    rx_cube = parse_radar_log(filename)
    
    # Get dimensions dynamically from the loaded data
    Nchirp, Nsample, Nrx = rx_cube.shape
    Tr = Tchirp # Assuming PRI is roughly Tchirp
    
    print(f"Loaded Cube: {Nchirp} chirps, {Nsample} samples, {Nrx} RX antennas")

    # %% ------- PRE-PROCESSING (Filter -> DC Remove -> Hilbert) -------
    # Filter Parameters
    filter_order = 4
    cutoff_freq = 0.35  # Normalized (0 to 1, where 1 is Nyquist)
    
    # Design Lowpass Filter
    b, a = signal.butter(filter_order, cutoff_freq, btype='low')
    
    # 1. Apply Zero-Phase Lowpass Filter
    # We apply this along axis=1 (the samples/fast-time axis)
    rx_cube = signal.filtfilt(b, a, rx_cube, axis=1)
    
    # 2. Remove DC Offset
    # Calculate mean along axis 1, keeping dimensions to allow broadcasting subtraction
    # Shape becomes (Nchirp, 1, Nrx) so we can subtract it from (Nchirp, Nsample, Nrx)
    rx_cube = rx_cube - np.mean(rx_cube, axis=1, keepdims=True)
    
    # 3. Reconstruct Analytic Signal (Hilbert Transform)
    # This converts real-valued data to complex (I + jQ)
    rx_cube = signal.hilbert(rx_cube, axis=1)

else:
    raise ValueError("Unknown mode. Use 'sim' or 'real'.")

ant_idx = 0   # antenna index to visualize (0-based index)

# %% -------- Range FFT --------
# FFT along fast-time (axis 1)
range_cube = np.fft.fft(rx_cube, n=Nsample, axis=1)
df = fs / Nsample
f_axis = np.arange(Nsample) * df

# Use positive frequencies (skip DC, index 0)
N_half = int(np.floor(Nsample / 2))
# Indices 1 to N_half-1 (Python slicing is exclusive at the end)
range_indices = np.arange(1, N_half) 

# Range calibration from MATLAB script
range_res = (3 * 10**8) / (2 * (fc_end - fc_start))
range_axis = range_indices * range_res

# Range profile (first chirp, selected antenna)
range_prof = range_cube[0, range_indices, ant_idx]

# --- Range peak detection (multi-target) ---
range_mag = np.abs(range_prof)
threshold = np.max(range_mag) * 0.25
# find_peaks returns indices relative to range_mag input
pk_locs, properties = signal.find_peaks(range_mag, height=threshold, distance=2)
pk_vals = properties['peak_heights']
num_targets = len(pk_locs)

# --- Print range-only detection info ---
print(f'\nDetected {num_targets} targets (by range peaks):')
for ti in range(num_targets):
    r_val = range_axis[pk_locs[ti]]
    print(f'  Range peak {ti+1} at {r_val:.2f} m (mag={pk_vals[ti]:.2f})')

# Plot Range FFT
if plot_RangeFFT:
    plt.figure()
    plt.plot(range_axis, 20 * np.log10(np.abs(range_prof)))
    plt.plot(range_axis[pk_locs], 20 * np.log10(pk_vals), 'ro', markersize=8)
    plt.xlabel('Range (m)')
    plt.ylabel('Magnitude (dB)')
    plt.title('Range Profile (Chirp 1, Antenna 1)')
    plt.grid(True)

# %% -------- Doppler FFT --------
doppler_cube = np.zeros((Nchirp, Nsample, Nrx), dtype=complex)
dop_win = np.hanning(Nchirp) 

# Apply window and FFT along slow-time (axis 0)
# Reshape window for broadcasting: (Nchirp, 1, 1)
dop_win_broad = dop_win.reshape(Nchirp, 1, 1)

# Windowing
windowed_cube = range_cube * dop_win_broad

# FFT and Shift
doppler_cube = np.fft.fftshift(np.fft.fft(windowed_cube, n=Nchirp, axis=0), axes=0)

# Frequency Axis
fd_axis = np.arange(-Nchirp/2, Nchirp/2) * (1 / (Nchirp * Tr))
vel_axis = (lam / 2) * fd_axis

# %% -------- Multi-Target Processing (Range + Doppler + AoA) --------
target_results = np.zeros((num_targets, 3)) # [range, vel, AoA]

for ti in range(num_targets):
    # Range bin index relative to the sliced array (range_indices)
    idx_rel = pk_locs[ti]
    
    # Map to absolute range bin in the original FFT matrix
    # range_indices started at index 1
    range_bin_abs = range_indices[idx_rel]
    
    range_val = range_axis[idx_rel]
    
    # Doppler spectrum at this range bin (antenna 0)
    dop_spec = doppler_cube[:, range_bin_abs, ant_idx]
    dop_peak_idx = np.argmax(np.abs(dop_spec))
    vel_val = vel_axis[dop_peak_idx]
    
    # AoA using DBF if we have at least 2 RX
    aoa_val = np.nan
    if Nrx >= 2:
        # Snapshot vector across antennas
        x_vec = doppler_cube[dop_peak_idx, range_bin_abs, :] # Shape (Nrx,)
        
        scan_deg = np.arange(-40, 40.2, 0.2)
        k0 = 2 * np.pi / lam
        m_rx_idx = np.arange(Nrx)
        
        # DBF Scan
        # Create Steering Matrix (Nrx x Nangles)
        # theta in radians
        th_scan = np.deg2rad(scan_deg)
        # Broadcating: (Nrx, 1) * (1, Nangles)
        steering_vecs = np.exp(1j * k0 * d * m_rx_idx[:, None] * np.sin(th_scan[None, :]))
        
        # Beamforming: P = |a^H * x|^2
        # (Nangles x Nrx) dot (Nrx,) -> (Nangles,)
        bf_response = np.abs(np.dot(steering_vecs.conj().T, x_vec))**2
        
        idx_max = np.argmax(bf_response)
        aoa_val = scan_deg[idx_max]
        
    target_results[ti, :] = [range_val, vel_val, aoa_val]

# -------- Print Final Target Table --------
print('\nFinal Target Estimates:')
for ti in range(num_targets):
    print(f'Target {ti+1}: Range = {target_results[ti,0]:.2f} m   '
          f'Vel = {target_results[ti,1]:.2f} m/s   AoA = {target_results[ti,2]:.1f} deg')

# %% -------- Doppler Plot (first detected target) --------
if plot_DopplerFFT and num_targets > 0:
    idx_rel = pk_locs[0]
    range_bin_abs = range_indices[idx_rel]
    dop_spec = doppler_cube[:, range_bin_abs, ant_idx]
    
    plt.figure()
    plt.plot(vel_axis, 20 * np.log10(np.abs(dop_spec)))
    plt.xlabel('Velocity (m/s)')
    plt.ylabel('Magnitude (dB)')
    plt.title(f'Doppler Spectrum at Range ≈ {target_results[0,0]:.2f} m')
    plt.grid(True)

# %% -------- Range-Doppler Map (All Antennas) --------
if plot_RD_map:
    for i_ant in range(Nrx):
        # Slice Doppler cube to match range axis (remove DC, take first half)
        RD = doppler_cube[:, range_indices, i_ant]
        RDdB = 20 * np.log10(np.abs(RD))
        RDdB = RDdB - np.max(RDdB) # Normalize to 0 dB
        
        plt.figure()
        # extent=[xmin, xmax, ymin, ymax]
        # Note: imshow origin is top-left by default, we want 'lower' for standard plots
        plt.imshow(RDdB, aspect='auto', origin='lower', cmap='jet', vmin=-40, vmax=0,
                   extent=[range_axis[0], range_axis[-1], vel_axis[0], vel_axis[-1]])
        
        plt.xlabel('Range (m)')
        plt.ylabel('Velocity (m/s)')
        plt.title(f'Range–Doppler Map (Antenna {i_ant + 1})')
        plt.colorbar()
        
        # Mark detected targets
        for ti in range(num_targets):
            plt.plot(target_results[ti, 0], target_results[ti, 1], 'wx', markersize=10, markeredgewidth=1.5)

# %% -------- DBF Beampattern Plot (first target) --------
if plot_DBF and Nrx >= 2 and num_targets > 0:
    # Use first detected target logic again for plotting
    idx_rel = pk_locs[0]
    range_bin_abs = range_indices[idx_rel]
    vel_val = target_results[0, 1]
    
    # Find nearest Doppler bin
    dop_idx = np.argmin(np.abs(vel_axis - vel_val))
    
    x_vec = doppler_cube[dop_idx, range_bin_abs, :]
    
    # --- CHANGE IS HERE ---
    # Scan from -40 to 40 degrees (40.2 ensures 40 is included)
    scan_deg = np.arange(-40, 40.2, 0.2) 
    # ----------------------

    th_scan = np.deg2rad(scan_deg)
    m_rx_idx = np.arange(Nrx)
    k0 = 2 * np.pi / lam
    
    # Re-calculate pattern for plotting
    steering_vecs = np.exp(1j * k0 * d * m_rx_idx[:, None] * np.sin(th_scan[None, :]))
    P = np.abs(np.dot(steering_vecs.conj().T, x_vec))**2
    P = P / np.max(P) # Normalize
    
    plt.figure()
    plt.plot(scan_deg, 10 * np.log10(P))
    plt.xlabel('Angle (deg)')
    plt.ylabel('Normalized Power (dB)')
    plt.grid(True)
    plt.title(f'DBF Beampattern at Range={target_results[0,0]:.2f} m, Vel={vel_val:.2f} m/s')

plt.show()