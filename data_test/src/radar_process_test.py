import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
import scipy.ndimage as ndimage
import re
import os


# --- 1. Data Parsing ---
def parse_frames_txt(filepath):
    """
    Parses frames.txt to extract radar data.
    Returns a list of frames, where each frame is (Nchirp, Nsample, Nrx).
    """
    frames = []
    current_frame_chirps = []
    current_chirp_samples = []

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("Frame"):
                if current_frame_chirps:
                    if current_chirp_samples:
                        current_frame_chirps.append(np.array(current_chirp_samples))
                        current_chirp_samples = []
                    frames.append(np.array(current_frame_chirps))
                    current_frame_chirps = []
                current_chirp_samples = []
                continue

            if line.startswith("Chirp"):
                if current_chirp_samples:
                    current_frame_chirps.append(np.array(current_chirp_samples))
                    current_chirp_samples = []
                continue

            if line.startswith("Sample"):
                match = re.search(r"\[(.*?)\]", line)
                if match:
                    vals = [float(x) for x in match.group(1).split(",")]
                    current_chirp_samples.append(vals)

    if current_chirp_samples:
        current_frame_chirps.append(np.array(current_chirp_samples))
    if current_frame_chirps:
        frames.append(np.array(current_frame_chirps))

    return frames


def detect_peaks(rd_map_db, range_axis, v_axis, threshold_db=None, neighborhood_size=5):
    """
    Finds local maxima in the Range-Doppler map.
    Returns a list of tuples: (range, velocity, power_db)
    """
    # Default threshold: 15 dB below the global maximum of this frame
    if threshold_db is None:
        threshold_db = np.max(rd_map_db) - 15

    # Find local maxima using a maximum filter
    data_max = ndimage.maximum_filter(rd_map_db, size=neighborhood_size)
    maxima = rd_map_db == data_max

    # Apply threshold
    maxima = maxima & (rd_map_db > threshold_db)

    # Get indices of peaks
    peak_indices = np.argwhere(maxima)

    detections = []
    for idx in peak_indices:
        # idx is (doppler_idx, range_idx) based on shape (Nc, Nr)
        d_idx, r_idx = idx

        # Map to physical units
        r_val = range_axis[r_idx]
        v_val = v_axis[d_idx]
        power = rd_map_db[d_idx, r_idx]

        detections.append((r_val, v_val, power))

    return detections


# --- 2. Constants & Parameters ---
fc_start = 61020099000.0
fc_end = 61479903000.0
fc = 0.5 * (fc_start + fc_end)
c = 299792458.0
lam = c / fc
Tchirp = 6.99625e-05
fr = 1 / Tchirp

# --- 3. Load Data ---
# Try to locate frames.txt
possible_paths = ["data_test/frames.txt", "frames.txt"]
filepath = None
for p in possible_paths:
    if os.path.exists(p):
        filepath = p
        break

if filepath is None:
    print("Error: frames.txt not found in data_test/ or current directory.")
    exit(1)

print(f"Loading data from {filepath}...")
frames_list = parse_frames_txt(filepath)

if not frames_list:
    print("No frames found.")
    exit(1)

# Stack frames into (Nf, Nc, Ns, Na)
# Note: frames.txt might have variable chirps if interrupted, but assuming consistent structure
try:
    raw_data = np.stack(frames_list, axis=0)
except ValueError:
    print("Error: Frames have inconsistent shapes. Truncating to minimum common shape.")
    min_chirps = min(f.shape[0] for f in frames_list)
    min_samples = min(f.shape[1] for f in frames_list)
    frames_list = [f[:min_chirps, :min_samples, :] for f in frames_list]
    raw_data = np.stack(frames_list, axis=0)

Nf, Nc, Ns, Na = raw_data.shape
print(f"Data Shape: [Frames: {Nf}, Chirps: {Nc}, Samples: {Ns}, Antennas: {Na}]")

# --- 4. Low-pass Filter + DC Removal ---
print("Applying Low-pass Filter and DC Removal...")
filter_order = 4
cutoff_freq = 0.35
b, a = signal.butter(filter_order, cutoff_freq, "low", output="ba")

# Apply filter along Sample axis (axis 2)
rx_cube_filt = signal.filtfilt(b, a, raw_data, axis=2)

# Remove DC (Mean subtraction per chirp/antenna)
rx_cube = rx_cube_filt - np.mean(rx_cube_filt, axis=2, keepdims=True)

# --- 5. Range FFT ---
print("Computing Range FFT...")
# FFT along Sample axis
range_cube = np.fft.fft(rx_cube, axis=2)

# Keep positive ranges (skip DC)
# MATLAB: 2:Nfft/2 -> Python: 1:Ns//2
range_cube = range_cube[:, :, 1 : Ns // 2, :]
Nr = range_cube.shape[2]

# --- 6. Smart MTI Filter (IIR) ---
print("Applying Smart MTI Filter...")
alpha = 0.1
H = np.zeros((Nr, Na), dtype=complex)
range_mti = np.zeros_like(range_cube)

# Loop over frames and chirps sequentially
for f in range(Nf):
    for c in range(Nc):
        In = range_cube[f, c, :, :]  # Shape (Nr, Na)

        # Filtered Result
        Fn = In - H

        # Update History
        H = (alpha * In) + ((1 - alpha) * H)

        range_mti[f, c, :, :] = Fn

# --- 7. Doppler FFT ---
print("Computing Doppler FFT...")
# Window along slow-time (Chirp axis, axis 1)
window = np.hanning(Nc).reshape(1, Nc, 1, 1)
range_mti_win = range_mti * window

# FFT along Chirp axis
doppler_cube = np.fft.fft(range_mti_win, axis=1)
doppler_cube = np.fft.fftshift(doppler_cube, axes=1)

# --- 8. Power Calculation ---
# Sum power across antennas (axis 3)
doppler_pow = np.sum(np.abs(doppler_cube) ** 2, axis=3)  # Shape (Nf, Nc, Nr)

# --- 9. Axis Calibration ---
Nd = Nc
fd_axis = np.arange(-Nd // 2, Nd // 2) * (fr / Nd)
v_axis = (lam / 2) * fd_axis

# Range calibration from MATLAB script
range_res = (3 * 10**8) / (2 * (fc_end - fc_start))
range_axis = np.arange(Nr) * range_res
print(range_res)

dv = (lam / 2) * (fr / Nd)
vmax = (lam * fr) / 4
print(f"Δv ≈ {dv:.3f} m/s, v_max ≈ ±{vmax:.2f} m/s")

# --- 10. Plotting ---
print("Plotting...")
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))

# Initial plot
# Note: imshow extent is [left, right, bottom, top]
extent = [range_axis[0], range_axis[-1], v_axis[0], v_axis[-1]]
im = ax.imshow(
    np.zeros((Nc, Nr)), aspect="auto", origin="lower", extent=extent, cmap="jet"
)

ax.set_xlabel("Range (m)")
ax.set_ylabel("Velocity (m/s)")
title = ax.set_title("Range-Doppler")
cbar = plt.colorbar(im)
cbar.set_label("Power (dB)")

# Fixed scale from MATLAB
im.set_clim(75, 100)

try:
    for f in range(Nf):
        RD = doppler_pow[f, :, :]  # Shape (Nc, Nr)
        RD_dB = 10 * np.log10(RD + 1e-9)

        # Detect and print peaks
        peaks = detect_peaks(RD_dB, range_axis, v_axis)
        print(f"Frame {f+1}: Found {len(peaks)} peaks")
        for p in peaks:
            print(
                f"  Range: {p[0]:.2f} m, Velocity: {p[1]:.2f} m/s, Power: {p[2]:.2f} dB"
            )

        im.set_data(RD_dB)
        title.set_text(f"Range-Doppler (Frame {f+1}/{Nf})")

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.1)

except KeyboardInterrupt:
    print("Animation stopped by user.")

plt.ioff()
plt.show()
