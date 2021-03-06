# %%
""" 
    Imports 
"""
import sys
import time
import csv

# Signal processing
import numpy as np
# import scipy.signal as signal
# from scipy.fftpack import fft

# Plotting
# from pyqtgraph.Qt import QtGui, QtCore
# import pyqtgraph as pg
import matplotlib.pyplot as plt


# Hardware interfacing
import iio
import adi

# %%
"""
    Initialize pluto SDR
"""
# sdr = adi.Pluto(uri='ip:192.168.2.1')
sdr = adi.Pluto(uri='ip:pluto.local')

# %%
"""
    Configuration
"""
sample_rate = int(10e6)
buffer_size = int(2**14)
rx_lo = int(2e9)
tx_lo = int(2e9)
adc_size = 2**12

# Configure pluto
# Rx config
sdr.sample_rate = sample_rate
sdr.rx_buffer_size = buffer_size
sdr.rx_lo = rx_lo
sdr.gain_control_mode_chan0 = "slow_attack"

# Tx config
sdr.tx_lo = tx_lo
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = -20

# Print configuration
print("RX LO: {:0.2f}MHz".format(rx_lo / 1e6))
print("TX LO: {:0.2f}MHz".format(tx_lo / 1e6))

# %%

# Generate output waveform
f_s = int(sdr.sample_rate)
f_c = 2e9
T_s = 1 / float(f_s)
M = buffer_size
n = np.arange(0, M * T_s, T_s)
i = np.cos(2 * np.pi * f_c * n) * buffer_size
q = np.cos(2 * np.pi * f_c * n) * buffer_size
iq = i + (1j * q)

# Transmit IQ waveform
sdr.tx(iq)

# %%
# Matplotlib setup
plt.ion()

# %%

""" 
    Receive signal and do simple processing 
"""
output_file = 'data.csv'
file = open(output_file, 'w')
writer = csv.writer(file)

# Collect buffer_size samples
# num_samples = 20
# for i in range(num_samples):
#     x_n = sdr.rx()
#     writer.writerow(x_n)

# Collect samples
x_n = sdr.rx()

# Number of collected samples (i.e. buffer size)
M = len(x_n)          

# Window signal 
w_n = np.hamming(M)
x_n *= w_n

# Minimum SNR to be classified as a signal [dB]
min_snr = 10
noise_floor = -70

# Compute DFT 
X_k = np.absolute(np.fft.fft(x_n))
X_k = X_k[1:-1]

# Shift from 0->2pi to -pi->pi
X_k = np.fft.fftshift(X_k)

# Scale by window and 1/2 because only using half of FFT
X_k_mag = np.abs(X_k) * 2 / np.sum(w_n)    

# Convert to dBFS
X_k_dbfs = 20 * np.log10(X_k_mag / adc_size)

# Get real frequencies [MHz] and shift to -pi->pi
freq = np.fft.fftfreq(M, T_s)
freq = np.fft.fftshift(freq[1:-1]) / 1e6

# Create figure
fig, ax = plt.subplots()
plt.title("Received Signal - Frequency Domain")
plt.xlabel("frequency [MHz]")
plt.ylabel("dBFS")
plt.ylim([-120, -20])

line1, = ax.plot(freq, X_k_dbfs)

real_time_plot = True
while (real_time_plot):
    # Collect new samples
    x_n = sdr.rx()
    # x_n *= w_n

    # Calculate DFT, take magnitude, normalize, convert to dBFS
    X_k = np.absolute(np.fft.fft(x_n))
    X_k = X_k[1:-1]
    X_k = np.fft.fftshift(X_k)
    X_k_mag = np.abs(X_k) * 2 / np.sum(w_n)    
    X_k_dbfs = 20*np.log10(X_k_mag / adc_size)

    # Update plot
    line1.set_ydata(X_k_dbfs)
    fig.canvas.draw()
    fig.canvas.flush_events()
    

# %%
