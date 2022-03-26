"""
	Filename: 	Phase_Testing.py
	Desc.:	 
	Author:		Marshall Bruner
	Date:		2022-02-22
"""

# %%
# Imports
from random import sample
import numpy as np
from numpy import log10, pi
from numpy.fft import fft, ifft, fftshift, fftfreq, rfft, irfft
import matplotlib.pyplot as plt
from scipy import signal
from target_detection import cfar
from Phaser_Functions import range_bin

# from matplotlib import style
# style.use('seaborn-dark')

import netCDF4 as nc

# %%
"""
	Read dataset and plot DFT
"""
ds = nc.Dataset('Data/Test_Track_2.nc', 'r')

# Track was moving at bin 60
time_start = 60
x_n = ds.variables['i'][time_start,:] + 1j * ds.variables['q'][time_start,:]
total_N = x_n.size
# %%
x_n_1 = x_n[0:int(total_N / 2)]
x_n_2 = x_n[int(total_N / 2):total_N]
X_k_1 = fft(x_n_1)
X_k_2 = fft(x_n_2)
N = x_n_1.size

# %%
sample_rate = 600e3
sample_period = 1 / sample_rate

time_diff = N * sample_period
print('Time delta between signals: %0.2fms' % (time_diff * 1e3))

f = 12.1e9
c = 3e8
wavelength = c / f


# %%

# Time 
time = np.linspace(0, N * sample_period, N)

freq = fftshift(np.linspace(-sample_rate / 2, sample_rate / 2, N))

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.set_figheight(16)
fig.set_figwidth(16)
ax1.plot(freq, 10 * log10(X_k_1))
ax2.plot(freq, 10 * log10(X_k_2))
ax2.set_xlabel('Frequency [rad]', fontsize=22)

"""
	Filter and replot
"""
cutoffs = (105e3, 107e3)
order = 10
max_ripple = 5
w_sos = signal.cheby1(order, max_ripple, cutoffs, btype='bp', 
	fs=sample_rate, output='sos')
x_n_1_filt = signal.sosfilt(w_sos, x_n_1)
x_n_2_filt = signal.sosfilt(w_sos, x_n_2)

X_k_1_filt = fft(x_n_1_filt)
X_k_2_filt = fft(x_n_2_filt)

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.set_figheight(16)
fig.set_figwidth(16)
ax1.plot(freq, 10 * log10(X_k_1_filt))
ax2.plot(freq, 10 * log10(X_k_2_filt))
ax2.set_xlabel('Frequency [rad]', fontsize=22)

# Apply CFAR classification
num_guard_cells = 10
num_ref_cells = 30
bias = 3
cfar_method = 'greatest'
X_k_1_cfar, X_k_1_targets = cfar(X_k_1_filt, num_guard_cells, 
	num_ref_cells, bias, cfar_method)
X_k_2_cfar, X_k_2_targets = cfar(X_k_2_filt, num_guard_cells, 
	num_ref_cells, bias, cfar_method)

X_k_1_targets[np.where(abs(X_k_1_targets) <= abs(X_k_1_cfar))] = np.ma.masked
X_k_2_targets[np.where(abs(X_k_2_targets) <= abs(X_k_2_cfar))] = np.ma.masked

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.set_figheight(16)
fig.set_figwidth(16)
ax1.plot(freq / 1e3, 10 * log10(X_k_1_targets))
ax2.plot(freq / 1e3, 10 * log10(X_k_2_targets))
ax1.set_xlim([-150, 150])
ax2.set_xlim([-150, 150])
ax2.set_xlabel('Frequency [kHz]', fontsize=22)

# %%
# Obtain single target and see if it maintains phase info
max_index = X_k_1_targets.argmax()

single_freq_1 = np.copy(X_k_1_targets)
single_freq_2 = np.copy(X_k_2_targets)

single_freq_1[np.where(X_k_1_targets != np.max(X_k_1_targets))] = 0
single_freq_2[np.where(X_k_1_targets != np.max(X_k_1_targets))] = 0

x_n_1_single = ifft(single_freq_1)
x_n_2_single = ifft(single_freq_2)

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.set_figheight(16)
fig.set_figwidth(16)
ax1.set_title('All removed except max')
ax1.stem(freq / 1e3, 10 * log10(single_freq_1))
ax2.stem(freq / 1e3, 10 * log10(single_freq_2))
# ax1.set_xlim([-150, 150])
# ax2.set_xlim([-150, 150])
ax2.set_xlabel('Frequency [kHz]', fontsize=22)

fig, ax1 = plt.subplots()
fig.set_figheight(8)
fig.set_figwidth(16)
ax1.set_title('Subsequently Received Signals - Time Domain\n$T_{diff}=%0.2f$ms\nAll removed except max' % 
	(time_diff * 1e3), fontsize=24)
ax1.plot(time * 1e3, x_n_2_single, c='r', label='$t_0+T_{diff}$')
ax1.plot(time * 1e3, x_n_1_single, c='b', label='$t_0$')
ax1.set_xlabel('Time [ms]', fontsize=22)
ax1.set_ylabel('Amplitude', fontsize=22)
ax1.set_xlim([3, 3.1])
ax1.legend(loc='upper left', fontsize='16')

# %%
# Normalize to remove amplitude interference
x_n_1_filt /= np.max(x_n_1_filt)
x_n_2_filt /= np.max(x_n_2_filt)

# %%
"""
	Plot filtered time domain
"""
fig, ax1 = plt.subplots()
fig.set_figheight(8)
fig.set_figwidth(16)
ax1.set_title('Subsequently Received Signals - Time Domain\n$T_{diff}=%0.2f$ms' % 
	(time_diff * 1e3), fontsize=24)
ax1.plot(time * 1e3, x_n_2_filt, c='r', label='$t_0+T_{diff}$')
ax1.plot(time * 1e3, x_n_1_filt, c='b', label='$t_0$')
ax1.set_xlabel('Time [ms]', fontsize=22)
ax1.set_ylabel('Amplitude', fontsize=22)
ax1.legend(loc='upper left', fontsize='16')

# %%
fig.savefig('Figures/Subsequent_Signals_Overlayed')

# %%
"""
	Calculate cross-correlation between different times then from the phase shift
	calculate target velocity.
"""
# Cross correlation between different times
corr = signal.correlate(x_n_1_single, x_n_2_single, mode='full')
lags = signal.correlation_lags(N, N, mode='full')
dt = np.linspace(-time[-1], time[-1], 2 * N - 1)

# Obtain max value of correlation function
max_corr = corr.argmax()
lag_max = lags[max_corr]

# Time shift amount is where the max correlation occurs
recovered_time_shift = dt[max_corr]
recovered_phase_shift = 2 * pi * (((0.5 + recovered_time_shift * f) % 1.0) - 0.5)

fig, ax = plt.subplots()
fig.set_figheight(8)
fig.set_figwidth(16)
ax.set_title('Cross-correlation Between Subsequent Buffers\n$T_{diff}=13.65$ms', 
	fontsize=24)
ax.set_xlabel('Lag', fontsize=22)
ax.set_ylabel('Amplitude', fontsize=22)

ax.axvline(lag_max, c='r', linestyle='dashed', label='Phase shift: %0.2f$\pi$' % 
    (recovered_phase_shift / pi))
ax.legend(loc='upper right', fontsize=16)

ax.plot(lags, corr, label='a')
fig.savefig('Figures/Cross_Correlation_Subsequent_Signals')

# Print corresponding velocity
V_r = (wavelength * recovered_phase_shift) / (4 * pi * time_diff)
print('Target velocity: %0.4f m/s = %0.4f f/s' % (V_r, V_r * 3.28084))

# %%
"""
	Testing if the signal can be shifted to completely overlay the t-1 
	signal with the found phase shift.
"""
X_k_1_shift = rfft(x_n_1_filt)
X_k_2_shift = rfft(x_n_2_filt) * np.exp(1.0j * pi/2)

x_n_1_shift = irfft(X_k_1_shift)
x_n_2_shift = irfft(X_k_2_shift)

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.set_figheight(16)
fig.set_figwidth(16)
ax1.plot(time * 1e3, x_n_1_filt, c='b', label='t=0')
ax1.plot(time * 1e3, x_n_2_filt, c='r', label='t=1')
ax2.plot(time * 1e3, x_n_1_shift, c='b', label='t=0')
ax2.plot(time * 1e3, x_n_2_shift, c='r', label='t=1')
ax2.set_xlabel('Time [ms]', fontsize=22)

# %%
angles = np.tan((np.imag(x_n_1) - np.imag(x_n_2)) / (np.real(x_n_1) - np.real(x_n_2)))

fig, ax1 = plt.subplots()
fig.set_figheight(8)
fig.set_figwidth(16)
ax1.set_title('Phase Difference - Time Domain\n$T_{diff}=%0.2f$ms' % 
	(time_diff * 1e3), fontsize=24)
ax1.plot(time * 1e3, angles / pi, c='b', label='$t_0$')
ax1.set_xlabel('Time [ms]', fontsize=22)
ax1.set_ylabel('Amplitude', fontsize=22)

# %%

mode = 'same'
Rxx = signal.correlate(x_n, x_n, mode=mode)
PSD = fft(Rxx)

freq_full = fftshift(np.linspace(-sample_rate / 2, sample_rate / 2, PSD.size))
lags = signal.correlation_lags(x_n.size, x_n.size, mode=mode)

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.set_figheight(16)
fig.set_figwidth(16)
ax1.set_title('PSD')
ax1.plot(freq_full, 10 * log10(PSD))
ax1.set_xlabel('Frequency [kHz]', fontsize=22)
ax2.plot(lags, Rxx)
ax2.set_xlabel('Lag', fontsize=22)
# ax1.set_xlim([-150, 150])
# ax2.set_xlim([-150, 150])

# %%

N = x_n_1.size
sample_rate = 600e3

max_time = N * (1 / sample_rate)

X_k_1 = fft(x_n_1)
X_k_2 = fft(x_n_2)
X_k_1, R_res = range_bin(X_k_1, N)
X_k_2, R_res = range_bin(X_k_2, N)

N_new = X_k_1.size
lags = signal.correlation_lags(N_new, N_new, mode='full')
dt = np.linspace(-max_time, max_time, 2 * N_new - 1)

single_bins = np.zeros(lags.shape)

for index in range(N_new):
	tmp_1 = np.copy(single_bins)
	tmp_2 = np.copy(single_bins)

	tmp_1[index] = X_k_1[index]
	tmp_2[index] = X_k_2[index]

	Rxx = signal.correlate(ifft(tmp_2), ifft(tmp_1), mode='full')

	# Obtain max value of correlation function
	max_corr = Rxx.argmax()
	lag_max = lags[max_corr]

	# Time shift amount is where the max correlation occurs
	recovered_time_shift = dt[max_corr]
	recovered_phase_shift = 2 * pi * (((0.5 + recovered_time_shift * sample_rate) % 1.0) - 0.5)

	V_r = (wavelength * recovered_phase_shift) / (4 * pi / sample_rate) 
	print("%0.4f m/s" % V_r)

# %%
fig, ax = plt.subplots()
fig.set_figheight(8)
fig.set_figwidth(16)

ax.axvline(lag_max, c='r', linestyle='dashed', label='Phase shift: %0.2f$\pi$' % 
    (recovered_phase_shift / pi))
ax.legend(loc='upper right', fontsize=16)

ax.plot(lags, Rxx, label='a')