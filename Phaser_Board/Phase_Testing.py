"""
	Filename: 	Phase_Testing.py
	Desc.:	 
	Author:		Marshall Bruner
	Date:		2022-02-22
"""

# %%
# Imports
import numpy as np
from numpy import log10, pi
from numpy.fft import fft, ifft, fftshift, fftfreq, rfft, irfft
import matplotlib.pyplot as plt
from scipy import signal
from target_detection import cfar

from matplotlib import style
style.use('seaborn-dark')

import netCDF4 as nc

# %%
sample_rate = 600e3
sample_period = 1 / sample_rate

time_diff = 120e-3

f = 12.1e9
c = 3e8
wavelength = c / f

# %%
"""
	Read dataset and plot DFT
"""
ds = nc.Dataset('Data/Test_Track_2.nc', 'r')

time_start = 60
time_end = time_start + 2
x_n = ds.variables['i'][time_start:time_end,:] + 1j * ds.variables['q'][time_start:time_end,:]
x_n_1, x_n_2 = x_n
X_k_1 = fft(x_n_1)
X_k_2 = fft(x_n_2)
N = x_n_1.size

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
"""
	Plot filtered time domain
"""
fig, ax1 = plt.subplots()
fig.set_figheight(8)
fig.set_figwidth(16)
ax1.plot(time * 1e3, x_n_1_filt, c='b', label='t=0')
ax1.plot(time * 1e3, x_n_2_filt, c='r', label='t=1')
ax1.set_xlabel('Time [ms]', fontsize=22)

# %%
"""
	Calculate cross-correlation between different times then from the phase shift
	calculate target velocity.
"""
# Cross correlation between different times
corr = signal.correlate(x_n_1_filt, x_n_2_filt, mode='full')
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

ax.axvline(lag_max, c='r', linestyle='dashed', label='Phase shift: %0.2f$\pi$' % 
    (recovered_phase_shift / pi))
ax.legend(loc='upper right', fontsize=16)

ax.plot(lags, corr, label='a')

# Print corresponding velocity
V_r = (wavelength * recovered_phase_shift) / (4 * pi * time_diff)
print('Target velocity: %0.4f m/s' % V_r)

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