# %%
# Copyright (C) 2019 Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     - Neither the name of Analog Devices, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#     - The use of this software may or may not infringe the patent rights
#       of one or more patent holders.  This license does not release you
#       from the requirement that you obtain separate licenses from these
#       patent holders to use this software.
#     - Use of the software either in source or binary form, must be run
#       on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''To test this script, shine a 10.5GHz HB100 source on the array'''

from operator import index
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
import numpy as np
from numpy import log10
from numpy.fft import fft, ifft, fftshift, fft2
from scipy import signal
import target_detection
import adi
import faulthandler
faulthandler.enable()
from importlib import reload
reload(target_detection)

# %%

import time
from time import sleep

# Instantiate all the Devices
try:
    import phaser_config
    rpi_ip = phaser_config.rpi_ip
    sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block
except:
    print('No config file found...')
    rpi_ip = "ip:phaser.local"  # IP address of the Raspberry Pi
    sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block

my_sdr = adi.ad9361(uri=sdr_ip)
time.sleep(0.5)

gpios = adi.one_bit_adc_dac(rpi_ip)
gpios.gpio_vctrl_1 = 1 # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
gpios.gpio_vctrl_2 = 0 # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)
gpios.gpio_burst =0    # High to low causes Pluto to start a new Rx buffer and a burst of TDD pulses
time.sleep(0.5)

my_phaser = adi.CN0566(uri=rpi_ip, rx_dev=my_sdr)
time.sleep(0.5)

# Parameters
sample_rate = 0.6e6
sample_rate = 10e6 
center_freq = 2.1e9
signal_freq = 1e6 
num_slices = 200
fft_size = 1024*16

# Create radio
'''This script is for Pluto Rev C, dual channel setup'''
my_sdr.sample_rate = int(sample_rate)

# Configure Rx
my_sdr.rx_lo = int(center_freq)   # set this to output_freq - (the freq of the HB100)
my_sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
my_sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
my_sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
my_sdr.rx_hardwaregain_chan0 = int(60)   # must be between -3 and 70
my_sdr.rx_hardwaregain_chan1 = int(60)   # must be between -3 and 70
# Configure Tx
my_sdr.tx_lo = int(center_freq)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True      # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
my_sdr.tx_hardwaregain_chan0 = -88   # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = -3   # must be between 0 and -88

# Read properties
print("RX LO %s" % (my_sdr.rx_lo))

# Enable phaser logic in pluto
gpio = adi.one_bit_adc_dac(sdr_ip)
time.sleep(0.5)
gpio.gpio_phaser_enable = True
time.sleep(0.5)
gpio.gpio_tdd_ext_sync = True   # If set to True, this enables external capture triggering using the L24N GPIO on the Pluto.  When set to false, an internal trigger pulse will be generated every second

# Configure the ADF4159 Rampling PLL
output_freq = 12.1e9
BW = 500e6/4
num_steps = 1000
ramp_time = 1e3 # us
my_phaser.frequency = int(output_freq/4) # Output frequency divided by 4
my_phaser.freq_dev_range = int(BW) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
my_phaser.freq_dev_step = int(BW/num_steps) # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
my_phaser.freq_dev_time = int(ramp_time) # total time (in us) of the complete frequency ramp
my_phaser.ramp_mode = "single_sawtooth_burst"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
my_phaser.delay_word = 4095     # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
my_phaser.delay_clk = 'PFD'     # can be 'PFD' or 'PFD*CLK1'
my_phaser.delay_start_en = 0         # delay start
my_phaser.ramp_delay_en = 0          # delay between ramps.  
my_phaser.trig_delay_en = 0          # triangle delay
my_phaser.sing_ful_tri = 0           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
my_phaser.tx_trig_en = 1             # start a ramp with TXdata
#my_phaser.clk1_value = 100
#my_phaser.phase_value = 3
my_phaser.enable = 0                 # 0 = PLL enable.  Write this last to update all the registers


# Configure TDD controller
tdd = adi.tdd(sdr_ip)
tdd.frame_length_ms = 4    # each GPIO toggle is spaced 4ms apart
tdd.burst_count = 20 # there is a burst of 20 toggles, then off for a long time
tdd.rx_rf_ms = [0.5,0.9, 0, 0]    # each GPIO pulse will be 100us (0.6ms - 0.5ms).  And the first trigger will happen 0.5ms into the buffer
tdd.secondary = False
tdd.en = True

# buffer size needs to be greater than the frame_time
frame_time = tdd.frame_length_ms*tdd.burst_count   # time in ms
print("frame_time:  ", frame_time, "ms")
buffer_time = 0
power=12
while frame_time > buffer_time:     
    power=power+1
    buffer_size = int(2**power) 
    buffer_time = buffer_size/my_sdr.sample_rate*1000   # buffer time in ms
    if power==23:
        break     # max pluto buffer size is 2**23, but for tdd burst mode, set to 2**22
print("buffer_size:", buffer_size)
my_sdr.rx_buffer_size = buffer_size
print("buffer_time:", buffer_time, " ms")  

# Create a sinewave waveform
fs = int(my_sdr.sample_rate)
print("sample_rate:", fs)
N = buffer_size
fc = int(signal_freq / (fs / N)) * (fs / N)
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 0.9* (i + 1j * q)

# Send data
my_sdr.tx([iq, iq])  # only send data to the 2nd channel (that's all we need)

my_sdr._ctx.set_timeout(30000)
my_sdr._rx_init_channels() 
    
print('Collecting...')

# Collect data
for r in range(5):    # grab several buffers to let the AGC settle
    gpios.gpio_burst = 0
    gpios.gpio_burst = 1
    # time.sleep(0.001)
    gpios.gpio_burst = 0
    
    data = my_sdr.rx()
    
    chan1 = data[0]
    chan2 = data[1]
    
my_sdr.tx_destroy_buffer()

PRI = tdd.frame_length_ms / 1e3
PRF = 1 / PRI

# Don't take first burst because it is contaminated with noise
num_bursts = tdd.burst_count - 1

N = chan1.size
time = np.linspace(0, ts * N, N)

# Plot full time domain signal
fig1, ax1 = plt.subplots(figsize=(16, 8))
ax1.plot(time * 1e3, np.abs(chan1))
ax1.set_title('Received Signal', fontsize=20)
ax1.set_xlabel('Time [ms]', fontsize=18)

# %%
# Split into frames
N_frame = int(PRI / ts)
rx_bursts = np.zeros((num_bursts, N_frame), dtype=complex)

# First ramp starts at 0.5ms
start_offset_time = 0.5e-3
start_offset_index = int((start_offset_time / (frame_time / 1e3)) * N_frame)

# fig, ax = plt.subplots(figsize=(16, 8))
for burst in range(num_bursts):
    rx_bursts[burst] = chan1[start_offset_index + (burst + 1) * N_frame:
        start_offset_index + (burst + 2) * N_frame]
    # ax.plot(abs(rx_bursts[burst]))

freq = np.linspace(-fs / 2, fs / 2, N_frame)

# Obtain range-FFT x-axis
c = 3e8
wavelength = c / output_freq
ramp_time_s = ramp_time / 1e6
slope = (BW * 4) / ramp_time_s
dist = (freq - signal_freq) * c / (2 * slope)

# %%

# Resolutions
R_res = c / (2 * (BW * 4))
v_res = wavelength / (2 * num_bursts * PRI)

# Doppler spectrum
PRF = 1 / PRI
max_doppler_freq = PRF / 2
max_doppler_vel = max_doppler_freq * wavelength / 2

# %%
rx_bursts_fft = fftshift(abs(fft2(rx_bursts)))
thresholds = np.zeros(rx_bursts_fft.shape)
rx_bursts_fft_cfar = np.ma.masked_all(rx_bursts_fft.shape)

# Uncomment to perform CFAR thresholding before plotting
# This significantly increases run time
"""
# CFAR for entire range Doppler spectrum
fig, ax = plt.subplots(figsize=(16, 8))
for m in range(num_bursts):
    threshold, targets, noise = target_detection.cfar(rx_bursts_fft[m], 
        num_guard_cells=5, num_ref_cells=10, cfar_method='false_alarm', 
        fa_rate=0.05)
    thresholds[m] = threshold
    if (m % 4 == 0):
        ax.plot(dist, log10(targets), label=m)
    rx_bursts_fft_cfar[m] = targets

ax.plot(dist, log10(rx_bursts_fft_cfar[4]), label='target')
ax.plot(dist, log10(thresholds[4]), label='threshold')
ax.legend(loc='upper right', fontsize=16)

# Plot frames overlayed frequency spectrum
# for burst_index in range(1, num_bursts):
#     if (burst_index % 4 == 0):
#         ax.plot(dist, log10(fftshift(abs(rx_bursts_fft[burst_index]))), label=burst_index)

ax.set_title('Overlaid Frequency Spectrum\nSpacing: %0.2fms' % (PRI * 1e3), fontsize=24)
ax.set_xlabel('Range [m]', fontsize=22)
# ax.set_xlabel('Frequency [Hz]', fontsize=22)
ax.set_ylabel('Magnitude [dB]', fontsize=22)
# ax.set_xlim([70e3, 130e3])
ax.set_xlim([0, 10])
ax.legend(loc='upper right', fontsize=16)

# rx_bursts_fft[np.where(log10(rx_bursts) < 3)] = 0
"""

# %%
fig, ax = plt.subplots(figsize=(16, 16))

extent = [-max_doppler_vel, max_doppler_vel, dist.min(), dist.max()]
range_doppler = ax.imshow(log10(rx_bursts_fft).T, aspect='auto', 
    extent=extent, origin='lower', cmap=get_cmap('bwr'), vmin=2, vmax=7)

ax.set_title('Range Doppler Spectrum', fontsize=24)
ax.set_xlabel('Velocity [m/s]', fontsize=22)
ax.set_ylabel('Range [m]', fontsize=22)

max_range = 15
ax.set_ylim([0, max_range])
ax.set_yticks(np.arange(0, max_range, 2))
colorbar = fig.colorbar(range_doppler, cmap=get_cmap('bwr'), 
    orientation='vertical')
colorbar.set_label(label='Magnitude [dB]', size=22)

fig.savefig('Figures/Range_Doppler_Moving_next2_Refl_0-%i_bwr.png' % (max_range)) #, facecolor='w')

# %%
# 3D Plot
print('Plotting 3D')
X = np.linspace(dist.min(), dist.max(), rx_bursts_fft.shape[1])
Y = np.arange(-max_doppler_vel, max_doppler_vel, rx_bursts_fft.shape[0])
X, Y = np.meshgrid(X, Y)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot(X, Y, log10(rx_bursts_fft), cmap=get_cmap('bwr'), vmin=2, vmax=7)
# ax.set_xlim([0, 15])
plt.show()