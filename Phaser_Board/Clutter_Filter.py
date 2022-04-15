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

import sys
sys.path.insert(0, '/home/marchall/documents/chill/pyadi-iio')
sys.path.insert(0, '/home/marchall/documents/chill/ADI_Radar_DSP')

from operator import index
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
import numpy as np
from numpy import log10
from numpy.fft import fft, ifft, fftshift, ifftshift, fft2, ifft2
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

# %%
# Configure TDD controller
tdd = adi.tdd(sdr_ip)
tdd.frame_length_ms = 4    # each GPIO toggle is spaced 4ms apart
tdd.burst_count = 40 # there is a burst of 20 toggles, then off for a long time
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

my_sdr._ctx.set_timeout(30000)
my_sdr._rx_init_channels() 

# %%    
# Send data
my_sdr.tx([iq, iq])  # only send data to the 2nd channel (that's all we need)

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

plt.show()

# %%
# Split into frames
N_frame = int(PRI / ts)
rx_bursts = np.zeros((num_bursts, N_frame), dtype=complex)

# First ramp starts at 0.5ms
start_offset_time = 0.5e-3
start_offset_index = int((start_offset_time / (frame_time / 1e3)) * N_frame)

for burst in range(num_bursts):
    rx_bursts[burst] = chan1[start_offset_index + (burst + 1) * N_frame:
        start_offset_index + (burst + 2) * N_frame]

freq = np.linspace(-fs / 2, fs / 2, N_frame)

# %%

# Obtain range-FFT x-axis
c = 3e8
wavelength = c / output_freq
ramp_time_s = ramp_time / 1e6
slope = (BW * 4) / ramp_time_s
dist = (freq - signal_freq) * c / (2 * slope)

# Resolutions
R_res = c / (2 * (BW * 4))
v_res = wavelength / (2 * num_bursts * PRI)

# Doppler spectrum
PRF = 1 / PRI
max_doppler_freq = PRF / 2
max_doppler_vel = max_doppler_freq * wavelength / 2
velocity_axis = np.linspace(-max_doppler_vel, max_doppler_vel, num_bursts)

# %%
"""
    Plot range-Doppler spectrum - no filtering
"""
rx_bursts_fft = fft2(rx_bursts)

fig, ax = plt.subplots(figsize=(16, 16))
ax.set_title('Range Doppler Spectrum', fontsize=30)
ax.set_xlabel('Velocity [m/s]', fontsize=28)
ax.set_ylabel('Range [m]', fontsize=28)
max_range = 10
ax.set_ylim([0, max_range])
ax.set_yticks(np.arange(0, max_range, 2))
plt.xticks(fontsize=24)
plt.yticks(fontsize=24)

extent = [-max_doppler_vel, max_doppler_vel, dist.min(), dist.max()]
range_doppler = ax.imshow(10 * log10(fftshift(abs(rx_bursts_fft))).T, aspect='auto', 
    extent=extent, origin='lower', cmap=get_cmap('gist_rainbow'), vmin=30, vmax=70)
# colorbar = fig.colorbar(range_doppler, 
#     orientation='horizontal')
# colorbar.set_label(label='Magnitude [dB]', size=22)



fig.savefig('Figures/Clutter/Range_Doppler_No_CF.png', bbox_inches='tight')

# %%
# Generate colorbar separately and save
colorbar = fig.colorbar(range_doppler, cmap=get_cmap('bwr'), 
    orientation='horizontal')
colorbar.set_label(label='Magnitude [dB]', size=22)
range_doppler.figure.axes[1].tick_params(axis="x", labelsize=18)
ax.remove()
fig.savefig('Figures/Clutter/Range_Doppler_Colorbar.png', bbox_inches='tight')

# %%
"""
    Do some zero-Doppler filtering and plot
"""

max_clutter_vel = 0.1 # m/s
rx_bursts_cf_fft = fftshift(rx_bursts_fft.copy())
rx_bursts_cf_fft[np.where((velocity_axis > -max_clutter_vel) & 
    (velocity_axis < max_clutter_vel))] = 0

fig, ax = plt.subplots(figsize=(16, 16))
ax.set_title('Range Doppler Spectrum\nClutter Filtered @ $-%0.2f<v<%0.2fms^{-1}$' % 
    (max_clutter_vel, max_clutter_vel), fontsize=30)
ax.set_xlabel('Velocity [m/s]', fontsize=28)
ax.set_ylabel('Range [m]', fontsize=28)
max_range = 10
ax.set_ylim([0, max_range])
ax.set_yticks(np.arange(0, max_range, 2))
plt.xticks(fontsize=24)
plt.yticks(fontsize=24)

range_doppler_cf = ax.imshow(10 * log10(abs(rx_bursts_cf_fft)).T, aspect='auto', 
    extent=extent, origin='lower', cmap=get_cmap('bwr'), vmin=20, vmax=70)

# colorbar = fig.colorbar(range_doppler_cf, cmap=get_cmap('bwr'), 
#     orientation='vertical')
# colorbar.set_label(label='Magnitude [dB]', size=22)

fig.savefig('Figures/Clutter/Range_Doppler_CF.png', bbox_inches='tight')

# %%
rx_bursts_cf = ifft2(ifftshift(rx_bursts_cf_fft))

i = 7
test_fft = fft(rx_bursts[i])
test_fft_cf = fft(rx_bursts_cf[i])

# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 16))
# plt.suptitle('Comparison of Range-FFT Spectrum\nWith and Without Clutter Filtering', 
#     fontsize=26)
fig1, ax1 = plt.subplots(figsize=(16, 8))
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
fig2, ax2 = plt.subplots(figsize=(16, 8))
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
ax1.set_title('Range-FFT Spectrum\nNo Clutter Filter', fontsize=24)
ax2.set_title('Range-FFT Spectrum\nClutter Filter', fontsize=24)
ax1.set_xlabel('Range [m]', fontsize=22)
# ax2.set_xlabel('Frequency [kHz]', fontsize=22)
ax2.set_xlabel('Range [m]', fontsize=22)
ax1.set_ylabel('Magnitude [dB]', fontsize=22)
ax2.set_ylabel('Magnitude [dB]', fontsize=22)
ax1.plot(dist, 10 * log10(fftshift(abs(test_fft))), label='No Filter', c='b')
ax2.plot(dist, 10 * log10(fftshift(abs(test_fft_cf))), label='Clutter Filtered', c='b')

xlim = np.array([-max_range, max_range]) 
ax1.set_xlim(xlim)
ax2.set_xlim(xlim)

ylim = [20, 70]
ax1.set_ylim(ylim)
ax2.set_ylim(ylim)

subdist = np.where((dist > 0.5) & (dist < 2.5))
y2_target = 10 * log10(abs(fftshift(test_fft_cf)[subdist].max()))
target_index = np.where(10 * log10(abs(fftshift(test_fft_cf))) == y2_target)
x1_target = dist[target_index]
x2_target = dist[target_index]
y1_target = 10 * log10(abs(fftshift(test_fft_cf)))[target_index]

ax1.annotate('Target', xy=(x1_target, y1_target), xycoords='data',
    xytext=(x1_target + 0.5, y1_target + 5), textcoords='data',
    arrowprops = dict(
            connectionstyle='arc3,rad=0.',
            shrinkA=0, shrinkB=0,
            arrowstyle='-|>', ls='-', linewidth=3
        ), fontsize=22)
ax2.annotate('Target', xy=(x2_target, y2_target), xycoords='data',
    xytext=(x2_target + 0.5, y2_target + 5), textcoords='data',
    arrowprops = dict(
            connectionstyle='arc3,rad=0.',
            shrinkA=0, shrinkB=0,
            arrowstyle='-|>', ls='-', linewidth=3
        ), fontsize=22)


fig1.savefig('Figures/Clutter/Without_CF.png', bbox_inches='tight')
fig2.savefig('Figures/Clutter/With_CF.png', bbox_inches='tight')

# %%
plt.show()


# Plot frequnecy domain stackplot
fig, ax = plt.subplots(figsize=(16, 8))

rx_bursts_fft = np.zeros(rx_bursts.shape)
for index, rx_burst in enumerate(rx_bursts):
    rx_bursts_fft[index] = log10(fftshift(abs(fft(rx_burst))))

ax.stackplot(freq / 1e6, *(rx_bursts_fft[2:10]))
ax.set_xlim([signal_freq - 100e3, signal_freq + 100e3])
ax.set_xlim([signal_freq / 1e6 - 0.1, signal_freq / 1e6 + 0.1])
ax.set_ylabel('Slow time', fontsize=22)
ax.set_xlabel('Frequency [MHz]\nDFT of Fast Time', fontsize=22)
ax.set_title('Stack plot of all M buffers', fontsize=24)
plt.tick_params(axis='y', left=False, which='both', labelleft=False)
plt.xticks(size=16)
plt.yticks(size=16)

fig.savefig('Figures/Stack_Plot.png')

# plt.show()
