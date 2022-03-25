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

import time
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, fftshift
from scipy import signal
import adi
import faulthandler
faulthandler.enable()

# Instantiate all the Devices
try:
    import phaser_config
    rpi_ip = phaser_config.rpi_ip
    sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block
except:
    print('No config file found...')
    rpi_ip = "ip:phaser.local"  # IP address of the Raspberry Pi
    sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block

try:
    x = my_sdr.uri
    print("Pluto already connected")
except NameError:
    print("Pluto not connected...")
    my_sdr = adi.ad9361(uri=sdr_ip)

time.sleep(0.5)

try:
    x = my_phaser.uri
    print("cn0566 already connected")
except NameError:
    print("cn0566 not open...")
    my_phaser = adi.CN0566(uri=rpi_ip, rx_dev=my_sdr)

sample_rate = 30.72e6
sample_rate = 0.6e6
center_freq = 2.1e9
signal_freq = 100e3
num_slices = 200
fft_size = 1024*16

# Create radio
'''This script is for Pluto Rev C, dual channel setup'''
my_sdr.sample_rate = int(sample_rate)

"""
# Configure Rx
my_sdr.rx_lo = int(1.895e9)   # set this to output_freq - (the freq of the HB100)
my_sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
my_sdr.gain_control_mode_chan0 = 'manual'  # manual of slow_attack
my_sdr.gain_control_mode_chan1 = 'manual'  # manual of slow_attack
my_sdr.rx_hardwaregain_chan0 = int(60)   # must be between -3 and 70
my_sdr.rx_hardwaregain_chan1 = int(60)   # must be between -3 and 70
# Configure Tx
my_sdr.tx_lo = int(5.5e9)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True      # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
my_sdr.tx_hardwaregain_chan0 = -88   # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = -0   # must be between 0 and -88
"""

# Configure Rx
my_sdr.rx_lo = int(center_freq)   # set this to output_freq - (the freq of the HB100)
my_sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
# my_sdr.rx_buffer_size = int(fft_size)
my_sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
my_sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
my_sdr.rx_hardwaregain_chan0 = int(60)   # must be between -3 and 70
my_sdr.rx_hardwaregain_chan1 = int(60)   # must be between -3 and 70
# Configure Tx
my_sdr.tx_lo = int(center_freq)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True      # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
my_sdr.tx_hardwaregain_chan0 = -88   # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = -0   # must be between 0 and -88

# Read properties
print("RX LO %s" % (my_sdr.rx_lo))

# Enable phaser logic in pluto
gpio = adi.one_bit_adc_dac(sdr_ip)
gpio.gpio_phaser_enable = True

# Configure the ADF4159 Rampling PLL
output_freq = 12.1e9
BW = 500e6 / 4
num_steps = 1000
ramp_time = 1e3 # us
ramp_time_s = ramp_time / 1e6
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
tdd.frame_length_ms = 8 # each GPIO toggle is spaced 4ms apart, 
tdd.burst_count = 5       # there is a burst of 20 toggles, then off for a long time
tdd.rx_rf_ms = [0.5,1.0, 0, 0]    # each GPIO pulse will be 100us (0.6ms - 0.5ms).  And the first trigger will happen 0.5ms into the buffer
tdd.secondary = False
tdd.en = True

# buffer size needs to be greater than the frame_time
frame_time = tdd.frame_length_ms*tdd.burst_count/2
print("frame_time:", frame_time, "ms")
buffer_time = 0
power=12
while frame_time > buffer_time:     
    power=power+1
    buffer_size = int(2**power)
    buffer_time = buffer_size/my_sdr.sample_rate*1000
    if power==23:
        break     # max pluto buffer size is 2**23, but for tdd burst mode, set to 2**22
print("buffer_size:", buffer_size)
my_sdr.rx_buffer_size = buffer_size
print("buffer_time:", buffer_time, " ms")  
#
# DDS send data (either do this or the Tx Method -- not both)
# my_sdr.dds_single_tone(int(3e6), 0.9, 0)   # my_sdr.dds_single_tone(tone_freq_hz, tone_scale, tx_channel)

# Create a sinewave waveform
fs = int(my_sdr.sample_rate)
print("sample_rate:", fs)
N = buffer_size
fc = int(3e6 / (fs / N)) * (fs / N)
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 1 * (i + 1j * q)

# Send data
my_sdr._ctx.set_timeout(0)
my_sdr.tx([iq*0.5, iq])  # only send data to the 2nd channel (that's all we need)

# my_sdr.tx
# Collect data
for r in range(5):    # grab several buffers to let the AGC settle
    data = my_sdr.rx()
    chan1 = data[0]
    chan2 = data[1]
my_sdr.tx_destroy_buffer()

# chan1 += chan2

# %%

# plt.clf()
# plt.plot(np.abs(chan1))
# plt.draw()
# plt.show()
spec, freqs, times, im = plt.specgram(chan1, Fs=fs, NFFT=256, noverlap=128)
# plt.draw()

# plt.show()

# plt.plot(fft(chan1 + chan2))

# %%

# get 256 point fft surrounding a ramp

start_time = 0.5e-3
end_time = 1e-3
N = chan1.size

num_points = 512 

num_ramps = 4

ramps = []

for ramp_index in range(num_ramps):
    # ramp_index = 2
    mid_index = int(start_time * (ramp_index + 1) * N / (buffer_time / 1e3))

    ramps.append(chan1[mid_index - int(num_points / 2):mid_index + int(num_points / 2)])

    single_ramp_fft = fft(ramps[ramp_index])

    freq = np.linspace(-sample_rate / 2, sample_rate / 2, num_points)

    fig = plt.figure(figsize=(16, 8))
    ax = plt.axes()
    ax.set_title('Ramp #%i' % ramp_index, fontsize=24)
    ax.set_xlabel('Frequency [kHz]', fontsize=22)
    ax.set_ylabel('Magnitude [dB]', fontsize=22)
    ax.axvline(x=0, linestyle='--', c='r')
    ax.plot(fftshift(freq) / 1e3, 10 * np.log10(single_ramp_fft))
# %%

from Phaser_Functions import range_bin


c = 3e8
slope = BW / ramp_time_s
signal_freq = 0

max_dist = 10
max_freq = max_dist * slope / c + signal_freq
max_freq = np.max(freq)

for ramp_index, single_ramp in enumerate(ramps):
    single_ramp_fft = fft(single_ramp)
    single_ramp_fft_bin, N_res = range_bin(single_ramp_fft, single_ramp_fft.size)
    N_new = single_ramp_fft_bin.size

    freq = fftshift(np.linspace(-sample_rate / 2, sample_rate / 2, N_new))

    bb_indices = np.where((freq >= signal_freq) & (freq <= max_freq))
    freq_bb = freq[bb_indices]

    single_ramp_fft_bin = single_ramp_fft_bin[bb_indices]

    fig = plt.figure(figsize=(16, 8))
    ax = plt.axes()
    ax.set_title('Ramp #%i' % ramp_index, fontsize=24)
    ax.set_xlabel('Frequency [kHz]', fontsize=22)
    ax.set_ylabel('Magnitude [dB]', fontsize=22)
    ax.axvline(x=0, linestyle='--', c='r')
    ax.plot(freq_bb / 1e3, 10 * np.log10(single_ramp_fft_bin))

# %%

from numpy.fft import ifft
from numpy import pi

max_time = 4e-3
lags = signal.correlation_lags(N_new, N_new, mode='full')
dt = np.linspace(-max_time, max_time, 2 * N_new - 1)

wavelength = c / output_freq 

for ramp_index in range(0, len(ramps) - 1):
    phases = np.zeros(single_ramp_fft_bin.shape)
    vel = np.zeros(single_ramp_fft_bin.shape)
    zeros = np.zeros(single_ramp_fft_bin.shape)
    for index, val in enumerate(zeros):
        X_k_1 = np.copy(zeros)
        X_k_2 = np.copy(zeros)
        X_k_1[index] = fft(ramps[ramp_index])[index]
        X_k_2[index] = fft(ramps[ramp_index + 1])[index]

        x_n_1 = ifft(X_k_1)
        x_n_2 = ifft(X_k_2)

        Rxx = signal.correlate(x_n_2, x_n_1, mode='full')

        # Obtain max value of correlation function
        max_corr = Rxx.argmax()
        lag_max = lags[max_corr]

        # Time shift amount is where the max correlation occurs
        recovered_time_shift = dt[max_corr]
        recovered_phase_shift = 2 * pi * (((0.5 + recovered_time_shift * sample_rate) % 1.0) - 0.5)

        # i1, q1 = np.real(ramps[ramp_index][index]), np.imag(ramps[ramp_index][index])
        # i2, q2 = np.real(ramps[ramp_index + 1][index]), np.imag(ramps[ramp_index + 1][index])

        # recovered_phase_shift = np.arctan((q2 - q1) / (i2 - i1))

        phases[index] = recovered_phase_shift
        V_r = (wavelength * recovered_phase_shift) / (4 * pi / sample_rate) 
        # print("%0.4f m/s" % V_r)
        vel[index] = V_r

    # get distance x-axis
    dist_bb = freq_bb * c / slope

    fig = plt.figure(figsize=(16, 8))
    ax = plt.axes()
    ax.set_title('Velocity between %i and %i' % (ramp_index, ramp_index + 1), fontsize=24)
    ax.set_xlabel('Distance [m]', fontsize=22)
    # ax.set_xlabel('Frequency [kHz]', fontsize=22)
    ax.set_ylabel('Velocity [m/s]', fontsize=22)
    ax.plot(dist_bb, vel)
    ax.axhline(y=0, c='orange', linestyle='--')
    # ax.set_ylim([-20e3, 20e3])

    ax2 = ax.twinx()
    ax2.plot(dist_bb, phases / np.pi, c='r')
    ax2.set_ylabel('Phase shift [$\\frac{rad}{\pi}$]', fontsize=22)

# fig = plt.figure(figsize=(16, 8))

# ax = plt.axes()
# ax.set_title('fft', fontsize=24)
# ax.set_xlabel('Frequency [kHz]', fontsize=22)
# ax.set_ylabel('Velocity [m/s]', fontsize=22)
# ax.plot(freq_bb / 1e3, 10 * np.log10(fft(ramps[2])))
# %%
plt.show()