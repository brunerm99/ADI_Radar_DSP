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
import time
import matplotlib.pyplot as plt
import numpy as np
from numpy import log10
from numpy.fft import fft, ifft, fftshift
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
sample_rate = 3e6
center_freq = 2.1e9
signal_freq = 100e3
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
my_sdr.tx_hardwaregain_chan1 = -0   # must be between 0 and -88

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
tdd.burst_count = 20       # there is a burst of 20 toggles, then off for a long time
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
fc = int(0.5e6 / (fs / N)) * (fs / N)
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 0.9* (i + 1j * q)

# Send data
my_sdr.tx([iq*0.5, iq])  # only send data to the 2nd channel (that's all we need)

my_sdr._ctx.set_timeout(30000)
my_sdr._rx_init_channels() 
    
# %%

# time.sleep(3)    
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

# %%

frame_length = tdd.frame_length_ms / 1e3
num_bursts = tdd.burst_count

N = chan1.size
time = np.linspace(0, ts * N, N)

# %%

fig1, ax1 = plt.subplots(figsize=(16, 8))
ax1.plot(time * 1e3, np.abs(chan1))
ax1.set_title('Received Signal', fontsize=20)
ax1.set_xlabel('Time [ms]', fontsize=18)

# %%
"""
    Looking at the 4rd and 5th frames received.
    Plot their frequency spectrums overlaid and try plotting with CCD applied. 
"""

index_step = int(frame_length / ts)
rx_bursts = np.zeros((num_bursts, index_step), dtype=complex)
for burst in range(num_bursts):
    rx_bursts[burst] = chan1[burst * index_step:(burst + 1) * index_step]

fig1, ax1 = plt.subplots(figsize=(16, 8))
freq = np.linspace(-fs / 2, fs / 2, index_step)

c = 3e8
wavelength = c / output_freq
ramp_time_s = ramp_time / 1e6
slope = (BW * 4) / ramp_time_s
dist = freq * c / (2 * slope)

burst_fft_1 = fft(rx_bursts[3])
burst_fft_2 = fft(rx_bursts[4])

for burst_index in range(1, num_bursts):
    burst_fft_tmp = fft(rx_bursts[burst_index])

    # ax1.plot(dist, log10(fftshift(burst_fft_1)), label='First')
    ax1.plot(dist, log10(fftshift(abs(burst_fft_2))), label=burst_index)

ax1.set_title('Frequency Spectrum\nSpacing: %0.2fms' % (frame_length * 1e3), fontsize=24)
ax1.set_xlabel('Range [m]]', fontsize=22)
# ax1.set_xlabel('Frequency [Hz]', fontsize=22)
ax1.set_ylabel('Magnitude [dB]', fontsize=22)
ax1.legend(loc='upper right', fontsize=16)
ax1.set_xlim([-10, 10])

# %%

fig2, ax2 = plt.subplots(figsize=(16, 8))

ccd = rx_bursts[4] - rx_bursts[3]
ccd_fft = fft(ccd)
ax2.plot(freq, log10(fftshift(ccd_fft)))
# ax2.plot(abs(rx_bursts[3]))
# ax2.plot(abs(rx_bursts[4]))
ax2.set_xlabel('n', fontsize=22)
ax2.set_title('CCD', fontsize=24)

# %%
"""
    1. Iterate through whole spectrum
    2. Grab (2 * ~num_points~ + 1) frequencies per iteration (sweeping window)
    3. Calculate phase difference between windows from each frame
    4. Calculate velocity from phase difference
"""

lags = signal.correlation_lags(index_step, index_step, mode='full')
dt = np.linspace(-frame_length, frame_length, 2 * index_step - 1)
phases = np.zeros(index_step)
time_shifts = np.zeros(index_step)
vel = np.zeros(index_step)

rx_burst_fft_1 = fft(rx_bursts[3])
rx_burst_fft_2 = fft(rx_bursts[4])

num_points = 20
window_freq = (num_points * 2 + 1) * sample_rate / index_step
window_range = window_freq * c / (8 * BW)
print('Window size: %0.2fHz or %0.2fm' % (window_freq, window_range))


empty = np.zeros(index_step, dtype=complex)
for index in range(index_step):
    single_tone_1 = np.copy(empty)
    single_tone_2 = np.copy(empty)

    offset_left     = min(num_points, index)
    offset_right    = min(num_points, index_step - index)

    single_tone_1[index - offset_left:index + offset_right] = rx_burst_fft_1[index - offset_left:index + offset_right]
    single_tone_2[index - offset_left:index + offset_right] = rx_burst_fft_2[index - offset_left:index + offset_right]

    single_tone_1_ifft = ifft(single_tone_1)
    single_tone_2_ifft = ifft(single_tone_2)

    Rxx = signal.correlate(single_tone_2_ifft, single_tone_1_ifft, mode='full')

    # Obtain max value of correlation function
    max_corr = Rxx.argmax()
    lag_max = lags[max_corr]

    # Time shift amount is where the max correlation occurs
    recovered_time_shift = dt[max_corr]
    recovered_phase_shift = 2 * np.pi * (((recovered_time_shift / frame_length)))
    
    time_shifts[index] = recovered_time_shift
    phases[index] = recovered_phase_shift
    V_r = (wavelength * recovered_phase_shift) / (4 * np.pi / frame_length) 
    # print("%0.4f m/s" % V_r)
    vel[index] = V_r

# %%

fig, ax = plt.subplots(figsize=(16, 8))
ax.set_title('Last cross-correlation')
ax.plot(lags, Rxx)

# %%
"""
    Plot time shift, phase shift, and velocity as a function of frequency/distance
"""

fig, ax = plt.subplots(figsize=(16, 8))
ax2 = ax.twinx()
ax.plot(dist, fftshift(time_shifts))
ax2.plot(dist, fftshift(phases), c='r')
# ax.set_xlim([0, 500e3])
# ax.set_xlim([0, 30])
ax.set_title('Phase and time shifts')
ax.set_xlabel('Distance [m]')
ax.set_ylabel('Time shift [s]')
ax2.set_ylabel('Phase shift [rad]')

# %%

fig, ax = plt.subplots(figsize=(16, 8))
ax.plot(dist, fftshift(vel))
ax.set_xlim([0, 30])
ax.set_title('Range-Velocity', fontsize=24)
ax.set_xlabel('Distance [m]', fontsize=22)
ax.set_ylabel('Velocity [m/s]', fontsize=22)

# %%

# time_single_frame = np.linspace(0, frame_length, index_step)

# fig2, ax2 = plt.subplots(figsize=(16, 8))
# ax2.plot(time_single_frame * 1e3, single_tone_1_ifft)
# ax2.set_xlabel('Time [ms]')
# ax2.set_xlim([0, 0.2])


# %%

# fig2, ax2 = plt.subplots(figsize=(16, 8))
# ax2.specgram(chan1, Fs=fs)
# ax2.set_ylabel('Frequency [Hz]')
# ax2.set_xlabel('Time [s]')
plt.show()