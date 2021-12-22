#!/usr/bin/env python
#  Must use Python 3

import sys
sys.path.append('/lib/python3.7/site-packages/')
import warnings
import time
import numpy as np
import matplotlib.pyplot as plt
import adi

from ADAR_pyadi_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
from SDR_functions import *    #import the SDR functions (These all start with SDR_xxxx)
try:
    from pluto_config import *    # this has all the key parameters that the user would want to change (i.e. calibration phase and antenna element spacing)
except:
    print("Make sure that the file pluto_config_xxxx.py is in the same directory as this python file.")
    sys.exit(0)

sdr_address = 'ip:192.168.2.1'   # Pluto IP address
LO_freq    = 12.3e9       # Frequency to program the ADF4159's VCO to.  It is the external LO of the LTC555x mixers
SignalFreq = 10e9
Rx_freq    = LO_freq - SignalFreq
Tx_freq = Rx_freq
SampleRate = 4e6
Rx_gain = 20
Tx_gain = -3
RxGain1 = 127
RxGain2 = 127
RxGain3 = 127
RxGain4 = 127
Rx1_cal = 0  # you can put phase cal values here (to compensate for phase mismatches in the lines, etc.)
Rx2_cal = 0
Rx3_cal = 0
Rx4_cal = 0

print("Frequencies:\nLO: {}MHz\nRF: {}MHz\nRx: {}MHz\n".format(LO_freq / 1e6, SignalFreq / 1e6, Rx_freq / 1e6))

'''Intialize Pluto'''
sdr = SDR_init(sdr_address, 1, SampleRate, Tx_freq, Rx_freq, Rx_gain, Tx_gain)
buff_size = 2**14
sdr.rx_buffer_size = int(buff_size)
print("Capture time: {}ms".format((buff_size / SampleRate) * 1e3))
sdr.dds_single_tone(int(0), 0.9, 0)    # sdr.dds_single_tone(tone_freq_hz, tone_scale_0to1, tx_channel)

'''Intialize ADF4159 and Set to Ramping'''
pll = adi.adf4159()
output_freq = int(LO_freq)
pll.frequency = int(output_freq/4)
BW = 500e6
c = 3e8
num_steps = 1000
on_time = 1e-3
delay_num = 4095   # max of 4095
delay_time = delay_num*(1/100e6)  # delay time if delay_clk is set to 0
slope = BW / on_time
print("BW: {}MHz".format(BW / 1e6))
print("Ramp time: {}ms".format((on_time) * 1e3))
print("Delay time: {}ms".format((delay_time) * 1e3))
print("Total ramp time: {}ms".format((on_time + delay_time) * 1e3))
print("Slope: {}GHz/sec".format(slope / 1e9))
pll.freq_dev_range = int(BW) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
pll.freq_dev_step = int(BW/num_steps) # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
pll.freq_dev_time = int(on_time) # total time (in us) of the complete frequency ramp
pll.ramp_mode = "continuous_sawtooth"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
pll.delay_word = delay_num     # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
pll.delay_clk = 0    
pll.delay_start_en = 0         # delay start
pll.ramp_delay_en = 1          # delay between ramps.  
pll.trig_delay_en = 0          # triangle delay
pll.sing_ful_tri = 0           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
pll.tx_trig_en = 0             # start a ramp with TXdata
pll.enable = 0                 # 0 = PLL enable.  Write this last to update all the registers

'''Intialize ADAR1000'''
beam0 = adi.adar1000(
    chip_id="BEAM0",
    array_element_map=[[1, 2, 3, 4]],
    channel_element_map=[2, 1, 4, 3])
ADAR_init(beam0)
ADAR_set_mode(beam0, "rx")
ADAR_set_Taper(beam0, "rx", RxGain1, RxGain2, RxGain3, RxGain4)
ADAR_set_Phase(beam0, 'rx', 0, Rx1_cal, Rx2_cal, Rx3_cal, Rx4_cal)
data=SDR_getData(sdr)

# Sampling period
t_sample = 1 / SampleRate
print("Sampling Rate: {}MHz\nSampling Period: {}us".format(SampleRate / 1e6, t_sample * 1e6))


# Plot time domain samples
fig1 = plt.figure(1)
plt.plot(np.array(range(data.size)) * t_sample * 1e3, np.real(data))
plt.title("Sampled Data\nSamples: {}".format(buff_size))
plt.xlabel("Time (Sampled) [ms]")
plt.ylabel("Amplitude")
# plt.show()

### Processing ###
data_fft = np.fft.fft(data)
data_fft = np.abs(data_fft)
# data_fft = 10 * np.log10(data_fft / np.max(data_fft))
fft_freq = np.linspace(-1 * SampleRate, SampleRate, buff_size)
distance = (fft_freq * c) / (2 * slope)
data_fft = np.fft.fftshift(data_fft)

# Plot FFT
fig2 = plt.figure(2)
plt.plot(distance, data_fft)
plt.title("FFT\nSamples: {}".format(buff_size))
plt.xlim((-10, 40))
#plt.ylim((0, 250000))
#plt.xlabel("Frequency [kHz]")
plt.xlabel("Distance [m]")
plt.ylabel("Amplitude")
plt.show()

'''
Get subsets of the data containing 1 chirp each. Subtract subsequenct chirps to get only the data that matters.
This will only work for moving targets.
'''


'''try to use correlation to find the start of each ramp'''
'''warnings.filterwarnings('ignore')   # ignore numpy's real/complex warning
ramp_mask = np.append(np.ones(int(SampleRate*on_time)), np.zeros(int(SampleRate*delay_time)))
N = (on_time+delay_time)*SampleRate - 1
print('Number capture points per ramp = ', N)
fs = int(sdr.sample_rate)
fc = int(100000 / (SampleRate / N)) * (SampleRate / N)
ts = 1 / float(SampleRate)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = i + 1j * q

ramp_corr = np.correlate(iq*ramp_mask, data, 'full')
plt.plot(ramp_corr)
plt.show()'''


