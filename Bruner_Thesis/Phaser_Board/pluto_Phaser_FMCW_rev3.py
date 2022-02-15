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

"""
Simple FMCW demo with PHASER and ADALM-PLUTO
waterfall plot modified from:  https://amyboyle.ninja/Pyqtgraph-live-spectrogram
"""

import PyQt5
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import faulthandler
faulthandler.enable()

import adi


# Instantiate all the Devices
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

# Initialize both ADAR1000s, set gains to max, and all phases to 0
my_phaser.configure(device_mode="rx")
for i in range(0, 8):
    my_phaser.set_chan_gain(i, 127)
    my_phaser.set_chan_phase(i, 0)


sample_rate = 2e6
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
my_sdr.rx_buffer_size = int(fft_size)
my_sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
my_sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
my_sdr.rx_hardwaregain_chan0 = int(30)   # must be between -3 and 70
my_sdr.rx_hardwaregain_chan1 = int(30)   # must be between -3 and 70
# Configure Tx
my_sdr.tx_lo = int(center_freq)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True      # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
my_sdr.tx_hardwaregain_chan0 = -88   # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = -0   # must be between 0 and -88

# Enable TDD logic in pluto (this is for synchronizing Rx Buffer to ADF4159 TX input)
#gpio = adi.one_bit_adc_dac(sdr_ip)
#gpio.gpio_phaser_enable = True

# Configure the ADF4159 Rampling PLL
output_freq = 12.1e9
BW = 500e6/4
num_steps = 1000
ramp_time = 1e3
my_phaser.frequency = int(output_freq/4) # Output frequency divided by 4
my_phaser.freq_dev_range = int(BW) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
my_phaser.freq_dev_step = int(BW/num_steps) # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
my_phaser.freq_dev_time = int(ramp_time) # total time (in us) of the complete frequency ramp
my_phaser.delay_word = 4095     # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
my_phaser.delay_clk = 'PFD'     # can be 'PFD' or 'PFD*CLK1'
my_phaser.delay_start_en = 0         # delay start
my_phaser.ramp_delay_en = 0          # delay between ramps.  
my_phaser.trig_delay_en = 0          # triangle delay
my_phaser.ramp_mode = "continuous_triangular"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
my_phaser.sing_ful_tri = 0           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
my_phaser.tx_trig_en = 0             # start a ramp with TXdata
# Enable ADF4159 TX input and generate a single triangular ramp with each trigger
# my_phaser.ramp_mode = "single_ramp_burst"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
# my_phaser.sing_ful_tri = 1           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
# my_phaser.tx_trig_en = 1             # start a ramp with TXdata
my_phaser.enable = 0                 # 0 = PLL enable.  Write this last to update all the registers


# Configure the TDD logic to instruct how many ADF4159 ramps will appear in one Pluto Rx buffer
# tdd = adi.tdd(sdr_ip)
# tdd.frame_length_ms = 4    # each GPIO toggle is spaced 4ms apart
# tdd.burst_count = 2       # there is a burst of 20 toggles, then off for a long time
# tdd.rx_rf_ms = [0.5,0.6, 0, 0]    # each GPIO pulse will be 100us (0.6ms - 0.5ms).  And the first trigger will happen 0.5ms into the buffer
# tdd.secondary = False
# tdd.en = True

# # buffer size needs to be greater than the frame_time
# frame_time = tdd.frame_length_ms*tdd.burst_count/2
# print("frame_time:", frame_time, "ms")
# buffer_time = 0
# power=12
# while frame_time > buffer_time:     
#     power=power+1
#     buffer_size = int(2**power)
#     buffer_time = buffer_size/my_sdr.sample_rate*1000
#     if power==23:
#         break     # max pluto buffer size is 2**23, but for tdd burst mode, set to 2**22
# print("buffer_size:", buffer_size)
# my_sdr.rx_buffer_size = buffer_size
# print("buffer_time:", buffer_time, " ms")  

# DDS send data (either do this or the Tx Method -- not both)
#my_sdr.dds_single_tone(int(3e6), 0.9, 0)   # my_sdr.dds_single_tone(tone_freq_hz, tone_scale, tx_channel)

# Create a sinewave waveform
fs = int(my_sdr.sample_rate)
print("sample_rate:", fs)
N = int(my_sdr.rx_buffer_size)
fc = int(signal_freq / (fs / N)) * (fs / N)
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 1 * (i + 1j * q)

# Send data
my_sdr._ctx.set_timeout(30000)
my_sdr.tx([iq*0, iq])  # only send data to the 2nd channel (that's all we need)


# create the object for drawing the waterfall plot
class PlutoSDR():
    def __init__(self, signal):
        self.signal = signal
    def read(self):
        raw_data = my_sdr.rx()
        data=raw_data[0]+raw_data[1]
        self.signal.emit(data)

class SpectrogramWidget(pg.PlotWidget):
    read_collected = QtCore.pyqtSignal(np.ndarray)
    def __init__(self):
        super(SpectrogramWidget, self).__init__()

        self.img = pg.ImageItem()
        self.addItem(self.img)
        self.setWindowTitle("Waterfall Plot")
        # self.setAntialiasing(aa=True)
        self.img_array = np.zeros((num_slices, fft_size-2))
        self.img.scale(100e-3, sample_rate/N)    # scale horizontal and vertical axes
        #self.setRange(yRange=(0,sample_rate/2))  # limit plot range to the first Nyquist zone frequencies
        zoom_freq = 10e3
        #self.setRange(yRange=(sample_rate/2-zoom_freq,sample_rate/2+zoom_freq))  # zoom in on the radar return
        self.setRange(yRange=(sample_rate/2+signal_freq-zoom_freq,sample_rate/2+signal_freq))  # zoom in on the signal frequency
        #self.translate(0, -sample_rate/2)
        self.setLabel('left', 'Frequency', units='Hz')
        self.setLabel('bottom', 'Time', units = 'sec')
        self.show()

    def update(self, data):
        win = np.blackman(len(data))
        y = data * win
        sp = np.absolute(np.fft.fft(y))
        sp = sp[1:-1]
        sp = np.fft.fftshift(sp)
        s_mag = np.abs(sp) * 2 / np.sum(win)    # Scale FFT by window and /2 since we are using half the FFT spectrum
        s_mag = np.maximum(s_mag, 10**(-15))
        s_dbfs = 20*np.log10(s_mag/(2**12))     # Pluto is a 12 bit ADC, so use that to convert to dBFS

        # roll down one and replace leading edge with new data
        self.img_array = np.roll(self.img_array, -1, axis=0)
        self.img_array[-1] = s_dbfs
        self.img.setLevels([-60,-20])
        self.img.setImage(self.img_array, autoLevels=False)
        

if __name__ == '__main__':
    app = QtGui.QApplication([])
    w = SpectrogramWidget()
    w.read_collected.connect(w.update)
    pluto = PlutoSDR(w.read_collected)

    # time (seconds) between reads
    t = QtCore.QTimer()
    t.timeout.connect(pluto.read)
    t.start(100) #QTimer takes ms

    app.exec_()
    my_sdr.tx_destroy_buffer()   # If sdr.tx_cyclic_buffer=true, then when the program ends, destroy the Tx buffer.  Otherwise, you'll get an error when you run the program again.



