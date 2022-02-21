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

import time
from venv import create
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import faulthandler
faulthandler.enable()

# Signal processing stuff
from numpy.fft import fft, ifft, fftshift, fftfreq
from numpy import absolute, pi
from target_detection import cfar

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


sample_rate = 0.6e6
center_freq = 2.1e9
signal_freq = 100e3
num_slices = 200
fft_size = 1024*16
# fft_size = 2**10

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
BW = 500e6
num_steps = 1000
ramp_time = 1e3 # us
ramp_time_s = ramp_time / 1e6
my_phaser.frequency = int(output_freq/4) # Output frequency divided by 4
my_phaser.freq_dev_range = int(BW/4) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
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

"""
    Print config
"""
print("""
CONFIG:
Sample rate: {sample_rate}MHz
Num samples: 2^{Nlog2}
Bandwidth: {BW}MHz
Ramp time: {ramp_time}ms
Output frequency: {output_freq}MHz
""".format(sample_rate=sample_rate / 1e6, Nlog2=int(np.log2(fft_size)), 
    BW=BW / 1e6, ramp_time=ramp_time / 1e3, output_freq=output_freq / 1e6))

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
my_sdr._ctx.set_timeout(0)
my_sdr.tx([iq*0, iq])  # only send data to the 2nd channel (that's all we need)

phases_0 = np.zeros(8)
phases_0[:4] = 15

def plot_rx(realtime_plot=False, plot_dist=False, cfar_params=None, yaxis_limits=[20, 50],
        masked=False, scanning=False):

    if (scanning):
        for i in range(phases_0.size):
            my_phaser.set_chan_phase(i, phases_0[i])

    # Collect raw data buffer, take DFT, and do basic processing
    x_n = my_sdr.rx()
    x_n = x_n[0] + x_n[1]

    X_k = absolute(fft(x_n))
    X_k = fftshift(X_k)

    # Create figure
    plt.ion()
    fig, ax = plt.subplots()
    fig.set_figheight(8)
    fig.set_figwidth(16)
    ax.set_ylabel("Magnitude [dB]", fontsize=22)
    ax.set_ylim(yaxis_limits)

    # Create frequency axis
    N = len(X_k)
    freq = fftshift(fftfreq(N, 1 / sample_rate))
    freq_kHz = freq / 1e3

    # Create range-FFT scale
    c = 3e8
    slope = BW / ramp_time_s
    dist = (c / slope) * freq

    if (plot_dist):
        ax.set_xlim([-15, 15])
        ax.set_xlabel("Distance [m]", fontsize=22)
        xaxis = dist
    else:
        # ax.set_xlim([-30, 30])
        ax.set_xlabel("Frequency [kHz]", fontsize=22)
        xaxis = freq_kHz

    if (cfar_params):
        # Get CFAR values and mask non-targets
        cfar_values, targets_only = cfar(X_k, cfar_params['num_guard_cells'], 
            cfar_params['num_ref_cells'], cfar_params['bias'], cfar_params['method'])
        if (masked):
            X_k = targets_only
        line1, = ax.plot(xaxis, 10 * np.log10(X_k), c='b', label="Received targets")
        line2, = ax.plot(xaxis, 10 * np.log10(cfar_values), c='r', label="CFAR Threshold")

        ax.set_title("Received Signal - Frequency Domain\nCFAR Method: %s" % 
            (cfar_params['method']), fontsize=24)
        ax.legend(fontsize=18, loc='upper left')
    else:
        line1, = ax.plot(xaxis, 10 * np.log10(X_k), c='b')
        ax.set_title("Received Signal - Frequency Domain", fontsize=24)

    while (realtime_plot):
        x_n = my_sdr.rx()
        x_n = x_n[0] + x_n[1]

        X_k = absolute(fft(x_n))
        X_k = fftshift(X_k)

        # Get CFAR values and mask non-targets
        if (cfar_params):
            cfar_values, targets_only = cfar(X_k, cfar_params['num_guard_cells'], 
                cfar_params['num_ref_cells'], cfar_params['bias'], cfar_params['method'])
            if (masked):
                X_k = targets_only
            line1.set_ydata(10 * np.log10(X_k))
            line2.set_ydata(10 * np.log10(cfar_values))
        else:
            line1.set_ydata(10 * np.log10(X_k))
        fig.canvas.draw()
        fig.canvas.flush_events()

    fig.canvas.draw()
    return fig

"""
    Function: create_figures()
    Description: Creates and saves figures for use in my thesis. This includes all CFAR methods
        both with and without the values below the threshold masked. 
"""
def create_figures(num_guard_cells, num_ref_cells, bias, fig_dir='Figures/'):
    cfar_methods = ['greatest', 'average', 'smallest']
    cfar_params = {
        'num_guard_cells': num_guard_cells,
        'num_ref_cells': num_ref_cells,
        'bias': bias,
        'method': '',
    }
    for method in cfar_methods:
        cfar_params['method'] = method

        fig = plot_rx(realtime_plot=False, plot_dist=False, cfar_params=cfar_params, masked=True)
        output_filename = 'CFAR_' + method + '_Masked_Frequency.png'
        fig.savefig(fig_dir + output_filename)

        fig = plot_rx(realtime_plot=False, plot_dist=True, cfar_params=cfar_params, masked=True)
        output_filename = 'CFAR_' + method + '_Masked_Distance.png'
        fig.savefig(fig_dir + output_filename)

        fig = plot_rx(realtime_plot=False, plot_dist=False, cfar_params=cfar_params, masked=False)
        output_filename = 'CFAR_' + method + '_Frequency.png'
        fig.savefig(fig_dir + output_filename)

        fig = plot_rx(realtime_plot=False, plot_dist=True, cfar_params=cfar_params, masked=False)
        output_filename = 'CFAR_' + method + '_Distance.png'
        fig.savefig(fig_dir + output_filename)

if __name__ == '__main__':
    cfar_params = {
        'num_guard_cells': 10,
        'num_ref_cells': 30,
        'bias': 2,
        'method': 'average',
    }
    fig1 = plot_rx(False, False, yaxis_limits=[0, 40], scanning=False)
    fig1_fname = 'fig1.png'
    fig1.savefig('Figures/' + fig1_fname)

    fig2 = plot_rx(False, False, yaxis_limits=[0, 40], scanning=True)
    fig2_fname = 'fig2.png'
    fig2.savefig('Figures/' + fig2_fname)

    # create_figures(10, 30, 2)
    # plot_rx(True, True, yaxis_limits=[0, 40], cfar_params=cfar_params, scanning=True)