# %%
#  https://github.com/analogdevicesinc/pyadi-iio/blob/ensm-example/examples/pluto.py


import time
import sys
import iio
import adi
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# Create radio
sdr = adi.Pluto()

'''Configure Rx properties'''
sdr.sample_rate = int(10e6)
#sdr.filter = "/home/pi/Documents/PlutoFilters/samprate_40p0.ftr"  #pyadi-iio auto applies filters based on sample rate
#sdr.rx_rf_bandwidth = int(20e6)
sdr.rx_buffer_size = int(2**14)
sdr.rx_lo = int(2e9)
sdr.gain_control_mode_chan0 = "slow_attack"
#sdr.gain_control_mode_chan0 = "manual"
#sdr.rx_hardwaregain_chan0 = int(40)

'''Configure Tx properties'''
#sdr.tx_rf_bandwidth = int(10e6)
sdr.tx_lo = int(2e9)
#sdr.tx_buffer_size = int(2**18)
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = -20
# sdr.dds_enabled = [1, 1, 1, 1]                  #DDS generator enable state (if enabled, then don't use the sdr.tx(iq) command below
# sdr.dds_frequencies = [0.1e6, 0.1e6, 0.1e6, 0.1e6]      #Frequencies of DDSs in Hz
# sdr.dds_scales = [1, 1, 0, 0]                   #Scale of DDS signal generators Ranges [0,1]            

# Read properties
print("RX LO %s" % (sdr.rx_lo))

# Create a sinewave waveform
fs = int(sdr.sample_rate)
fc = 3000000
N = 2**14
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = i + 1j * q

# Send data
sdr.tx(iq)       # Send Tx data.  But don't use this if the dds generator is enabled above

# Collect data
for r in range(40):    # grab several buffers to give the AGC time to react (if AGC is set to "slow_attack" instead of "manual")
    data = sdr.rx()

NumSamples = len(data)          #number of samples
win = np.hamming(NumSamples)
y = data * win
sp = np.absolute(np.fft.fft(y))
sp = sp[1:-1]
sp = np.fft.fftshift(sp)
s_mag = np.abs(sp) * 2 / np.sum(win)    # Scale FFT by window and /2 since we are using half the FFT spectrum
s_dbfs = 20*np.log10(s_mag/(2**12))     # Pluto is a 12 bit ADC, so use that to convert to dBFS
xf = np.fft.fftfreq(NumSamples, ts)
xf = np.fft.fftshift(xf[1:-1])/1e6
plt.plot(xf, s_dbfs)
plt.xlabel("frequency [MHz]")
plt.ylabel("dBfs")
plt.draw()
plt.pause(0.05)
time.sleep(0.1)

plt.show()

