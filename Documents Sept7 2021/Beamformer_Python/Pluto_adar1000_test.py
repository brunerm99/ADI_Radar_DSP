#  https://github.com/analogdevicesinc/pyadi-iio/blob/ensm-example/examples/pluto.py

import time
import sys
try:
    import iiopy as iio
except:
    # By default the iio python bindings are not in path
    #sys.path.append('/usr/lib/python2.7/site-packages/')
    sys.path.append('/lib/python3.7/site-packages/')
    import iiopy as iio
import adi
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import spidev
from ADAR_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
from SDR_functions import *    #import the SDR functions (These all start with SDR_xxxx)

spi = spidev.SpiDev()
spi.open(0, 0)  # set bus = 0 and chip select (device) = 0
spi.max_speed_hz = 500000
spi.mode = 0
  
'''SET DEFAULT VALUES'''
sdr_address = 'ip:192.168.2.1'   # This is the default Pluto address (You can check/change this in the config.txt file on the Pluto "usb drive")
SignalFreq = 10.495e9
TX_freq    = 5.81e9          # TX LO freq.  This is the freq that Pluto generates (if using the Tx of Pluto as a crude LO)
LO_freq    = int(SignalFreq-TX_freq)              # RX LO freq
SampleRate = 40e6
Rx_gain = 25
Averages = 1
Taper = 0
SymTaper = 0
RxGain1 = 127
RxGain2 = 127
RxGain3 = 127
RxGain4 = 127
RxGain5 = 127
RxGain6 = 127
RxGain7 = 127
RxGain8 = 127
Rx1_cal = -19.6875   # you can put phase cal values here (to compensate for phase mismatches in the lines, etc.)
Rx2_cal = 0
Rx3_cal = -8.4375
Rx4_cal = 16.8750
Rx5_cal = 0
Rx6_cal = 0
Rx7_cal = 0
Rx8_cal = 0
ADDR1=0x20         
ADDR2=0x40         
num_ADARs = 1      # Number of ADAR1000's connected -- this can be either 1 or 2. no other values are allowed
num_Rx = 1         # Number of Rx channels (i.e. Pluto this must be 1, but AD9361 SOM this could be 1 or 2)
sel_Tx = 0         # 0 = Rx mode, 1 = Tx mode
'''Intialize the ADAR1000 and SDR'''
sdr = SDR_init(sdr_address, SampleRate, TX_freq, LO_freq, Rx_gain)
ADAR_init(spi, ADDR1)
ADAR_set_RxTaper(spi, ADDR1, RxGain1, RxGain2, RxGain3, RxGain4)
ADAR_set_RxPhase(spi, ADDR1, num_ADARs, 0, 2.8125, Rx1_cal, Rx2_cal, Rx3_cal, Rx4_cal)
ADAR_update_Rx(spi, ADDR1)

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
ts = 1 / float(SampleRate)
xf = np.fft.fftfreq(NumSamples, ts)
xf = np.fft.fftshift(xf[1:-1])/1e6
plt.plot(xf, s_dbfs)
plt.xlabel("frequency [MHz]")
plt.ylabel("dBfs")
plt.draw()
plt.pause(0.05)
time.sleep(0.1)

plt.show()

