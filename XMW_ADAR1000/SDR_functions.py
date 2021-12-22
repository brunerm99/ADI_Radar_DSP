# SDR_functions.py 
# These are Pluto control functions

import time
import sys
import numpy as np
try:
    import iio
except:
    # By default the iio python bindings are not in path
    #sys.path.append('/usr/lib/python2.7/site-packages/')
    sys.path.append('/lib/python3.7/site-packages/')
    import iio
import adi
from ADAR_pyadi_functions import *   #import the ADAR1000 write functions (like ADAR_init and writeBeam functions)

def SDR_LO_init(LO_freq):   #program the ADF4159 to be the LO of the external LTC555x mixers
    pll = adi.adf4159()
    pll.frequency = int(LO_freq/4) # Output frequency divided by 4
    pll.enable = 0 # Power down
    #pll.freq_dev_range = 0 # frequency deviation range
    #pll.freq_dev_step = 5690 # frequency deviation step
    #pll.freq_dev_time = 0 # frequency deviation time
    pll.ramp_mode = "disabled"

def SDR_init(sdr_address, NumRx, SampleRate, TX_freq, RX_freq, Rx_gain, Tx_gain):
    '''Setup contexts'''
    if NumRx==1:  # 1 Rx, so this is Pluto
        #sdr=adi.Pluto()     #This finds pluto over usb.  But communicating with its ip address gives us more flexibility
        sdr=adi.Pluto(uri=sdr_address)      #This finds the device at that ip address
    if NumRx==2:  # 2 Rx, so this is the ADRV9361-SOM
        #sdr=adi.ad9361(uri='ip:192.168.0.3')
        sdr = adi.ad9361(uri=sdr_address)
        sdr._ctrl.debug_attrs["adi,frequency-division-duplex-mode-enable"].value = "1"  # move to fdd mode.  see https://github.com/analogdevicesinc/pyadi-iio/blob/ensm-example/examples/ad9361_advanced_ensm.py
        sdr._ctrl.debug_attrs["adi,ensm-enable-txnrx-control-enable"].value = "0"       # Disable pin control so spi can move the states
        sdr._ctrl.debug_attrs["initialize"].value = "1"
        sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
        sdr.gain_control_mode_chan0 = 'manual'      #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
        sdr.gain_control_mode_chan1 = 'manual'      #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
    sdr._rxadc.set_kernel_buffers_count(1)   #Default is 4 Rx buffers are stored, but we want to change and immediately measure the result, so buffers=1
    rx = sdr._ctrl.find_channel('voltage0') 
    rx.attrs['quadrature_tracking_en'].value = '1'   # set to '1' to enable quadrature tracking
    sdr.sample_rate = int(SampleRate)
    #sdr.filter = "/home/pi/Documents/PlutoFilters/samprate_40p0.ftr"  #pyadi-iio auto applies filters based on sample rate
    #sdr.rx_rf_bandwidth = int(40e6)
    #sdr.tx_rf_bandwidth = int(1e6)
    sdr.rx_buffer_size = int(4*256)
    sdr.tx_lo = int(TX_freq)
    sdr.tx_cyclic_buffer = True
    sdr.tx_buffer_size = int(2**18)
    if NumRx==1:
        sdr.tx_hardwaregain_chan0 = int(Tx_gain)  # this is a negative number between 0 and -88
        sdr.gain_control_mode_chan0 = "manual"                #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
        sdr.rx_hardwaregain_chan0 = int(Rx_gain)
    if NumRx==2:
        sdr.tx_hardwaregain_chan0 = int(Tx_gain)   # Make sure the Tx channels are attenuated (or off) and their freq is far away from Rx
        sdr.tx_hardwaregain_chan1 = int(Tx_gain)
        sdr.rx_hardwaregain_chan0 = int(Rx_gain)
        sdr.rx_hardwaregain_chan1 = int(Rx_gain)
    #sdr.dds_enabled = [1, 1, 1, 1]                  #DDS generator enable state
    #sdr.dds_frequencies = [0.1e6, 0.1e6, 0.1e6, 0.1e6]      #Frequencies of DDSs in Hz
    #sdr.dds_scales = [1, 1, 0, 0]                   #Scale of DDS signal generators Ranges [0,1]            
    sdr.dds_single_tone(int(0.0e6), 0.9, 0)    # sdr.dds_single_tone(tone_freq_hz, tone_scale_0to1, tx_channel)
    sdr.rx_lo = int(RX_freq)
    return sdr

def SDR_setRx(sdr, NumRx, RX_freq, Rx_gain):
    sdr.rx_lo = int(RX_freq)
    if NumRx==1:
        sdr.rx_hardwaregain_chan0 = int(Rx_gain)
    if NumRx==2:
        sdr.rx_hardwaregain_chan0 = int(Rx_gain)
        sdr.rx_hardwaregain_chan1 = int(Rx_gain)

def SDR_getData(sdr):
    data=sdr.rx()          #read a buffer of data from Pluto using pyadi-iio library (adi.py)        
    return data

def SDR_get_dBFS(sdr, Averages, num_Rx):
    # Get signal level from Pluto and return dBFS measurement
    total=0
    for count in range (0, Averages):
        data_raw=SDR_getData(sdr)
        data = data_raw
        NumSamples = len(data)          #number of samples
        win = np.blackman(NumSamples)
        y = data * win
        sp = np.absolute(np.fft.fft(y))
        sp = sp[1:-1]
        sp = np.fft.fftshift(sp)
        s_mag = np.abs(sp) * 2 / np.sum(win)    # Scale FFT by window and /2 since we are using half the FFT spectrum
        s_mag = np.maximum(s_mag, 10**(-15))
        s_dbfs = 20*np.log10(s_mag/(2**12))     # Pluto is a 12 bit ADC, so use that to convert to dBFS
        total=total+max(s_dbfs)   # sum up all the loops, then we'll average
    PeakValue=total/Averages
    return PeakValue

