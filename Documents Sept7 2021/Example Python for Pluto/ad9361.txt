import sys
sys.path.append('/usr/lib/python2.7/site-packages/')
sys.path.append('/lib/python3.7/site-packages/')
#import iio
import iiopy as iio
import adi

sdr_address = str('ip:192.168.0.4')

sdr=adi.ad9361(uri=sdr_address)
#sdr._rxadc.set_kernel_buffers_count(1)   #Default is 4 Rx buffers are stored, but we want to change and immediately measure the result, so buffers=1
#rx = sdr._ctrl.find_channel('voltage0') 
#rx.attrs['quadrature_tracking_en'].value = '0'   # set to '1' to enable quadrature tracking
print(sdr._ctrl.attrs['ensm_mode'].value)
sdr._ctrl.debug_attrs['adi,frequency-division-duplex-mode-enable'].value = '1'
sdr._ctrl.debug_attrs['initialize'].value = '1'
print(sdr._ctrl.attrs['ensm_mode'].value)
#print(rx.attrs['quadrature_tracking_en'].value) # we are receiving a real (not complex) signal.  Therefore disable QEC, or that tracking loop will cause amplitude variation
#sdr.rx_enabled_channels = [0]     # enable Rx1 (voltage0)
sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
sdr.sample_rate = int(40000000)
#sdr.filter = "/home/pi/Documents/PlutoFilters/samprate_40p0.ftr"  #pyadi-iio auto applies filters based on sample rate
#sdr.rx_rf_bandwidth = int(1000000)
#sdr.tx_rf_bandwidth = int(500000)
#sdr.tx_cyclic_buffer = True
#sdr.tx_buffer_size = int(2**18)
sdr.gain_control_mode_chan0 = "manual"                #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
sdr.gain_control_mode_chan1 = 'manual'                #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
#sdr._set_iio_attr("voltage1", "gain_control_mode", False, "manual")
sdr.rx_buffer_size = int(1024)    # We only need a few samples to get the gain.  And a small buffer will greatly speed up the sweep
#sdr.tx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
sdr.tx_hardwaregain_chan0 = -50
sdr.tx_hardwaregain_chan1 = -50
sdr.tx_lo = int(1000000000)
sdr.rx_lo = int(1000000000)
sdr.rx_hardwaregain_chan0 = int(33)
sdr.rx_hardwaregain_chan1 = int(33)
print(sdr.rx_hardwaregain_chan0)
