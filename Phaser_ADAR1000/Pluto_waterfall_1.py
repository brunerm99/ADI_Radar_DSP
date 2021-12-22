# Waterfall Plot with PlutoSDR
# Modified from: https://stackoverflow.com/questions/66990758/how-to-update-spectrum-waterfall-for-live-data
# Modified from: https://www.geeksforgeeks.org/how-to-update-a-plot-in-matplotlib/
# https://github.com/analogdevicesinc/pyadi-iio/tree/master/examples
# https://wiki.analog.com/resources/tools-software/linux-drivers/iio-pll/adf4159

import adi
from ADAR_pyadi_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



pll = adi.adf4159()
output_freq = 12e9
pll.frequency = int(output_freq/4) # Output frequency divided by 4
pll.enable = 0 # Power down
pll.freq_dev_range = int(100e6) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
pll.freq_dev_step = int(500e6/1000) # frequency deviation step in Hz.  This is fDEV, in Hz
pll.freq_dev_time = int(10e3) # total time (in us) of the complete frequency ramp
pll.ramp_mode = "disabled"

sdr = adi.Pluto()
# configure device
sample_rate = 2e6
center_freq = 1.8e9
fft_size = 1024*64
num_samps = fft_size * 32
sdr.sample_rate = int(sample_rate)  # Hz
sdr.rx_lo = int(center_freq)  # Hz
sdr.tx_lo = int(center_freq-0.01e6)  # Hz
#sdr.gain_control_mode_chan0 = "slow_attack"
sdr.gain_control_mode_chan0 = "manual"                #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
sdr.rx_hardwaregain_chan0 = int(30)
sdr.tx_hardwaregain_chan0 = int(0)  # this is a negative number between 0 and -88


sdr.rx_buffer_size = int(num_samps)
sdr.dds_single_tone(int(0.01e6), 0.9, 0)    # sdr.dds_single_tone(tone_freq_hz, tone_scale_0to1, tx_channel)


beam0 = adi.adar1000(
    chip_id="BEAM0",
    array_element_map=[[1, 2, 3, 4]],
    channel_element_map=[2, 1, 4, 3])
ADAR_init(beam0)
ADAR_set_mode(beam0, "rx")
ADAR_set_Taper(beam0, "rx", 127, 127, 127, 127)
ADAR_set_Phase(beam0, "rx", 0, 0, 0, 0, 0)

plt.ion()
fig = plt.figure()
ax=fig.add_subplot(111)

for x in range(0,100):
    samples = sdr.rx() # receive samples off Pluto
    # Create Waterfall matrix
    num_slices = int(np.floor(num_samps/fft_size))
    waterfall = np.zeros((num_slices, fft_size))
    for i in range(num_slices):
        waterfall[i,:] = np.log10(np.fft.fftshift(np.abs(np.fft.fft(samples[i*fft_size:(i+1)*fft_size]))**2))

    # Plot waterfall
    time_per_row = 1.0/sample_rate * fft_size
    #fmin = (center_freq - sample_rate/2.0)/1e6 # MHz
    #fmax = (center_freq + sample_rate/2.0)/1e6 # MHz
    fmin = (center_freq+0.0e6 - 1e3)
    fmax = (center_freq+0.0e6 + 1e3)
    ax.imshow(waterfall, extent=[fmin, fmax, time_per_row*num_slices, 0], aspect='auto', cmap=plt.get_cmap('jet'))
    fig.canvas.draw()





