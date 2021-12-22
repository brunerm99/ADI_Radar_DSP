# Waterfall Plot with PlutoSDR
# Kind of klunky first rev by Jon Kraft
# Modified from: https://stackoverflow.com/questions/66990758/how-to-update-spectrum-waterfall-for-live-data
# Modified from: https://www.geeksforgeeks.org/how-to-update-a-plot-in-matplotlib/
# https://github.com/analogdevicesinc/pyadi-iio/tree/master/examples
# https://wiki.analog.com/resources/tools-software/linux-drivers/iio-pll/adf4159

import adi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sdr = adi.Pluto()
# configure device
sample_rate = 2e6
center_freq = 1.8e9
num_slices = 32
fft_size = 1024
num_samps = fft_size * num_slices
sdr.sample_rate = int(sample_rate)  # Hz
sdr.rx_lo = int(center_freq)  # Hz
sdr.tx_lo = int(center_freq)  # Hz
#sdr.gain_control_mode_chan0 = "slow_attack"
sdr.gain_control_mode_chan0 = "manual"                #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
sdr.rx_hardwaregain_chan0 = int(30)
sdr.tx_hardwaregain_chan0 = int(0)  # this is a negative number between 0 and -88


sdr.rx_buffer_size = int(num_samps)
sdr.dds_single_tone(int(0.01e6), 0.9, 0)    # sdr.dds_single_tone(tone_freq_hz, tone_scale_0to1, tx_channel)


plt.ion()
fig = plt.figure()
ax=fig.add_subplot(111)

samples = sdr.rx() # receive samples off Pluto
# Create Waterfall matrix
#num_slices = int(np.floor(num_samps/fft_size))
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

for z in range(0,100):
    samples = sdr.rx()
    for i in range(num_slices):
        for x in range(num_slices-1):
            waterfall[x,:] = waterfall[x+1,:]
        waterfall[(num_slices-1),:] = np.log10(np.fft.fftshift(np.abs(np.fft.fft(samples[i*fft_size:(i+1)*fft_size]))**2))
        ax.imshow(waterfall, extent=[fmin, fmax, time_per_row*num_slices, 0], aspect='auto', cmap=plt.get_cmap('jet'))
        fig.canvas.draw()


