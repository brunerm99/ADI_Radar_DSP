"""
    Make a 2-D plot.
"""

# %%

import time
import matplotlib.pyplot as plt
from matplotlib import animation
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import faulthandler
faulthandler.enable()
import colorcet

# Signal processing stuff
import numpy as np
from numpy import sin, cos
from scipy import signal
from scipy.signal import windows
from numpy.fft import fft, ifft, fftshift, fftfreq
from numpy import absolute, pi, log10
from target_detection import cfar

# Phaser board interaction
import adi
from Phaser_Functions import update_gains, update_phases, range_bin, range_norm

# Instantiate all the Devices
try:
    import phaser_config
    rpi_ip = phaser_config.rpi_ip
    sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block
except:
    print('No config file found...')
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
my_sdr.tx([iq*0.5, iq])  # only send data to the 2nd channel (that's all we need)
# %%

# Reduce size of data for quicker computation
def reduce_array_size(arr, factor, bb_indices):
    N = arr.size
    arr_rs = arr[bb_indices]
    arr_rs = arr_rs[0:round(N / factor)]
    N_rs = arr_rs.size
    return arr_rs, N_rs

# Down sample signal by a factor for quicker computation
def downsample(arr, factor):
    N = arr.size
    arr_ds = signal.resample(arr, int(N / factor))
    return arr_ds, arr_ds.size

def polar_animation():
    global frame, img
    angle = int((frame * beamwidth / 2) % 180)
    if (angle > int(beamwidth / 2)):
        x_n = my_sdr.rx()
        x_n = x_n[0] + x_n[1]

        X_k = absolute(fft(x_n))
        X_k = fftshift(X_k)

        X_k_rs, _ = reduce_array_size(X_k, rs_factor, bb_indices)
        # X_k_ds, _ = downsample(X_k_rs, ds_factor)
        X_k_ds = X_k_rs

        # _, targets_only = cfar(X_k_ds, 10, 30, 3, 'greatest')

        X_k_width = np.ma.masked_all((beamwidth, N_ds))
        X_k_width[:] = X_k_ds

        zdata[:,angle - int(beamwidth / 2):angle + int(beamwidth / 2)] = X_k_width.T

        img.setImage(zdata.T)
    frame += 1

def fft_animation():
    global curve, w_n, freq_ds, dist
    x_n = my_sdr.rx()
    x_n = x_n[0] + x_n[1]
    # x_n *= w_n

    # sos = signal.cheby1(10, 1, 105000, btype='hp', fs=fs, output='sos')
    # x_n_filtered = signal.sosfilt(sos, x_n)

    X_k = absolute(fft(x_n))
    X_k = fftshift(X_k)
    X_k_ds = X_k
    X_k_ds, _ = reduce_array_size(X_k, rs_factor, bb_indices)

    # X_k_ds, d_res = range_bin(X_k_ds, N)
    N_ds = X_k_ds.size

    # _, X_k_ds = cfar(X_k_ds, 10, 30, 3, 'greatest')

    # X_k_ds = range_norm(X_k_ds, dist, 1)
    X_k_dbfs = 20 * log10(X_k_ds / 2**12)
    curve.setData(freq_ds, X_k_dbfs)

if __name__ == '__main__':
    # Apply blackman taper
    num_devs = 2
    num_channels = 4
    taper = windows.blackman(num_devs * num_channels + 
        2)[1:num_devs * num_channels + 1]
    update_gains(my_phaser, taper, 70, num_channels)

   
    x_n = my_sdr.rx()
    x_n = x_n[0] + x_n[1]
    N = x_n.size

    w_n = signal.windows.hamming(N)
    x_n *= w_n

    X_k = absolute(fft(x_n))
    X_k = fftshift(X_k)

    # Create frequency axis
    # Shift down to baseband and only keep positive frequncies
    # Also only keep to a max frequency
    freq = fftshift(fftfreq(N, 1 / sample_rate))
    
    c = 3e8
    slope = BW / ramp_time_s

    # signal_freq is the IF frequency
    max_dist = float(input("Enter max distance: "))
    max_freq = max_dist * slope / c + signal_freq
    max_freq = np.max(freq)
    bb_indices = np.where((freq >= signal_freq) & (freq <= max_freq))
    freq_bb = freq[bb_indices]
    freq_kHz = freq_bb / 1e3

    # Reduce array size by a factor, rs_factor
    rs_factor = 1
    X_k_ds, N_ds = reduce_array_size(X_k, rs_factor, bb_indices)
    freq_ds, _ = reduce_array_size(freq, rs_factor, bb_indices)

    # X_k_ds, d_res = range_bin(X_k_ds, N)
    N_ds = X_k_ds.size
    freq_ds = np.linspace(signal_freq, max_freq, N_ds)
    # print('Range bins: %i @ %0.2fm' % (N_ds, d_res))

    # Create range-FFT scale
    dist = (c / slope) * (freq_ds - signal_freq) # meters

    R_max = np.max(dist)
    print('Data size reduced from %i to %i' % (N, X_k_ds.size))
    print('Range reduced to: %0.2fm' % R_max)

    # R_max = 150
    N_test = 20
    N_theta = 360

    frame = 0
    POLAR = False
    FFT = True
    if (POLAR):
        theta = np.linspace(0, 2 * pi, N_theta)
        ranges = np.linspace(1, R_max, N_ds)
        zdata = np.ma.masked_all((N_ds, N_theta))

        beamwidth = 20
        X_k_width = np.ma.masked_all((beamwidth, N_ds))
        X_k_width[:] = X_k_ds

        zdata[:,0:beamwidth] = X_k_width.T

        max_frame = int(N_theta / (beamwidth))

        app = pg.mkQApp('2D Viewer')
        win = pg.GraphicsLayoutWidget()
        ax2D = win.addPlot(title='2D spectrum', row=0, col=0)
        ax2D.setXRange(0, 180)
        ax2D.setYRange(0, 1000)

        img = pg.ImageItem()
        img.setImage(zdata.T)
        ax2D.addItem(img)

        timer = QtCore.QTimer()
        timer.timeout.connect(polar_animation)
        timer.start(0)

        win.show()
        win.raise_()
        app.exec_()

    elif (FFT):
        app = pg.mkQApp('FFT Viewer')

        win = QtWidgets.QMainWindow()
        win.resize(800, 800)
        p = pg.plot()
        p.setWindowTitle('DFT Plot')
        p.setLabel('bottom', 'Frequency', units='Hz')
        p.setLabel('left', 'Magnitude', units='dBFS')
        p.setTitle('Received Signal - Frequency Spectrum')
        p.setYRange(0, 70)
        # X_k_ds = range_norm(X_k_ds, dist, 1)
        X_k_dbfs = 20 * log10(X_k_ds / 2**12)
        curve = p.plot(freq_ds, X_k_dbfs)

        timer = QtCore.QTimer()
        timer.timeout.connect(fft_animation)
        timer.start(0)

        app.exec_()



# %%
