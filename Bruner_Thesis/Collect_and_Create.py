# %%
""" 
    Imports 
"""
# Hardware interfacing
import iio
import adi

# Signal processing
import numpy as np
# import scipy.signal as signal
# from scipy.fftpack import fft

# Plotting
# from pyqtgraph.Qt import QtGui, QtCore
# import pyqtgraph as pg
# import matplotlib.pyplot as plt
import netCDF4 as nc

# %%
# Initialize pluto SDR
sdr = adi.Pluto()

sample_rate = int(10e6)
buffer_size = int(2**14)
rx_lo = int(2e9)
tx_lo = int(2e9)
adc_size = 2**12

# Configure pluto
# Rx config
sdr.sample_rate = sample_rate
sdr.rx_buffer_size = buffer_size
sdr.rx_lo = rx_lo
sdr.gain_control_mode_chan0 = "slow_attack"

# Tx config
sdr.tx_lo = tx_lo
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = -20

# Print configuration
print("RX LO: {:0.2f}MHz".format(rx_lo / 1e6))
print("TX LO: {:0.2f}MHz".format(tx_lo / 1e6))

"""
    Create netCDF file
"""
output_filename = 'data.nc'

ds = nc.Dataset(output_filename, 'w', format='NETCDF4')

time = ds.createDimension('time', None)
bin = ds.createDimension('bin', buffer_size)

times = ds.createVariable('time', 'f4', ('time',))
bins = ds.createVariable('bin', 'f4', ('bin',))
value = ds.createVariable('value', 'f4', ('time', 'bin',))
value.units = 'Voltage relative to full scale (2^{:0.0f} bits)'.format(np.log2(adc_size))

curr_time = 0
# %%

# Generate output waveform
f_s = int(sdr.sample_rate)
f_c = 3e9
T_s = 1 / float(f_s)
M = buffer_size
n = np.arange(0, M * T_s, T_s)
i = np.cos(2 * np.pi * f_c * n) * buffer_size
q = np.cos(2 * np.pi * f_c * n) * buffer_size
iq = i + (1j * q)

# Transmit IQ waveform
sdr.tx(iq)

""" 
    Receive signal and do simple processing 
"""
num_loops = 20

for _ in range(num_loops):
    # Collect buffer_size samples
    x_n = sdr.rx()

    # Number of collected samples (i.e. buffer size)
    M = len(x_n)          

    # Add data to nc file
    value[curr_time, :] = np.random.uniform(0, 100, size=(buffer_size))
    curr_time = value.shape[0]

# Close dataset when done
ds.close()