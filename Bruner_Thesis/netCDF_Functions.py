# %%
# Imports
%reset
import netCDF4 as nc
import numpy as np
import pandas as pd

# %%
# Passed in parameters
buffer_size = 2**12 
adc_len = 2**12

# %%
# Create nc file
filename = 'test.nc'
def create_dataset(output_filename, data, buffer_size, adc_len):
    ds = nc.Dataset(output_filename, 'w', format='NETCDF4')

    time = ds.createDimension('time', None)
    bin = ds.createDimension('bin', buffer_size)

    times = ds.createVariable('time', 'f4', ('time',))
    bins = ds.createVariable('bin', 'f4', ('bin',))
    i = ds.createVariable('i', 'f4', ('time', 'bin',))
    i.units = 'In-phase voltage relative to full scale (2^{:0.0f} bits)'.format(np.log2(adc_len))
    q = ds.createVariable('q', 'f4', ('time', 'bin',))
    q.units = 'Quadrature voltage relative to full scale (2^{:0.0f} bits)'.format(np.log2(adc_len))

    for row in data.iloc:
        curr_time = i.shape[0]
        bins[:] = np.arange(buffer_size)
        iq_data = list(map(lambda x : complex(x), row))
        i[curr_time, :] = list(map(lambda x : x.real, iq_data))
        q[curr_time, :] = list(map(lambda x : x.imag, iq_data))
    print(i.shape)

    # Close dataset when done
    ds.close()
# %%
# Read netCDF file

output_filename = 'radar_data.nc'
data = pd.read_csv('data.csv')
create_dataset(output_filename, data, 2**14, 2**12)

ds_read = nc.Dataset(output_filename, 'r')