# %%
# Imports
%reset
import netCDF4 as nc
import numpy as np

# %%
# Passed in parameters
buffer_size = 2**12 
adc_len = 2**12

# %%
# Create nc file
filename = 'test.nc'
ds = nc.Dataset(filename, 'w', format='NETCDF4')

time = ds.createDimension('time', None)
bin = ds.createDimension('bin', buffer_size)

times = ds.createVariable('time', 'f4', ('time',))
bins = ds.createVariable('bin', 'f4', ('bin',))
value = ds.createVariable('value', 'f4', ('time', 'bin',))
value.units = 'Voltage relative to full scale (2^{:0.0f} bits)'.format(np.log2(adc_len))

bins[:] = np.arange(buffer_size)
value[0, :] = np.random.uniform(0, 100, size=(buffer_size))
curr_time = value.shape[0]
value[curr_time, :] = np.random.uniform(0, 100, size=(buffer_size))
print(value.shape)

# Close dataset when done
ds.close()
# %%
# Read netCDF file

ds_read = nc.Dataset(filename, 'r')

