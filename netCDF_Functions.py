# %%
# Imports
# %reset
import netCDF4 as nc
import numpy as np

# %%
# Create nc file
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

    for curr_time, row in enumerate(data):
        bins[:] = np.arange(buffer_size)
        i[curr_time, :] = list(map(lambda x : complex(x).real, row))
        q[curr_time, :] = list(map(lambda x : complex(x).imag, row))

    # Close dataset when done
    ds.close()
# %%
# Read netCDF file

if __name__ == '__main__':
    import pandas as pd

    output_filename = 'radar_data.nc'
    data = pd.read_csv('data.csv')
    create_dataset(output_filename, data, 2**14, 2**12)

    ds_read = nc.Dataset(output_filename, 'r')