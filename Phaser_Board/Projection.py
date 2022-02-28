"""
	Filename: 	Projection.py
	Desc.:	 
	Author:		Marshall Bruner
	Date:		2022-02-28
"""
# %%
# Imports
import numpy as np
from numpy import log10, pi
from numpy.fft import fft, ifft, fftshift, fftfreq
import matplotlib.pyplot as plt
from scipy import signal

from matplotlib import style
style.use('seaborn-dark')

# %%
def polar_to_cart(polar_data, theta_step, range_step, x, y, order=3):

    from scipy.ndimage.interpolation import map_coordinates as mp

    # "x" and "y" are numpy arrays with the desired cartesian coordinates
    # we make a meshgrid with them
    X, Y = np.meshgrid(x, y)

    # Now that we have the X and Y coordinates of each point in the output plane
    # we can calculate their corresponding theta and range
    Tc = np.degrees(np.arctan2(Y, X)).ravel()
    Rc = (np.sqrt(X**2 + Y**2)).ravel()

    # Negative angles are corrected
    Tc[Tc < 0] = 360 + Tc[Tc < 0]

    # Using the known theta and range steps, the coordinates are mapped to
    # those of the data grid
    Tc = Tc / theta_step
    Rc = Rc / range_step

    # An array of polar coordinates is created stacking the previous arrays
    coords = np.vstack((Tc, Rc))

    # To avoid holes in the 360º - 0º boundary, the last column of the data
    # copied in the begining
    polar_data = np.vstack((polar_data, polar_data[-1,:]))

    # The data is mapped to the new coordinates
    # Values outside range are substituted with nans
    cart_data = mp(polar_data, coords, order=order, mode='constant', cval=np.nan)

    # The data is reshaped and returned
    return(cart_data.reshape(len(y), len(x)).T)

# %%
if __name__ == '__main__':
    polar_data = np.zeros((360, 10)) # Here a 2D array of data is assumed, with shape thetas x ranges
    polar_data[180:,2:5] = 2

	# We create the x and y axes of the output cartesian data
    x = np.arange(-100000, 100000, 1000)
    y = np.arange(-100000, 100000, 1000)

	# We call the mapping function assuming 1 degree of theta step and 500 meters of
	# range step. The default order of 3 is used.
    # cart_data = polar_to_cart(polar_data, 1, 500, x, y)
    # plt.plot(cart_data)

    # from scipy import ndimage
    # polar_data = np.ma.masked_all((100, 100))
    # polar_data[40:50,50:] = 1
    # polar_rot1 = ndimage.rotate(polar_data, 20, reshape=False, mode='nearest')
    # polar_rot2 = ndimage.rotate(polar_data, 45, reshape=False, mode='nearest')
    # polar_rot1[np.where(polar_rot2 != np.ma.masked)] = polar_rot2[np.where(polar_rot2 != np.ma.masked)]
    
    # fig, ax = plt.subplots()
    # ax.imshow(polar_data)
    # plt.imshow(polar_rot1)
    # fig.set_figwidth(8)
    # fig.set_figheight(8)
    
    angle_resolution = 1
    range_max = 400

    a, r = np.mgrid[0:int(360/angle_resolution),0:range_max]

    x = (range_max + r * np.cos(a*(2 * pi)/360.0)).astype(int)
    y = (range_max + r * np.sin(a*(2 * pi)/360.0)).astype(int)

    cart_grid = np.zeros((360))

    for i in range(0, int(360/angle_resolution)): 
        cart_grid[y[i,:],x[i,:]] = polar_data[i,:]
