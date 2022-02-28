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
    polar_data = np.zeros((180, 10)) # Here a 2D array of data is assumed, with shape thetas x ranges
    polar_data[:,2:5] = 2

	# We create the x and y axes of the output cartesian data
    x = np.arange(-100000, 100000, 1000)
    y = np.arange(-100000, 100000, 1000)

	# We call the mapping function assuming 1 degree of theta step and 500 meters of
	# range step. The default order of 3 is used.
    cart_data = polar_to_cart(polar_data, 1, 500, x, y)
    plt.plot(cart_data)