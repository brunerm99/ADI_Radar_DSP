
import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from matplotlib import animation

# Trying with radar plot
R_max = 150
N_test = 20
N_theta = 360
fig = plt.figure()
fig.set_figheight(8)
fig.set_figwidth(8)


ax = plt.subplot(111, polar=True)
theta = np.linspace(0, 2 * pi, N_theta)
ranges = np.linspace(0, R_max, N_test)
zdata = np.ma.masked_all((N_test, N_theta))
zdata[:,5] = np.linspace(0, 50, N_test)

range_fft = np.linspace(0, 50, N_test)

beamwidth = 20
empty_arr = np.ma.masked_all((beamwidth, N_test))
full_arr = np.copy(empty_arr)
full_arr[:] = range_fft

pc = ax.pcolormesh(theta, ranges, zdata)

def polar_animation(frame):
    if (frame > int(beamwidth / 2)):
        zdata[:,frame - int(beamwidth / 2):frame + int(beamwidth / 2)] = full_arr.T
        pc.set_array(zdata)

anim = animation.FuncAnimation(fig, polar_animation, round(N_theta / 2), interval=1, 
    blit=False, repeat=True)
plt.show()