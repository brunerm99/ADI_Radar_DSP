import sys
sys.path.append('/lib/python3.7/site-packages/')
import numpy as np
import matplotlib.pyplot as plt

on_time = 1e-3
samp_rate = 4e6
delay_time = 40.95e-6
a = np.ones(int(samp_rate*on_time))
a=np.append(a, np.zeros(int(samp_rate*delay_time)))
b=a
a=np.append(a,a)
a=np.append(a,a)
z=np.correlate(a, b, 'full')

plt.plot(a*4000)
plt.plot(z)
plt.show()
