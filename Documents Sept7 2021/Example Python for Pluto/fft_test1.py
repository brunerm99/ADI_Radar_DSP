import numpy as np
import matplotlib.pyplot as plt
from scipy import pi
from scipy.fftpack import fft
sample_rate = 2048
N = (2 - 0) * sample_rate
time = np.linspace(0, 2, N)
freq1 = 60
magnitude1 = 25
freq2 = 250
magnitude2 = 60
freq3 = 450
magnitude3 = 30
waveform1 = magnitude1 * np.sin (2 * pi * freq1 * time)
waveform2 = magnitude2 * np.sin (2 * pi * freq2 * time)
waveform3 = magnitude3 * np.sin (2 * pi * freq3 * time)
noise = np.random.normal (0, 3, N)
time_data = waveform1 + waveform2 + waveform3 + noise
# Plotting the time domain response
plt.plot (time [0:300], time_data [0:300])
plt.title ('Time Domain Signal')
plt.xlabel ('Time')
plt.ylabel ('Amplitude')
plt.grid('on')
plt.show ()
frequency = np.linspace (0.0, 1024, int (N/2))
freq_data = fft(time_data)
y = 2/N * np.abs (freq_data [0:np.int (N/2)])
# Plotting the FFT spectrum
plt.plot(frequency, y)
plt.title('Frequency domain Signal')
plt.xlabel('Frequency in Hz')
plt.ylabel('Amplitude')
plt.grid('on')
plt.show()