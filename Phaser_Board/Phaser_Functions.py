"""
	Filename: 	Phaser_Functions.py
	Desc.:	 
	Author:		Marshall Bruner
	Date:		2022-02-26
"""

# Imports
import numpy as np
from numpy import log10, pi
from numpy.fft import fft, ifft, fftshift, fftfreq
import matplotlib.pyplot as plt
from scipy import signal

"""
    update_phases()
    Description: Updates each channel's phase depending on an input scan angle.
    @param my_phaser: Phaser object to be modified.
    @param scan_angle: Desired scan angle
"""
def update_phases(my_phaser, scan_angle, output_freq, num_devs=2, num_channels=4):
    # Calculate phase offsets from scan angle
    phases_rad = np.arange(0, num_devs * num_channels, 1, dtype=float)
    phase_diff = (2 * np.pi * output_freq * my_phaser.element_spacing *
        np.sin(scan_angle * np.pi / 180)) / my_phaser.c
    phases_rad *= phase_diff
    phases_deg = np.degrees(phases_rad)

    for index in range(num_devs * num_channels):
        my_phaser.elements.get(index + 1).rx_phase = phases_deg[index]
    my_phaser.latch_rx_settings()

    # Update each channel with calculated phases
    # for dev_index, (dev_name, dev_obj) in enumerate(my_phaser.devices.items()):
    #     for channel_index, channel in enumerate(dev_obj.channels):
    #         # print('Old: %i-%i: %0.4f degrees' % (dev_index, channel_index, channel.rx_phase))
    #         channel.rx_phase = phases_deg[(dev_index * num_channels) + channel_index]
    #         # print('New: %i-%i: %0.4f degrees' % (dev_index, channel_index, channel.rx_phase), 
    #             # end='\n\n')
    #         dev_obj.latch_rx_settings()

"""
    update_gains()
    Description: Updates each channel's gain given a taper values.
    @param my_phaser: Phaser object to be modified.
    @param taper: Gain taper values for all elements
"""
def update_gains(my_phaser, taper, max_phaser_gain, num_channels=4, num_devs=2):
    for index in range(num_devs * num_channels):
        my_phaser.elements.get(index + 1).rx_gain = taper[index]
        my_phaser.elements.get(index + 1).rx_attenuator = not bool(taper[index]) 
    my_phaser.latch_rx_settings()

    # print('Setting gain weights:')
    # for dev_index, (dev_name, dev_obj) in enumerate(my_phaser.devices.items()):
    #     for channel_index, channel in enumerate(dev_obj.channels):
    #         print('Old: %i-%i: %0.4f' % (dev_index, channel_index, channel.rx_gain))
    #         channel.rx_gain = max_phaser_gain * taper[(dev_index * num_channels) + channel_index]
    #         print('New: %i-%i: %0.4f' % (dev_index, channel_index, channel.rx_gain), 
    #             end='\n\n')
    #         dev_obj.latch_rx_settings()


"""
    range_bin()
"""
def range_bin(X_k, N_total, BW=500e6, sample_rate=600e3, slope=500e9):
    # Resolution
    c = 3e8
    r_res = c / (2 * BW)
    f_res = 2 * r_res * slope / c

    # Spectrum separation
    f_diff = sample_rate / N_total

    # Range bin size
    n = int(np.ceil(f_res / f_diff))
    n = n if n >= 1 else 1

    # print('Frequency resolution (from BW): %0.2fHz' % f_res)
    # print('Frequency resolution (from Ts and N): %0.2fHz' % f_diff)
    # print('Range resolution: %0.2fm' % r_res)

    pad_amount = n - (X_k.size % n)
    X_k_bin = np.pad(X_k, (0, pad_amount), 'constant')

    X_k_bin = np.max(X_k_bin.reshape(-1, n), axis=1)

    return X_k_bin, r_res

"""
    range_norm()
"""
def range_norm(X_k, ranges, ref_range=1):
    norm_factors = (ranges / ref_range)**2
    X_k *= norm_factors
    return X_k