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
    print('Scan angle: %0.2f' % scan_angle)
    # Calculate phase offsets from scan angle
    phases_rad = np.arange(0, num_devs * num_channels, 1, dtype=float)
    phase_diff = (2 * np.pi * output_freq * my_phaser.element_spacing *
        np.sin(scan_angle * np.pi / 180)) / my_phaser.c
    phases_rad *= phase_diff
    phases_deg = np.degrees(phases_rad)
    # print(np.degrees(phases_rad))

    # TODO add phase calibration offset

    # Update each channel with calculated phases
    for dev_index, (dev_name, dev_obj) in enumerate(my_phaser.devices.items()):
        for channel_index, channel in enumerate(dev_obj.channels):
            # print('Old: %i-%i: %0.4f degrees' % (dev_index, channel_index, channel.rx_phase))
            channel.rx_phase = phases_deg[(dev_index * num_channels) + channel_index]
            # print('New: %i-%i: %0.4f degrees' % (dev_index, channel_index, channel.rx_phase), 
                # end='\n\n')
            dev_obj.latch_rx_settings()

"""
    TODO: check if max Rx gain is actually 70 rather than 127
    update_gains()
    Description: Updates each channel's gain given a taper values.
    @param my_phaser: Phaser object to be modified.
    @param taper: Gain taper values for all elements
"""
def update_gains(my_phaser, taper, max_phaser_gain, num_channels=4):
    print('Setting gain weights:')
    for dev_index, (dev_name, dev_obj) in enumerate(my_phaser.devices.items()):
        for channel_index, channel in enumerate(dev_obj.channels):
            print('Old: %i-%i: %0.4f' % (dev_index, channel_index, channel.rx_gain))
            channel.rx_gain = max_phaser_gain * taper[(dev_index * num_channels) + channel_index]
            print('New: %i-%i: %0.4f' % (dev_index, channel_index, channel.rx_gain), 
                end='\n\n')
            dev_obj.latch_rx_settings()

