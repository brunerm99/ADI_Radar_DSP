# Simple testing of the XMW Phased Array Cube with the ADAR1000
# ADAR1000 python use info is here:  https://analogdevicesinc.github.io/pyadi-iio/devices/adi.adar1000.html
# Examples here:  https://github.com/analogdevicesinc/pyadi-iio/tree/master/examples
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# From the device tree:  (found here:  https://github.com/analogdevicesinc/linux/blob/rpi-5.4.y/arch/arm/boot/dts/overlays/rpi-adar1000-overlay.dts)
# BEAM0 = ADAR1000_0@0, label="BEAM0", ADDR 0x00, CS=GPIO8
# BEAM1 = ADAR1000_1@1, label="BEAM1", ADDR 0x20
# BEAM2 = ADAR1000_2@2, label="BEAM2", ADDR 0x40
# BEAM3 = ADAR1000_3@3, label="BEAM3", ADDR 0x60

# The driver uses readback on the ADAR1000.  So make sure that MISO/CIPO is connected to ADAR1000 SDO and MOSI/COPI is connected to ADAR1000 SDIO

import adi
from ADAR_pyadi_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
from SDR_functions import *    #import the SDR functions (These all start with SDR_xxxx)
try:
    from pluto_config import *    # this has all the key parameters that the user would want to change (i.e. calibration phase and antenna element spacing)
except:
    print("Make sure that the file pluto_config_xxxx.py is in the same directory as this python file.")
    sys.exit(0)
    
# Setup Pluto with the config values from the pluto_config.py file
sdr = SDR_init(config.sdr_address, config.num_Rx, config.SampleRate, config.Tx_freq, config.Rx_freq, config.Rx_gain, config.Tx_gain)

# Create device handle for a each ADAR1000
# Instantiation arguments:
#     chip_id: Must match the ADAR1000 label in the device tree (i.e. "BEAM0")
#     array_element_map: Maps the array elements to a 1x4 array. See below:
#         (El. #1)    (El. #2)    (El. #3)    (El. #4)
#     channel_element_map: Maps the ADAR1000's channels to the array elements. See below:
#         Ch. #1 -> El. #2
#         Ch. #2 -> El. #1
#         Ch. #3 -> El. #4
#         Ch. #4 -> El. #3

# Each XMW ADAR1000 board has 4 elements, linearly spaced.  BEAM0 is elements 1-4, BEAM1 is elements 5-8.
# The ADAR1000 layout is such that channel 1 is the first element, channel 0 is the second,
# channel 4 is the third, and channel 3 is the fourth
beam0 = adi.adar1000(
    chip_id="BEAM0",
    array_element_map=[[1, 2, 3, 4]],
    channel_element_map=[2, 1, 4, 3],
)
beam1 = adi.adar1000(
    chip_id="BEAM1",
    array_element_map=[[5, 6, 7, 8]],
    channel_element_map=[6, 5, 8, 7],
)

# initialize the ADAR1000
ADAR_init(beam0)        # resets the ADAR1000, then reprograms it to the standard config
ADAR_init(beam1)        # resets the ADAR1000, then reprograms it to the standard config

# Program the beams

DEVICE_MODE = "rx"    # This can be "rx" or "rx"
ADAR_set_mode(beam0, DEVICE_MODE)  # configures for rx or tx.  And sets the LNAs for Receive mode or the PA's for Transmit mode
ADAR_set_mode(beam1, DEVICE_MODE)  # configures for rx or tx.  And sets the LNAs for Receive mode or the PA's for Transmit mode


# Set the gain of each element
for channel in beam0.channels:
    channel.rx_gain = 127
for channel in beam1.channels:
    channel.rx_gain = 127
ADAR_update_Rx(beam0)
ADAR_update_Rx(beam1)
    
ArrayGain = []
ArrayAngle = []

for PhDelta in list(np.arange(-180, 180, 5)):
    # Set the phase of each element
    for channel in beam0.channels:
        channel.rx_phase = channel.array_element_number * PhDelta   #writes to I and Q registers.
        if PhDelta==45:  #do a spot check to see if we are giving the elements the correct phase
            print("Element ", channel.array_element_number, " has Phase = ", channel.rx_phase)
    for channel in beam1.channels:
        channel.rx_phase = channel.array_element_number * PhDelta   #writes to I and Q registers.
        if PhDelta==45:
            print("Element ", channel.array_element_number, " has Phase = ", channel.rx_phase)
    ADAR_update_Rx(beam0)
    ADAR_update_Rx(beam1)
        
    value1 = (299792458 * np.radians(np.abs(PhDelta)))/(2*3.14159*config.SignalFreq*config.d)
    clamped_value1 = max(min(1, value1), -1)     #arcsin argument must be between 1 and -1, or numpy will throw a warning
    theta = np.degrees(np.arcsin(clamped_value1))
    if PhDelta>=0:
        SteerAngle = theta   # positive PhaseDelta covers 0deg to 90 deg
    else:
        SteerAngle = -theta   # negative phase delta covers 0 deg to -90 deg
    # Get data
    Averages=2
    total=0
    for count in range (0, Averages):
        data_raw=SDR_getData(sdr)
        data = data_raw
        NumSamples = len(data)          #number of samples
        win = np.blackman(NumSamples)
        y = data * win
        sp = np.absolute(np.fft.fft(y))
        sp = sp[1:-1]
        sp = np.fft.fftshift(sp)
        s_mag = np.abs(sp) * 2 / np.sum(win)    # Scale FFT by window and /2 since we are using half the FFT spectrum
        s_mag = np.maximum(s_mag, 10**(-15))
        s_dbfs = 20*np.log10(s_mag/(2**12))     # Pluto is a 12 bit ADC, so use that to convert to dBFS
        total=total+max(s_dbfs)   # sum up all the loops, then we'll average
    PeakValue=total/Averages
    ArrayGain.append(PeakValue)
    ArrayAngle.append(SteerAngle)

fig, ax = plt.subplots()
ax.plot(ArrayAngle, ArrayGain)

ax.set(xlabel='Steering Angle (deg)', ylabel='Peak Signal (dBfs)',)
ax.grid()
plt.show()
