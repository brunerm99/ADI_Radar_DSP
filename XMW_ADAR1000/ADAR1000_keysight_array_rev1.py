# Copyright (C) 2020 Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     - Neither the name of Analog Devices, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#     - The use of this software may or may not infringe the patent rights
#       of one or more patent holders.  This license does not release you
#       from the requirement that you obtain separate licenses from these
#       patent holders to use this software.
#     - Use of the software either in source or binary form, must be run
#       on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
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

# Modified from adar1000_array class example here:
#     https://github.com/analogdevicesinc/pyadi-iio/blob/master/examples/adar1000_array_example.py
#
# Create handle for an array of 4 ADAR1000s with the channels configured in a 4x4 array.
#
# Instantiation arguments:
#     chip_ids: Must match the ADAR1000 labels in the device tree
#
#     device_map: Maps the ADAR1000s to their location in the array. See below:
#         (ADAR1000 #1)    (ADAR1000 #2)    (ADAR1000 #3)    (ADAR1000 #4)
#
#     element_map: Maps the element numbers in the array. Orientation is looking at the back of the array.  See below:
#         (El. #1)    (El. #5)   (El. #9)    (El. #13)
#         (El. #2)    (El. #6)   (El. #10)   (El. #14)
#         (El. #3)    (El. #7)   (El. #11)   (El. #15)
#         (El. #4)    (El. #8)   (El. #12)   (El. #16)
#     channel_map: Maps the ADAR1000 channels in the array. On the XMW ADAR1000 board each CHx is labled RFCx. See below:
#         (Ch. #3)    (Ch. #3)   (Ch. #3)   (Ch. #3)
#         (Ch. #4)    (Ch. #4)   (Ch. #4)   (Ch. #4)
#         (Ch. #1)    (Ch. #1)   (Ch. #1)   (Ch. #1)
#         (Ch. #2)    (Ch. #2)   (Ch. #2)   (Ch. #2)
#
#     device_element_map: Maps the ADAR1000s to specific elements in the array, in channel order. See below:
#         ADAR1000 #1:
#             Ch. #1 -> El. #3
#             Ch. #2 -> El. #4
#             Ch. #3 -> El. #1
#             Ch. #4 -> El. #2
#         ADAR1000 #2:
#             Ch. #1 -> El. #7
#             Ch. #2 -> El. #8
#             Ch. #3 -> El. #5
#             Ch. #4 -> El. #6
#         ADAR1000 #3:
#             Ch. #1 -> El. #11
#             Ch. #2 -> El. #12
#             Ch. #3 -> El. #9
#             Ch. #4 -> El. #10
#         ADAR1000 #4:
#             Ch. #1 -> El. #15
#             Ch. #2 -> El. #16
#             Ch. #3 -> El. #13
#             Ch. #4 -> El. #14

array = adi.adar1000_array(
    chip_ids=["BEAM0", "BEAM1", "BEAM2", "BEAM3"],   #change to "BEAMx" in example folder
    device_map=[
        [1, 2, 3, 4],
        []                 #does this need to be two lines???? It won't work if I delete this empty line though....
    ],   
    element_map=[
        [1, 5, 9, 13],
        [2, 6, 10, 14],
        [3, 7, 11, 15],
        [4, 8, 12, 16]
    ],
    device_element_map={
        1: [3, 4, 1, 2],
        2: [7, 8, 5, 6],
        3: [11, 12, 9, 10],
        4: [15, 16, 13, 14]    # fix comma in example folder
    }
)

# Set the array frequency to 10.1GHz and the element spacing to 15mm so that the array can be accurately steered
array.frequency = 10.1e9
array.element_spacing = 0.015

ArrayGain = []
ElevationAngle = []
AzimuthAngle = [] 
    
DEVICE_MODE = "rx"

for device in array.devices.values():
    ADAR_init(device)        # resets the ADAR1000, then reprograms it to the standard config
    ADAR_set_mode(device, DEVICE_MODE)  # configures for rx or tx.  And sets the LNAs for Receive mode or the PA's for Transmit mode

for element in array.elements.values():
    element.rx_gain = 127
    element.tx_gain = 127
array.latch_rx_settings()
array.latch_tx_settings()

# Steer the Rx array
if DEVICE_MODE == "rx":
    for el in np.arange(-80, 80, 5):
        for az in np.arange(-80, 80, 5):
            array.steer_rx(azimuth=az, elevation=el)
            array.latch_rx_settings()
            ElevationAngle.append(el)
            AzimuthAngle.append(az)
            ArrayGain.append(SDR_get_dBFS(sdr,5,1))

# Steer the Tx array
else:
    array.steer_tx(azimuth=10, elevation=45)
    array.latch_tx_settings()


# Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(ElevationAngle, AzimuthAngle, ArrayGain, color='white', edgecolors='grey', alpha=0.5)  # alpha controls opacity of surface
#ax.scatter(X, Y, Z, c='red')
plt.show()

