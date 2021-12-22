# Simple testing of the XMW Phased Array Cube with the ADAR1000
# ADAR1000 python use info is here:  https://analogdevicesinc.github.io/pyadi-iio/devices/adi.adar1000.html
# Examples here:  https://github.com/analogdevicesinc/pyadi-iio/tree/master/examples

# From the device tree:
# BEAM0 = ADAR1000@0, ADDR 0x00, CS=GPIO8
# BEAM1 = ADAR1000@1, ADDR 0x00, CS=GPIO7
# BEAM2 = ADAR1000@2, ADDR 0x00, CS=GPIO27
# BEAM3 = ADAR1000@3, ADDR 0x00, CS=GPIO22

# The driver uses readback on the ADAR1000.  So make sure that MISO/CIPO is connected to ADAR1000 SDO and MOSI/COPI is connected to ADAR1000 SDIO

import adi
from ADAR_pyadi_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)

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
    uri='ip:analog.local',
    context=None,
    chip_id="BEAM0",
    array_element_map=[[1, 2, 3, 4]],
    channel_element_map=[2, 1, 4, 3],
)
beam1 = adi.adar1000(
    chip_id="BEAM1",
    array_element_map=[[5, 6, 7, 8]],
    channel_element_map=[6, 5, 8, 7],
)
'''beam2 = adi.adar1000(
    chip_id="BEAM2",
    array_element_map=[[9, 10, 11, 12]],
    channel_element_map=[10, 9, 12, 11],
)
beam3 = adi.adar1000(
    chip_id="BEAM3",
    array_element_map=[[13, 14, 15, 16]],
    channel_element_map=[14, 13, 16, 15],
)'''

# initialize the ADAR1000
ADAR_init(beam0)        # resets the ADAR1000, then reprograms it to the standard config

# Program the beams

DEVICE_MODE = "rx"
if DEVICE_MODE == "rx":
    # Configure the device for Rx mode
    beam0.mode = "rx"   # Mode of operation, bit 5 of reg 0x31. "rx", "tx", or "disabled"
    SELF_BIASED_LNAs = True
    if SELF_BIASED_LNAs:
        beam0.lna_bias_out_enable = False    # Allow the external LNAs to self-bias
    else:
        beam0.lna_bias_on = -0.7       # Set the external LNA bias
    # Enable the Rx path for each channel
    for channel in beam0.channels:
        channel.rx_enable = True  #this writes reg0x2E with data 0x00, then reg0x2E with data 0x20.  So it overwrites 0x2E, and enables only one channel

# Configure the device for Tx mode
else:
    beam0.mode = "tx"
    # Enable the Tx path for each channel and set the external PA bias
    for channel in beam0.channels:
        channel.tx_enable = True
        channel.pa_bias_on = -2

# Set the array phases to 10°, 20°, 30°, and 40° and the gains to max gain (0x7f, or 127)
for channel in beam0.channels:
    # Set the gain and phase depending on the device mode
    if DEVICE_MODE == "rx":
        channel.rx_phase = channel.array_element_number * 0   #writes to I and Q registers.
        channel.rx_gain = 127
    else:
        channel.tx_phase = channel.array_element_number * 10  #writes to I and Q registers.
        channel.tx_gain = 127

# Latch in the new gains & phases
if DEVICE_MODE == "rx":
    beam0.latch_rx_settings()  # writes 0x01 to reg 0x28.
else:
    beam0.latch_tx_settings()  # writes 0x02 to reg 0x28.

