# Create device handle for a single ADAR1000 with the channels configured in a 1x4 array.
# ADAR1000 Rpi device tree is here:  https://github.com/analogdevicesinc/linux/blob/rpi-4.19.y/arch/arm/boot/dts/overlays/rpi-adar1000-overlay.dts

import adi

# Create device handle for a single ADAR1000 with the channels configured in a 1x4 array.
# Instantiation arguments:
#     chip_id: Must match the ADAR1000 label in the device tree. i.e. "BEAM0" for ADDR0=low, ADDR1=low
#
#     array_element_map: Maps the array elements to a 1x4 array. See below:
#         (El. #1)    (El. #2)    (El. #3)    (El. #4)
#
#     channel_element_map: Maps the ADAR1000's channels to the array elements. See below:
#         Ch. #1 -> El. #2
#         Ch. #2 -> El. #1
#         Ch. #3 -> El. #4
#         Ch. #4 -> El. #3

# In this example, assume 8 elements, linearly spaced.  BEAM0 is elements 1-4, BEAM1 is elements 5-8.
# Elements are arrayed from left to right.  The ADAR1000 layout is such that channel 1 is the first element, channel 0 is the second,
# channel 4 is the third, and channel 3 is the fourth
device0 = adi.adar1000(
    chip_id="BEAM0",
    array_element_map=[[1, 2, 3, 4]],
    channel_element_map=[2, 1, 4, 3],
)
device = adi.adar1000(
    chip_id="BEAM1",
    array_element_map=[[5, 6, 7, 8]],
    channel_element_map=[6, 5, 8, 7],
)

# initialize the ADAR1000
device0.reset()                       #For some reason, only address 0 will reset the devices--but it resets all 4 of them, not just address 0
device.reset()                        #Performs a soft reset of the device (writes 0x81 to reg 0x00).  But this doesn't reset the part, only the above command works.

device._ctrl.reg_write(0x00, 0x18)    #Sets the SDO pin to active, i.e. a 4 wire SPI bus.  Before each reg_write, 0x18 is also written to 0x00.
device._ctrl.reg_write(0x400, 0x55)   #This trims the LDO value to approx. 1.8V (to the center of its range)
device._ctrl.reg_write(0x38, 0x60)    #Bypasses the beam and bias RAM, i.e. we use the SPI writes to set the Gain, Phase and bias values instead of pulling the values the memory
device._ctrl.reg_write(0x34, 0x08)    #Sets the LNA bias to the middle of its range
device._ctrl.reg_write(0x35, 0x16)    #Sets the VGA bias to [0010] and the vector modulator bias to [110]


# Program the beams

DEVICE_MODE = "rx"
if DEVICE_MODE == "rx":
    # Configure the device for Rx mode
    device.mode = "rx"   # this writes 0xA0 0x31 0x00.  What does this do?  What does A0 mean??? Just a mistake?
    device._ctrl.reg_write(0x31, 0xB0)   #Enables the whole Rx and sets ADTR1107 switch high (Rx mode)
    SELF_BIASED_LNAs = True
    if SELF_BIASED_LNAs:
        # Allow the external LNAs to self-bias
        device.lna_bias_out_enable = False    # this writes 0xA0 0x30 0x00.  What does this do?
        device._ctrl.reg_write(0x30, 0x00)   #Disables PA and DAC bias
    else:
        # Set the external LNA bias
        device.lna_bias_on = -0.7       # this writes 0x25 to register 0x2D.  This is correct.  But oddly enough, it doesn't first write 0x18 to reg 0x00....
        device._ctrl.reg_write(0x30, 0x20)   #Enables PA and DAC bias.  I think this would be needed too?

    # Enable the Rx path for each channel
    for channel in device.channels:
        #channel.rx_enable = True  #this writes reg0x2E with data 0x00, then reg0x2E with data 0x20.  So it overwrites 0x2E, and enables only one channel
        device._ctrl.reg_write(0x2E, 0x7F)    #Enables all four Rx channels, the Rx LNA, Rx Vector Modulator and Rx VGA



# Configure the device for Tx mode
else:
    device.mode = "tx"   # this writes "A0 31 00" then "20 31 40"

    # Enable the Tx path for each channel and set the external PA bias
    for channel in device.channels:
        channel.tx_enable = True  #this writes A0 2F 00 then 20 2F 20
        channel.pa_bias_on = -2   #this writes 20 2A 6A.  Again, there is no 0x18 write to reg 0x00, just odd

# Set the array phases to 10°, 20°, 30°, and 40° and the gains to max gain (0x7f, or 127)
for channel in device.channels:
    # Set the gain and phase depending on the device mode
    if DEVICE_MODE == "rx":
        channel.rx_phase = channel.array_element_number * 150  #writes to I and Q registers.  Looks correct
        channel.rx_gain = 127  #writes A0 11 00 then 20 11 7F.  Other than the weird A0 write, it looks correct (7F to reg 0x11)
    else:
        channel.tx_phase = channel.array_element_number * 10  #writes to I and Q registers.  Looks correct
        channel.tx_gain = 127  #writes A0 1D 00 then 20 1D 7F, so looks correct

# Latch in the new gains & phases
if DEVICE_MODE == "rx":
    device.latch_rx_settings()  # writes 0x01 to reg 0x28.  This is correct
else:
    device.latch_tx_settings()  # writes 0x02 to reg 0x28.  This is correct

