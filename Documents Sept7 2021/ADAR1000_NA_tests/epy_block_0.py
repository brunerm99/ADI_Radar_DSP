# ADAR1000 Phase Sweeper and Pluto data collection

'''
Useful resources:
    Introduction to Phased Array Antennas: https://www.analog.com/en/analog-dialogue/articles/phased-array-antenna-patterns-part1.html#
    Full workshop build instructions at www.github.com/jonkraft/phasedarray
    Analog Devices Python Interfaces:  https://analogdevicesinc.github.io/pyadi-iio/
    Python examples:  https://github.com/analogdevicesinc/pyadi-iio/tree/master/examples
    GNU Radio and IIO Devices:  https://wiki.analog.com/resources/tools-software/linux-software/gnuradio
    ADI Kuiper Linux for Raspberry Pi:  https://wiki.analog.com/resources/tools-software/linux-software/gnuradio
'''

# Copyright (C) 2019 Analog Devices, Inc.
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


import numpy as np
import time
import spidev
from gnuradio import gr
import sys

def ADAR_init(spi, ADDR, Rx_mode):
    # Initialize the ADAR1000
    spi.xfer2([ADDR, 0x00, 0x81])  # reset the device
    spi.xfer2([ADDR, 0x00, 0x18])  # Sets SDO  pin to active (4 wire SPI)
    spi.xfer2([ADDR+0x04, 0x00, 0x55])  # Trims LDO to 1.8V
    spi.xfer2([ADDR, 0x38, 0x60])  # Bypasses beam and bias RAM (use SPI for gain/phase)
    if Rx_mode == 1:
        print("ADAR1000 is in Rx Mode")
        spi.xfer2([ADDR, 0x2E, 0x7F])  # Enables all 4 Rx channels, LNA, VGA, and Vector Mod
        spi.xfer2([ADDR, 0x34, 0x08])  # Sets LNA bias to middle of its range
        spi.xfer2([ADDR, 0x35, 0x16])  # Sets VGA bias to [0010] and vector mod bias to [110]
        spi.xfer2([ADDR, 0x31, 0xB0])  # Enables the whole Rx and sets the ADTR1107 switch high (Rx mode)
        spi.xfer2([ADDR, 0x10, int(128+127)])  # Sets Rx1 VGA gain
        spi.xfer2([ADDR, 0x11, int(128+127)])  # Sets Rx2 VGA gain
        spi.xfer2([ADDR, 0x12, int(128+127)])  # Sets Rx3 VGA gain
        spi.xfer2([ADDR, 0x13, int(128+127)])  # Sets Rx4 VGA gain
    else:
        print("ADAR1000 is in Tx Mode")
        spi.xfer2([ADDR, 0x2F, 0x7F])  # Enables all four Tx channels, the Tx Driver, Tx Vector Modulator and Tx VGA
        spi.xfer2([ADDR, 0x36, 0x16])  # Sets the TX VGA bias to [0010] and the TX vector modulator bias to [110]
        spi.xfer2([ADDR, 0x37, 0x06])  # Sets the Tx Driver bias to [110]
        #spi.xfer2([ADDR, 0x31, 0x42])  # Enables the whole Tx and enable TR_SPI
        spi.xfer2([ADDR, 0x31, 0xD0])  # Enables Tx and the switch, but puts part into Rx mode until we bias the PA's properly.  TR_SW_POS should be at 3.3V now
        spi.xfer2([ADDR, 0x30, 0x40])  # Sets bit 6 high (enables control of pins for external biasing)
        spi.xfer2([ADDR, 0x29, 0x39])  # Sets "on" PA gate bias to about -1.1V
        spi.xfer2([ADDR, 0x2A, 0x39])  # Sets "on" PA gate bias to about -1.1V
        spi.xfer2([ADDR, 0x2B, 0x39])  # Sets "on" PA gate bias to about -1.1V
        spi.xfer2([ADDR, 0x2C, 0x39])  # Sets "on" PA gate bias to about -1.1V
        spi.xfer2([ADDR, 0x46, 0x68])  # Sets "off" PA gate bias to about -2V
        spi.xfer2([ADDR, 0x47, 0x68])  # Sets "off" PA gate bias to about -2V
        spi.xfer2([ADDR, 0x48, 0x68])  # Sets "off" PA gate bias to about -2V
        spi.xfer2([ADDR, 0x49, 0x68])  # Sets "off" PA gate bias to about -2V
        spi.xfer2([ADDR, 0x31, 0x90])  # Changes Reg 0x31 to TX_EN=0.  PA_BIAS should be at -2V now.    

        # Write registers to set Tx1-4 to 45 deg and Max Gain
        spi.xfer2([ADDR, 0x1C, 128])  # Sets Tx1 gain
        spi.xfer2([ADDR, 0x20, 0x3F])  # Sets Tx1 I vector to positive and [10110]
        spi.xfer2([ADDR, 0x21, 0x20])  # Sets Tx1 Q vector to positive and [10110]
        spi.xfer2([ADDR, 0x1D, 128])  # Sets Tx2 gain
        spi.xfer2([ADDR, 0x22, 0x3F])  # Sets Tx2 I vector to positive and [10110]
        spi.xfer2([ADDR, 0x23, 0x20])  # Sets Tx2 Q vector to positive and [10110]
        spi.xfer2([ADDR, 0x1E, 128])  # Sets Tx3 gain
        spi.xfer2([ADDR, 0x24, 0x3F])  # Sets Tx3 I vector to positive and [10110]
        spi.xfer2([ADDR, 0x25, 0x20])  # Sets Tx3 Q vector to positive and [10110]
        spi.xfer2([ADDR, 0x1F, 128])  # Sets Tx4 gain
        spi.xfer2([ADDR, 0x26, 0x3F])  # Sets Tx4 I vector to positive and [10110]
        spi.xfer2([ADDR, 0x27, 0x20])  # Sets Tx4 Q vector to positive and [10110]

        spi.xfer2([ADDR, 0x28, 0x02])  # Loads Tx vectors from SPI.

        spi.xfer2([ADDR, 0x31, 0xD2])  # Put part into Tx mode (bit 1=High).  TR_SW_POS should be 0V and PA bias should be -1.1V.

    time.sleep(0.1)


def ADAR_set_RxTaper(spi, ADDR, Rx_mode, RxGain1, RxGain2, RxGain3, RxGain4):
    # set the ADAR1000's VGA gain of each of the Rx channels.  RxGainx needs to be between 0 and 127
    if Rx_mode == 1:
        spi.xfer2([ADDR, 0x10, int(128+RxGain1)])  # Sets Rx1 VGA gain
        spi.xfer2([ADDR, 0x11, int(128+RxGain2)])  # Sets Rx2 VGA gain
        spi.xfer2([ADDR, 0x12, int(128+RxGain3)])  # Sets Rx3 VGA gain
        spi.xfer2([ADDR, 0x13, int(128+RxGain4)])  # Sets Rx4 VGA gain
    else:
        spi.xfer2([ADDR, 0x1C, int(128+RxGain1)])  # Sets Tx1 VGA gain
        spi.xfer2([ADDR, 0x1D, int(128+RxGain2)])  # Sets Tx2 VGA gain
        spi.xfer2([ADDR, 0x1E, int(128+RxGain3)])  # Sets Tx3 VGA gain
        spi.xfer2([ADDR, 0x1F, int(128+RxGain4)])  # Sets Tx4 VGA gain
        #print("TX mode, ADDR = ", ADDR, RxGain1, RxGain2, RxGain3, RxGain4)
        spi.xfer2([ADDR, 0x31, 0xD2])  # Put part into Tx mode (bit 1=High).  TR_SW_POS should be 0V and PA bias should be -1.1V.

    if Rx_mode == 1:
        spi.xfer2([ADDR, 0x28, 0x01])  # Loads Rx vectors from SPI.  0x08 is all ADAR1000 devices
    else:
        spi.xfer2([ADDR, 0x28, 0x02])  # Loads Tx vectors from SPI.  0x08 is all ADAR1000 devices
        
def ADAR_set_RxPhase(spi, address, num_ADARs, Rx_mode, PhDelta, phase_step_size, RxPhase1, RxPhase2, RxPhase3, RxPhase4):
    step_size = 360  #2.8125
    Phase_A = ((np.rint(PhDelta*0/step_size)*step_size) + RxPhase1) % 360
    Phase_B = ((np.rint(PhDelta*1/step_size)*step_size) + RxPhase2) % 360
    Phase_C = ((np.rint(PhDelta*2/step_size)*step_size) + RxPhase3) % 360
    Phase_D = ((np.rint(PhDelta*3/step_size)*step_size) + RxPhase4) % 360
    if num_ADARs == 2:
        Phase_A = ((np.rint(PhDelta*4/step_size)*step_size) + RxPhase1) % 360
        Phase_B = ((np.rint(PhDelta*5/step_size)*step_size) + RxPhase2) % 360
        Phase_C = ((np.rint(PhDelta*6/step_size)*step_size) + RxPhase3) % 360
        Phase_D = ((np.rint(PhDelta*7/step_size)*step_size) + RxPhase4) % 360
    channels = [Phase_A, Phase_B, Phase_C, Phase_D]

    # Write vector I and Q to set phase shift (see Table 13 in ADAR1000 datasheet)
    i=1
    for Channel_Phase in channels:
        #round_Phase = np.rint(Channel_Phase/step_size)*step_size
        if i==1:
            if Rx_mode == 1:
                I = 0x14   # Rx1_I vector register address = 0x14
                Q = 0x15   # Rx1_Q vector register address = 0x15
            else:
                I = 0x20   # Tx1
                Q = 0x21   # Tx1
        if i==2:
            if Rx_mode == 1:
                I = 0x16   # Rx2_I vector register address = 0x16
                Q = 0x17   # Rx2_Q vector register address = 0x17
            else:
                I = 0x22   # Tx2
                Q = 0x23   # Tx2
        if i==3:
            if Rx_mode == 1:
                I = 0x18   # Rx3_I vector register address = 0x18
                Q = 0x19   # Rx3_Q vector register address = 0x19
            else:
                I = 0x24   # Tx3
                Q = 0x25   # Tx3
        if i==4:
            if Rx_mode == 1:
                I = 0x1A   # Rx4_I vector register address = 0x1A
                Q = 0x1B   # Rx4_Q vector register address = 0x1B
            else:
                I = 0x26   # Tx4
                Q = 0x27   # Tx4
        
        ADAR_write_RxPhase(spi, address, Channel_Phase, I, Q)
        i = i+1
    if Rx_mode == 1:
        spi.xfer2([address, 0x28, 0x01])  # Loads Rx vectors from SPI.  0x08 is all ADAR1000 devices
    else:
        spi.xfer2([address, 0x28, 0x02])  # Loads Tx vectors from SPI.  0x08 is all ADAR1000 devices

def ADAR_write_RxPhase(spi, ADDR, Channel_Phase, I, Q):
    # See Table 13 in the ADAR1000 datasheet
    # Quadrant 1
    if Channel_Phase==0:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x20])
    if Channel_Phase==2.8125:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x21])
    if Channel_Phase==5.625:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x23])
    if Channel_Phase==8.4375:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x24])
    if Channel_Phase==11.25:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x26])
    if Channel_Phase==14.0625:
        spi.xfer2([ADDR, I, 0x3E])
        spi.xfer2([ADDR, Q, 0x27])
    if Channel_Phase==16.875:
        spi.xfer2([ADDR, I, 0x3E])
        spi.xfer2([ADDR, Q, 0x28])
    if Channel_Phase==19.6875:
        spi.xfer2([ADDR, I, 0x3D])
        spi.xfer2([ADDR, Q, 0x2A])
    if Channel_Phase==22.5:
        spi.xfer2([ADDR, I, 0x3D])
        spi.xfer2([ADDR, Q, 0x2B])
    if Channel_Phase==25.3125:
        spi.xfer2([ADDR, I, 0x3C])
        spi.xfer2([ADDR, Q, 0x2D])
    if Channel_Phase==28.125:
        spi.xfer2([ADDR, I, 0x3C])
        spi.xfer2([ADDR, Q, 0x2E])
    if Channel_Phase==30.9375:
        spi.xfer2([ADDR, I, 0x3B])
        spi.xfer2([ADDR, Q, 0x2F])
    if Channel_Phase==33.75:
        spi.xfer2([ADDR, I, 0x3A])
        spi.xfer2([ADDR, Q, 0x30])
    if Channel_Phase==36.5625:
        spi.xfer2([ADDR, I, 0x39])
        spi.xfer2([ADDR, Q, 0x31])
    if Channel_Phase==39.375:
        spi.xfer2([ADDR, I, 0x38])
        spi.xfer2([ADDR, Q, 0x33])
    if Channel_Phase==42.1875:
        spi.xfer2([ADDR, I, 0x37])
        spi.xfer2([ADDR, Q, 0x34])
    if Channel_Phase==45:
        spi.xfer2([ADDR, I, 0x36])
        spi.xfer2([ADDR, Q, 0x35])
    if Channel_Phase==47.8125:
        spi.xfer2([ADDR, I, 0x35])
        spi.xfer2([ADDR, Q, 0x36])
    if Channel_Phase==50.625:
        spi.xfer2([ADDR, I, 0x34])
        spi.xfer2([ADDR, Q, 0x37])
    if Channel_Phase==53.4375:
        spi.xfer2([ADDR, I, 0x33])
        spi.xfer2([ADDR, Q, 0x38])
    if Channel_Phase==56.25:
        spi.xfer2([ADDR, I, 0x32])
        spi.xfer2([ADDR, Q, 0x38])
        spi.xfer2([ADDR, Q, 0x39])
    if Channel_Phase==61.875:
        spi.xfer2([ADDR, I, 0x2F])
        spi.xfer2([ADDR, Q, 0x3A])
    if Channel_Phase==64.6875:
        spi.xfer2([ADDR, I, 0x2E])
        spi.xfer2([ADDR, Q, 0x3A])
    if Channel_Phase==67.5:
        spi.xfer2([ADDR, I, 0x2C])
        spi.xfer2([ADDR, Q, 0x3B])
    if Channel_Phase==70.3125:
        spi.xfer2([ADDR, I, 0x2B])
        spi.xfer2([ADDR, Q, 0x3C])
    if Channel_Phase==73.125:
        spi.xfer2([ADDR, I, 0x2A])
        spi.xfer2([ADDR, Q, 0x3C])
    if Channel_Phase==75.9375:
        spi.xfer2([ADDR, I, 0x28])
        spi.xfer2([ADDR, Q, 0x3C])
    if Channel_Phase==78.75:
        spi.xfer2([ADDR, I, 0x27])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==81.5625:
        spi.xfer2([ADDR, I, 0x25])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==84.375:
        spi.xfer2([ADDR, I, 0x24])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==87.1875:
        spi.xfer2([ADDR, I, 0x22])
        spi.xfer2([ADDR, Q, 0x3D])
# Quadrant 2
    if Channel_Phase==90:
        spi.xfer2([ADDR, I, 0x21])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==92.8125:
        spi.xfer2([ADDR, I, 0x01])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==95.625:
        spi.xfer2([ADDR, I, 0x03])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==98.4375:
        spi.xfer2([ADDR, I, 0x04])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==101.25:
        spi.xfer2([ADDR, I, 0x06])
        spi.xfer2([ADDR, Q, 0x3D])
    if Channel_Phase==104.0625:
        spi.xfer2([ADDR, I, 0x07])
        spi.xfer2([ADDR, Q, 0x3C])
    if Channel_Phase==106.875:
        spi.xfer2([ADDR, I, 0x08])
        spi.xfer2([ADDR, Q, 0x3C])
    if Channel_Phase==109.6875:
        spi.xfer2([ADDR, I, 0x0A])
        spi.xfer2([ADDR, Q, 0x3C])
    if Channel_Phase==112.5:
        spi.xfer2([ADDR, I, 0x0B])
        spi.xfer2([ADDR, Q, 0x3B])
    if Channel_Phase==115.3125:
        spi.xfer2([ADDR, I, 0x0D])
        spi.xfer2([ADDR, Q, 0x3A])
    if Channel_Phase==118.125:
        spi.xfer2([ADDR, I, 0x0E])
        spi.xfer2([ADDR, Q, 0x3A])
    if Channel_Phase==120.9375:
        spi.xfer2([ADDR, I, 0x0F])
        spi.xfer2([ADDR, Q, 0x39])
    if Channel_Phase==123.75:
        spi.xfer2([ADDR, I, 0x11])
        spi.xfer2([ADDR, Q, 0x38])
    if Channel_Phase==126.5625:
        spi.xfer2([ADDR, I, 0x12])
        spi.xfer2([ADDR, Q, 0x38])
    if Channel_Phase==129.375:
        spi.xfer2([ADDR, I, 0x13])
        spi.xfer2([ADDR, Q, 0x37])
    if Channel_Phase==132.1875:
        spi.xfer2([ADDR, I, 0x14])
        spi.xfer2([ADDR, Q, 0x36])
    if Channel_Phase==135:
        spi.xfer2([ADDR, I, 0x16])
        spi.xfer2([ADDR, Q, 0x35])
    if Channel_Phase==137.8125:
        spi.xfer2([ADDR, I, 0x17])
        spi.xfer2([ADDR, Q, 0x34])
    if Channel_Phase==140.625:
        spi.xfer2([ADDR, I, 0x18])
        spi.xfer2([ADDR, Q, 0x33])
    if Channel_Phase==143.4375:
        spi.xfer2([ADDR, I, 0x19])
        spi.xfer2([ADDR, Q, 0x31])
    if Channel_Phase==146.25:
        spi.xfer2([ADDR, I, 0x19])
        spi.xfer2([ADDR, Q, 0x30])
    if Channel_Phase==149.0625:
        spi.xfer2([ADDR, I, 0x1A])
        spi.xfer2([ADDR, Q, 0x2F])
    if Channel_Phase==151.875:
        spi.xfer2([ADDR, I, 0x1B])
        spi.xfer2([ADDR, Q, 0x2E])
    if Channel_Phase==154.6875:
        spi.xfer2([ADDR, I, 0x1C])
        spi.xfer2([ADDR, Q, 0x2D])
    if Channel_Phase==157.5:
        spi.xfer2([ADDR, I, 0x1C])
        spi.xfer2([ADDR, Q, 0x2B])
    if Channel_Phase==160.3125:
        spi.xfer2([ADDR, I, 0x1D])
        spi.xfer2([ADDR, Q, 0x2A])
    if Channel_Phase==163.125:
        spi.xfer2([ADDR, I, 0X1E])
        spi.xfer2([ADDR, Q, 0x28])
    if Channel_Phase==165.9375:
        spi.xfer2([ADDR, I, 0x1E])
        spi.xfer2([ADDR, Q, 0x27])
    if Channel_Phase==168.75:
        spi.xfer2([ADDR, I, 0x1E])
        spi.xfer2([ADDR, Q, 0x26])
    if Channel_Phase==171.5625:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x24])
    if Channel_Phase==174.375:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x23])
    if Channel_Phase==177.1875:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x21])
# Quadrant 3
    if Channel_Phase==180:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x20])
    if Channel_Phase==182.8125:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x20])
    if Channel_Phase==185.625:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x03])
    if Channel_Phase==188.4375:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x04])
    if Channel_Phase==191.25:
        spi.xfer2([ADDR, I, 0x1F])
        spi.xfer2([ADDR, Q, 0x06])
    if Channel_Phase==194.0625:
        spi.xfer2([ADDR, I, 0x1E])
        spi.xfer2([ADDR, Q, 0x07])
    if Channel_Phase==196.875:
        spi.xfer2([ADDR, I, 0x1E])
        spi.xfer2([ADDR, Q, 0x08])
    if Channel_Phase==199.6875:
        spi.xfer2([ADDR, I, 0x1D])
        spi.xfer2([ADDR, Q, 0x0A])
    if Channel_Phase==202.5:
        spi.xfer2([ADDR, I, 0x1D])
        spi.xfer2([ADDR, Q, 0x0B])
    if Channel_Phase==205.3125:
        spi.xfer2([ADDR, I, 0x1C])
        spi.xfer2([ADDR, Q, 0x0D])
    if Channel_Phase==208.125:
        spi.xfer2([ADDR, I, 0x1C])
        spi.xfer2([ADDR, Q, 0x0E])
    if Channel_Phase==210.9375:
        spi.xfer2([ADDR, I, 0x1B])
        spi.xfer2([ADDR, Q, 0x0F])
    if Channel_Phase==213.75:
        spi.xfer2([ADDR, I, 0x1A])
        spi.xfer2([ADDR, Q, 0x10])
    if Channel_Phase==216.5625:
        spi.xfer2([ADDR, I, 0x19])
        spi.xfer2([ADDR, Q, 0x11])
    if Channel_Phase==219.375:
        spi.xfer2([ADDR, I, 0x18])
        spi.xfer2([ADDR, Q, 0x13])
    if Channel_Phase==222.1875:
        spi.xfer2([ADDR, I, 0x17])
        spi.xfer2([ADDR, Q, 0x14])
    if Channel_Phase==225:
        spi.xfer2([ADDR, I, 0x16])
        spi.xfer2([ADDR, Q, 0x15])
    if Channel_Phase==227.8125:
        spi.xfer2([ADDR, I, 0x15])
        spi.xfer2([ADDR, Q, 0x16])
    if Channel_Phase==230.625:
        spi.xfer2([ADDR, I, 0x14])
        spi.xfer2([ADDR, Q, 0x17])
    if Channel_Phase==233.4375:
        spi.xfer2([ADDR, I, 0x13])
        spi.xfer2([ADDR, Q, 0x18])
    if Channel_Phase==236.25:
        spi.xfer2([ADDR, I, 0x12])
        spi.xfer2([ADDR, Q, 0x18])
    if Channel_Phase==239.0625:
        spi.xfer2([ADDR, I, 0x10])
        spi.xfer2([ADDR, Q, 0x19])
    if Channel_Phase==241.875:
        spi.xfer2([ADDR, I, 0x0F])
        spi.xfer2([ADDR, Q, 0x1A])
    if Channel_Phase==244.6875:
        spi.xfer2([ADDR, I, 0x0E])
        spi.xfer2([ADDR, Q, 0x1A])
    if Channel_Phase==247.5:
        spi.xfer2([ADDR, I, 0x0C])
        spi.xfer2([ADDR, Q, 0x1B])
    if Channel_Phase==250.3125:
        spi.xfer2([ADDR, I, 0x0B])
        spi.xfer2([ADDR, Q, 0x1C])
    if Channel_Phase==253.125:
        spi.xfer2([ADDR, I, 0x0A])
        spi.xfer2([ADDR, Q, 0x1C])
    if Channel_Phase==255.9375:
        spi.xfer2([ADDR, I, 0x08])
        spi.xfer2([ADDR, Q, 0x1C])
    if Channel_Phase==258.75:
        spi.xfer2([ADDR, I, 0x07])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==261.5625:
        spi.xfer2([ADDR, I, 0x05])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==264.375:
        spi.xfer2([ADDR, I, 0x04])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==267.1875:
        spi.xfer2([ADDR, I, 0x02])
        spi.xfer2([ADDR, Q, 0x1D])
# Quadrant 4
    if Channel_Phase==270:
        spi.xfer2([ADDR, I, 0x01])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==272.8125:
        spi.xfer2([ADDR, I, 0x21])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==275.625:
        spi.xfer2([ADDR, I, 0x23])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==278.4375:
        spi.xfer2([ADDR, I, 0x24])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==281.25:
        spi.xfer2([ADDR, I, 0x26])
        spi.xfer2([ADDR, Q, 0x1D])
    if Channel_Phase==284.0625:
        spi.xfer2([ADDR, I, 0x27])
        spi.xfer2([ADDR, Q, 0x1C])
    if Channel_Phase==286.875:
        spi.xfer2([ADDR, I, 0x28])
        spi.xfer2([ADDR, Q, 0x1C])
    if Channel_Phase==289.6875:
        spi.xfer2([ADDR, I, 0x2A])
        spi.xfer2([ADDR, Q, 0x1C])
    if Channel_Phase==292.5:
        spi.xfer2([ADDR, I, 0x2B])
        spi.xfer2([ADDR, Q, 0x1B])
    if Channel_Phase==295.3125:
        spi.xfer2([ADDR, I, 0x2D])
        spi.xfer2([ADDR, Q, 0x1A])
    if Channel_Phase==298.125:
        spi.xfer2([ADDR, I, 0x2E])
        spi.xfer2([ADDR, Q, 0x1A])
    if Channel_Phase==300.9375:
        spi.xfer2([ADDR, I, 0x2F])
        spi.xfer2([ADDR, Q, 0x19])
    if Channel_Phase==303.75:
        spi.xfer2([ADDR, I, 0x31])
        spi.xfer2([ADDR, Q, 0x18])
    if Channel_Phase==306.5625:
        spi.xfer2([ADDR, I, 0x32])
        spi.xfer2([ADDR, Q, 0x18])
    if Channel_Phase==309.375:
        spi.xfer2([ADDR, I, 0x33])
        spi.xfer2([ADDR, Q, 0x17])
    if Channel_Phase==312.1875:
        spi.xfer2([ADDR, I, 0x34])
        spi.xfer2([ADDR, Q, 0x16])
    if Channel_Phase==315:
        spi.xfer2([ADDR, I, 0x36])
        spi.xfer2([ADDR, Q, 0x15])
    if Channel_Phase==317.8125:
        spi.xfer2([ADDR, I, 0x37])
        spi.xfer2([ADDR, Q, 0x14])
    if Channel_Phase==320.625:
        spi.xfer2([ADDR, I, 0x38])
        spi.xfer2([ADDR, Q, 0x13])
    if Channel_Phase==323.4375:
        spi.xfer2([ADDR, I, 0x39])
        spi.xfer2([ADDR, Q, 0x11])
    if Channel_Phase==326.25:
        spi.xfer2([ADDR, I, 0x39])
        spi.xfer2([ADDR, Q, 0x10])
    if Channel_Phase==329.0625:
        spi.xfer2([ADDR, I, 0x3A])
        spi.xfer2([ADDR, Q, 0x0F])
    if Channel_Phase==331.875:
        spi.xfer2([ADDR, I, 0x3B])
        spi.xfer2([ADDR, Q, 0x0E])
    if Channel_Phase==334.6875:
        spi.xfer2([ADDR, I, 0x3C])
        spi.xfer2([ADDR, Q, 0x0D])
    if Channel_Phase==337.5:
        spi.xfer2([ADDR, I, 0x3C])
        spi.xfer2([ADDR, Q, 0x0B])
    if Channel_Phase==340.3125:
        spi.xfer2([ADDR, I, 0x3D])
        spi.xfer2([ADDR, Q, 0x0A])
    if Channel_Phase==343.125:
        spi.xfer2([ADDR, I, 0x3E])
        spi.xfer2([ADDR, Q, 0x08])
    if Channel_Phase==345.9375:
        spi.xfer2([ADDR, I, 0x3E])
        spi.xfer2([ADDR, Q, 0x07])
    if Channel_Phase==348.75:
        spi.xfer2([ADDR, I, 0x3E])
        spi.xfer2([ADDR, Q, 0x06])
    if Channel_Phase==351.5625:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x04])
    if Channel_Phase==354.375:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x03])
    if Channel_Phase==357.1875:
        spi.xfer2([ADDR, I, 0x3F])
        spi.xfer2([ADDR, Q, 0x01])


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block

    def __init__(self, addr=0, reprogram=0, rx_en=1, Taper=1, SymTaper=0, PhaseCal=0, SignalFreq=10525000000, RxGain1=127, RxGain2=127, RxGain3=127, RxGain4=127, Rx1_cal=0, Rx2_cal=0, Rx3_cal=0, Rx4_cal=0):  
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='ADAR1000 Sweeper',   # will show up in GRC
            in_sig=[],
            out_sig=[np.complex64, np.float32]
        )
        self.addr = addr
        self.rx_en = rx_en   # set to 1 for ADAR1000 Rx mode, or 0 for ADAR1000 Tx mode
        self.reprogram = reprogram

        self.Taper = Taper
        self.SymTaper = SymTaper
        self.PhaseCal = PhaseCal
        self.RxGain1 = RxGain1
        self.RxGain2 = RxGain2
        self.RxGain3 = RxGain3
        self.RxGain4 = RxGain4
        self.Rx1_cal=Rx1_cal
        self.Rx2_cal=Rx2_cal
        self.Rx3_cal=Rx3_cal
        self.Rx4_cal=Rx4_cal
                       
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  #set bus=0 and device=0
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0

        # The ADDR is set by the address pins on the ADAR1000
        if self.addr == 0:
            self.ADDR1=0x00            
        if self.addr == 1:
            self.ADDR1=0x20
        if self.addr == 2:
            self.ADDR1=0x40
        if self.addr == 3:
            self.ADDR1=0x60
            
        ADAR_init(self.spi, self.ADDR1, self.rx_en)
        


    def work(self, input_items, output_items):
        # The ADDR is set by the address pins on the ADAR1000
        if self.addr == 0:
            self.ADDR1=0x00            
        if self.addr == 1:
            self.ADDR1=0x20
        if self.addr == 2:
            self.ADDR1=0x40
        if self.addr == 3:
            self.ADDR1=0x60

        if self.reprogram == 1:
            print("reprogram")
            ADAR_init(self.spi, self.ADDR1, self.rx_en)
        
        time.sleep(0.1)

        if self.Taper==0:
            if self.SymTaper==0:
                Gain4=self.RxGain4  # Sets Rx4 VGA gain
            else:
                Gain4=self.RxGain1  # Sets Rx4 VGA gain
            ADAR_set_RxTaper(self.spi, self.ADDR1, self.rx_en, self.RxGain1, self.RxGain2, self.RxGain3, Gain4)
        else:
            ADAR_set_RxTaper(self.spi, self.ADDR1, self.rx_en, 127, 127, 127, 127)
        
        if self.PhaseCal == 0:
            Rx1_Phase_Cal = self.Rx1_cal
            Rx2_Phase_Cal = self.Rx2_cal
            Rx3_Phase_Cal = self.Rx3_cal
            Rx4_Phase_Cal = self.Rx4_cal
        else:
            Rx1_Phase_Cal = 0
            Rx2_Phase_Cal = 0
            Rx3_Phase_Cal = 0
            Rx4_Phase_Cal = 0

        PhaseValues = np.arange(0, 1, 1)   
        PhaseStepNumber=0    # this is the number of phase steps we'll take (140 in total).  At each phase step, we set the individual phases of each of the Rx channels
        for PhDelta in PhaseValues:
            ADAR_set_RxPhase(self.spi, self.ADDR1, 1, self.rx_en, 0, 360, Rx1_Phase_Cal, Rx2_Phase_Cal, Rx3_Phase_Cal, Rx4_Phase_Cal)               
            output_items[0][PhaseStepNumber]=((1) + (1j))  # output this as a complex number so we can do an x-y plot with the constellation graph
            PhaseStepNumber=PhaseStepNumber+1    # increment the phase delta and start this whole thing again.  This will repeat 140 times
        output_items[0]=output_items[0][0:PhaseStepNumber]
        output_items[1][:] = 1
        
        return len(output_items[0])
