# Closed loop monopulse tracking using the ADRV9361-SOM and 2 ADAR1000 ICs

'''
Useful resources:
    Introduction to Phased Array Antennas: https://www.analog.com/en/analog-dialogue/articles/phased-array-antenna-patterns-part1.html#
    Full workshop build instructions at www.github.com/jonkraft/phasedarray
    Analog Devices Python Interfaces:  https://analogdevicesinc.github.io/pyadi-iio/
    Python examples:  https://github.com/analogdevicesinc/pyadi-iio/tree/ensm-example/examples
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

sys.path.append('/usr/lib/python2.7/site-packages/')
sys.path.append('/lib/python3.7/site-packages/')
#import iio          # see temporary issue here https://ez.analog.com/sw-interface-tools/f/q-a/534115/pyadi-iio-operation-in-gnuradio-3-8
import iiopy as iio  # this is a temporary workaround due to a conflict between gr-iio and pyadi-iio
import adi

def ADAR_init(spi, ADDR):
    # Initialize the ADAR1000
    spi.xfer2([ADDR, 0x00, 0x81])  # reset the device
    spi.xfer2([ADDR, 0x00, 0x18])  # Sets SDO  pin to active (4 wire SPI)
    spi.xfer2([ADDR+0x04, 0x00, 0x55])  # Trims LDO to 1.8V
    spi.xfer2([ADDR, 0x38, 0x60])  # Bypasses beam and bias RAM (use SPI for gain/phase)
    spi.xfer2([ADDR, 0x2E, 0x7F])  # Enables all 4 Rx channels, LNA, VGA, and Vector Mod
    spi.xfer2([ADDR, 0x34, 0x08])  # Sets LNA bias to middle of its range
    spi.xfer2([ADDR, 0x35, 0x16])  # Sets VGA bias to [0010] and vector mod bias to [110]
    spi.xfer2([ADDR, 0x31, 0xB0])  # Enables the whole Rx and sets the ADTR1107 switch high (Rx mode)
    spi.xfer2([ADDR, 0x10, int(128+127)])  # Sets Rx1 VGA gain
    spi.xfer2([ADDR, 0x11, int(128+127)])  # Sets Rx2 VGA gain
    spi.xfer2([ADDR, 0x12, int(128+127)])  # Sets Rx3 VGA gain
    spi.xfer2([ADDR, 0x13, int(128+127)])  # Sets Rx4 VGA gain

def ADAR_set_RxTaper(spi, ADDR, RxGain1, RxGain2, RxGain3, RxGain4):
    # set the ADAR1000's VGA gain of each of the Rx channels.  RxGainx needs to be between 0 and 127
    spi.xfer2([ADDR, 0x10, int(128+RxGain1)])  # Sets Rx1 VGA gain
    spi.xfer2([ADDR, 0x11, int(128+RxGain2)])  # Sets Rx2 VGA gain
    spi.xfer2([ADDR, 0x12, int(128+RxGain3)])  # Sets Rx3 VGA gain
    spi.xfer2([ADDR, 0x13, int(128+RxGain4)])  # Sets Rx4 VGA gain

def ADAR_set_RxPhase(spi, address, num_ADARs, PhDelta, phase_step_size, RxPhase1, RxPhase2, RxPhase3, RxPhase4):
    step_size = phase_step_size  #  Must be a multiple of 2.8125 
    Phase_A = ((np.rint(PhDelta*0/step_size)*step_size) + RxPhase1) % 360   # rounds phase values to nearest 2.8125 value
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
            I = 0x14   # Rx1_I vector register address = 0x14
            Q = 0x15   # Rx1_Q vector register address = 0x15
        if i==2:
            I = 0x16   # Rx2_I vector register address = 0x16
            Q = 0x17   # Rx2_Q vector register address = 0x17
        if i==3:
            I = 0x18   # Rx3_I vector register address = 0x18
            Q = 0x19   # Rx3_Q vector register address = 0x19
        if i==4:
            I = 0x1A   # Rx4_I vector register address = 0x1A
            Q = 0x1B   # Rx4_Q vector register address = 0x1B
        ADAR_write_RxPhase(spi, address, Channel_Phase, I, Q)
        i = i+1
    spi.xfer2([address, 0x28, 0x01])  # Loads Rx vectors from SPI.  0x08 is all ADAR1000 devices

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
    if Channel_Phase==59.0625:
        spi.xfer2([ADDR, I, 0x30])
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

    def __init__(self, IP=[192, 168, 0, 3], LO_freq=2400000000, TX_freq=5810000000, SampleRate=3000000, Rx_gain=30, Averages=1, Taper=1, SymTaper=0, PhaseCal=0, ScanMaxAngle=0, Error_Threshold=0.1, SignalFreq=10525000000, RxGain1=127, RxGain2=127, RxGain3=127, RxGain4=127, Rx1_cal=0, Rx2_cal=0, Rx3_cal=0, Rx4_cal=0, RxGain5=127, RxGain6=127, RxGain7=127, RxGain8=127, Rx5_cal=0, Rx6_cal=0, Rx7_cal=0, Rx8_cal=0):  
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='ADAR1000 Sweeper',   # will show up in GRC
            in_sig=[],
            #out_sig=[np.complex64, np.complex64, np.complex64, np.complex64, np.float32]
            out_sig=[np.float32, np.float32]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        #sdr_address = 'ip:192.168.0.4'
        sdr_address = "ip:"+str(IP[0])+"."+str(IP[1])+"."+str(IP[2])+"."+str(IP[3])  # I couldn't get GNUradio to import the IP address from the flowgraph block as a string.... Not sure why.  So I broke it up into integers
        self.LO_freq = LO_freq              # RX LO freq
        self.TX_freq = TX_freq              # TX LO freq
        self.SampleRate = SampleRate
        self.Rx_gain = Rx_gain
        self.Averages = Averages
        self.Taper = Taper
        self.SymTaper = SymTaper
        self.PhaseCal = PhaseCal
        
        self.RxGain1 = RxGain1
        self.RxGain2 = RxGain2
        self.RxGain3 = RxGain3
        self.RxGain4 = RxGain4
        self.RxGain5 = RxGain5
        self.RxGain6 = RxGain6
        self.RxGain7 = RxGain7
        self.RxGain8 = RxGain8
        self.Rx1_cal=Rx1_cal
        self.Rx2_cal=Rx2_cal
        self.Rx3_cal=Rx3_cal
        self.Rx4_cal=Rx4_cal
        self.Rx5_cal=Rx5_cal
        self.Rx6_cal=Rx6_cal
        self.Rx7_cal=Rx7_cal
        self.Rx8_cal=Rx8_cal
        
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  #set bus=0 and device=0
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0
        
        # The ADDR is set by the address pins on the ADAR1000.  This is set by P10 on the eval board.
        self.ADDR1=0x20            # ADDR 0x20 is set by jumpering pins 4 and 6 on P10
        #self.ADDR1=0x00           # ADDR 0x00 is set by leaving all jumpers off of P10
        self.ADDR2 = 0x40
        ADAR_init(self.spi, self.ADDR1)
        ADAR_init(self.spi, self.ADDR2)
        
        self.c = 299792458    # speed of light in m/s
        self.d = 0.015        # element to element spacing of the antenna
        self.SignalFreq = SignalFreq
        
        self.ScanMaxAngle = ScanMaxAngle
        self.PeakPhaseDelta = 0
        self.Error_Threshold = Error_Threshold
        self.loop_count = 0
        self.buffer_array = []
        for i in range(0,4000):
            self.buffer_array.append(0)  # array of zeros
        
        '''Setup SDR Context and Configure Settings'''        
        #self.sdr=adi.ad9361(uri='ip:192.168.0.3')
        self.sdr = adi.ad9361(uri=sdr_address)
        self.sdr._ctrl.debug_attrs["adi,frequency-division-duplex-mode-enable"].value = "1"  # move to fdd mode.  see https://github.com/analogdevicesinc/pyadi-iio/blob/ensm-example/examples/ad9361_advanced_ensm.py
        self.sdr._ctrl.debug_attrs["adi,ensm-enable-txnrx-control-enable"].value = "0"       # Disable pin control so spi can move the states
        self.sdr._ctrl.debug_attrs["initialize"].value = "1"
        self.sdr._rxadc.set_kernel_buffers_count(1)   #Default is 4 Rx buffers are stored, but we want to change and immediately measure the result, so buffers=1
        rx = self.sdr._ctrl.find_channel('voltage0') 
        rx.attrs['quadrature_tracking_en'].value = '0'   # set to '1' to enable quadrature tracking
        self.sdr.rx_enabled_channels = [0, 1]   # enable both Rx1 (voltage0) and Rx2 (voltage1)
        self.sdr.sample_rate = int(self.SampleRate)
        #self.sdr.filter = "/home/pi/Documents/PlutoFilters/samprate_40p0.ftr"  #pyadi-iio auto applies filters based on sample rate
        #self.sdr.rx_rf_bandwidth = int(1000000)
        #self.sdr.tx_rf_bandwidth = int(500000)
        self.sdr.gain_control_mode_chan0 = 'manual'      #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
        self.sdr.gain_control_mode_chan1 = 'manual'      #We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
        self.sdr.rx_buffer_size = int(1024)    # We only need a few samples to get the gain.  And a small buffer will greatly speed up the sweep
        self.sdr.tx_hardwaregain_chan0 = -80   # Make sure the Tx channels are attenuated (or off) and their freq is far away from Rx
        self.sdr.tx_hardwaregain_chan1 = -80
        self.sdr.tx_lo = int(1000000000)
        print(self.sdr)
        
    def findMaxAngle(self, ADDR1, ADDR2, phase_step_size, RxPhase1, RxPhase2, RxPhase3, RxPhase4, RxPhase5, RxPhase6, RxPhase7, RxPhase8):
        PhaseValues = np.arange(-196.875, 196.875, 2.8125)   # These are all the phase deltas (i.e. phase difference between Rx1 and Rx2, then Rx2 and Rx3, etc.) we'll sweep.     
        max_signal = -1000
        max_PhaseDelta = 0        # Reset max_angle.  This is the angle where we saw the max signal.  This is where our compass will point.
        for PhDelta in PhaseValues:
            ADAR_set_RxPhase(self.spi, ADDR1, 1, PhDelta, phase_step_size, RxPhase1, RxPhase2, RxPhase3, RxPhase4)
            ADAR_set_RxPhase(self.spi, ADDR2, 2, PhDelta, phase_step_size, RxPhase5, RxPhase6, RxPhase7, RxPhase8)
            data=self.sdr.rx()          #read a buffer of data from Pluto using pyadi-iio library (adi.py)
            chan1 = data[0]   # Rx1 data
            chan2 = data[1]   # Rx2 data
            sum_chan = chan1+chan2
            N = len(sum_chan)               #number of samples
            win = np.hamming(N)
            y_sum = sum_chan * win
            s_sum = np.fft.fftshift(y_sum)
            max_index = np.argmax(s_sum)             
            s_mag_sum = np.abs(s_sum[max_index]) #* 2 / np.sum(win)
            if s_mag_sum > max_signal:
                max_signal = s_mag_sum
                max_PhaseDelta = PhDelta
        return(max_PhaseDelta)
    
    def GetTrackingError(self):
        total_sum=0
        total_delta=0
        total_angle=0
        for count in range (0,self.Averages):       # repeat loop and average the results
            data=self.sdr.rx()          #read a buffer of data from Pluto using pyadi-iio library (adi.py)
            chan1 = data[0]   # Rx1 data
            chan2 = data[1]   # Rx2 data
            sum_chan = chan1+chan2
            delta_chan = chan1-chan2
            N = len(sum_chan)               #number of samples
            win = np.hamming(N)
            y_sum = sum_chan * win
            y_delta = delta_chan * win
            s_sum = np.fft.fftshift(y_sum)
            s_delta = np.fft.fftshift(y_delta)
            max_index = np.argmax(s_sum)
            total_angle = total_angle + (np.angle(s_sum[max_index]) - np.angle(s_delta[max_index]))
            s_mag_sum = np.abs(s_sum[max_index]) #* 2 / np.sum(win)
            s_mag_delta = np.abs(s_delta[max_index]) #* 2 / np.sum(win)
            s_dbfs_sum = 20*np.log10(np.max([s_mag_sum, 10**(-15)])/(2**12))        # make sure the log10 argument isn't zero (hence np.max)
            s_dbfs_delta = 20*np.log10(np.max([s_mag_delta, 10**(-15)])/(2**12))    # make sure the log10 argument isn't zero (hence np.max)
            total_sum=total_sum+(s_dbfs_sum)   # sum up all the loops, then we'll average
            total_delta=total_delta+(s_dbfs_delta)   # sum up all the loops, then we'll average
        PeakValue_sum = total_sum/self.Averages
        PeakValue_delta = total_delta/self.Averages
        PeakValue_angle = total_angle/self.Averages
        if PeakValue_angle > 0:
            diff_angle = 1
            diff_chan = max((diff_angle * (PeakValue_sum - PeakValue_delta) + diff_angle * (PeakValue_sum + PeakValue_delta)/2) / (PeakValue_sum + PeakValue_delta), 0.01)
        else:
            diff_angle = -1
            diff_chan = min((diff_angle * (PeakValue_sum - PeakValue_delta) + diff_angle * (PeakValue_sum + PeakValue_delta)/2) / (PeakValue_sum + PeakValue_delta), -0.01)
        return [diff_chan, PeakValue_sum]
    
    def ConvertPhaseToSteerAngle(self, PhDelta):
        # steering angle theta = arcsin(c*deltaphase/(2*pi*f*d)
        # if element spacing is lambda/2 then this simplifies to theta=arcsin(deltaphase(in radians)/pi)
        value1 = (self.c * np.radians(np.abs(PhDelta)))/(2*3.14159*self.SignalFreq*self.d)
        clamped_value1 = max(min(1, value1), -1)     #arcsin argument must be between 1 and -1, or numpy will throw a warning
        theta = np.degrees(np.arcsin(clamped_value1))
        if PhDelta>=0:
            SteerAngle = 0 - theta   # positive PhaseDelta covers 0deg to 90 deg
        else:
            SteerAngle = 0 + theta # negative phase delta covers 0 deg to -90 deg
        return SteerAngle
    
    def work(self, input_items, output_items):
        self.sdr.rx_lo = int(self.LO_freq)
        self.sdr.rx_hardwaregain_chan0 = int(self.Rx_gain)
        self.sdr.rx_hardwaregain_chan1 = int(self.Rx_gain)

        if self.Taper==0:
            if self.SymTaper==0:
                ADAR_set_RxTaper(self.spi, self.ADDR1, self.RxGain1, self.RxGain2, self.RxGain3, self.RxGain4)
                ADAR_set_RxTaper(self.spi, self.ADDR2, self.RxGain5, self.RxGain6, self.RxGain7, self.RxGain8)
            else:
                ADAR_set_RxTaper(self.spi, self.ADDR1, self.RxGain1, self.RxGain2, self.RxGain3, self.RxGain4)
                ADAR_set_RxTaper(self.spi, self.ADDR2, self.RxGain4, self.RxGain3, self.RxGain2, self.RxGain1)
        else:
            ADAR_set_RxTaper(self.spi, self.ADDR1, 127, 127, 127, 127)
            ADAR_set_RxTaper(self.spi, self.ADDR2, 127, 127, 127, 127)

        phase_step_size = 2.8125
        if self.PhaseCal == 0:
            Rx1_Phase_Cal = self.Rx1_cal
            Rx2_Phase_Cal = self.Rx2_cal
            Rx3_Phase_Cal = self.Rx3_cal
            Rx4_Phase_Cal = self.Rx4_cal
            Rx5_Phase_Cal = self.Rx5_cal
            Rx6_Phase_Cal = self.Rx6_cal
            Rx7_Phase_Cal = self.Rx7_cal
            Rx8_Phase_Cal = self.Rx8_cal
        else:
            Rx1_Phase_Cal = 0
            Rx2_Phase_Cal = 0
            Rx3_Phase_Cal = 0
            Rx4_Phase_Cal = 0
            Rx5_Phase_Cal = 0
            Rx6_Phase_Cal = 0
            Rx7_Phase_Cal = 0
            Rx8_Phase_Cal = 0
      
        if self.ScanMaxAngle > 0:
            self.PeakPhaseDelta = self.findMaxAngle(self.ADDR1, self.ADDR2, phase_step_size, Rx1_Phase_Cal, Rx2_Phase_Cal, Rx3_Phase_Cal, Rx4_Phase_Cal, Rx5_Phase_Cal, Rx6_Phase_Cal, Rx7_Phase_Cal, Rx8_Phase_Cal)
            self.ScanMaxAngle = -1
            print('Intial Max Phase Delta = ', self.PeakPhaseDelta)
            ADAR_set_RxPhase(self.spi, self.ADDR1, 1, self.PeakPhaseDelta, phase_step_size, Rx1_Phase_Cal, Rx2_Phase_Cal, Rx3_Phase_Cal, Rx4_Phase_Cal)
            ADAR_set_RxPhase(self.spi, self.ADDR2, 2, self.PeakPhaseDelta, phase_step_size, Rx1_Phase_Cal, Rx2_Phase_Cal, Rx3_Phase_Cal, Rx4_Phase_Cal)
            for n_count in range(-32, 0):
                self.buffer_array[n_count] = (n_count+16)*5   # make a line in the graph when we scan for max angle
        
        track = self.GetTrackingError()
        error = track[0]
        signal_mag = track[1]
        steer_angle = self.ConvertPhaseToSteerAngle(self.PeakPhaseDelta)
        if error < (-1*self.Error_Threshold):
            self.PeakPhaseDelta = self.PeakPhaseDelta + phase_step_size
            ADAR_set_RxPhase(self.spi, self.ADDR1, 1, self.PeakPhaseDelta, phase_step_size, Rx1_Phase_Cal, Rx2_Phase_Cal, Rx3_Phase_Cal, Rx4_Phase_Cal)
            ADAR_set_RxPhase(self.spi, self.ADDR2, 2, self.PeakPhaseDelta, phase_step_size, Rx5_Phase_Cal, Rx6_Phase_Cal, Rx7_Phase_Cal, Rx8_Phase_Cal)
            print_en=1
        elif error > (1*self.Error_Threshold):
            self.PeakPhaseDelta = self.PeakPhaseDelta - phase_step_size
            ADAR_set_RxPhase(self.spi, self.ADDR1, 1, self.PeakPhaseDelta, phase_step_size, Rx1_Phase_Cal, Rx2_Phase_Cal, Rx3_Phase_Cal, Rx4_Phase_Cal)
            ADAR_set_RxPhase(self.spi, self.ADDR2, 2, self.PeakPhaseDelta, phase_step_size, Rx5_Phase_Cal, Rx6_Phase_Cal, Rx7_Phase_Cal, Rx8_Phase_Cal)
        
        self.buffer_array.append(steer_angle)
        self.buffer_array = self.buffer_array[-4000:]  # just use the last elements
        output_items[0] = output_items[0][0:4000]
        for i_count in range(0,4000):
            output_items[0][i_count] = self.buffer_array[i_count]
            output_items[1][i_count] = i_count
        return len(output_items[0])



