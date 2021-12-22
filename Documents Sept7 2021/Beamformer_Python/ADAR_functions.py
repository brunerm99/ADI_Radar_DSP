# ADAR_functions.py 

import numpy as np

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
    
def ADAR_update_Rx(spi, ADDR):
    spi.xfer2([ADDR, 0x28, 0x01])  # Loads Rx vectors from SPI.  ADDR=0x08 updates all ADAR1000 devices

def ADAR_set_RxTaper(spi, ADDR, RxGain1, RxGain2, RxGain3, RxGain4):
    # set the ADAR1000's VGA gain of each of the Rx channels.  RxGainx needs to be between 0 and 127
    spi.xfer2([ADDR, 0x10, int(128+RxGain1)])  # Sets Rx1 VGA gain
    spi.xfer2([ADDR, 0x11, int(128+RxGain2)])  # Sets Rx2 VGA gain
    spi.xfer2([ADDR, 0x12, int(128+RxGain3)])  # Sets Rx3 VGA gain
    spi.xfer2([ADDR, 0x13, int(128+RxGain4)])  # Sets Rx4 VGA gain

def ADAR_set_RxPhase(spi, address, ADAR_number, PhDelta, phase_step_size, RxPhase1, RxPhase2, RxPhase3, RxPhase4):
    step_size = phase_step_size  #2.8125*16
    Phase_A = ((np.rint(PhDelta*0/step_size)*step_size) + RxPhase1) % 360
    Phase_B = ((np.rint(PhDelta*1/step_size)*step_size) + RxPhase2) % 360
    Phase_C = ((np.rint(PhDelta*2/step_size)*step_size) + RxPhase3) % 360
    Phase_D = ((np.rint(PhDelta*3/step_size)*step_size) + RxPhase4) % 360
    if ADAR_number == 2:
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

def ADAR_write_RxPhase(spi, ADDR, Channel_Phase, I, Q):
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
   