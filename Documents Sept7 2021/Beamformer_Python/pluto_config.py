# standard config files for the beamformer setups
# This is for use with Pluto (and either 1 or 2 ADAR1000 boards)

class config_variables:
    def __init__(self):
        x = 0
config = config_variables()

config.sdr_address = 'ip:192.168.2.1'   # This is the default Pluto address (You can check/change this in the config.txt file on the Pluto "usb drive")
config.SignalFreq = 10.30e9
config.TX_freq    = 2.2e9          # TX LO freq.  This is the freq that Pluto generates (if using the Tx of Pluto as a crude LO)
#config.Mixer_LO   = 10e9          #config.TX_freq  # This is the Mixer LO.  Could be equal to self.TX_freq.  Or if ADF4371 is used for the LO freq, then set this to 9.5 GHz because I haven't built a way to arbitrarily set the 4371 freq....  I should have just used the ADF4371 linux driver.... I'll work on it....
config.Rx_freq    = 2.2e9
config.SampleRate = 40e6
config.Rx_gain = 20
config.Averages = 1
config.RxGain1 = 127
config.RxGain2 = 127
config.RxGain3 = 127
config.RxGain4 = 127
config.RxGain5 = 127
config.RxGain6 = 127
config.RxGain7 = 127
config.RxGain8 = 127
config.Rx1_cal = -2.8125  # you can put phase cal values here (to compensate for phase mismatches in the lines, etc.)
config.Rx2_cal = 0
config.Rx3_cal = -5.625
config.Rx4_cal = -19.6875
config.Rx5_cal = 0
config.Rx6_cal = 0
config.Rx7_cal = 0
config.Rx8_cal = 0
config.refresh_time = 1000 # refresh time in ms.  Auto beam sweep will update at this rate.  Too fast makes it hard to adjust the GUI values when sweeping is active
# The ADAR1000 address is set by the address pins on the ADAR1000.  This is set by P10 on the eval board.
# ADDR 00 (BEAM0, 0x00) is set by leaving all jumpers off of P10
# ADDR 01 (BEAM1, 0x20) is set by jumpering pins 4 and 6 on P10
# ADDR 10 (BEAM2, 0x40) is set by jumpering pins 3 and 5 on P10
# ADDR 11 (BEAM3, 0x60) is set by jumpering both 4+6 and 3+5 on P10
config.ADDR1=0x20         
config.ADDR2=0x40         
config.num_ADARs = 1      # Number of ADAR1000's connected -- this can be either 1 or 2. no other values are allowed
config.num_Rx = 1         # Number of Rx channels (i.e. Pluto this must be 1, but AD9361 SOM this could be 1 or 2)
config.d = 0.0151          # element to element spacing of the antenna
