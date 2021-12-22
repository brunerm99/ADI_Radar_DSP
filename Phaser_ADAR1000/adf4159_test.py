# %%
#  https://wiki.analog.com/resources/tools-software/linux-drivers/iio-pll/adf4159

import adi
import time

pll = adi.adf4159()
output_freq = 12e9
pll.frequency = int(output_freq/4) # Output frequency divided by 4
BW = 500e6
num_steps = 1000
pll.freq_dev_range = int(BW) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
pll.freq_dev_step = int(BW/num_steps) # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
pll.freq_dev_time = int(1e3) # total time (in us) of the complete frequency ramp
pll.ramp_mode = "continuous_sawtooth"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
pll.delay_word = 4095     # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
pll.delay_clk = 'PFD'     # can be 'PFD' or 'PFD*CLK1'
pll.delay_start_en = 0         # delay start
pll.ramp_delay_en = 1          # delay between ramps.  
pll.trig_delay_en = 0          # triangle delay
pll.sing_ful_tri = 0           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
pll.tx_trig_en = 0             # start a ramp with TXdata
#pll.clk1_value = 100
#pll.phase_value = 3
pll.enable = 0                 # 0 = PLL enable.  Write this last to update all the registers


while False:  # probe vtune and check if the values change after each 5 seconds
    pll.ramp_mode = "disabled"
    print("disabled")
    time.sleep(5)
    pll.ramp_mode = "continuous_sawtooth"
    print("cst")
    time.sleep(5)
    pll.ramp_mode = "continuous_triangular"
    print("ctt")
    time.sleep(5)
    pll.ramp_mode = "single_sawtooth_burst"
    print("ssb")
    time.sleep(5)
    pll.ramp_mode = "single_ramp_burst"
    print("srb")
    time.sleep(5)
    print("Done")
