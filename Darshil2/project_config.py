#  Set all the configuration parameters in this File
#  Mention numbe of adars used transreceiver source all Constatnst and variables

sw_tx = 1  # Switch to toggle between Pluto and ad9361. Pluto == 1(1r1t) and ad9361 == 2(2r2t)
tx_source = "adf4159"
rpi_ip = "ip:10.84.1.56"
lo_ip = "ip:192.168.2.1"  # IP address of the Transreceiver Block
device_mode = "rx"  # Mode of operation of beamformer. It can be Tx, Rx or Detector

SignalFreq = 10106000000  # Frequency of source
Averages = 1  # Number of Avg too be taken. We be user input value on GUI
phase_step_size = 2.8125  # it is 360/2**number of bits. For now number of bits == 6. change later
steer_res = 2.8125  # It is steering resolution. This would be user selected value
c = 299792458  # speed of light in m/s
d = 0.015  # element to element spacing of the antenna
num_ADARs = 1  # address According to P10 jumper of EVAL-board
res_bits = 1  # res_bits and bits are two different vaiable. It can be variable but it is hardset to 1 for now

rx_gain = [0x7F, 0x7F, 0x7F, 0x7F]  # gain of individual channel
# This migth be useful in creating a calibration routine for gain values
# set gai of one of the channel to max and rest others to 0 plot the graph and check for gain at boardside
# make relative values same for all, That is gain equal to minimum value.
calibrated_values = []
gcalibrated_values = []
gcal = []