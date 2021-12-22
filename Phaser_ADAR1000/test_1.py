import adi
import time
from ADAR_pyadi_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
beam0 = adi.adar1000(
    chip_id="BEAM0",
    array_element_map=[[1, 2, 3, 4]],
    channel_element_map=[2, 1, 4, 3])
ADAR_init(beam0)
ADAR_set_mode(beam0, "rx")
#time.sleep(3)
# beam0._ctrl.reg_write(0x400, 0x55)
#for channel in beam0.channels:
#    channel.rx_enable = True  #this writes reg0x2E with data 0x00, then reg0x2E with data 0x20.  So it overwrites 0x2E, and enables only one channel


