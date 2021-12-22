# ADAR_functions.py 

import numpy as np

def ADAR_init(beam):
    # Initialize the ADAR1000
    beam.reset()                       #Performs a soft reset of the device (writes 0x81 to reg 0x00)
    beam._ctrl.reg_write(0x400, 0x55)   #This trims the LDO value to approx. 1.8V (to the center of its range)

    beam.sequencer_enable = False
    beam.beam_mem_enable = False        # RAM control vs SPI control of the beam state, reg 0x38, bit 6.  False sets bit high and SPI control
    beam.bias_mem_enable = False        # RAM control vs SPI control of the bias state, reg 0x38, bit 5.  False sets bit high and SPI control
    beam.pol_state = False              #Polarity switch state, reg 0x31, bit 0. True outputs -5V, False outputs 0V
    beam.pol_switch_enable = False      #Enables switch driver for ADTR1107 switch, reg 0x31, bit 3
    beam.tr_source = 'spi'              #TR source for chip, reg 0x31 bit 2.  'external' sets bit high, 'spi' sets bit low
    beam.tr_spi = 'rx'                  #TR SPI control, reg 0x31 bit 1.  'tx' sets bit high, 'rx' sets bit low
    beam.tr_switch_enable = True        #Switch driver for external switch, reg0x31, bit 4
    beam.external_tr_polarity = True    #Sets polarity of TR switch compared to TR state of ADAR1000.  True outputs 0V in Rx mode

    beam.rx_vga_enable = True           #Enables Rx VGA, reg 0x2E, bit 0.  
    beam.rx_vm_enable = True            #Enables Rx VGA, reg 0x2E, bit 1.  
    beam.rx_lna_enable = True           #Enables Rx LNA, reg 0x2E, bit 2.  
    beam.rx_lna_bias_current = 8        #Sets the LNA bias to the middle of its range
    beam.rx_vga_vm_bias_current = 22    #Sets the VGA and vector modulator bias.  I thought this should be 22d, but Stingray has it as 85d????

    beam.tx_vga_enable = True           #Enables Tx VGA, reg 0x2F, bit0
    beam.tx_vm_enable = True            #Enables Tx Vector Modulator, reg 0x2F, bit1
    beam.tx_pa_enable = True            #Enables Tx channel drivers, reg 0x2F, bit2
    beam.tx_pa_bias_current = 6         #Sets Tx driver bias current
    beam.tx_vga_vm_bias_current = 22    #Sets Tx VGA and VM bias.  I thought this should be 22d, but stingray has as 45d??????
    
def ADAR_update_Rx(beam):
    beam.latch_rx_settings()  # Loads Rx vectors from SPI.  Writes 0x01 to reg 0x28.

def ADAR_update_Tx(beam):
    beam.latch_tx_settings()  # Loads Tx vectors from SPI.  Writes 0x02 to reg 0x28.
    
def ADAR_set_mode(beam, mode):
    if mode == "rx":
        # Configure the device for Rx mode
        beam.mode = "rx"   # Mode of operation, bit 5 of reg 0x31. "rx", "tx", or "disabled"
        SELF_BIASED_LNAs = True
        if SELF_BIASED_LNAs:
            beam.lna_bias_out_enable = False    # Allow the external LNAs to self-bias
        else:
            beam.lna_bias_on = -0.7       # Set the external LNA bias
        # Enable the Rx path for each channel
        for channel in beam.channels:
            channel.rx_enable = True  #this writes reg0x2E with data 0x00, then reg0x2E with data 0x20.  So it overwrites 0x2E, and enables only one channel
            #beam0._ctrl.reg_write(0x2E, 0x7F)    #Enables all four Rx channels, the Rx LNA, Rx Vector Modulator and Rx VGA

    # Configure the device for Tx mode
    else:
        beam.mode = "tx"   # Mode of operation, bit 5 of reg 0x31. "rx", "tx", or "disabled"
        # Enable the Tx path for each channel and set the external PA bias
        for channel in beam.channels:
            channel.tx_enable = True
            channel.pa_bias_on = -2
        
        
def ADAR_set_Taper(beam, mode, Gain0, Gain1, Gain2, Gain3):
    if mode == "rx":
        beam.channels[0].rx_gain=Gain0
        beam.channels[1].rx_gain=Gain1
        beam.channels[2].rx_gain=Gain2
        beam.channels[3].rx_gain=Gain3
        array.latch_rx_settings()

    else:
        beam.channels[0].tx_gain=Gain0
        beam.channels[1].tx_gain=Gain1
        beam.channels[2].tx_gain=Gain2
        beam.channels[3].tx_gain=Gain3
        array.latch_tx_settings()

def ADAR_set_Phase(beam, mode, phase, offset):
    for channel in beam.channels:
        if mode == "rx":
            channel.rx_phase = channel.array_element_number * phase + offset
            beam.latch_rx_settings()
        else:
            channel.tx_phase = channel.array_element_number * phase + offset
            beam.latch_tx_settings()

