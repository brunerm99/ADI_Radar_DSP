import adi
import sys
from functions import *
from project_config import *


def main():
    # This Main function would be eventually function of GUI
    # all other things will be called inside the GUI
    # Select and connect the Transreciever
    if "adf4159" in tx_source:
        pll = adi.adf4159()
        if sw_tx == 2:
            sdr = adi.ad9361(uri=lo_ip)
        else:
            sdr = adi.Pluto(uri=lo_ip)
        sdr_init(sdr, pll)

    elif "pluto" in tx_source:
        if sw_tx == 2:
            sdr = adi.ad9361(uri=lo_ip)
            sdr_init(sdr, sdr)
        else:
            sdr = adi.Pluto(uri=lo_ip)
            sdr_init(sdr, sdr)

    else:
        raise Exception("Please select appropriate transmitter source. Valid options 'pluto' or 'adf4159'")

    # channel 4 is the third, and channel 3 is the fourth
    # Select and connect the Beamformer/s. Each Beamformer has 4 channels
    if sw_tx == 2:
        adar = adi.adar1000_array(
            chip_ids=["BEAM0", "BEAM1"],
            device_map=[[1], [2]],
            element_map=[[1, 2, 3, 4], [5, 6, 7, 8]],
            device_element_map={
                1: [2, 1, 4, 3],
                2: [6, 5, 8, 7],
            },
        )
    else:
        adar = adi.adar1000_array(
            chip_ids=["BEAM0"],
            device_map=[[1]],
            element_map=[[1, 2, 3, 4]],
            device_element_map={
                1: [2, 1, 4, 3]
            },
        )
                
    
    beam_list = []  # List of Beamformers in order to steup and configure Individually.
    for device in adar.devices.values():
        beam_list.append(device)
    print(beam_list)
    # initialize the ADAR1000
    for adar in beam_list:
        ADAR_init(adar)  # resets the ADAR1000, then reprograms it to the standard config/ Known state
#         ADAR_set_RxTaper(adar)  # Set gain of each channel of all beamformer according to the Cal Values

    cal = input("Do you want to Calibrate the system?\n")
    if ("yes" or "Yes") in cal:
        mode = input("What type of calibration do you want to do? 'gain' or 'phase' or 'full'\n")
        if "gain" in mode:
            gain_calibration(beam_list, sdr)
        elif "phase" in mode:
            Phase_calibration(beam_list, sdr)
        elif "full" in mode:
            gain_calibration(beam_list, sdr)
            Phase_calibration(beam_list, sdr)
        for adar in beam_list:
            ADAR_init(adar)  # resets the ADAR1000, then reprograms it to the standard config/ Known state
        print("The Calibration is done. Restart the script and continue without calibration.")
        sys.exit()
    ADAR_set_RxTaper(beam_list)  # Set gain of each channel of all beamformer according to the Cal Values
    ADAR_Plotter(beam_list, sdr)  # Rx down-converted signal and plot it to get sinc pattern


main()
