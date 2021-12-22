import numpy as np
import matplotlib.pyplot as plt
import time
from project_config import *

try:
    from gain_cal_val import *
    from phase_cal_val import *
except:
    None


def sdr_init(my_pluto, my_pll):
    if sw_tx == 2:
        # move to fdd mode.  see https://github.com/analogdevicesinc/pyadi-iio/blob/ensm-example/examples
        # /ad9361_advanced_ensm.py
        my_pluto._ctrl.debug_attrs["adi,frequency-division-duplex-mode-enable"].value = "1"
        my_pluto._ctrl.debug_attrs[
            "adi,ensm-enable-txnrx-control-enable"].value = "0"  # Disable pin control so spi can move the states
        my_pluto._ctrl.debug_attrs["initialize"].value = "1"
        my_pluto.rx_enabled_channels = [0, 1]  # enable Rx1 (voltage0) and Rx2 (voltage1)
        my_pluto.gain_control_mode_chan1 = 'manual'  # We must be in manual gain control mode (otherwise we won't see
        # the peaks and nulls!)
        my_pluto.rx_hardwaregain_chan1 = 30
        #  my_pluto.dds_single_tone(int(0.0e6), 0.9, 1)
        # my_pluto.dds_single_tone(tone_freq_hz, tone_scale_0to1, tx_channel)

    # Default is 4 Rx buffers are stored, but we want to change and immediately measure the result, so buffers=1
    my_pluto._rxadc.set_kernel_buffers_count(1)
    rx = my_pluto._ctrl.find_channel('voltage0')
    rx.attrs['quadrature_tracking_en'].value = '1'  # set to '1' to enable quadrature tracking
    my_pluto.sample_rate = int(2e6)  # Sampling rate
    my_pluto.rx_buffer_size = int(4 * 256)
    my_pluto.rx_rf_bandwidth = int(10e6)
    # We must be in manual gain control mode (otherwise we won't see the peaks and nulls!)
    my_pluto.gain_control_mode_chan0 = 'manual'
    my_pluto.rx_hardwaregain_chan0 = 30
    my_pluto.rx_lo = int(12e9-SignalFreq)  # 4495000000  # Recieve Freq
    #  my_pluto.dds_single_tone(int(0.0e6), 0.9, 0)
    #  my_pluto.dds_single_tone(tone_freq_hz, tone_scale_0to1, tx_channel)

    if tx_source == "adf4159":
        my_pll.frequency = int(12e9/4)
        my_pll.freq_dev_step = 5690
        my_pll.freq_dev_range = 0
        my_pll.freq_dev_time = 0
        my_pll.enable = 0
        my_pll.ramp_mode = "disabled"
    #         print(my_pll.phase_value)
    #         my_pll.phase_value = 1

    else:
        my_pluto.tx_lo = 6000000000
        my_pluto.tx_cyclic_buffer = True
        my_pluto._tx_buffer_size = int(2 ** 18)
        my_pluto.rx_lo = 4495000000  # Recieve Freq
        # Make sure the Tx channels are attenuated (or off) and their freq is far away from Rx
        my_pluto.tx_hardwaregain_chan0 = -10
        if sw_tx == 2:
            my_pluto.tx_hardwaregain_chan1 = -10


def ADAR_init(beam):
    # Configure ADAR1000
    # beam.initialize_devices()  # Always Intialize the device 1st as reset is performed at Initialization
    # If ADAR1000 array is used initialization work other wise reset each adar individually
    beam.reset()  # Performs a soft reset of the device (writes 0x81 to reg 0x00)
    time.sleep(0.1)
    beam._ctrl.reg_write(0x400, 0x55)  # This trims the LDO value to approx. 1.8V (to the center of its range)

    beam.sequencer_enable = False
    beam.beam_mem_enable = False  # RAM control vs SPI control of the beam state, reg 0x38, bit 6.  False sets bit high and SPI control This code is writing value 40 to reg 38 Next line writes 20 so 60 in all. This is done by single command in SPI writes.
    beam.bias_mem_enable = False  # RAM control vs SPI control of the bias state, reg 0x38, bit 5.  False sets bit high and SPI control
    beam.pol_state = False  # Polarity switch state, reg 0x31, bit 0. True outputs -5V, False outputs 0V
    beam.pol_switch_enable = False  # Enables switch driver for ADTR1107 switch, reg 0x31, bit 3
    beam.tr_source = 'spi'  # TR source for chip, reg 0x31 bit 2.  'external' sets bit high, 'spi' sets bit low
    beam.tr_spi = 'rx'  # TR SPI control, reg 0x31 bit 1.  'tx' sets bit high, 'rx' sets bit low
    beam.tr_switch_enable = True  # Switch driver for external switch, reg0x31, bit 4
    beam.external_tr_polarity = True  # Sets polarity of TR switch compared to TR state of ADAR1000.  True outputs 0V in Rx mode

    beam.rx_vga_enable = True  # Enables Rx VGA, reg 0x2E, bit 0.
    beam.rx_vm_enable = True  # Enables Rx VGA, reg 0x2E, bit 1.
    beam.rx_lna_enable = True  # Enables Rx LNA, reg 0x2E, bit 2. bit3,4,5,6 enables RX for all the channels
    beam._ctrl.reg_write(0x2E, 0x7F)  # bit3,4,5,6 enables RX for all the channels. It is never set if not in this line
    beam.rx_lna_bias_current = 8  # Sets the LNA bias to the middle of its range
    beam.rx_vga_vm_bias_current = 22  # Sets the VGA and vector modulator bias.  I thought this should be 22d, but Stingray has it as 85d????

    beam.tx_vga_enable = True  # Enables Tx VGA, reg 0x2F, bit0
    beam.tx_vm_enable = True  # Enables Tx Vector Modulator, reg 0x2F, bit1
    beam.tx_pa_enable = True  # Enables Tx channel drivers, reg 0x2F, bit2
    beam.tx_pa_bias_current = 6  # Sets Tx driver bias current
    beam.tx_vga_vm_bias_current = 22  # Sets Tx VGA and VM bias.  I thought this should be 22d, but stingray has as 45d??????

    if device_mode == "rx":
        # Configure the device for Rx mode
        beam.mode = "rx"  # Mode of operation, bit 5 of reg 0x31. "rx", "tx", or "disabled".

        SELF_BIASED_LNAs = True
        if SELF_BIASED_LNAs:
            # Allow the external LNAs to self-bias
            beam.lna_bias_out_enable = False  # this writes 0xA0 0x30 0x00.  What does this do? # Disabling it allows LNAs to stay in self bias mode all the time
            # beam._ctrl.reg_write(0x30, 0x00)   #Disables PA and DAC bias
        else:
            # Set the external LNA bias
            beam.lna_bias_on = -0.7  # this writes 0x25 to register 0x2D.  This is correct.  But oddly enough, it doesn't first write 0x18 to reg 0x00....
            # beam._ctrl.reg_write(0x30, 0x20)   #Enables PA and DAC bias.  I think this would be needed too?

        # Enable the Rx path for each channel
        for channel in beam.channels:
            channel.rx_enable = True  # this writes reg0x2E with data 0x00, then reg0x2E with data 0x20.  So it overwrites 0x2E, and enables only one channel
            # beam._ctrl.reg_write(0x2E, 0x7F)    #Enables all four Rx channels, the Rx LNA, Rx Vector Modulator and Rx VGA

    # Configure the device for Tx mode
    else:
        beam.mode = "tx"

        # Enable the Tx path for each channel and set the external PA bias
        for channel in beam.channels:
            channel.tx_enable = True
            channel.pa_bias_on = -2

    if device_mode == "rx":
        beam.latch_rx_settings()  # writes 0x01 to reg 0x28.
    else:
        beam.latch_tx_settings()  # writes 0x02 to reg 0x28.


def ADAR_set_RxTaper(beam_list):
    # Set the gains to max gain (0x7f, or 127)
    for beam in beam_list:
        i = 0  # Gain of Individual channel
        for channel in beam.channels:
            if beam == beam_list[0]:
                channel.rx_gain = cal_gain0[i]
            elif beam == beam_list[1]:
                channel.rx_gain = cal_gain1[i]
            i += 1
        beam.latch_rx_settings()  # writes 0x01 to reg 0x28


def ADAR_set_RxPhase(beam, Ph_Diff, beam_no):
    if beam_no == 1:
        Phase_A = ((np.rint(
            Ph_Diff * 0 / phase_step_size) * phase_step_size) + Rx1_Phase_Cal) % 360  # round each value to the nearest step size increment
        Phase_B = ((np.rint(
            Ph_Diff * 1 / phase_step_size) * phase_step_size) + Rx2_Phase_Cal) % 360  # Cal values defined in project_config file.
        Phase_C = ((np.rint(Ph_Diff * 2 / phase_step_size) * phase_step_size) + Rx3_Phase_Cal) % 360
        Phase_D = ((np.rint(Ph_Diff * 3 / phase_step_size) * phase_step_size) + Rx4_Phase_Cal) % 360

    elif beam_no == 2:
        Phase_A = ((np.rint(Ph_Diff * 4 / phase_step_size) * phase_step_size) + Rx5_Phase_Cal) % 360
        Phase_B = ((np.rint(Ph_Diff * 5 / phase_step_size) * phase_step_size) + Rx6_Phase_Cal) % 360
        Phase_C = ((np.rint(Ph_Diff * 6 / phase_step_size) * phase_step_size) + Rx7_Phase_Cal) % 360
        Phase_D = ((np.rint(Ph_Diff * 7 / phase_step_size) * phase_step_size) + Rx8_Phase_Cal) % 360
    channel_phase_value = [Phase_A, Phase_B, Phase_C, Phase_D]
    #     print(channel_phase_value)

    i = 0
    for channel in beam.channels:
        # Set phase depending on the device mode
        if device_mode == "rx":
            channel.rx_phase = channel_phase_value[
                i]  # writes to I and Q registers values according to Table 13-16 from datasheet.
        i = i + 1
    if device_mode == "rx":
        beam.latch_rx_settings()
    else:
        beam.latch_tx_settings()


def ADAR_Plotter(beam_list, sdr):
    if sw_tx == 2:
        while True:

            PhaseValues = np.arange(-196.875, 196.875,
                                    phase_step_size)  # These are all the phase deltas (i.e. phase difference between Rx1 and Rx2, then Rx2 and Rx3, etc.) we'll sweep.
            PhaseStepNumber = 0  # this is the number of phase steps we'll take (140 in total).  At each phase step, we set the individual phases of each of the Rx channels
            max_signal = -100000  # Reset max_signal.  We'll keep track of the maximum signal we get as we do this 140 loop.
            max_angle = -90  # Reset max_angle.  This is the angle where we saw the max signal.  This is where our compass will point.
            gain = []
            delta = []
            beam_phase = []
            angle = []
            diff_error = []
            for PhDelta in PhaseValues:
                beam_no = 1
                for i in range(0, len(beam_list)):  # Set Phase Values of all the adar1000 connected.
                    ADAR_set_RxPhase(beam_list[i], PhDelta, beam_no)
                    beam_no = 2

                value1 = (c * np.radians(np.abs(PhDelta))) / (2 * 3.14159 * SignalFreq * d)
                clamped_value1 = max(min(1, value1),
                                     -1)  # arcsin argument must be between 1 and -1, or numpy will throw a warning
                theta = np.degrees(np.arcsin(clamped_value1))
                if PhDelta >= 0:
                    SteerAngle = theta  # positive PhaseDelta covers 0deg to 90 deg
                else:
                    SteerAngle = -theta  # negative phase delta covers 0 deg to -90 deg

                total_sum = 0
                total_delta = 0
                total_angle = 0
                for count in range(0, Averages):  # repeat loop and average the results
                    data = sdr.rx()  # read a buffer of data from Pluto using pyadi-iio library (adi.py)
                    '''chan1 = data[
                        0]  # Rx1 data. data is a list of values chan1 and chan2 are just 1st and 2nd element of list Do we have to discard all other values?
                    chan2 = data[1]  # Rx2 data. Changing data[0] to data. delta chan is all 0.'''
                    sum_chan = data
                    delta_chan = data*0
                    N = len(sum_chan)  # number of samples  len(sum_chan) = 1 as just 1st element of list is taken
                    win = np.blackman(N)
                    y_sum = sum_chan * win
                    y_delta = delta_chan * win

                    sp = np.absolute(np.fft.fft(y_sum))
                    sp = sp[1:-1]
                    s_sum = np.fft.fftshift(sp)

                    dp = np.absolute(np.fft.fft(y_delta))
                    dp = dp[1:-1]
                    s_delta = np.fft.fftshift(dp)

                    max_index = np.argmax(s_sum)
                    total_angle = total_angle + (np.angle(s_sum[max_index]) - np.angle(s_delta[max_index]))

                    s_mag_sum = np.abs(s_sum[max_index]) * 2 / np.sum(win)
                    s_mag_delta = np.abs(s_delta[max_index]) * 2 / np.sum(win)
                    s_mag_sum = np.maximum(s_mag_sum, 10 ** (-15))
                    s_mag_delta = np.maximum(s_mag_delta, 10 ** (-15))
                    s_dbfs_sum = 20 * np.log10(
                        s_mag_sum / (2 ** 12))  # make sure the log10 argument isn't zero (hence np.max)
                    s_dbfs_delta = 20 * np.log10(
                        s_mag_delta / (2 ** 12))  # make sure the log10 argument isn't zero (hence np.max)
                    total_sum = total_sum + (s_dbfs_sum)  # sum up all the loops, then we'll average
                    total_delta = total_delta + (s_dbfs_delta)  # sum up all the loops, then we'll average
                PeakValue_sum = total_sum / Averages
                PeakValue_delta = total_delta / Averages
                PeakValue_angle = total_angle / Averages

                if np.sign(PeakValue_angle) == -1:
                    target_error = min(-0.01, (
                            np.sign(PeakValue_angle) * (PeakValue_sum - PeakValue_delta) + np.sign(
                        PeakValue_angle) * (PeakValue_sum + PeakValue_delta) / 2) / (
                                               PeakValue_sum + PeakValue_delta))
                else:
                    target_error = max(0.01, (
                            np.sign(PeakValue_angle) * (PeakValue_sum - PeakValue_delta) + np.sign(
                        PeakValue_angle) * (PeakValue_sum + PeakValue_delta) / 2) / (
                                               PeakValue_sum + PeakValue_delta))

                if PeakValue_sum > max_signal:  # take the largest value, so that we know where to point the compass
                    max_signal = PeakValue_sum
                    max_angle = PeakValue_angle
                    max_PhDelta = PhDelta
                    data_fft = sum_chan
                gain.append(PeakValue_sum)
                delta.append(PeakValue_delta)
                beam_phase.append(PeakValue_angle)
                angle.append(SteerAngle)
                diff_error.append(target_error)

            NumSamples = len(data_fft)  # number of samples
            win = np.blackman(NumSamples)
            y = data_fft * win
            sp = np.absolute(np.fft.fft(y))
            sp = sp[1:-1]
            sp = np.fft.fftshift(sp)
            s_mag = np.abs(sp) * 2 / np.sum(win)  # Scale FFT by window and /2 since we are using half the FFT spectrum
            s_mag = np.maximum(s_mag, 10 ** (-15))
            max_gain = 20 * np.log10(s_mag / (2 ** 12))  # Pluto is a 12 bit ADC, so use that to convert to dBFS
            ts = 1 / float(sdr.sample_rate)
            xf = np.fft.fftfreq(NumSamples, ts)
            xf = np.fft.fftshift(xf[1:-1])  # this is the x axis (freq in Hz) for our fft plot

            ArrayGain = gain
            ArrayDelta = delta
            ArrayBeamPhase = beam_phase
            ArrayAngle = angle
            ArrayError = diff_error
            peak_gain = max(ArrayGain)
            index_peak_gain = np.where(ArrayGain == peak_gain)
            index_peak_gain = index_peak_gain[0]
            max_angle = ArrayAngle[int(index_peak_gain[0])]
            plt.clf()
            #             plt.plot(xf/1e6, max_gain)
            plt.scatter(ArrayAngle,
                        ArrayGain)  # Gain plots sum_chan. Delta plots the difference and Error plots the diff of sum & delta chans
            plt.draw()
            plt.pause(0.05)
            time.sleep(0.05)
            print(angle[gain.index(max(gain))])  # This givens angle frequency source

    elif sw_tx == 1:
        while True:
            # Write code for plotting o/p using single channel
            # We have to step gain over hear. For now it is fixed gain and set in main function
            # Print details such as Tx freq, Rx Freq, bandwidth and Beam Measured/calculated
            # We have to call/set Transreciever in this step. Make it a global variable

            # SteerValues is a sudo line
            # SteerValues = np.arange(-90, 90+steer_res, steer_res)  # convert degrees to radians
            # # Phase delta = 2*Pi*d*sin(theta)/lambda = 2*Pi*d*sin(theta)*f/c
            # PhaseValues = np.degrees(2*3.14159*d*np.sin(np.radians(SteerValues))*SignalFreq/c)

            if res_bits == 1:
                phase_limit = int(225 / phase_step_size) * phase_step_size + phase_step_size
                PhaseValues = np.arange(-phase_limit, phase_limit, phase_step_size)

            gain = []
            angle = []
            max_gain = []
            max_signal = -100000
            max_angle = -90
            beam_no = 1
            for PhDelta in PhaseValues:
                for i in range(0, len(beam_list)):  # change according to number of adar1000 connected
                    ADAR_set_RxPhase(beam_list[i], PhDelta, beam_no)

                value1 = (c * np.radians(np.abs(PhDelta))) / (2 * 3.14159 * SignalFreq * d)
                #  - sdr.tx_rf_bandwidth * 1000
                clamped_value1 = max(min(1, value1),
                                     -1)  # arcsin argument must be between 1 and -1, or numpy will throw a warning
                theta = np.degrees(np.arcsin(clamped_value1))
                if PhDelta >= 0:
                    SteerAngle = -theta  # positive PhaseDelta covers 0deg to 90 deg
                else:
                    SteerAngle = theta  # negative phase delta covers 0 deg to -90 deg

                total = 0
                for count in range(0, Averages):
                    data_raw = sdr.rx()
                    data = data_raw  # Saving raw data to do calculation for peak value
                    NumSamples = len(data)  # number of samples
                    win = np.blackman(NumSamples)
                    y = data * win
                    sp = np.absolute(np.fft.fft(y))
                    sp = sp[1:-1]
                    sp = np.fft.fftshift(sp)
                    s_mag = np.abs(sp) * 2 / np.sum(
                        win)  # Scale FFT by window and /2 since we are using half the FFT spectrum
                    s_mag = np.maximum(s_mag, 10 ** (-15))
                    s_dbfs = 20 * np.log10(s_mag / (2 ** 12))  # Pluto is a 12 bit ADC, so use that to convert to dBFS
                    total = total + max(s_dbfs)  # sum up all the loops, then we'll average
                PeakValue = total / Averages
                if PeakValue > max_signal:  # for the largest value, save the data so we can plot it in the FFT window
                    max_signal = PeakValue
                    max_angle = SteerAngle
                    data = data_raw
                    NumSamples = len(data)  # number of samples
                    win = np.blackman(NumSamples)
                    y = data * win
                    sp = np.absolute(np.fft.fft(y))
                    sp = sp[1:-1]
                    sp = np.fft.fftshift(sp)
                    s_mag = np.abs(sp) * 2 / np.sum(
                        win)  # Scale FFT by window and /2 since we are using half the FFT spectrum
                    s_mag = np.maximum(s_mag, 10 ** (-15))
                    max_gain = 20 * np.log10(
                        s_mag / (2 ** 12))  # Pluto is a 12 bit ADC, so use that to convert to dBFS
                    ts = 1 / float(sdr.sample_rate)
                    xf = np.fft.fftfreq(NumSamples, ts)
                    xf = np.fft.fftshift(xf[1:-1])  # this is the x axis (freq in Hz) for our fft plot
                gain.append(PeakValue)
                angle.append(SteerAngle)
            ArrayGain = gain
            ArrayAngle = angle
            plt.clf()
            # plt.ylim(-60, -10)
            # plt.plot(xf/1e6, max_gain)
            plt.scatter(ArrayAngle, ArrayGain)
            # plt.plot(ArrayAngle, ArrayGain, '-o', ms=5, alpha=0.7, mfc='blue')

            max_gain = max(ArrayGain)
            index_max_gain = np.where(ArrayGain == max_gain)
            index_max_gain = index_max_gain[0]
            max_angle = ArrayAngle[int(index_max_gain[0])]
            # plt.axhline(y=max_gain, color='blue', linestyle="--", alpha=0.3)
            # plt.axvline(x=max_angle, color = 'red', linestyle=":", alpha=0.3)
            plt.pause(0.05)
            time.sleep(0.05)
            print(angle[gain.index(max(gain))])  # This givens angle frequency source


def Phase_calibration(beam_list, sdr):
    for cal_element in range(1, 8):
        if cal_element == 1:
            beam0_cal = [0x7F, 0x7F, 0, 0]
            beam1_cal = [0, 0, 0, 0]

        elif cal_element == 2:
            beam0_cal = [0, 0x7F, 0x7F, 0]
            beam1_cal = [0, 0, 0, 0]

        elif cal_element == 3:
            beam0_cal = [0, 0, 0x7F, 0x7F]
            beam1_cal = [0, 0, 0, 0]

        elif cal_element == 4:
            beam0_cal = [0, 0, 0, 0x7F]
            beam1_cal = [0x7F, 0, 0, 0]

        elif cal_element == 5:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0x7F, 0x7F, 0, 0]

        elif cal_element == 6:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0, 0x7F, 0x7F, 0]

        elif cal_element == 7:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0, 0, 0x7F, 0x7F]

        for beam in beam_list:
            i = 0  # Gain of Individual channel
            for channel in beam.channels:
                if beam == beam_list[0]:
                    channel.rx_gain = beam0_cal[i]
                elif beam == beam_list[1]:
                    channel.rx_gain = beam1_cal[i]
                i += 1
            beam.latch_rx_settings()  # writes 0x01 to reg 0x28

        cal_val = cal_plot(beam_list, sdr, cal_element)
        calibrated_values.append(-1 * cal_val)
    calibrated_values[1] = calibrated_values[1] + calibrated_values[0]
    calibrated_values[2] = calibrated_values[2] + calibrated_values[1]
    calibrated_values[3] = calibrated_values[3] + calibrated_values[2]
    calibrated_values[4] = calibrated_values[4] + calibrated_values[3]
    calibrated_values[5] = calibrated_values[5] + calibrated_values[4]
    calibrated_values[6] = calibrated_values[6] + calibrated_values[5]

    print(calibrated_values)
    try:
        with open("phase_cal_val.py", "x") as f:
            f.write('Rx1_Phase_Cal = 0')
            f.write('\nRx2_Phase_Cal = {}'.format(calibrated_values[0]))
            f.write('\nRx3_Phase_Cal = {}'.format(calibrated_values[1]))
            f.write('\nRx4_Phase_Cal = {}'.format(calibrated_values[2]))
            f.write('\nRx5_Phase_Cal = {}'.format(calibrated_values[3]))
            f.write('\nRx6_Phase_Cal = {}'.format(calibrated_values[4]))
            f.write('\nRx7_Phase_Cal = {}'.format(calibrated_values[5]))
            f.write('\nRx8_Phase_Cal = {}'.format(calibrated_values[6]))

    except:
        with open("phase_cal_val.py", "w") as f:
            f.write('Rx1_Phase_Cal = 0')
            f.write('\nRx2_Phase_Cal = {}'.format(calibrated_values[0]))
            f.write('\nRx3_Phase_Cal = {}'.format(calibrated_values[1]))
            f.write('\nRx4_Phase_Cal = {}'.format(calibrated_values[2]))
            f.write('\nRx5_Phase_Cal = {}'.format(calibrated_values[3]))
            f.write('\nRx6_Phase_Cal = {}'.format(calibrated_values[4]))
            f.write('\nRx7_Phase_Cal = {}'.format(calibrated_values[5]))
            f.write('\nRx8_Phase_Cal = {}'.format(calibrated_values[6]))


def cal_plot(beam_list, sdr, cal_element):
    PhaseValues = np.arange(-196.875, 196.875, phase_step_size)
    max_signal = -100000  # Reset max_signal.  We'll keep track of the maximum signal we get as we do this 140 loop.
    gain = []
    delta = []
    beam_phase = []
    angle = []
    diff_error = []
    for PhDelta in PhaseValues:

        ADAR_set_CalRxPhase(beam_list, PhDelta, cal_element)
        value1 = (c * np.radians(np.abs(PhDelta))) / (2 * 3.14159 * SignalFreq * d)
        clamped_value1 = max(min(1, value1),
                             -1)  # arcsin argument must be between 1 and -1, or numpy will throw a warning
        theta = np.degrees(np.arcsin(clamped_value1))
        if PhDelta >= 0:
            SteerAngle = theta  # positive PhaseDelta covers 0deg to 90 deg
        else:
            SteerAngle = -theta  # negative phase delta covers 0 deg to -90 deg

        total_sum = 0
        total_delta = 0
        total_angle = 0
        for count in range(0, 5):  # repeat loop and average the results
            data = sdr.rx()  # read a buffer of data from Pluto using pyadi-iio library (adi.py)
            '''chan1 = data[0]
            chan2 = data[1]'''
            sum_chan = data
            delta_chan = data*0
            N = len(sum_chan)  # number of samples  len(sum_chan) = 1 as just 1st element of list is taken
            win = np.blackman(N)
            y_sum = sum_chan * win
            y_delta = delta_chan * win

            sp = np.absolute(np.fft.fft(y_sum))
            sp = sp[1:-1]
            s_sum = np.fft.fftshift(sp)

            dp = np.absolute(np.fft.fft(y_delta))
            dp = dp[1:-1]
            s_delta = np.fft.fftshift(dp)

            max_index = np.argmax(s_sum)
            total_angle = total_angle + (np.angle(s_sum[max_index]) - np.angle(s_delta[max_index]))

            s_mag_sum = np.abs(s_sum[max_index]) * 2 / np.sum(win)
            s_mag_delta = np.abs(s_delta[max_index]) * 2 / np.sum(win)
            s_mag_sum = np.maximum(s_mag_sum, 10 ** (-15))
            s_mag_delta = np.maximum(s_mag_delta, 10 ** (-15))
            s_dbfs_sum = 20 * np.log10(
                s_mag_sum / (2 ** 12))  # make sure the log10 argument isn't zero (hence np.max)
            s_dbfs_delta = 20 * np.log10(
                s_mag_delta / (2 ** 12))  # make sure the log10 argument isn't zero (hence np.max)
            total_sum = total_sum + (s_dbfs_sum)  # sum up all the loops, then we'll average
            total_delta = total_delta + (s_dbfs_delta)  # sum up all the loops, then we'll average
        PeakValue_sum = total_sum / 5
        PeakValue_delta = total_delta / 5
        PeakValue_angle = total_angle / 5

        if np.sign(PeakValue_angle) == -1:
            target_error = min(-0.01, (
                    np.sign(PeakValue_angle) * (PeakValue_sum - PeakValue_delta) + np.sign(
                PeakValue_angle) * (PeakValue_sum + PeakValue_delta) / 2) / (
                                       PeakValue_sum + PeakValue_delta))
        else:
            target_error = max(0.01, (
                    np.sign(PeakValue_angle) * (PeakValue_sum - PeakValue_delta) + np.sign(
                PeakValue_angle) * (PeakValue_sum + PeakValue_delta) / 2) / (
                                       PeakValue_sum + PeakValue_delta))

        if PeakValue_sum > max_signal:  # take the largest value, so that we know where to point the compass
            max_signal = PeakValue_sum
            # max_angle = PeakValue_angle
            # max_PhDelta = PhDelta
            # data_fft = sum_chan
        gain.append(PeakValue_sum)
        delta.append(PeakValue_delta)
        beam_phase.append(PeakValue_angle)
        angle.append(SteerAngle)
        diff_error.append(target_error)
    #     ArrayGain = gain
    #     # ArrayDelta = delta
    #     # ArrayBeamPhase = beam_phase
    #     ArrayAngle = angle
    #     # ArrayError = diff_error
    #     # peak_gain = max(ArrayGain)
    #     # index_peak_gain = np.where(ArrayGain == peak_gain)
    #     # index_peak_gain = index_peak_gain[0]
    #     # max_angle = ArrayAngle[int(index_peak_gain[0])]
    #     plt.clf()
    #     # #             plt.plot(xf/1e6, max_gain)
    #     plt.scatter(ArrayAngle,
    #     ArrayGain)  # Gain plots sum_chan. Delta plots the difference and Error plots the diff of sum & delta chans
    #     plt.show()
    return angle[gain.index(min(gain))]  # This givens angle frequency source


def ADAR_set_CalRxPhase(beam_list, Ph_Diff, cal_element):
    Phase_1 = (np.rint(Ph_Diff * 1 / phase_step_size) * phase_step_size) % 360
    Phase_2 = Phase_1 - 180

    if cal_element == 1:
        beam0_ph = [Phase_2, Phase_1, 0, 0]
        beam1_ph = [0, 0, 0, 0]

    elif cal_element == 2:
        beam0_ph = [0, Phase_2, Phase_1, 0]
        beam1_ph = [0, 0, 0, 0]

    elif cal_element == 3:
        beam0_ph = [0, 0, Phase_2, Phase_1]
        beam1_ph = [0, 0, 0, 0]

    elif cal_element == 4:
        beam0_ph = [0, 0, 0, Phase_2]
        beam1_ph = [Phase_1, 0, 0, 0]

    elif cal_element == 5:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [Phase_2, Phase_1, 0, 0]

    elif cal_element == 6:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [0, Phase_1, Phase_2, 0]

    elif cal_element == 7:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [0, 0, Phase_1, Phase_2]

    for beam in beam_list:
        i = 0  # Gain of Individual channel
        for channel in beam.channels:
            if beam == beam_list[0]:
                channel.rx_phase = beam0_ph[i]
            elif beam == beam_list[1]:
                channel.rx_phase = beam1_ph[i]
            i += 1
        beam.latch_rx_settings()


def gain_calibration(beam_list, sdr):
    for gcal_element in range(1, 9):
        if gcal_element == 1:
            beam0_cal = [0x7F, 0, 0, 0]
            beam1_cal = [0, 0, 0, 0]

        elif gcal_element == 2:
            beam0_cal = [0, 0x7F, 0, 0]
            beam1_cal = [0, 0, 0, 0]

        elif gcal_element == 3:
            beam0_cal = [0, 0, 0x7F, 0]
            beam1_cal = [0, 0, 0, 0]

        elif gcal_element == 4:
            beam0_cal = [0, 0, 0, 0x7F]
            beam1_cal = [0, 0, 0, 0]

        elif gcal_element == 5:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0x7F, 0, 0, 0]

        elif gcal_element == 6:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0, 0x7F, 0, 0]

        elif gcal_element == 7:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0, 0, 0x7F, 0]

        elif gcal_element == 8:
            beam0_cal = [0, 0, 0, 0]
            beam1_cal = [0, 0, 0, 0x7F]

        for beam in beam_list:
            i = 0  # Gain of Individual channel
            for channel in beam.channels:
                if beam == beam_list[0]:
                    channel.rx_gain = beam0_cal[i]
                elif beam == beam_list[1]:
                    channel.rx_gain = beam1_cal[i]
                i += 1
            beam.latch_rx_settings()  # writes 0x01 to reg 0x28

        gcal_val = gcal_plot(beam_list, sdr, gcal_element)
        gcalibrated_values.append(gcal_val)

    for k in range(0, 8):
        x = ((gcalibrated_values[k] * 127) / (min(gcalibrated_values)))
        gcal.append(int(x))
    #     print(gcal)
    cal_gain0 = gcal[:4]
    cal_gain1 = gcal[4:]
    #     print(cal_gain0 , cal_gain1)
    try:
        with open("gain_cal_val.py", "x") as f:
            f.write('cal_gain0 = {}'.format(cal_gain0))
            f.write('\ncal_gain1 = {}'.format(cal_gain1))
    except:
        with open("gain_cal_val.py", "w") as f:
            f.write('cal_gain0 = {}'.format(cal_gain0))
            f.write('\ncal_gain1 = {}'.format(cal_gain1))


def gcal_plot(beam_list, sdr, gcal_element):
    PhaseValues = np.arange(-196.875, 196.875, phase_step_size)
    max_signal = -100000  # Reset max_signal.  We'll keep track of the maximum signal we get as we do this 140 loop.
    gain = []
    delta = []
    beam_phase = []
    angle = []
    diff_error = []
    for PhDelta in PhaseValues:

        ADAR_set_gCalRxPhase(beam_list, PhDelta, gcal_element)
        value1 = (c * np.radians(np.abs(PhDelta))) / (2 * 3.14159 * SignalFreq * d)
        clamped_value1 = max(min(1, value1),
                             -1)  # arcsin argument must be between 1 and -1, or numpy will throw a warning
        theta = np.degrees(np.arcsin(clamped_value1))
        if PhDelta >= 0:
            SteerAngle = theta  # positive PhaseDelta covers 0deg to 90 deg
        else:
            SteerAngle = -theta  # negative phase delta covers 0 deg to -90 deg

        total_sum = 0
        total_delta = 0
        total_angle = 0
        for count in range(0, 5):  # repeat loop and average the results
            data = sdr.rx()  # read a buffer of data from Pluto using pyadi-iio library (adi.py)
            '''chan1 = data[0]
            chan2 = data[1]'''
            sum_chan = data
            delta_chan = data*0
            N = len(sum_chan)  # number of samples  len(sum_chan) = 1 as just 1st element of list is taken
            win = np.blackman(N)
            y_sum = sum_chan * win
            y_delta = delta_chan * win

            sp = np.absolute(np.fft.fft(y_sum))
            sp = sp[1:-1]
            s_sum = np.fft.fftshift(sp)

            dp = np.absolute(np.fft.fft(y_delta))
            dp = dp[1:-1]
            s_delta = np.fft.fftshift(dp)

            max_index = np.argmax(s_sum)
            total_angle = total_angle + (np.angle(s_sum[max_index]) - np.angle(s_delta[max_index]))

            s_mag_sum = np.abs(s_sum[max_index]) * 2 / np.sum(win)
            s_mag_delta = np.abs(s_delta[max_index]) * 2 / np.sum(win)
            s_mag_sum = np.maximum(s_mag_sum, 10 ** (-15))
            s_mag_delta = np.maximum(s_mag_delta, 10 ** (-15))
            s_dbfs_sum = 20 * np.log10(
                s_mag_sum / (2 ** 12))  # make sure the log10 argument isn't zero (hence np.max)
            s_dbfs_delta = 20 * np.log10(
                s_mag_delta / (2 ** 12))  # make sure the log10 argument isn't zero (hence np.max)
            total_sum = total_sum + (s_dbfs_sum)  # sum up all the loops, then we'll average
            total_delta = total_delta + (s_dbfs_delta)  # sum up all the loops, then we'll average
        PeakValue_sum = total_sum / 5
        PeakValue_delta = total_delta / 5
        PeakValue_angle = total_angle / 5

        if np.sign(PeakValue_angle) == -1:
            target_error = min(-0.01, (
                    np.sign(PeakValue_angle) * (PeakValue_sum - PeakValue_delta) + np.sign(
                PeakValue_angle) * (PeakValue_sum + PeakValue_delta) / 2) / (
                                       PeakValue_sum + PeakValue_delta))
        else:
            target_error = max(0.01, (
                    np.sign(PeakValue_angle) * (PeakValue_sum - PeakValue_delta) + np.sign(
                PeakValue_angle) * (PeakValue_sum + PeakValue_delta) / 2) / (
                                       PeakValue_sum + PeakValue_delta))

        if PeakValue_sum > max_signal:  # take the largest value, so that we know where to point the compass
            max_signal = PeakValue_sum
            # max_angle = PeakValue_angle
            # max_PhDelta = PhDelta
            # data_fft = sum_chan
        gain.append(PeakValue_sum)
        delta.append(PeakValue_delta)
        beam_phase.append(PeakValue_angle)
        angle.append(SteerAngle)
        diff_error.append(target_error)
    #     ArrayGain = gain
    #     # ArrayDelta = delta
    #     # ArrayBeamPhase = beam_phase
    #     ArrayAngle = angle
    #     # ArrayError = diff_error
    #     # peak_gain = max(ArrayGain)
    #     # index_peak_gain = np.where(ArrayGain == peak_gain)
    #     # index_peak_gain = index_peak_gain[0]
    #     # max_angle = ArrayAngle[int(index_peak_gain[0])]
    #     plt.clf()
    #     # #             plt.plot(xf/1e6, max_gain)
    #     plt.scatter(ArrayAngle,
    #     ArrayGain)  # Gain plots sum_chan. Delta plots the difference and Error plots the diff of sum & delta chans
    #     plt.show()
    return max(gain)  # This givens angle frequency source


def ADAR_set_gCalRxPhase(beam_list, Ph_Diff, gcal_element):
    Phase_1 = (np.rint(Ph_Diff * 1 / phase_step_size) * phase_step_size) % 360
    # Phase_2 = Phase_1 - 180

    if gcal_element == 1:
        beam0_ph = [Phase_1, 0, 0, 0]
        beam1_ph = [0, 0, 0, 0]

    elif gcal_element == 2:
        beam0_ph = [0, Phase_1, 0, 0]
        beam1_ph = [0, 0, 0, 0]

    elif gcal_element == 3:
        beam0_ph = [0, 0, Phase_1, 0]
        beam1_ph = [0, 0, 0, 0]

    elif gcal_element == 4:
        beam0_ph = [0, 0, 0, Phase_1]
        beam1_ph = [0, 0, 0, 0]

    elif gcal_element == 5:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [Phase_1, 0, 0, 0]

    elif gcal_element == 6:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [0, Phase_1, 0, 0]

    elif gcal_element == 7:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [0, 0, Phase_1, 0]

    elif gcal_element == 8:
        beam0_ph = [0, 0, 0, 0]
        beam1_ph = [0, 0, 0, Phase_1]

    for beam in beam_list:
        i = 0  # Gain of Individual channel
        for channel in beam.channels:
            if beam == beam_list[0]:
                channel.rx_phase = beam0_ph[i]
            elif beam == beam_list[1]:
                channel.rx_phase = beam1_ph[i]
            i += 1
        beam.latch_rx_settings()
