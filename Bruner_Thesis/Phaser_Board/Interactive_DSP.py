"""
    This is a CLI tool to test out different combinations of windows, filters, thresholding, etc.
    without having to modify code and rerun everytime. I am still actively working on it, so it
    doesn't include much yet. 
"""

from mimetypes import init
import time
import matplotlib.pyplot as plt
import faulthandler
faulthandler.enable()

# Signal processing stuff
import numpy as np
from numpy.fft import fft, ifft, fftshift, fftfreq
from numpy import absolute, pi
from scipy import signal
from scipy.signal import windows
from target_detection import cfar

# Import settings
# Ignore divide by zero for np.log10(*)
np.seterr(divide='ignore')

import adi


# Instantiate all the Devices
rpi_ip = "ip:phaser.local"  # IP address of the Raspberry Pi
sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block

try:
    x = my_sdr.uri
    print("Pluto already connected")
except NameError:
    print("Pluto not connected...")
    my_sdr = adi.ad9361(uri=sdr_ip)

time.sleep(0.5)

try:
    x = my_phaser.uri
    print("cn0566 already connected")
except NameError:
    print("cn0566 not open...")
    my_phaser = adi.CN0566(uri=rpi_ip, rx_dev=my_sdr)

# Initialize both ADAR1000s, set gains to max, and all phases to 0
my_phaser.configure(device_mode="rx")
for i in range(0, 8):
    my_phaser.set_chan_gain(i, 127)
    my_phaser.set_chan_phase(i, 0)


sample_rate = 0.6e6
center_freq = 2.1e9
signal_freq = 100e3
num_slices = 200
fft_size = 1024*16
# fft_size = 2**10

# Create radio
'''This script is for Pluto Rev C, dual channel setup'''
my_sdr.sample_rate = int(sample_rate)

# Configure Rx
my_sdr.rx_lo = int(center_freq)   # set this to output_freq - (the freq of the HB100)
my_sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
my_sdr.rx_buffer_size = int(fft_size)
my_sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
my_sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
my_sdr.rx_hardwaregain_chan0 = int(30)   # must be between -3 and 70
my_sdr.rx_hardwaregain_chan1 = int(30)   # must be between -3 and 70
# Configure Tx
my_sdr.tx_lo = int(center_freq)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True      # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
my_sdr.tx_hardwaregain_chan0 = -88   # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = -0   # must be between 0 and -88

# Enable TDD logic in pluto (this is for synchronizing Rx Buffer to ADF4159 TX input)
#gpio = adi.one_bit_adc_dac(sdr_ip)
#gpio.gpio_phaser_enable = True

# Configure the ADF4159 Rampling PLL
output_freq = 12.1e9
c = 3e8
wavelength = c / output_freq
BW = 500e6
num_steps = 1000
ramp_time = 1e3 # us
ramp_time_s = ramp_time / 1e6
my_phaser.frequency = int(output_freq/4) # Output frequency divided by 4
my_phaser.freq_dev_range = int(BW/4) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
my_phaser.freq_dev_step = int(BW/num_steps) # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
my_phaser.freq_dev_time = int(ramp_time) # total time (in us) of the complete frequency ramp
my_phaser.delay_word = 4095     # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
my_phaser.delay_clk = 'PFD'     # can be 'PFD' or 'PFD*CLK1'
my_phaser.delay_start_en = 0         # delay start
my_phaser.ramp_delay_en = 0          # delay between ramps.  
my_phaser.trig_delay_en = 0          # triangle delay
my_phaser.ramp_mode = "continuous_triangular"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
my_phaser.sing_ful_tri = 0           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
my_phaser.tx_trig_en = 0             # start a ramp with TXdata
# Enable ADF4159 TX input and generate a single triangular ramp with each trigger
# my_phaser.ramp_mode = "single_ramp_burst"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
# my_phaser.sing_ful_tri = 1           # full triangle enable/disable -- this is used with the single_ramp_burst mode 
# my_phaser.tx_trig_en = 1             # start a ramp with TXdata
my_phaser.enable = 0                 # 0 = PLL enable.  Write this last to update all the registers

"""
    Print config
"""
print("""
CONFIG:
Sample rate: {sample_rate}MHz
Num samples: 2^{Nlog2}
Bandwidth: {BW}MHz
Ramp time: {ramp_time}ms
Output frequency: {output_freq}MHz
""".format(sample_rate=sample_rate / 1e6, Nlog2=int(np.log2(fft_size)), 
    BW=BW / 1e6, ramp_time=ramp_time / 1e3, output_freq=output_freq / 1e6))

# Create a sinewave waveform
fs = int(my_sdr.sample_rate)
print("sample_rate:", fs)
N = int(my_sdr.rx_buffer_size)
fc = int(signal_freq / (fs / N)) * (fs / N)
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 1 * (i + 1j * q)

# Send data
my_sdr._ctx.set_timeout(0)
my_sdr.tx([iq*0, iq])  # only send data to the 2nd channel (that's all we need)

def init_plot(cfar_params=None, axis_limits=[-sample_rate / 2e3, sample_rate / 2e3, 20, 50], 
    masked=False):
    # Collect raw data buffer, take DFT, and do basic processing
    x_n = my_sdr.rx()
    x_n = x_n[0] + x_n[1]

    X_k = absolute(fft(x_n))
    X_k = fftshift(X_k)

    # Create figure
    plt.ion()
    fig, ax = plt.subplots()
    fig.set_figheight(8)
    fig.set_figwidth(16)
    ax.set_title("Received Signal - Frequency Domain", fontsize=24)
    ax.set_xlabel("Frequency [kHz]", fontsize=22)
    ax.set_ylabel("Magnitude [dB]", fontsize=22)
    ax.set_xlim(axis_limits[:2])
    ax.set_ylim(axis_limits[2:])

    # Create frequency axis
    N = len(X_k)
    freq = fftshift(fftfreq(N, 1 / sample_rate))
    freq_kHz = freq / 1e3

    # Create range-FFT scale
    c = 3e8
    slope = BW / ramp_time_s
    dist = (c / slope) * freq

    line1, = ax.plot(freq_kHz, 10 * np.log10(abs(X_k)), c='b', label="Received targets")

    # CFAR plot placeholder
    cfar_placeholder = np.ma.masked_all(X_k.shape)
    line2, = ax.plot(freq_kHz, 10 * np.log10(abs(cfar_placeholder)), c='r', label='CFAR Threshold')

    # Update plot
    fig.canvas.flush_events()
    fig.canvas.draw()

    line1.set_ydata(10 * np.log10(abs(X_k)))
    line2.set_ydata(10 * np.log10(abs(cfar_placeholder)))

    # Update plot
    fig.canvas.flush_events()
    fig.canvas.draw()

    return fig, ax, line1, line2, x_n, axis_limits, cfar_params, freq_kHz, dist

def update_plot(fig, ax, line1, line2, x_n, axis_limits, cfar_params=None, xdata=np.array([]),
    xlabel=None):
    X_k = absolute(fft(x_n))
    X_k = fftshift(X_k)

    ax.set_xlim(axis_limits[:2])
    ax.set_ylim(axis_limits[2:])

    # Create frequency axis
    N = len(X_k)
    freq = fftshift(fftfreq(N, 1 / sample_rate))
    freq_kHz = freq / 1e3

    # Get CFAR values and mask non-targets
    if (cfar_params):
        cfar_values, targets_only = cfar(X_k, cfar_params['num_guard_cells'], 
            cfar_params['num_ref_cells'], cfar_params['bias'], cfar_params['method'])
        if (cfar_params['mask']):
            X_k = targets_only
        line1.set_ydata(10 * np.log10(abs(X_k)))
        line2.set_ydata(10 * np.log10(abs(cfar_values)))
        ax.set_title("Received Signal - Frequency Domain\nCFAR Method: %s" % 
            (cfar_params['method']), fontsize=24)
        ax.legend(fontsize=18, loc='upper left')
    else:
        ax.set_title("Received Signal - Frequency Domain", fontsize=24)
        line1.set_ydata(10 * np.log10(abs(X_k)))
        line2.set_ydata(np.ma.masked_all(X_k.shape))
        try:
            ax.get_legend().remove()
        except:
            pass

    # Toggle xaxis to frequency/distance
    if (xdata.size > 0):
        line1.set_xdata(xdata)

        xdata_max = np.round(np.max(xdata), -1)
        xdata_min = np.round(np.min(xdata), -1)
        step = np.round((xdata_max - xdata_min) / 6, -1)

        xticks = np.concatenate((np.arange(xdata_min, xdata_max, step), [xdata_max]))

        ax.set_xticks(xticks)
        ax.set_xlim([xdata_min, xdata_max])
        ax.set_xlabel(xlabel)

    # Update xticks after changing axes or limits
    # xdata_min, xdata_max = ax.get_xlim()
    # step = np.round((xdata_max - xdata_min) / 6, -1)

    # xticks = np.concatenate((np.arange(xdata_min, xdata_max, step), [xdata_max]))
    # ax.set_xticks(xticks)

    # Update plot
    fig.canvas.flush_events()
    fig.canvas.draw()

    return fig, ax, line1, line2, x_n, axis_limits, cfar_params

def new_buffer(fig, ax, line1, line2, axis_limits):
    print('Collecting new buffer...', end='')
    # Collect raw data buffer, take DFT, and do basic processing
    x_n = my_sdr.rx()
    x_n = x_n[0] + x_n[1]
    print('Done')

    fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, 
        line1, line2, x_n, axis_limits, None)

"""
    update_phases()
    Description: Updates each channel's phase depending on an input scan angle.
    @param my_phaser: Phaser object to be modified.
    @param scan_angle: Desired scan angle
"""
def update_phases(my_phaser, scan_angle):
    num_devs = 2
    num_channels = 4

    # Calculate phase offsets from scan angle
    phases_rad = np.arange(0, num_devs * num_channels, 1, dtype=float)
    phase_diff = (2 * np.pi * output_freq * my_phaser.element_spacing *
        np.sin(scan_angle * np.pi / 180)) / my_phaser.c
    phases_rad *= phase_diff
    phases_deg = np.degrees(phases_rad)
    # print(np.degrees(phases_rad))

    # Update each channel with calculated phases
    for dev_index, (dev_name, dev_obj) in enumerate(my_phaser.devices.items()):
        for channel_index, channel in enumerate(dev_obj.channels):
            # print('Old: %i-%i: %0.4f degrees' % (dev_index, channel_index, channel.rx_phase))
            channel.rx_phase = phases_deg[(dev_index * num_channels) + channel_index]
            # print('New: %i-%i: %0.4f degrees' % (dev_index, channel_index, channel.rx_phase), 
                # end='\n\n')
            dev_obj.latch_rx_settings()



"""
    dsp_cli()
    Description: This is the main function which contains all of the handling for the various
        commands available to the user. It also initializes windows and the figure on first run
        (could be a separate 'init' function). 
"""
def dsp_cli():
    """
        Initialize plot, windows, and other variables for faster runtime.
    """
    print('Initializing plot...', end='')
    fig, ax, line1, line2, x_n, axis_limits, cfar_params, freq, dist = init_plot()
    curr_x_freq = True
    x_n_orig = x_n
    print('Done')

    print('Pre-loading window functions...', end='')
    N = x_n.size
    x_n_windows = {
        'Rectangular'   : x_n,
        'Hanning'       : x_n * windows.hann(N),
        'Hamming'       : x_n * windows.hamming(N),
        'Blackman'      : x_n * windows.blackman(N),
    }
    print('Done')

    cfar_methods = [
        'average',
        'greatest',
        'smallest',
    ]

    # Default directory to store saved figures to
    fig_dir = 'Figures/'

    # Contains available filters
    filter_types = [
        'lowpass',
        'highpass',
    ]

    filters = {
        'Chebyshev1': signal.cheby1,
        'Chebyshev2': signal.cheby2,
    }

    # Contains all horizontal/vertical lines so they can be removed later
    lines = {}

    #############################################

    help = """
Options:
    1.\tSet axis limits
    2.\tChange window function
    3.\tCFAR
    4.\tRemove CFAR threshold
    5.\tFilter
    6.\tRemove filter
    7.\tToggle x-axis (frequency/distance)
    8.\tAdd horizontal/vertical line
    9.\tRemove horizontal/vertical lines
    10.\tBeam-steering
    11.\tScan multiple points between 2 angles
    s.\tSave figure as PNG
    r.\tReload
    q.\tQuit
"""

    while (True):
        print(help)
        cli_input = input('> ')  
        if (cli_input == 'q'):
            """
                Quit.
            """
            break
        elif (cli_input == '1'):
            """
                Sets the x- and y-axis limits.
            """
            print('Current limits:', axis_limits)
            try:
                cli_limits = input('Enter limits \'x0, x1, y0, y1\': ').split(',')
                cli_limits = list(map(lambda x : int(x), cli_limits))
                print('X: {}, Y: {}'.format(cli_limits[:2], cli_limits[2:]))
                fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                    line2, x_n, cli_limits, cfar_params)
            except:
                print('Invalid entry...')
                continue
        elif (cli_input == '2'):
            """
                Changes window function. 
                Default is rectangular/none and other options are available in the x_n_windows dict.
            """
            print('Window options:')
            for index, name in enumerate(x_n_windows.keys()):
                print('\t%i. %s' % (index + 1, name))
            print('\t%i. Go back' % (index + 2))
            try:
                cli_window = int(input('> '))
                x_n_window = x_n_windows[list(x_n_windows.keys())[cli_window - 1]]
                fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                    line2, x_n_window, axis_limits, cfar_params)
            except:
                print('Invalid entry...')
                continue
        elif (cli_input == '3'):
            """
                Adds CFAR target detection threshold. 
                This takes inputs for the parameters for the CFAR algorithm and calculates the 
                threshold. Masking values below the threshold is optional.
            """
            try:
                cli_cfar_params = input('Enter CFAR parameters \'Num guard cells, num ref cells, bias\': ').split(',')
                cli_cfar_params = list(map(lambda x : float(x), cli_cfar_params))
                print('CFAR method options:')
                for index, method in enumerate(cfar_methods):
                    print('\t%i. %s' % (index + 1, method))
                cli_cfar_method = input('> ')
                cli_mask = input('Mask values below threshold? y/n: ')
                cli_mask = True if cli_mask == 'y' else False
                cfar_params = {
                    'num_guard_cells': int(cli_cfar_params[0]),
                    'num_ref_cells': int(cli_cfar_params[1]),
                    'bias': cli_cfar_params[2],
                    'method': cfar_methods[int(cli_cfar_method) - 1],
                    'mask': cli_mask,
                }
                print(cfar_params)
                print(axis_limits)
                fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                    line2, x_n, axis_limits, cfar_params)
            except:
                print('Invalid entry...')
                continue
        elif (cli_input == '4'):
            """
                Remove the CFAR threshold and optional masking.
            """
            # Remove CFAR method
            fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                line2, x_n, axis_limits, None)
        elif (cli_input == '5'):
            """
                Add filtering to signal.
                Current options only allow for low- and high-pass, but these can be used 
                in combination to create the other two main filters: band and stop.
            """
            print('Filter type options:')
            try:
                for index, filter_type in enumerate(filter_types):
                    print('\t%i. %s' % (index + 1, filter_type.title()))
                cli_filter_type = filter_types[int(input('> ')) - 1]
                cli_cutoff = int(input('Enter cutoff frequency [Hz]: '))
                cli_rp = int(input('Enter maximum ripple [dB]: '))
                cli_filter_order = int(input('Enter filter order: '))

                print('Filter options:')
                for index, name in enumerate(filters.keys()):
                    print('\t%i. %s' % (index + 1, name))
                cli_filter = int(input('> ')) - 1
                fs = int(sample_rate)
                print(cli_filter_order, cli_rp, cli_cutoff, cli_filter_type, fs)
                sos = filters[list(filters.keys())[cli_filter]](cli_filter_order, 
                    cli_rp, cli_cutoff, btype=cli_filter_type, fs=fs, output='sos')
                x_n_filtered = signal.sosfilt(sos, x_n)
                fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                    line2, x_n_filtered, axis_limits, cfar_params)
            except:
                print('Invalid entry...')
                continue
        elif (cli_input == '6'):
            """
                Remove filtering.
            """
            fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                line2, x_n_orig, axis_limits, None)
        elif (cli_input == '7'):
            """
                Toggle between frequency and range for the x-axis.
                This is not perfect because it has to create new xticks and the time has
                not put in to make it work very well. It is mostly affected when using a 
                very small range as the xticks have a smallest increment of 10.
            """
            if (curr_x_freq):
                fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, 
                    line1, line2, x_n_orig, axis_limits, None, dist, xlabel='Distance [m]')
                curr_x_freq = False
            else:
                fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, 
                    line1, line2, x_n_orig, axis_limits, None, freq, xlabel='Frequency [kHz]')
                curr_x_freq = True
        elif (cli_input == '8'):
            """
                Add horizontal/vertical lines.
            """
            try:
                line_function = ax.axhline if input('Horizontal/vertical? (h/v): ') == 'h' else ax.axvline
                line_coord = float(input('Input value to insert line: '))
                lines.update({str(line_coord): 
                    line_function(line_coord, c='r', linestyle='dashed')})
            except:
                print('Invalid entry...')
                continue
        elif (cli_input == '9'):
            """
                Remove horizontal/vertical lines.
            """
            for line in lines.copy():
                lines.pop(line).remove()
        elif (cli_input == '10'):
            """
                TODO: INCOMPLETE
                Calculates phase angles from desired scan angle and sets channel phases.
            """
            try:
                cli_scan_angle = float(input('Enter desired scan angle [degrees]: '))
                update_phases(my_phaser, cli_scan_angle)
            except:
                print('Invalid entry...')
                continue

        elif (cli_input == '11'):
            """
                Scan between 2 angles. 
            """
            try:
                cli_scan_extremes = input('Enter min and max scan angles (min, max) [degrees]: ').split(',')
                scan_min = float(cli_scan_extremes[0])
                scan_max = float(cli_scan_extremes[1])
                cli_num_angles = int(input('Enter number of scan angles in the range: '))
                cli_interval_time = float(input('Input time between samples [s]: '))
                scan_angles = np.linspace(scan_min, scan_max, cli_num_angles)
                for scan_angle in scan_angles:
                    print('Setting angle: %0.4f' % scan_angle)
                    update_phases(my_phaser, scan_angle)
                    new_buffer(fig, ax, line1, line2, axis_limits)
                    time.sleep(cli_interval_time)
                
                # Reset to 0 degrees
                update_phases(my_phaser, 0)
                new_buffer(fig, ax, line1, line2, axis_limits)
            except:
                print('Invalid entry...')
                continue

        elif (cli_input == 's'):
            """
                Save current figure to a figure of filename './{fig_dir}/{filename}.png'.
            """
            cli_filename = input('Input filename (no extension): ')
            fig.savefig(fig_dir + cli_filename + '.png')
            print('Figure saved to: %s' % (fig_dir + cli_filename + '.png'))
        elif (cli_input == 'r'):
            """
                Collect and plot new buffer.
            """
            new_buffer(fig, ax, line1, line2, axis_limits)

if __name__ == '__main__':
    cfar_params = {
        'num_guard_cells': 10,
        'num_ref_cells': 30,
        'bias': 2,
        'method': 'average',
    }

    dsp_cli()