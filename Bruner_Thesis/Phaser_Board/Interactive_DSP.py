"""
Simple FMCW demo with PHASER and ADALM-PLUTO
waterfall plot modified from:  https://amyboyle.ninja/Pyqtgraph-live-spectrogram
"""

from mimetypes import init
import time
from turtle import update
from venv import create
import matplotlib.pyplot as plt
import numpy as np
import faulthandler
faulthandler.enable()

# Signal processing stuff
from scipy.signal import windows
from numpy.fft import fft, ifft, fftshift, fftfreq
from numpy import absolute, pi
from target_detection import cfar

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

    line1, = ax.plot(freq_kHz, 10 * np.log10(X_k), c='b', label="Received targets")

    # CFAR plot placeholder
    cfar_placeholder = np.ma.masked_all(X_k.shape)
    line2, = ax.plot(freq_kHz, 10 * np.log10(cfar_placeholder), c='r', label='CFAR Threshold')

    # Update plot
    fig.canvas.flush_events()
    fig.canvas.draw()

    line1.set_ydata(10 * np.log10(X_k))
    line2.set_ydata(10 * np.log10(cfar_placeholder))

    # Update plot
    fig.canvas.flush_events()
    fig.canvas.draw()

    return fig, ax, line1, line2, x_n, axis_limits, cfar_params

def update_plot(fig, ax, line1, line2, x_n, axis_limits, cfar_params=None):
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
        line1.set_ydata(10 * np.log10(X_k))
        line2.set_ydata(10 * np.log10(cfar_values))
        ax.set_title("Received Signal - Frequency Domain\nCFAR Method: %s" % 
            (cfar_params['method']), fontsize=24)
        ax.legend(fontsize=18, loc='upper left')
    else:
        ax.set_title("Received Signal - Frequency Domain", fontsize=24)
        line1.set_ydata(10 * np.log10(X_k))
        line2.set_ydata(10 * np.log10(np.ma.masked_all(X_k.shape)))
        try:
            ax.get_legend().remove()
        except:
            pass

    # Update plot
    fig.canvas.flush_events()
    fig.canvas.draw()

    return fig, ax, line1, line2, x_n, axis_limits, cfar_params

def dsp_cli():
    """
        Initialize plot, windows, and other variables for faster runtime.
    """
    print('Initializing plot...', end='')
    fig, ax, line1, line2, x_n, axis_limits, cfar_params = init_plot()
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

    #############################################

    help = """
Options:
    1. Set axis limits
    2. Change window function
    3. CFAR
    4. Remove CFAR threshold
    r. Reload
    q. Quit
"""

    while (True):
        print(help)
        cli_input = input('> ')  
        if (cli_input == 'q'):
            break
        elif (cli_input == '1'):
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
            # try:
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
            # except:
            #     print('Invalid entry...')
            #     continue
        elif (cli_input == '4'):
            # Remove CFAR method
            fig, ax, line1, line2, x_n, axis_limits, cfar_params = update_plot(fig, ax, line1, 
                line2, x_n, axis_limits, None)
        elif (cli_input == 'r'):
            plt.close(fig)
            dsp_cli()
            break
            

if __name__ == '__main__':
    cfar_params = {
        'num_guard_cells': 10,
        'num_ref_cells': 30,
        'bias': 2,
        'method': 'average',
    }

    dsp_cli()