#!/usr/bin/env python3

# Imports
import sys
sys.path.insert(0, '../Phaser_Board')
sys.path.insert(0, '..')
sys.path.insert(0, '/home/marchall/documents/chill/.packages/pyadi-iio')
sys.path.insert(0, '/home/marchall/documents/chill/.packages/libiio')

from pyqtgraph.Qt import QtGui, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5.QtSvg import QSvgWidget
import pyqtgraph as pg
import numpy as np
from numpy import arange, sin, cos, pi, log10
from numpy.fft import fft, fftshift, fft2, ifft2, ifftshift
from scipy import signal
from scipy import interpolate
import time
from matplotlib import cm
from io import BytesIO
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['mathtext.fontset'] = 'cm'

import adi
from Phaser_Functions import range_norm as range_norm_func
from target_detection import cfar


# Instantiate all the Devices
try:
    import phaser_config
    rpi_ip = phaser_config.rpi_ip
    sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block
except:
    print('No config file found...')
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

# Print config
print("""
CONFIG:
Sample rate: {sample_rate}MHz
Num samples: 2^{Nlog2}
Bandwidth: {BW}MHz
Ramp time: {ramp_time}ms
Output frequency: {output_freq}MHz
IF: {signal_freq}kHz
""".format(sample_rate=sample_rate / 1e6, Nlog2=int(np.log2(fft_size)),
	   BW=BW / 1e6, ramp_time=ramp_time / 1e3, output_freq=output_freq / 1e6,

	   signal_freq=signal_freq / 1e3))

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
my_sdr.tx([iq*0.5, iq])  # only send data to the 2nd channel (that's all we need)

c = 3e8
default_rf_bw = 250e6
N_frame = fft_size
freq = np.linspace(-fs / 2, fs / 2, int(N_frame))
slope = BW / 4 / ramp_time_s
max_dist = (fs / 2 - signal_freq) * c / slope
dist = (freq - signal_freq) * c / (2 * slope)

xdata = freq
plot_dist = False
range_norm = False
cfar_toggle = False

f = 200
omega = 2 * pi * f
x_n = np.sin(omega * t)
noise = 0.3 * np.random.normal(size=(10, N))

def tex2svg(formula, fontsize=12, dpi=300):
    """ Render TeX formula to SVG

	Thanks github/gmarull

    Args:
        formula (str): TeX formula.
        fontsize (int, optional): Font size.
        dpi (int, optional): DPI.

    Returns:
        str: SVG render.
    """

    fig = plt.figure(figsize=(0.01, 0.01))
    fig.text(0, 0, r'${}$'.format(formula), fontsize=fontsize)

    output = BytesIO()
    fig.savefig(output, dpi=dpi, transparent=True, format='svg',
                bbox_inches='tight', pad_inches=0.0, frameon=False)
    plt.close(fig)

    output.seek(0)
    return output.read()

class Window(QMainWindow):
	def __init__(self):
		super().__init__()
		self.setWindowTitle("Interactive FFT")
		self.setGeometry(100, 100, 600, 500)

		self.num_rows = 12
		self.UiComponents()

		# showing all the widgets
		self.show()

	# method for components
	def UiComponents(self):
		widget = QWidget()

		global layout
		layout = QGridLayout()

		# Control Panel
		control_label = QLabel('Control Panel')
		font = control_label.font()
		font.setPointSize(20)
		control_label.setFont(font)
		control_label.setAlignment(Qt.AlignHCenter) #| Qt.AlignVCenter)
		layout.addWidget(control_label, 0, 0, 1, 2)

		# Check boxes
		self.x_axis_check = QCheckBox("Range (On) / Frequency (Off) X-Axis")
		font = self.x_axis_check.font()
		font.setPointSize(15)
		self.x_axis_check.setFont(font)

		self.range_norm_check = QCheckBox("Range Normalize?")
		font = self.range_norm_check.font()
		font.setPointSize(15)
		self.range_norm_check.setFont(font)

		self.x_axis_check.stateChanged.connect(self.change_x_axis)
		self.range_norm_check.stateChanged.connect(self.do_range_norm)
		layout.addWidget(self.x_axis_check, 1, 0)
		layout.addWidget(self.range_norm_check, 2, 0)

		# CFAR toggle
		self.cfar_checkbox = QCheckBox('Toggle CFAR')
		font = self.cfar_checkbox.font()
		font.setPointSize(15)
		self.cfar_checkbox.setFont(font)
		self.cfar_checkbox.stateChanged.connect(self.toggle_cfar)
		layout.addWidget(self.cfar_checkbox, 3, 0, 1, 2)

		# Range resolution
		# Changes with the RF BW slider
		self.range_res_label = QLabel("B<sub>RF</sub>: %0.2f MHz - R<sub>res</sub>: %0.2f m" %
									  (default_rf_bw / 1e6, c / (2 * default_rf_bw)))
		font = self.range_res_label.font()
		font.setPointSize(15)
		self.range_res_label.setFont(font)
		self.range_res_label.setAlignment(Qt.AlignRight)
		self.range_res_label.setMinimumWidth(300)
		layout.addWidget(self.range_res_label, 4, 1)

		# RF bandwidth slider
		self.bw_slider = QSlider(Qt.Horizontal)
		self.bw_slider.setMinimum(1)
		self.bw_slider.setMaximum(500)
		self.bw_slider.setValue(int(default_rf_bw / 1e6))
		self.bw_slider.setTickInterval(30)
		self.bw_slider.setTickPosition(QSlider.TicksBelow)
		self.bw_slider.valueChanged.connect(self.get_range_res)
		layout.addWidget(self.bw_slider, 4, 0)

		self.set_bw = QPushButton('Set RF Bandwidth')
		self.set_bw.pressed.connect(self.set_range_res)
		layout.addWidget(self.set_bw, 5, 0, 1, 2)

		# Display formulas for reference
		range_formula = r'R = \frac{f_{beat}c}{2S}'
		range_res_formula = r'R_{res} = \frac{c}{2B_{RF}}'
		slope_formula = r'S = \frac{B_{RF}}{t_{ramp}}'
		self.range_svg = QSvgWidget()
		self.range_svg.load(tex2svg(range_formula, fontsize=6, dpi=100))
		self.slope_svg = QSvgWidget()
		self.slope_svg.load(tex2svg(slope_formula, fontsize=6, dpi=100))
		self.range_res_svg = QSvgWidget()
		self.range_res_svg.load(tex2svg(range_res_formula, fontsize=6, dpi=100))
		# layout.addWidget(self.range_res_svg, self.num_rows - 2, 0, 1, 1)
		# layout.addWidget(self.slope_svg, self.num_rows - 2, 1, 1, 1)
		# layout.addWidget(self.range_svg, self.num_rows - 1, 0, 1, 1)

		# FFT plot
		self.plot = pg.plot()
		self.plot.setMinimumWidth(1200)
		self.curve = self.plot.plot(freq, pen='y', width=6)
		title_style = {'size': '20pt'}
		label_style = {'color': '#FFF', 'font-size': '14pt'}
		self.plot.setLabel('bottom', text='Frequency', units='Hz', **label_style)
		self.plot.setLabel('left', text='Magnitude', units='dB', **label_style)
		self.plot.setTitle('Received Signal - Frequency Spectrum', **title_style)
		layout.addWidget(self.plot, 0, 2, self.num_rows, 1)
		self.plot.setYRange(0, 10)
		self.plot.setXRange(signal_freq, fs / 2)

		widget.setLayout(layout)

		# setting this widget as central widget of the main window
		self.setCentralWidget(widget)

	def get_range_res(self):
		""" Updates the slider bar label with RF bandwidth and range resolution

		Returns:
			None
		"""
		bw = self.bw_slider.value() * 1e6
		range_res = c / (2 * bw)
		self.range_res_label.setText("B<sub>RF</sub>: %0.2f MHz - R<sub>res</sub>: %0.2f m" %
									 (bw / 1e6, c / (2 * bw)))

	def set_range_res(self):
		""" Sets the RF bandwidth

		Returns:
			None
		"""
		global dist, slope
		bw = self.bw_slider.value() * 1e6
		slope = bw / ramp_time_s
		dist = (freq - signal_freq) * c / (2 * slope)
		print('New slope: %0.2fMHz/s' % (slope / 1e6))
		my_phaser.freq_dev_range = int(bw/4) # frequency deviation range in Hz
		my_phaser.enable = 0

	def do_range_norm(self, state):
		""" Toggles range normalization

		Args:
			state (QtCore.Qt.Checked) : State of check box

		Returns:
			None
		"""
		global range_norm
		if (state == QtCore.Qt.Checked):
			print('Range norm on')
			range_norm = True
		else:
			print('Range norm off')
			range_norm = False

	def change_x_axis(self, state):
		""" Toggles between showing frequency and range for the x-axis

		Args:
			state (QtCore.Qt.Checked) : State of check box

		Returns:
			None
		"""
		global plot_dist, slope
		plot_state = win.plot.getViewBox().state
		target_range = plot_state['targetRange']
		range_x = np.array(target_range[0])
		range_y = np.array(target_range[1])
		if (state == QtCore.Qt.Checked):
			print('Range axis')
			plot_dist = True
			print(range_x[0] / signal_freq)
			range_x = (range_x - signal_freq) * c / (2 * slope)
			self.plot.setXRange(*range_x)
		else:
			print('Frequency axis')
			plot_dist = False
			range_x = (range_x * (2 * slope) / c) + signal_freq
			self.plot.setXRange(*range_x)

	def toggle_cfar(self, state):
		""" Toggles CFAR thresholding

		Returns:
			None
		"""
		global cfar_toggle
		if (state == QtCore.Qt.Checked):
			print('CFAR on')
			cfar_toggle = True
		else:
			print('CFAR off')
			cfar_toggle = False

# create pyqt5 app
App = QApplication(sys.argv)

# create the instance of our Window
win = Window()

index = 0
def update():
	""" Updates the FFT in the window

	Returns:
		None
	"""
	global index, xdata, plot_dist, range_norm, freq, dist
	label_style = {'color': '#FFF', 'font-size': '14pt'}

	x_n = my_sdr.rx()
	x_n = x_n[0] + x_n[1]

	X_k = fft(x_n, n=fft_size)

	# A bunch of options defined in the UI components
	bias = 3
	num_guard_cells = 10
	num_ref_cells = 30
	cfar_method = 'greatest'
	if (cfar_toggle):
		threshold, targets = cfar(abs(X_k), num_guard_cells, num_ref_cells, bias, cfar_method)
		X_k = targets

	if (range_norm):
		X_k = range_norm_func(X_k, dist, 1)
		# win.plot.enableAutoRange('xy', True)

	if (plot_dist):
		# print(win.plot.getViewBox().state)
		win.curve.setData(dist, fftshift(log10(abs(X_k))))
		# win.plot.setXRange(0, max_dist)
		win.plot.setLabel('bottom', text='Distance', units='m', **label_style)
	else:
		win.curve.setData(freq, fftshift(log10(abs(X_k))))
		# win.plot.setXRange(signal_freq, fs / 2)
		# win.plot.setXRange(-fs / 2, fs / 2)
		win.plot.setLabel('bottom', text='Frequency', units='Hz', **label_style)

	# win.range_doppler_view.setImage(log10(fftshift(abs(rx_bursts_FFT))).T)
	# win.range_doppler.enableAutoRange('xy', False)

	if index == 1:
		win.plot.enableAutoRange('xy', False)
	index += 1

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)

# start the app
sys.exit(App.exec())
