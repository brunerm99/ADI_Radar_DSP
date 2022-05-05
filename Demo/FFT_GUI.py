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
from PyQt5.QtGui import QTransform
import pyqtgraph as pg
import numpy as np
from numpy import arange, sin, cos, pi, log10
from numpy.fft import fft, fftshift, fft2, ifft2, ifftshift
from scipy import signal
import time
from matplotlib import cm

import adi
from Phaser_Functions import range_norm as range_norm_func
from target_detection import cfar

# Instantiate all the Devices
try:
	import phaser_config
	rpi_ip = phaser_config.rpi_ip
	sdr_ip = phaser_config.sdr_ip # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block
except:
	print('No config file found...')
	rpi_ip = "ip:phaser.local"	# IP address of the Raspberry Pi
	sdr_ip = "ip:192.168.2.1" # "192.168.2.1, or pluto.local"  # IP address of the Transreceiver Block

try:
	x = my_sdr.uri
	print("Pluto already connected")
except NameError:
	print("Pluto not connected...")
	my_sdr = adi.ad9361(uri=sdr_ip)

time.sleep(0.5)

gpios = adi.one_bit_adc_dac(rpi_ip)
gpios.gpio_vctrl_1 = 1 # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
gpios.gpio_vctrl_2 = 0 # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)
gpios.gpio_burst =0    # High to low causes Pluto to start a new Rx buffer and a burst of TDD pulses
time.sleep(0.5)

my_phaser = adi.CN0566(uri=rpi_ip, rx_dev=my_sdr)
time.sleep(0.5)

# Parameters
sample_rate = 0.6e6
# sample_rate = 10e6
center_freq = 2.1e9
signal_freq = 0.1e6
num_slices = 200
fft_size = 1024*16

# Create radio
'''This script is for Pluto Rev C, dual channel setup'''
my_sdr.sample_rate = int(sample_rate)

# Configure Rx
my_sdr.rx_lo = int(center_freq)   # set this to output_freq - (the freq of the HB100)
my_sdr.rx_enabled_channels = [0, 1]   # enable Rx1 (voltage0) and Rx2 (voltage1)
my_sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
my_sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
my_sdr.rx_hardwaregain_chan0 = int(60)   # must be between -3 and 70
my_sdr.rx_hardwaregain_chan1 = int(60)   # must be between -3 and 70
# Configure Tx
my_sdr.tx_lo = int(center_freq)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True      # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
my_sdr.tx_hardwaregain_chan0 = -88   # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = -3   # must be between 0 and -88

# Read properties
print("RX LO %s" % (my_sdr.rx_lo))

# Enable phaser logic in pluto
gpio = adi.one_bit_adc_dac(sdr_ip)
time.sleep(0.5)
gpio.gpio_phaser_enable = True
time.sleep(0.5)
gpio.gpio_tdd_ext_sync = True   # If set to True, this enables external capture triggering using the L24N GPIO on the Pluto.  When set to false, an internal trigger pulse will be generated every second

# Configure the ADF4159 Rampling PLL
output_freq = 12.1e9
BW = 500e6/4
num_steps = 1000
ramp_time = 1e3 # us
ramp_time_s = ramp_time / 1e6
my_phaser.frequency = int(output_freq/4) # Output frequency divided by 4
my_phaser.freq_dev_range = int(BW) # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
my_phaser.freq_dev_step = int(BW/num_steps) # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
my_phaser.freq_dev_time = int(ramp_time) # total time (in us) of the complete frequency ramp
my_phaser.ramp_mode = "single_sawtooth_burst"     # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
my_phaser.delay_word = 4095     # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
my_phaser.delay_clk = 'PFD'     # can be 'PFD' or 'PFD*CLK1'
my_phaser.delay_start_en = 0         # delay start
my_phaser.ramp_delay_en = 0          # delay between ramps.
my_phaser.trig_delay_en = 0          # triangle delay
my_phaser.sing_ful_tri = 0           # full triangle enable/disable -- this is used with the single_ramp_burst mode
my_phaser.tx_trig_en = 1             # start a ramp with TXdata
#my_phaser.clk1_value = 100
#my_phaser.phase_value = 3
my_phaser.enable = 0                 # 0 = PLL enable.  Write this last to update all the registers

# %%
# Configure TDD controller
tdd = adi.tdd(sdr_ip)
tdd.frame_length_ms = 4    # each GPIO toggle is spaced 4ms apart
tdd.burst_count = 40 # there is a burst of 20 toggles, then off for a long time
tdd.rx_rf_ms = [0.5,0.9, 0, 0]    # each GPIO pulse will be 100us (0.6ms - 0.5ms).  And the first trigger will happen 0.5ms into the buffer
tdd.secondary = False
tdd.en = True

# buffer size needs to be greater than the frame_time
frame_time = tdd.frame_length_ms*tdd.burst_count   # time in ms
print("frame_time:  ", frame_time, "ms")
buffer_time = 0
power=12
while frame_time > buffer_time:
    power=power+1
    buffer_size = int(2**power)
    buffer_time = buffer_size/my_sdr.sample_rate*1000   # buffer time in ms
    if power==23:
	    break     # max pluto buffer size is 2**23, but for tdd burst mode, set to 2**22
print("buffer_size:", buffer_size)
my_sdr.rx_buffer_size = buffer_size
print("buffer_time:", buffer_time, " ms")

# Create a sinewave waveform
fs = int(my_sdr.sample_rate)
print("sample_rate:", fs)
N = buffer_size
fc = int(signal_freq / (fs / N)) * (fs / N)
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 0.9* (i + 1j * q)


# Don't take first burst because it is contaminated with noise
num_bursts = tdd.burst_count - 1

PRI = tdd.frame_length_ms / 1e3
PRF = 1 / PRI
max_doppler_freq = PRF / 2
c = 3e8
wavelength = c / output_freq
max_doppler_vel = max_doppler_freq * wavelength / 2
velocity_axis = np.linspace(-max_doppler_vel, max_doppler_vel, num_bursts)

N_frame = int(PRI / ts)
rx_bursts = np.zeros((num_bursts, N_frame), dtype=complex)

start_offset_time = 0.5e-3
start_offset_index = int((start_offset_time / (frame_time / 1e3)) * N_frame)


my_sdr._ctx.set_timeout(30000)
my_sdr._rx_init_channels()



# Constants
c = 3e8
default_rf_bw = 250e6

# Remove later
# For testing only
# Create signal with some normal noise
N = 1000
# fft_size = 1024
t = np.linspace(0, 1, N)
# fs = 1e6
freq = np.linspace(-fs / 2, fs / 2, int(N_frame))
slope = BW / 4 / ramp_time_s
max_dist = (fs / 2 - signal_freq) * c / slope
# dist = np.linspace(-max_dist, max_dist, N_frame)
dist = (freq - signal_freq) * c / (2 * slope)

# Selector variables
xdata = freq
plot_dist = False
range_norm = False
cfar_toggle = False
clutter_toggle = False

f = 200
omega = 2 * pi * f
x_n = np.sin(omega * t)
noise = 0.3 * np.random.normal(size=(10, N))

class Window(QMainWindow):
	def __init__(self):
		super().__init__()
		self.setWindowTitle("Interactive FFT")
		self.setGeometry(100, 100, 600, 500)
		self.UiComponents()


		# showing all the widgets
		self.show()

	# method for components
	def UiComponents(self):
		widget = QWidget()

		global layout
		layout = QGridLayout()

		# creating a push button object
		btn = QPushButton('Push Button')

		# creating a line edit widget
		text = QLineEdit("Line Edit")

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

		# Range resolution
		# Changes with the RF BW slider
		self.range_res_label = QLabel("B<sub>RF</sub>: %0.2f MHz - R<sub>res</sub>: %0.2f m" %
									  (default_rf_bw / 1e6, c / (2 * default_rf_bw)))
		font = self.range_res_label.font()
		font.setPointSize(15)
		self.range_res_label.setFont(font)
		self.range_res_label.setAlignment(Qt.AlignRight)
		self.range_res_label.setMinimumWidth(300)
		layout.addWidget(self.range_res_label, 5, 1)

		# RF bandwidth slider
		self.bw_slider = QSlider(Qt.Horizontal)
		self.bw_slider.setMinimum(1)
		self.bw_slider.setMaximum(500)
		self.bw_slider.setValue(int(default_rf_bw / 1e6))
		self.bw_slider.setTickInterval(30)
		self.bw_slider.setTickPosition(QSlider.TicksBelow)
		self.bw_slider.valueChanged.connect(self.get_range_res)
		layout.addWidget(self.bw_slider, 5, 0)

		self.set_bw = QPushButton('Set RF Bandwidth')
		self.set_bw.pressed.connect(self.set_range_res)
		layout.addWidget(self.set_bw, 6, 0, 1, 2)

		# CFAR toggle
		self.cfar_checkbox = QCheckBox('Toggle CFAR')
		font = self.cfar_checkbox.font()
		font.setPointSize(15)
		self.cfar_checkbox.setFont(font)
		self.cfar_checkbox.stateChanged.connect(self.toggle_cfar)
		layout.addWidget(self.cfar_checkbox, 3, 0, 1, 2)

		# Clutter suppression toggle
		self.clutter_checkbox = QCheckBox('Toggle Clutter Suppression')
		font = self.clutter_checkbox.font()
		font.setPointSize(15)
		self.clutter_checkbox.setFont(font)
		self.clutter_checkbox.stateChanged.connect(self.toggle_clutter)
		layout.addWidget(self.clutter_checkbox, 4, 0, 1, 2)

		# FFT plot
		self.plot = pg.plot()
		self.plot.setMinimumWidth(1200)
		self.curve = self.plot.plot(freq, pen='y')
		title_style = {'size': '20pt'}
		label_style = {'color': '#FFF', 'font-size': '14pt'}
		self.plot.setLabel('bottom', text='Frequency', units='Hz', **label_style)
		self.plot.setLabel('left', text='Magnitude', units='dB', **label_style)
		self.plot.setTitle('Received Signal - Frequency Spectrum', **title_style)
		layout.addWidget(self.plot, 0, 2, 10, 1)
		self.plot.setYRange(0, 10)
		self.plot.setXRange(signal_freq, fs / 2)

		# Range-Doppler plot
		# There are significantly more fast time indices (N_frame)
		# than slow time (num_bursts)
		self.transform = QTransform()
		self.transform.translate(0, 0)
		y_scale = N_frame / num_bursts
		self.transform.scale(y_scale, 1)

		range_doppler_plot = pg.PlotItem()
		range_doppler_plot.setLabel(axis='left', text='Range', units='m')
		range_doppler_plot.setLabel(axis='bottom', text='Velocity', units='m/s')
		# range_doppler_plot.setXRange(-10, 10)
		range_doppler_plot.invertX(False)
		range_doppler_plot.invertY(True)
		# self.velocity_axis = range_doppler_plot.getAxis('bottom')
		# xticks = [[zip(range(num_bursts), velocity_axis)], [zip(range(num_bursts), velocity_axis)]]
		# self.velocity_axis.setTicks(xticks)
		self.range_doppler_view = pg.ImageView(view=range_doppler_plot)

		colormap = cm.get_cmap('bwr')  # cm.get_cmap("CMRmap")
		colormap._init()
		self.lut = (colormap._lut * 255).view(np.ndarray)
		colors = [
			(0, 0, 0),
			(4, 5, 61),
			(84, 42, 55),
			(15, 87, 60),
			(208, 17, 141),
			(255, 255, 255)
			]

		# Color map
		cmap = pg.ColorMap(pos=np.linspace(0.0, 1.0, 259), color=self.lut)
		self.range_doppler_view.setColorMap(cmap)

		# self.range_doppler_view.setFixedSize(300, 300)
		layout.addWidget(self.range_doppler_view, 6, 0, 4, 2)

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
		""" Updates the slider bar label with RF bandwidth and range resolution

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
		global plot_dist
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

	def toggle_clutter(self, state):
		""" Toggles CFAR thresholding

		Returns:
			None
		"""
		global clutter_toggle
		if (state == QtCore.Qt.Checked):
			print('Clutter suppression on')
			clutter_toggle = True
		else:
			print('CFAR off')
			clutter_toggle = False

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
	global index, xdata, plot_dist, range_norm, rx_bursts
	label_style = {'color': '#FFF', 'font-size': '14pt'}
	# x_n = np.sin(omega * t) + noise[index % 10]
	# X_k = fft(x_n, n=fft_size)

	my_sdr.tx([iq, iq])

	# Quick burst
	gpios.gpio_burst = 0
	gpios.gpio_burst = 1
	gpios.gpio_burst = 0

	data = my_sdr.rx()
	chan1 = data[0]
	chan2 = data[1]

	my_sdr.tx_destroy_buffer()

	# Split into frames starting at offset
	for burst in range(num_bursts):
		rx_bursts[burst] = chan1[start_offset_index + (burst + 1) * N_frame:
		start_offset_index + (burst + 2) * N_frame]

	burst_0 = rx_bursts[-1]
	X_k = fft(burst_0)
	rx_bursts_fft = fft2(rx_bursts)

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

	if (clutter_toggle):
		max_clutter_vel = 0.1
		rx_bursts_fft = fftshift(rx_bursts_fft.copy())
		rx_bursts_fft[np.where((velocity_axis > -max_clutter_vel) &
					  (velocity_axis < max_clutter_vel))] = 1e-50
		rx_bursts = ifft2(ifftshift(rx_bursts_fft))
		X_k = rx_bursts[-1]

	if (plot_dist):
		# print('dist')
		# print(win.plot.getViewBox().state)
		win.curve.setData(dist, fftshift(log10(abs(X_k))))
		# win.plot.setXRange(0, max_dist)
		win.plot.setLabel('bottom', text='Distance', units='m', **label_style)
	else:
		win.curve.setData(freq, fftshift(log10(abs(X_k))))
		# win.plot.setXRange(signal_freq, fs / 2)
		# win.plot.setXRange(-fs / 2, fs / 2)
		win.plot.setLabel('bottom', text='Frequency', units='Hz', **label_style)

	# Range Doppler plot
	print(rx_bursts_fft.shape)
	print(velocity_axis.shape, num_bursts)
	win.range_doppler_view.setImage(log10(fftshift(abs(rx_bursts_fft))),
					transform=win.transform, levels=(2, 8),
					autoLevels=False, autoHistogramRange=False,
					autoRange=False) #, xvals=velocity_axis)

	if index == 1:
		win.plot.enableAutoRange('xy', False)
	index += 1

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(500)

# start the app
sys.exit(App.exec())
