#!/usr/bin/env python3

from pyqtgraph.Qt import QtGui, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
import pyqtgraph as pg
import numpy as np
from numpy import arange, sin, cos, pi, log10
from numpy.fft import fft, fftshift
import sys
from scipy import signal

# Constants
c = 3e8
default_rf_bw = 250e6

# Create signal with some normal noise
N = 1000
fft_size = 1024
t = np.linspace(0, 1, N)
sample_rate = 1e6
freq = np.linspace(-sample_rate / 2, sample_rate / 2, fft_size)

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
		self.range_norm_check.stateChanged.connect(self.range_norm)
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
		layout.addWidget(self.range_res_label, 3, 1)

		# RF bandwidth slider
		self.bw_slider = QSlider(Qt.Horizontal)
		self.bw_slider.setMinimum(1)
		self.bw_slider.setMaximum(500)
		self.bw_slider.setValue(int(default_rf_bw / 1e6))
		self.bw_slider.setTickInterval(30)
		self.bw_slider.setTickPosition(QSlider.TicksBelow)
		self.bw_slider.valueChanged.connect(self.get_range_res)
		layout.addWidget(self.bw_slider, 3, 0)

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
		self.plot.setYRange(-2, 3)

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

	def range_norm(self, state):
		""" Toggles range normalization

		Args:
		state (QtCore.Qt.Checked) : State of check box

		Returns:
		None
		"""
		if (state == QtCore.Qt.Checked):
			print('Range norm on')
		else:
			print('Range norm off')

	def change_x_axis(self, state):
		""" Toggles between showing frequency and range for the x-axis

		Args:
		state (QtCore.Qt.Checked) : State of check box

		Returns:
		None
		"""
		if (state == QtCore.Qt.Checked):
			print('Range axis')
		else:
			print('Frequency axis')

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
	global index
	x_n = np.sin(omega * t) + noise[index % 10]
	X_k = fft(x_n, n=fft_size)

	win.curve.setData(freq, fftshift(log10(abs(X_k))))
	if index == 1:
		win.plot.enableAutoRange('xy', False)
	index += 1

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

# start the app
sys.exit(App.exec())
