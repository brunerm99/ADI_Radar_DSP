#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Beamformer_Ex4
# GNU Radio version: 3.8.1.0

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import eng_notation
from gnuradio import qtgui
import sip
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio.qtgui import Range, RangeWidget
import epy_block_0

from gnuradio import qtgui

class top_block(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Beamformer_Ex4")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Beamformer_Ex4")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "top_block")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.variable_qtgui_label_0 = variable_qtgui_label_0 = ' '
        self.samp_rate = samp_rate = 40000000
        self.SymmetricTaper = SymmetricTaper = 0
        self.Setup_ADAR1000 = Setup_ADAR1000 = 0
        self.Rx4_Cal = Rx4_Cal = 0
        self.Rx4Gain = Rx4Gain = 127
        self.Rx3_Cal = Rx3_Cal = 0
        self.Rx3Gain = Rx3Gain = 127
        self.Rx2_Cal = Rx2_Cal = 0
        self.Rx2Gain = Rx2Gain = 127
        self.Rx1_Cal = Rx1_Cal = 0
        self.Rx1Gain = Rx1Gain = 127
        self.IgnorePhaseCals = IgnorePhaseCals = 0
        self.BeamTaper = BeamTaper = 0
        self.ADAR_mode = ADAR_mode = 1
        self.ADAR_address = ADAR_address = 1

        ##################################################
        # Blocks
        ##################################################
        self.ControlTab = Qt.QTabWidget()
        self.ControlTab_widget_0 = Qt.QWidget()
        self.ControlTab_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.ControlTab_widget_0)
        self.ControlTab_grid_layout_0 = Qt.QGridLayout()
        self.ControlTab_layout_0.addLayout(self.ControlTab_grid_layout_0)
        self.ControlTab.addTab(self.ControlTab_widget_0, 'ADAR1000_Setup')
        self.ControlTab_widget_1 = Qt.QWidget()
        self.ControlTab_layout_1 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.ControlTab_widget_1)
        self.ControlTab_grid_layout_1 = Qt.QGridLayout()
        self.ControlTab_layout_1.addLayout(self.ControlTab_grid_layout_1)
        self.ControlTab.addTab(self.ControlTab_widget_1, 'Gain')
        self.ControlTab_widget_2 = Qt.QWidget()
        self.ControlTab_layout_2 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.ControlTab_widget_2)
        self.ControlTab_grid_layout_2 = Qt.QGridLayout()
        self.ControlTab_layout_2.addLayout(self.ControlTab_grid_layout_2)
        self.ControlTab.addTab(self.ControlTab_widget_2, 'Phase')
        self.top_grid_layout.addWidget(self.ControlTab, 0, 0, 6, 2)
        for r in range(0, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        _SymmetricTaper_check_box = Qt.QCheckBox('Set Gain4 = Gain1 (Symmetric Taper)')
        self._SymmetricTaper_choices = {True: 1, False: 0}
        self._SymmetricTaper_choices_inv = dict((v,k) for k,v in self._SymmetricTaper_choices.items())
        self._SymmetricTaper_callback = lambda i: Qt.QMetaObject.invokeMethod(_SymmetricTaper_check_box, "setChecked", Qt.Q_ARG("bool", self._SymmetricTaper_choices_inv[i]))
        self._SymmetricTaper_callback(self.SymmetricTaper)
        _SymmetricTaper_check_box.stateChanged.connect(lambda i: self.set_SymmetricTaper(self._SymmetricTaper_choices[bool(i)]))
        self.ControlTab_grid_layout_1.addWidget(_SymmetricTaper_check_box, 5, 0, 1, 1)
        for r in range(5, 6):
            self.ControlTab_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 1):
            self.ControlTab_grid_layout_1.setColumnStretch(c, 1)
        _Setup_ADAR1000_push_button = Qt.QPushButton('Setup the ADAR1000')
        _Setup_ADAR1000_push_button = Qt.QPushButton('Setup the ADAR1000')
        self._Setup_ADAR1000_choices = {'Pressed': 1, 'Released': 0}
        _Setup_ADAR1000_push_button.pressed.connect(lambda: self.set_Setup_ADAR1000(self._Setup_ADAR1000_choices['Pressed']))
        _Setup_ADAR1000_push_button.released.connect(lambda: self.set_Setup_ADAR1000(self._Setup_ADAR1000_choices['Released']))
        self.ControlTab_grid_layout_0.addWidget(_Setup_ADAR1000_push_button, 2, 0, 1, 2)
        for r in range(2, 3):
            self.ControlTab_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_0.setColumnStretch(c, 1)
        self._Rx4_Cal_range = Range(-180, 180, 2.8125, 0, 10)
        self._Rx4_Cal_win = RangeWidget(self._Rx4_Cal_range, self.set_Rx4_Cal, 'Phase4', "counter_slider", float)
        self.ControlTab_grid_layout_2.addWidget(self._Rx4_Cal_win, 3, 0, 1, 2)
        for r in range(3, 4):
            self.ControlTab_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_2.setColumnStretch(c, 1)
        self._Rx4Gain_range = Range(0, 127, 1, 127, 20)
        self._Rx4Gain_win = RangeWidget(self._Rx4Gain_range, self.set_Rx4Gain, 'Gain4', "counter_slider", int)
        self.ControlTab_grid_layout_1.addWidget(self._Rx4Gain_win, 3, 0, 1, 2)
        for r in range(3, 4):
            self.ControlTab_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_1.setColumnStretch(c, 1)
        self._Rx3_Cal_range = Range(-180, 180, 2.8125, 0, 10)
        self._Rx3_Cal_win = RangeWidget(self._Rx3_Cal_range, self.set_Rx3_Cal, 'Phase3', "counter_slider", float)
        self.ControlTab_grid_layout_2.addWidget(self._Rx3_Cal_win, 2, 0, 1, 2)
        for r in range(2, 3):
            self.ControlTab_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_2.setColumnStretch(c, 1)
        self._Rx3Gain_range = Range(0, 127, 1, 127, 20)
        self._Rx3Gain_win = RangeWidget(self._Rx3Gain_range, self.set_Rx3Gain, 'Gain3', "counter_slider", int)
        self.ControlTab_grid_layout_1.addWidget(self._Rx3Gain_win, 2, 0, 1, 2)
        for r in range(2, 3):
            self.ControlTab_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_1.setColumnStretch(c, 1)
        self._Rx2_Cal_range = Range(-180, 180, 2.8125, 0, 10)
        self._Rx2_Cal_win = RangeWidget(self._Rx2_Cal_range, self.set_Rx2_Cal, 'Phase2', "counter_slider", float)
        self.ControlTab_grid_layout_2.addWidget(self._Rx2_Cal_win, 1, 0, 1, 2)
        for r in range(1, 2):
            self.ControlTab_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_2.setColumnStretch(c, 1)
        self._Rx2Gain_range = Range(0, 127, 1, 127, 20)
        self._Rx2Gain_win = RangeWidget(self._Rx2Gain_range, self.set_Rx2Gain, 'Gain2', "counter_slider", int)
        self.ControlTab_grid_layout_1.addWidget(self._Rx2Gain_win, 1, 0, 1, 2)
        for r in range(1, 2):
            self.ControlTab_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_1.setColumnStretch(c, 1)
        self._Rx1_Cal_range = Range(-180, 180, 2.8125, 0, 10)
        self._Rx1_Cal_win = RangeWidget(self._Rx1_Cal_range, self.set_Rx1_Cal, 'Phase1', "counter_slider", float)
        self.ControlTab_grid_layout_2.addWidget(self._Rx1_Cal_win, 0, 0, 1, 2)
        for r in range(0, 1):
            self.ControlTab_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_2.setColumnStretch(c, 1)
        self._Rx1Gain_range = Range(0, 127, 1, 127, 20)
        self._Rx1Gain_win = RangeWidget(self._Rx1Gain_range, self.set_Rx1Gain, 'Gain1', "counter_slider", int)
        self.ControlTab_grid_layout_1.addWidget(self._Rx1Gain_win, 0, 0, 1, 2)
        for r in range(0, 1):
            self.ControlTab_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_1.setColumnStretch(c, 1)
        _IgnorePhaseCals_check_box = Qt.QCheckBox('Set All Phase Cals to 0 deg')
        self._IgnorePhaseCals_choices = {True: 1, False: 0}
        self._IgnorePhaseCals_choices_inv = dict((v,k) for k,v in self._IgnorePhaseCals_choices.items())
        self._IgnorePhaseCals_callback = lambda i: Qt.QMetaObject.invokeMethod(_IgnorePhaseCals_check_box, "setChecked", Qt.Q_ARG("bool", self._IgnorePhaseCals_choices_inv[i]))
        self._IgnorePhaseCals_callback(self.IgnorePhaseCals)
        _IgnorePhaseCals_check_box.stateChanged.connect(lambda i: self.set_IgnorePhaseCals(self._IgnorePhaseCals_choices[bool(i)]))
        self.ControlTab_grid_layout_2.addWidget(_IgnorePhaseCals_check_box, 4, 0, 1, 2)
        for r in range(4, 5):
            self.ControlTab_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_2.setColumnStretch(c, 1)
        _BeamTaper_check_box = Qt.QCheckBox('Set All Element Gains to Max (127)')
        self._BeamTaper_choices = {True: 1, False: 0}
        self._BeamTaper_choices_inv = dict((v,k) for k,v in self._BeamTaper_choices.items())
        self._BeamTaper_callback = lambda i: Qt.QMetaObject.invokeMethod(_BeamTaper_check_box, "setChecked", Qt.Q_ARG("bool", self._BeamTaper_choices_inv[i]))
        self._BeamTaper_callback(self.BeamTaper)
        _BeamTaper_check_box.stateChanged.connect(lambda i: self.set_BeamTaper(self._BeamTaper_choices[bool(i)]))
        self.ControlTab_grid_layout_1.addWidget(_BeamTaper_check_box, 4, 0, 1, 1)
        for r in range(4, 5):
            self.ControlTab_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 1):
            self.ControlTab_grid_layout_1.setColumnStretch(c, 1)
        # Create the options list
        self._ADAR_mode_options = (0, 1, )
        # Create the labels list
        self._ADAR_mode_labels = ('Transmit', 'Receive', )
        # Create the combo box
        # Create the radio buttons
        self._ADAR_mode_group_box = Qt.QGroupBox('ADAR_mode' + ": ")
        self._ADAR_mode_box = Qt.QHBoxLayout()
        class variable_chooser_button_group(Qt.QButtonGroup):
            def __init__(self, parent=None):
                Qt.QButtonGroup.__init__(self, parent)
            @pyqtSlot(int)
            def updateButtonChecked(self, button_id):
                self.button(button_id).setChecked(True)
        self._ADAR_mode_button_group = variable_chooser_button_group()
        self._ADAR_mode_group_box.setLayout(self._ADAR_mode_box)
        for i, _label in enumerate(self._ADAR_mode_labels):
            radio_button = Qt.QRadioButton(_label)
            self._ADAR_mode_box.addWidget(radio_button)
            self._ADAR_mode_button_group.addButton(radio_button, i)
        self._ADAR_mode_callback = lambda i: Qt.QMetaObject.invokeMethod(self._ADAR_mode_button_group, "updateButtonChecked", Qt.Q_ARG("int", self._ADAR_mode_options.index(i)))
        self._ADAR_mode_callback(self.ADAR_mode)
        self._ADAR_mode_button_group.buttonClicked[int].connect(
            lambda i: self.set_ADAR_mode(self._ADAR_mode_options[i]))
        self.ControlTab_grid_layout_0.addWidget(self._ADAR_mode_group_box, 1, 0, 1, 2)
        for r in range(1, 2):
            self.ControlTab_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_0.setColumnStretch(c, 1)
        # Create the options list
        self._ADAR_address_options = (0, 1, 2, 3, )
        # Create the labels list
        self._ADAR_address_labels = ('0x00', '0x20', '0x40', '0x60', )
        # Create the combo box
        # Create the radio buttons
        self._ADAR_address_group_box = Qt.QGroupBox('ADDR' + ": ")
        self._ADAR_address_box = Qt.QHBoxLayout()
        class variable_chooser_button_group(Qt.QButtonGroup):
            def __init__(self, parent=None):
                Qt.QButtonGroup.__init__(self, parent)
            @pyqtSlot(int)
            def updateButtonChecked(self, button_id):
                self.button(button_id).setChecked(True)
        self._ADAR_address_button_group = variable_chooser_button_group()
        self._ADAR_address_group_box.setLayout(self._ADAR_address_box)
        for i, _label in enumerate(self._ADAR_address_labels):
            radio_button = Qt.QRadioButton(_label)
            self._ADAR_address_box.addWidget(radio_button)
            self._ADAR_address_button_group.addButton(radio_button, i)
        self._ADAR_address_callback = lambda i: Qt.QMetaObject.invokeMethod(self._ADAR_address_button_group, "updateButtonChecked", Qt.Q_ARG("int", self._ADAR_address_options.index(i)))
        self._ADAR_address_callback(self.ADAR_address)
        self._ADAR_address_button_group.buttonClicked[int].connect(
            lambda i: self.set_ADAR_address(self._ADAR_address_options[i]))
        self.ControlTab_grid_layout_0.addWidget(self._ADAR_address_group_box, 0, 0, 1, 2)
        for r in range(0, 1):
            self.ControlTab_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_0.setColumnStretch(c, 1)
        self._variable_qtgui_label_0_tool_bar = Qt.QToolBar(self)

        if None:
            self._variable_qtgui_label_0_formatter = None
        else:
            self._variable_qtgui_label_0_formatter = lambda x: str(x)

        self._variable_qtgui_label_0_tool_bar.addWidget(Qt.QLabel(' ' + ": "))
        self._variable_qtgui_label_0_label = Qt.QLabel(str(self._variable_qtgui_label_0_formatter(self.variable_qtgui_label_0)))
        self._variable_qtgui_label_0_tool_bar.addWidget(self._variable_qtgui_label_0_label)
        self.ControlTab_grid_layout_0.addWidget(self._variable_qtgui_label_0_tool_bar, 3, 0, 6, 2)
        for r in range(3, 9):
            self.ControlTab_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 2):
            self.ControlTab_grid_layout_0.setColumnStretch(c, 1)
        self.qtgui_const_sink_x_0 = qtgui.const_sink_c(
            140, #size
            "Peak Signal vs Steering Angle", #name
            1 #number of inputs
        )
        self.qtgui_const_sink_x_0.set_update_time(0.10)
        self.qtgui_const_sink_x_0.set_y_axis(-50, 0)
        self.qtgui_const_sink_x_0.set_x_axis(-80, 80)
        self.qtgui_const_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_x_0.enable_autoscale(False)
        self.qtgui_const_sink_x_0.enable_grid(True)
        self.qtgui_const_sink_x_0.enable_axis_labels(True)

        self.qtgui_const_sink_x_0.disable_legend()

        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "red", "red", "red",
            "red", "red", "red", "red", "red"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_const_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_const_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_const_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_const_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_const_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_x_0_win = sip.wrapinstance(self.qtgui_const_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_const_sink_x_0_win, 0, 2, 5, 3)
        for r in range(0, 5):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 5):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.epy_block_0 = epy_block_0.blk(addr=int(ADAR_address), reprogram=int(Setup_ADAR1000), rx_en=ADAR_mode, Taper=int(BeamTaper), SymTaper=int(SymmetricTaper), PhaseCal=int(IgnorePhaseCals), SignalFreq=int((10107000-0)*1000000), RxGain1=int(Rx1Gain), RxGain2=int(Rx2Gain), RxGain3=int(Rx3Gain), RxGain4=int(Rx4Gain), Rx1_cal=Rx1_Cal, Rx2_cal=Rx2_Cal, Rx3_cal=Rx3_Cal, Rx4_cal=Rx4_Cal)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_float*1, samp_rate,True)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_float*1)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_throttle_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.epy_block_0, 1), (self.blocks_throttle_0, 0))
        self.connect((self.epy_block_0, 0), (self.qtgui_const_sink_x_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "top_block")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_variable_qtgui_label_0(self):
        return self.variable_qtgui_label_0

    def set_variable_qtgui_label_0(self, variable_qtgui_label_0):
        self.variable_qtgui_label_0 = variable_qtgui_label_0
        Qt.QMetaObject.invokeMethod(self._variable_qtgui_label_0_label, "setText", Qt.Q_ARG("QString", self.variable_qtgui_label_0))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)

    def get_SymmetricTaper(self):
        return self.SymmetricTaper

    def set_SymmetricTaper(self, SymmetricTaper):
        self.SymmetricTaper = SymmetricTaper
        self._SymmetricTaper_callback(self.SymmetricTaper)
        self.epy_block_0.SymTaper = int(self.SymmetricTaper)

    def get_Setup_ADAR1000(self):
        return self.Setup_ADAR1000

    def set_Setup_ADAR1000(self, Setup_ADAR1000):
        self.Setup_ADAR1000 = Setup_ADAR1000
        self.epy_block_0.reprogram = int(self.Setup_ADAR1000)

    def get_Rx4_Cal(self):
        return self.Rx4_Cal

    def set_Rx4_Cal(self, Rx4_Cal):
        self.Rx4_Cal = Rx4_Cal
        self.epy_block_0.Rx4_cal = self.Rx4_Cal

    def get_Rx4Gain(self):
        return self.Rx4Gain

    def set_Rx4Gain(self, Rx4Gain):
        self.Rx4Gain = Rx4Gain
        self.epy_block_0.RxGain4 = int(self.Rx4Gain)

    def get_Rx3_Cal(self):
        return self.Rx3_Cal

    def set_Rx3_Cal(self, Rx3_Cal):
        self.Rx3_Cal = Rx3_Cal
        self.epy_block_0.Rx3_cal = self.Rx3_Cal

    def get_Rx3Gain(self):
        return self.Rx3Gain

    def set_Rx3Gain(self, Rx3Gain):
        self.Rx3Gain = Rx3Gain
        self.epy_block_0.RxGain3 = int(self.Rx3Gain)

    def get_Rx2_Cal(self):
        return self.Rx2_Cal

    def set_Rx2_Cal(self, Rx2_Cal):
        self.Rx2_Cal = Rx2_Cal
        self.epy_block_0.Rx2_cal = self.Rx2_Cal

    def get_Rx2Gain(self):
        return self.Rx2Gain

    def set_Rx2Gain(self, Rx2Gain):
        self.Rx2Gain = Rx2Gain
        self.epy_block_0.RxGain2 = int(self.Rx2Gain)

    def get_Rx1_Cal(self):
        return self.Rx1_Cal

    def set_Rx1_Cal(self, Rx1_Cal):
        self.Rx1_Cal = Rx1_Cal
        self.epy_block_0.Rx1_cal = self.Rx1_Cal

    def get_Rx1Gain(self):
        return self.Rx1Gain

    def set_Rx1Gain(self, Rx1Gain):
        self.Rx1Gain = Rx1Gain
        self.epy_block_0.RxGain1 = int(self.Rx1Gain)

    def get_IgnorePhaseCals(self):
        return self.IgnorePhaseCals

    def set_IgnorePhaseCals(self, IgnorePhaseCals):
        self.IgnorePhaseCals = IgnorePhaseCals
        self._IgnorePhaseCals_callback(self.IgnorePhaseCals)
        self.epy_block_0.PhaseCal = int(self.IgnorePhaseCals)

    def get_BeamTaper(self):
        return self.BeamTaper

    def set_BeamTaper(self, BeamTaper):
        self.BeamTaper = BeamTaper
        self._BeamTaper_callback(self.BeamTaper)
        self.epy_block_0.Taper = int(self.BeamTaper)

    def get_ADAR_mode(self):
        return self.ADAR_mode

    def set_ADAR_mode(self, ADAR_mode):
        self.ADAR_mode = ADAR_mode
        self._ADAR_mode_callback(self.ADAR_mode)
        self.epy_block_0.rx_en = self.ADAR_mode

    def get_ADAR_address(self):
        return self.ADAR_address

    def set_ADAR_address(self, ADAR_address):
        self.ADAR_address = ADAR_address
        self._ADAR_address_callback(self.ADAR_address)
        self.epy_block_0.addr = int(self.ADAR_address)





def main(top_block_cls=top_block, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()

if __name__ == '__main__':
    main()
