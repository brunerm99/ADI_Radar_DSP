#!/usr/bin/env python
#  Must use Python 3!!

import sys
import os
import time
import numpy as np
import spidev
from threading import Thread
from tkinter import *
import tkinter as tk
from tkinter import ttk
import tkinter.messagebox as mb
import matplotlib.pyplot as plt
import warnings
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.widgets import Cursor
import matplotlib.collections as collections
from scipy import signal

from ADAR_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
from SDR_functions import *    #import the SDR functions (These all start with SDR_xxxx)
from ADF4371_functions import *    #import the ADF4371 functions (These all start with ADF4371_xxxx)
try:
    from adrv9361_config import *    # this has all the key parameters that the user would want to change (i.e. calibration phase and antenna element spacing)
except:
    print("Make sure that the file adrv9361_config_xxxx.py is in the same directory as this python file.")
    sys.exit(0)

# Create SPI object for the ADAR1000s
spi = spidev.SpiDev()
spi.open(0, 0)  # set bus = 0 and chip select (device) = 0
spi.max_speed_hz = 500000
spi.mode = 0

# Create SPI object for the ADF4371
spi_ADF4371 = spidev.SpiDev()
spi_ADF4371.open(0, 1)   # set bus = 0 and chip select (device) = 1
spi_ADF4371.max_speed_hz = 500000
spi_ADF4371.mode = 0

class App:  
    def __init__(self, master):      
        '''SET DEFAULT VALUES'''
        self.sdr_address = config.sdr_address
        self.SignalFreq = config.SignalFreq
        self.TX_freq    = config.TX_freq          # TX LO freq.  This is the freq that Pluto generates (if using the Tx of Pluto as a crude LO)
        self.Mixer_LO   = config.Mixer_LO         # This is the Mixer LO.  Could be equal to self.TX_freq.  Or if ADF4371 is used for the LO freq, then set this to 9.5 GHz because I haven't built a way to arbitrarily set the 4371 freq....  I should have just used the ADF4371 linux driver.... I'll work on it....
        self.LO_freq    = int(self.SignalFreq-self.Mixer_LO)    # RX LO freq of Pluto/ADRV9361
        self.SampleRate = config.SampleRate
        self.Rx_gain = config.Rx_gain
        self.Averages = config.Averages
        self.RxGain1 = config.RxGain1
        self.RxGain2 = config.RxGain2
        self.RxGain3 = config.RxGain3
        self.RxGain4 = config.RxGain4
        self.RxGain5 = config.RxGain5
        self.RxGain6 = config.RxGain6
        self.RxGain7 = config.RxGain7
        self.RxGain8 = config.RxGain8
        self.Rx1_cal = config.Rx1_cal
        self.Rx2_cal = config.Rx2_cal
        self.Rx3_cal = config.Rx3_cal
        self.Rx4_cal = config.Rx4_cal
        self.Rx5_cal = config.Rx5_cal
        self.Rx6_cal = config.Rx6_cal
        self.Rx7_cal = config.Rx7_cal
        self.Rx8_cal = config.Rx8_cal
        self.refresh_time = config.refresh_time
        self.ADDR1=config.ADDR1         
        self.ADDR2=config.ADDR2         
        self.num_ADARs = config.num_ADARs      # Number of ADAR1000's connected -- this can be either 1 or 2. no other values are allowed
        self.num_Rx = config.num_ADARs         # Number of Rx channels (i.e. Pluto this must be 1, but AD9361 SOM this could be 1 or 2)
        self.c = 299792458    # speed of light in m/s
        self.d = config.d
        self.saved_gain = []
        self.saved_angle = []
        self.ArrayGain = []
        self.ArrayAngle = []
        self.ArrayError = []
        self.TrackArray = []
        for i in range(0,1000):
            self.TrackArray.append(0)  # array of zeros
        self.max_hold = -1000
        self.min_hold = 1000
        '''Intialize the SDR, ADAR1000, and ADF4371'''
        self.sdr = SDR_init(self.sdr_address, self.num_Rx, self.SampleRate, self.TX_freq, self.LO_freq, self.Rx_gain, -80)
        ADAR_init(spi, self.ADDR1)
        if self.num_ADARs==2:
            ADAR_init(spi, self.ADDR2)
        ADF4371_init(spi_ADF4371)  # programs ADF4371 to 9.5 GHz
        '''BUILD THE GUI: Master Frame'''
        master.protocol('WM_DELETE_WINDOW', self.closeProgram)    # clicking the "x" button to close the window will shut things down properly (using the closeProgram method)
        self.master = master
        b1 = Button(self.master, text="Aquire Data", command=self.updater)
        b1.grid(row=14, column=2, columnspan=2, sticky=E+W)
        button_exit = Button(self.master, text="Close Program", command=self.closeProgram, bd=4, bg="LightYellow3", relief=RAISED)
        button_exit.grid(row=14, column=6, columnspan=2, padx=50, pady=10, sticky=E+W)
        button_save = Button(self.master, text="Copy Plot to Memory", command=self.savePlot)
        button_save.grid(row=14, column=4, columnspan=1, sticky=E+W)
        button_clear = Button(self.master, text="Clear Memory", command=self.clearPlot)
        button_clear.grid(row=14, column=5, columnspan=1, sticky=E+W)
        self.refresh = tk.IntVar()
        check_refresh = tk.Checkbutton(self.master, text="Auto Refresh Data", highlightthickness=0, variable=self.refresh, command=self.updater, onvalue=1, offvalue=0, anchor=W, relief=FLAT)
        check_refresh.grid(row=14, column=0, columnspan=2, sticky=E+W)
        self.refresh.set(1)
        cntrl_tabs = ttk.Notebook(self.master, height=500, width=400)
        cntrl_tabs.grid(padx=10, pady=10, row=0, column=0, columnspan=4, sticky=N)
        cntrl_width=400
        cntrl_height=500
        frame1 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame2 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame3 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame4 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame5 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        #frame6 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame7 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame1.grid(row=0, column=1)
        frame2.grid(row=0, column=1)
        frame3.grid(row=0, column=1)
        frame4.grid(row=0, column=1)
        frame5.grid(row=0, column=1)
        #frame6.grid(row=0, column=1)
        frame7.grid(row=0, column=1)
        cntrl_tabs.add(frame1, text="Config")
        cntrl_tabs.add(frame2, text="Gain")
        cntrl_tabs.add(frame3, text="Phase")
        cntrl_tabs.add(frame4, text="BW")
        cntrl_tabs.add(frame5, text="Bits")
        #cntrl_tabs.add(frame6, text="Tracking")
        cntrl_tabs.add(frame7, text="Plot Options")
        '''Frame1:  Config'''
        self.freq = tk.DoubleVar()
        self.RxGain = tk.DoubleVar()
        self.Avg = tk.IntVar()
        slide_SignalFreq=Scale(frame1, from_=10, to=10.6, variable=self.freq, resolution=0.001, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_RxGain=Scale(frame1, from_=-5, to=60, resolution=1, variable=self.RxGain, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Average=Scale(frame1, from_=1, to=20, resolution=1, variable=self.Avg, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_SignalFreq.grid(row=0, column=0, padx=10, pady=10, rowspan=3)
        slide_RxGain.grid(row=3, column=0, padx=10, pady=10, rowspan=3)
        slide_Average.grid(row=6, column=0, padx=10, pady=10, rowspan=3)
        slide_SignalFreq.set(self.SignalFreq/1e9)
        slide_RxGain.set(int(self.Rx_gain))
        slide_Average.set(1)
        tk.Label(frame1, text="Signal Freq", relief=SUNKEN, anchor=W).grid(row=1, column=2, sticky=E+W)
        tk.Label(frame1, text="Rx Gain", relief=SUNKEN, anchor=W).grid(row=4, column=2, sticky=E+W)
        tk.Label(frame1, text="Times to Average", relief=SUNKEN, anchor=W).grid(row=7, column=2, sticky=E+W)
        self.update_time = tk.IntVar()
        slide_refresh_time=Scale(frame1, from_=100, to=5000, variable=self.update_time, resolution=1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_refresh_time.grid(row=13, column=0, padx=10, pady=10, rowspan=3)
        tk.Label(frame1, text="Update Time (ms)", relief=SUNKEN, anchor=W).grid(row=14, column=2, pady=20, sticky=E+W)
        self.update_time.set(self.refresh_time)

        #self.Divider = tk.Label(frame1, relief=SUNKEN, anchor=W).grid(row=18, column=0, columnspan=3, pady=10, padx=5, sticky=E+W)
        self.RxPhaseDelta = tk.DoubleVar()
        slide_RxPhaseDelta=Scale(frame1, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.RxPhaseDelta, command=self.RxPhaseUpdate, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_RxPhaseDelta.grid(row=20, column=0, padx=10, pady=10, rowspan=3)
        slide_RxPhaseDelta.set(0)
        
        def zero_PhaseDelta():
            slide_RxPhaseDelta.set(0)
        static_phase_label = tk.Button(frame1, text="Static Phase Delta", relief=RAISED, anchor=W, command=zero_PhaseDelta, highlightthickness=0)
        static_phase_label.grid(row=21, column=2, sticky=E+W)
        
        def mode_select(value):
            if value == 'Static Phase':
                slide_RxPhaseDelta.grid()
                static_phase_label.grid()
            else:
                slide_RxPhaseDelta.grid_remove()
                static_phase_label.grid_remove()
            if value == 'Tracking':
                plot_tabs.select(3)
                self.find_peak()
            else:
                self.updater()

        self.mode_var = StringVar()
        self.mode_var.set('Beam Sweep')
        slide_RxPhaseDelta.grid_remove()
        static_phase_label.grid_remove()
        mode_Menu = OptionMenu(frame1, self.mode_var, 'Beam Sweep', 'Static Phase', 'Tracking', command=mode_select)
        mode_Menu.grid(row=16, column=0, padx=10, pady=10, rowspan=1, sticky=E+W)
        tk.Label(frame1, text="Mode Selection", relief=SUNKEN, anchor=W).grid(row=16, column=2, pady=20, sticky=E+W)

        '''Frame2:  Gain'''
        self.Rx1Gain_set = tk.IntVar()
        self.Rx2Gain_set = tk.IntVar()
        self.Rx3Gain_set = tk.IntVar()
        self.Rx4Gain_set = tk.IntVar()
        self.Rx5Gain_set = tk.IntVar()
        self.Rx6Gain_set = tk.IntVar()
        self.Rx7Gain_set = tk.IntVar()
        self.Rx8Gain_set = tk.IntVar()
        self.Sym_set = tk.IntVar()
        self.Sym_set = 0
        def sym_Rx1(val):
            if (self.Sym_set.get()==1 and self.num_ADARs==1):
                slide_Rx4Gain.configure(state='normal') 
                slide_Rx4Gain.set(val)
                slide_Rx4Gain.configure(state='disabled')
            if (self.Sym_set.get()==1 and self.num_ADARs==2):
                slide_Rx8Gain.configure(state='normal') 
                slide_Rx8Gain.set(val)
                slide_Rx8Gain.configure(state='disabled')
        def sym_Rx2(val):
            if (self.Sym_set.get()==1 and self.num_ADARs==1):
                slide_Rx3Gain.configure(state='normal') 
                slide_Rx3Gain.set(val)
                slide_Rx3Gain.configure(state='disabled')
            if (self.Sym_set.get()==1 and self.num_ADARs==2):
                slide_Rx7Gain.configure(state='normal') 
                slide_Rx7Gain.set(val)
                slide_Rx7Gain.configure(state='disabled')
        def sym_Rx3(val):
            if (self.Sym_set.get()==1 and self.num_ADARs==2):
                slide_Rx6Gain.configure(state='normal') 
                slide_Rx6Gain.set(val)
                slide_Rx6Gain.configure(state='disabled')
        def sym_Rx4(val):
            if (self.Sym_set.get()==1 and self.num_ADARs==2):
                slide_Rx5Gain.configure(state='normal') 
                slide_Rx5Gain.set(val)
                slide_Rx5Gain.configure(state='disabled')
        slide_Rx1Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx1Gain_set, command=sym_Rx1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx2Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx2Gain_set, command=sym_Rx2, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx3Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx3Gain_set, command=sym_Rx3, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx4Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx4Gain_set, command=sym_Rx4, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx1Gain.grid(row=0, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
        slide_Rx2Gain.grid(row=3, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
        slide_Rx3Gain.grid(row=6, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
        slide_Rx4Gain.grid(row=9, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
        slide_Rx1Gain.set(127)
        slide_Rx2Gain.set(127)
        slide_Rx3Gain.set(127)
        slide_Rx4Gain.set(127)
        def Rx1_toggle():
            if slide_Rx1Gain.get()==0:
                slide_Rx1Gain.set(127)
            else:
                slide_Rx1Gain.set(0)
        def Rx2_toggle():
            if slide_Rx2Gain.get()==0:
                slide_Rx2Gain.set(127)
            else:
                slide_Rx2Gain.set(0)
        def Rx3_toggle():
            if slide_Rx3Gain.get()==0:
                slide_Rx3Gain.set(127)
            else:
                slide_Rx3Gain.set(0)
        def Rx4_toggle():
            if slide_Rx4Gain.get()==0:
                slide_Rx4Gain.set(127)
            else:
                slide_Rx4Gain.set(0)
        def Rx5_toggle():
            if slide_Rx5Gain.get()==0:
                slide_Rx5Gain.set(127)
            else:
                slide_Rx5Gain.set(0)
        def Rx6_toggle():
            if slide_Rx6Gain.get()==0:
                slide_Rx6Gain.set(127)
            else:
                slide_Rx6Gain.set(0)
        def Rx7_toggle():
            if slide_Rx7Gain.get()==0:
                slide_Rx7Gain.set(127)
            else:
                slide_Rx7Gain.set(0)
        def Rx8_toggle():
            if slide_Rx8Gain.get()==0:
                slide_Rx8Gain.set(127)
            else:
                slide_Rx8Gain.set(0)
        tk.Button(frame2, text="Rx1 Gain", relief=RAISED, anchor=W, command=Rx1_toggle, highlightthickness=0).grid(row=1, column=3, sticky=E+W)
        tk.Button(frame2, text="Rx2 Gain", relief=RAISED, anchor=W, command=Rx2_toggle, highlightthickness=0).grid(row=4, column=3, sticky=E+W)
        tk.Button(frame2, text="Rx3 Gain", relief=RAISED, anchor=W, command=Rx3_toggle, highlightthickness=0).grid(row=7, column=3, sticky=E+W)
        tk.Button(frame2, text="Rx4 Gain", relief=RAISED, anchor=W, command=Rx4_toggle, highlightthickness=0).grid(row=10, column=3, sticky=E+W)
        def sym_sel():
            if self.num_ADARs==1:
                if self.Sym_set.get()==1:
                    slide_Rx4Gain.configure(state='normal')  # 'normal'
                    slide_Rx4Gain.set(self.Rx1Gain_set.get())
                    slide_Rx4Gain.configure(state='disabled')  # 'normal'
                if self.Sym_set.get()==0:
                    slide_Rx4Gain.configure(state='normal')  # 'disabled'
            if self.num_ADARs==2:
                if self.Sym_set.get()==1:
                    slide_Rx5Gain.configure(state='normal')  # 'normal'
                    slide_Rx6Gain.configure(state='normal')  # 'normal'
                    slide_Rx7Gain.configure(state='normal')  # 'normal'
                    slide_Rx8Gain.configure(state='normal')  # 'normal'
                    slide_Rx5Gain.set(self.Rx4Gain_set.get())
                    slide_Rx6Gain.set(self.Rx3Gain_set.get())
                    slide_Rx7Gain.set(self.Rx2Gain_set.get())
                    slide_Rx8Gain.set(self.Rx1Gain_set.get())
                    slide_Rx5Gain.configure(state='disabled')  # 'normal'
                    slide_Rx6Gain.configure(state='disabled')  # 'normal'
                    slide_Rx7Gain.configure(state='disabled')  # 'normal'
                    slide_Rx8Gain.configure(state='disabled')  # 'normal'
                if self.Sym_set.get()==0:
                    slide_Rx5Gain.configure(state='normal')  # 'disabled'
                    slide_Rx6Gain.configure(state='normal')  # 'disabled'
                    slide_Rx7Gain.configure(state='normal')  # 'disabled'
                    slide_Rx8Gain.configure(state='normal')  # 'disabled'
        self.Sym_set = tk.IntVar()
        check_Sym = tk.Checkbutton(frame2, text="Symmetric Taper", highlightthickness=0, variable=self.Sym_set, onvalue=1, offvalue=0, command=sym_sel, relief=SUNKEN, anchor=W)
        check_Sym.grid(row=24, column=0, columnspan=2, padx=5, pady=5, sticky=E+W)
        def taper_profile(taper_var):
            if self.num_ADARs == 1:
                if taper_var == 1:    # Rect Window
                    gain1 = 127 
                elif taper_var == 2:  # Chebyshev
                    gain1 = 64
                elif taper_var == 3:  # Hamming
                    gain1 = 55
                elif taper_var == 4:  # Hann
                    gain1 = 48
                elif taper_var == 5:  # Blackman
                    gain1 = 30
                slide_Rx1Gain.configure(state='normal')  # 'disabled'
                slide_Rx2Gain.configure(state='normal')  # 'disabled'
                slide_Rx3Gain.configure(state='normal')  # 'disabled'
                slide_Rx4Gain.configure(state='normal')  # 'disabled'
                slide_Rx1Gain.set(gain1)
                slide_Rx2Gain.set(127)
                slide_Rx3Gain.set(127)
                slide_Rx4Gain.set(gain1)
            if self.num_ADARs == 2:
                if taper_var == 1:    # Rect Window
                    gain1 = 127 
                    gain2 = 127 
                    gain3 = 127 
                    gain4 = 127 
                elif taper_var == 2:  # Chebyshev
                    gain1 = 5 
                    gain2 = 29 
                    gain3 = 79 
                    gain4 = 127 
                elif taper_var == 3:  # Hamming
                    gain1 = 11 
                    gain2 = 34 
                    gain3 = 85 
                    gain4 = 127 
                elif taper_var == 4:  # Hann
                    gain1 = 15 
                    gain2 = 54 
                    gain3 = 98 
                    gain4 = 127 
                elif taper_var == 5:  # Blackman
                    gain1 = 7 
                    gain2 = 34 
                    gain3 = 84 
                    gain4 = 127 
                slide_Rx1Gain.configure(state='normal')  # 'disabled'
                slide_Rx2Gain.configure(state='normal')  # 'disabled'
                slide_Rx3Gain.configure(state='normal')  # 'disabled'
                slide_Rx4Gain.configure(state='normal')  # 'disabled'
                slide_Rx5Gain.configure(state='normal')  # 'disabled'
                slide_Rx6Gain.configure(state='normal')  # 'disabled'
                slide_Rx7Gain.configure(state='normal')  # 'disabled'
                slide_Rx8Gain.configure(state='normal')  # 'disabled'
                slide_Rx1Gain.set(gain1)
                slide_Rx2Gain.set(gain2)
                slide_Rx3Gain.set(gain3)
                slide_Rx4Gain.set(gain4)
                slide_Rx5Gain.set(gain4)
                slide_Rx6Gain.set(gain3)
                slide_Rx7Gain.set(gain2)
                slide_Rx8Gain.set(gain1)
        if self.num_ADARs == 2:
            slide_Rx5Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx5Gain_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx6Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx6Gain_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx7Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx7Gain_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx8Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx8Gain_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx5Gain.grid(row=12, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
            slide_Rx6Gain.grid(row=15, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
            slide_Rx7Gain.grid(row=18, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
            slide_Rx8Gain.grid(row=21, column=0, padx=2, pady=2, rowspan=3, columnspan=3)
            slide_Rx5Gain.set(127)
            slide_Rx6Gain.set(127)
            slide_Rx7Gain.set(127)
            slide_Rx8Gain.set(127)
            tk.Button(frame2, text="Rx5 Gain", relief=RAISED, anchor=W, command=Rx5_toggle, highlightthickness=0).grid(row=13, column=3, sticky=E+W)
            tk.Button(frame2, text="Rx6 Gain", relief=RAISED, anchor=W, command=Rx6_toggle, highlightthickness=0).grid(row=16, column=3, sticky=E+W)
            tk.Button(frame2, text="Rx7 Gain", relief=RAISED, anchor=W, command=Rx7_toggle, highlightthickness=0).grid(row=19, column=3, sticky=E+W)
            tk.Button(frame2, text="Rx8 Gain", relief=RAISED, anchor=W, command=Rx8_toggle, highlightthickness=0).grid(row=22, column=3, sticky=E+W)
        button_rect = Button(frame2, text="Rect Window", command=lambda: taper_profile(1))
        button_rect.grid(row=25, column=0, columnspan=1, padx=2, pady=1, sticky=E+W)
        button_cheb = Button(frame2, text="Chebyshev", command=lambda: taper_profile(2))
        button_cheb.grid(row=25, column=1, columnspan=1, padx=2, pady=1, sticky=E+W)
        #button_hamming = Button(frame2, text="Hamming", command=lambda: taper_profile(3))
        #button_hamming.grid(row=25, column=2, columnspan=1, padx=2, pady=1, sticky=E+W)
        button_hann = Button(frame2, text="Hann", command=lambda: taper_profile(4))
        button_hann.grid(row=26, column=0, columnspan=1, padx=2, pady=1, sticky=E+W)
        button_black = Button(frame2, text="Blackman", command=lambda: taper_profile(5))
        button_black.grid(row=26, column=1, columnspan=1, padx=2, pady=1, sticky=E+W)
        '''Frame3:  Phase'''
        self.Rx1Phase_set = tk.DoubleVar()
        self.Rx2Phase_set = tk.DoubleVar()
        self.Rx3Phase_set = tk.DoubleVar()
        self.Rx4Phase_set = tk.DoubleVar()
        self.Rx5Phase_set = tk.DoubleVar()
        self.Rx6Phase_set = tk.DoubleVar()
        self.Rx7Phase_set = tk.DoubleVar()
        self.Rx8Phase_set = tk.DoubleVar()
        slide_Rx1Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx1Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx2Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx2Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx3Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx3Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx4Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx4Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx1Phase.grid(row=0, column=0, padx=2, pady=2, columnspan=3, rowspan=3)
        slide_Rx2Phase.grid(row=3, column=0, padx=2, pady=2, columnspan=3, rowspan=3)
        slide_Rx3Phase.grid(row=6, column=0, padx=2, pady=2, columnspan=3, rowspan=3)
        slide_Rx4Phase.grid(row=9, column=0, padx=2, pady=2, columnspan=3, rowspan=3)
        slide_Rx1Phase.set(0)
        slide_Rx2Phase.set(0)
        slide_Rx3Phase.set(0)
        slide_Rx4Phase.set(0)
        def zero_Rx1():
            slide_Rx1Phase.set(0)
        def zero_Rx2():
            slide_Rx2Phase.set(0)
        def zero_Rx3():
            slide_Rx3Phase.set(0)
        def zero_Rx4():
            slide_Rx4Phase.set(0)
        def zero_Rx5():
            slide_Rx5Phase.set(0)
        def zero_Rx6():
            slide_Rx6Phase.set(0)
        def zero_Rx7():
            slide_Rx7Phase.set(0)
        def zero_Rx8():
            slide_Rx8Phase.set(0)
        tk.Button(frame3, text="Rx1 Phase", relief=RAISED, anchor=W, command=zero_Rx1, highlightthickness=0).grid(row=1, column=3, sticky=E+W)
        tk.Button(frame3, text="Rx2 Phase", relief=RAISED, anchor=W, command=zero_Rx2, highlightthickness=0).grid(row=4, column=3, sticky=E+W)
        tk.Button(frame3, text="Rx3 Phase", relief=RAISED, anchor=W, command=zero_Rx3, highlightthickness=0).grid(row=7, column=3, sticky=E+W)
        tk.Button(frame3, text="Rx4 Phase", relief=RAISED, anchor=W, command=zero_Rx4, highlightthickness=0).grid(row=10, column=3, sticky=E+W)
        Phase_set = tk.IntVar()
        def phase_sel():
            global phase1
            global phase2
            global phase3
            global phase4
            global phase5
            global phase6
            global phase7
            global phase8
            if Phase_set.get()==1:
                phase1=slide_Rx1Phase.get()
                phase2=slide_Rx2Phase.get()
                phase3=slide_Rx3Phase.get()
                phase4=slide_Rx4Phase.get()
                slide_Rx1Phase.set(0)
                slide_Rx2Phase.set(0)
                slide_Rx3Phase.set(0)
                slide_Rx4Phase.set(0)
                slide_Rx1Phase.configure(state='disabled')  # 'normal'
                slide_Rx2Phase.configure(state='disabled')  # 'normal'
                slide_Rx3Phase.configure(state='disabled')  # 'normal'
                slide_Rx4Phase.configure(state='disabled')  # 'normal'
                if self.num_ADARs == 2:
                    phase5=slide_Rx5Phase.get()
                    phase6=slide_Rx6Phase.get()
                    phase7=slide_Rx7Phase.get()
                    phase8=slide_Rx8Phase.get()
                    slide_Rx5Phase.set(0)
                    slide_Rx6Phase.set(0)
                    slide_Rx7Phase.set(0)
                    slide_Rx8Phase.set(0)
                    slide_Rx5Phase.configure(state='disabled')  # 'normal'
                    slide_Rx6Phase.configure(state='disabled')  # 'normal'
                    slide_Rx7Phase.configure(state='disabled')  # 'normal'
                    slide_Rx8Phase.configure(state='disabled')  # 'normal'
            else:
                slide_Rx1Phase.configure(state='normal')  # 'disabled'
                slide_Rx2Phase.configure(state='normal')  # 'disabled'
                slide_Rx3Phase.configure(state='normal')  # 'disabled'
                slide_Rx4Phase.configure(state='normal')  # 'disabled'
                slide_Rx1Phase.set(phase1)
                slide_Rx2Phase.set(phase2)
                slide_Rx3Phase.set(phase3)
                slide_Rx4Phase.set(phase4)
                if self.num_ADARs == 2:
                    slide_Rx5Phase.configure(state='normal')  # 'disabled'
                    slide_Rx6Phase.configure(state='normal')  # 'disabled'
                    slide_Rx7Phase.configure(state='normal')  # 'disabled'
                    slide_Rx8Phase.configure(state='normal')  # 'disabled'
                    slide_Rx5Phase.set(phase5)
                    slide_Rx6Phase.set(phase6)
                    slide_Rx7Phase.set(phase7)
                    slide_Rx8Phase.set(phase8)
        check_Phase = tk.Checkbutton(frame3, text="Set All Phase to 0", variable=Phase_set, highlightthickness=0, onvalue=1, offvalue=0, command=phase_sel, anchor=W, relief=SUNKEN)
        check_Phase.grid(row=24, column=0, padx=10, pady=10, sticky=E+W)
        if self.num_ADARs == 2:
            slide_Rx5Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx5Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx6Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx6Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx7Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx7Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx8Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx8Phase_set, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx5Phase.grid(row=12, column=0, padx=2, pady=2, columnspan=3, rowspan=3, sticky=E+W)
            slide_Rx6Phase.grid(row=15, column=0, padx=2, pady=2, columnspan=3, rowspan=3, sticky=E+W)
            slide_Rx7Phase.grid(row=18, column=0, padx=2, pady=2, columnspan=3, rowspan=3, sticky=E+W)
            slide_Rx8Phase.grid(row=21, column=0, padx=2, pady=2, columnspan=3, rowspan=3, sticky=E+W)
            slide_Rx5Phase.set(0)
            slide_Rx6Phase.set(0)
            slide_Rx7Phase.set(0)
            slide_Rx8Phase.set(0)
            tk.Button(frame3, text="Rx5 Phase", relief=RAISED, anchor=W, command=zero_Rx5, highlightthickness=0).grid(row=13, column=3, padx=1, sticky=E+W)
            tk.Button(frame3, text="Rx6 Phase", relief=RAISED, anchor=W, command=zero_Rx6, highlightthickness=0).grid(row=16, column=3, padx=1, sticky=E+W)
            tk.Button(frame3, text="Rx7 Phase", relief=RAISED, anchor=W, command=zero_Rx7, highlightthickness=0).grid(row=19, column=3, padx=1, sticky=E+W)
            tk.Button(frame3, text="Rx8 Phase", relief=RAISED, anchor=W, command=zero_Rx8, highlightthickness=0).grid(row=22, column=3, padx=1, sticky=E+W)
        '''Frame4:  BW'''
        self.BW = tk.DoubleVar()
        slide_SignalBW=Scale(frame4, from_=0, to=2000, variable=self.BW, resolution=100, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_SignalBW.grid(row=0, column=0, padx=10, pady=10, rowspan=3)
        slide_SignalBW.set(self.SignalFreq/1e9)
        tk.Label(frame4, text="Signal BW (MHz)", relief=SUNKEN, anchor=W).grid(row=1, column=2, sticky=E+W)
        self.RxSignal_text = tk.StringVar()
        self.RxSignal = tk.Label(frame4, textvariable=self.RxSignal_text, relief=SUNKEN, anchor=W).grid(row=8, column=0, pady=10, padx=5, sticky=E+W)
        self.RxSignal_text.set("Signal Bandwidth = ")        
        self.MixerLO_text = tk.StringVar()
        self.MixerLO = tk.Label(frame4, textvariable=self.MixerLO_text, relief=SUNKEN, anchor=W).grid(row=9, column=0, pady=10, padx=5, sticky=E+W)
        self.MixerLO_text.set("Mixer LO Freq = ")
        self.PlutoRxLO_text = tk.StringVar()
        self.PlutoRxLO = tk.Label(frame4, textvariable=self.PlutoRxLO_text, relief=SUNKEN, anchor=W).grid(row=10, column=0, pady=10, padx=5, sticky=E+W)
        self.PlutoRxLO_text.set("Pluto Rx LO = ")
        self.BeamCalc_text = tk.StringVar()
        self.BeamCalc = tk.Label(frame4, textvariable=self.BeamCalc_text, relief=SUNKEN, anchor=W).grid(row=11, column=0, pady=10, padx=5, sticky=E+W)
        self.BeamCalc_text.set("Beam Calculated at ")
        self.AngleMeas_text = tk.StringVar()
        self.AngleMeas = tk.Label(frame4, textvariable=self.AngleMeas_text, relief=SUNKEN, anchor=W).grid(row=12, column=0, pady=10, padx=5, sticky=E+W)
        self.AngleMeas_text.set("Beam Measured at ")        
        '''Frame5:  Bits'''
        self.res = tk.DoubleVar()
        self.bits = tk.IntVar()
        slide_res=Scale(frame5, from_=0.1, to=5, variable=self.res, resolution=0.1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_bits=Scale(frame5, from_=1, to=7, resolution=1, variable=self.bits, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_res.grid(row=0, column=0, padx=10, pady=10, rowspan=3)
        slide_bits.grid(row=3, column=0, padx=10, pady=10, rowspan=3)
        slide_res.set(2.8125)
        slide_bits.set(7)
        tk.Label(frame5, text="Steer Resolution", relief=SUNKEN, anchor=W).grid(row=1, column=2, sticky=E+W)
        tk.Label(frame5, text="Phase Shift Bits", relief=SUNKEN, anchor=W).grid(row=4, column=2, sticky=E+W)
        self.res_text = tk.StringVar()
        self.res_degrees = tk.Label(frame5, textvariable=self.res_text, relief=SUNKEN, anchor=W).grid(row=8, column=0, columnspan=3, pady=10, padx=5, sticky=E+W)
        self.res_text.set("Phase Shift Steps = ")
        self.res_bits = tk.IntVar()
        check_res_bits = tk.Checkbutton(frame5, text="Ignore Steer Res", highlightthickness=0, variable=self.res_bits, onvalue=1, offvalue=0, anchor=W, relief=FLAT)
        check_res_bits.grid(row=10, column=0, columnspan=1, sticky=E+W)
        self.res_bits.set(1)
        '''Frame6:  Tracking'''
        

            
        #tk.Button(frame6, text="Scan and Initialize", relief=RAISED, anchor=W, command=find_peak, highlightthickness=0).grid(row=21, column=2, sticky=E+W)




        '''Frame7:  Plot Options'''
        def clearMax():
            if self.PlotMax_set.get()==0:
                self.max_hold = -1000
                self.min_hold = 1000
        self.PlotMax_set = tk.IntVar()
        check_PlotMax = tk.Checkbutton(frame7, text="Show Peak Gain", highlightthickness=0, variable=self.PlotMax_set, command=clearMax, onvalue=1, offvalue=0, anchor=W, relief=SUNKEN)
        check_PlotMax.grid(row=0, column=0, columnspan=3, padx=20, pady=20, sticky=E+W)
        self.AngleMax_set = tk.IntVar()
        check_AngleMax = tk.Checkbutton(frame7, text="Show Peak Angle", highlightthickness=0, variable=self.AngleMax_set, onvalue=1, offvalue=0, anchor=W, relief=SUNKEN)
        check_AngleMax.grid(row=1, column=0, columnspan=3, padx=20, pady=20, sticky=E+W)
        self.HPBW_set = tk.IntVar()
        check_HPBW = tk.Checkbutton(frame7, text="Shade 3dB Area (HPBW)", highlightthickness=0, variable=self.HPBW_set, onvalue=1, offvalue=0, anchor=W, relief=SUNKEN)
        check_HPBW.grid(row=2, column=0, columnspan=3, padx=20, pady=20, sticky=E+W)
        self.show_sum = tk.IntVar()
        check_sum = tk.Checkbutton(frame7, text="Show Sum", highlightthickness=0, variable=self.show_sum, onvalue=1, offvalue=0, anchor=W, relief=SUNKEN)
        check_sum.grid(row=0, column=3, columnspan=1, padx=20, pady=20, sticky=E+W)
        self.show_sum.set(1)
        self.show_delta = tk.IntVar()
        check_delta = tk.Checkbutton(frame7, text="Show Delta", highlightthickness=0, variable=self.show_delta, onvalue=1, offvalue=0, anchor=W, relief=SUNKEN)
        check_delta.grid(row=1, column=3, columnspan=1, padx=20, pady=20, sticky=E+W)
        self.show_delta.set(1)
        self.show_error = tk.IntVar()
        check_error = tk.Checkbutton(frame7, text="Show Error", highlightthickness=0, variable=self.show_error, onvalue=1, offvalue=0, anchor=W, relief=SUNKEN)
        check_error.grid(row=2, column=3, columnspan=1, padx=20, pady=20, sticky=E+W)
        self.show_error.set(1)
        self.x_min = tk.DoubleVar()
        self.x_max = tk.DoubleVar()
        self.y_min = tk.DoubleVar()
        self.y_max = tk.DoubleVar()
        def check_axis(var_axis):
            x_minval=slide_x_min.get()
            x_maxval=slide_x_max.get()
            y_minval=slide_y_min.get()
            y_maxval=slide_y_max.get()
            if (x_minval)>=x_maxval:
                slide_x_min.set(x_maxval-1)
            if y_minval>=y_maxval:
                slide_y_min.set(y_maxval-1)            
        slide_x_min=Scale(frame7, from_=-100, to=99, variable=self.x_min, command=check_axis, resolution=1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_x_min.grid(row=3, column=0, padx=10, pady=10, rowspan=3, columnspan=3)
        slide_x_min.set(-89)
        tk.Label(frame7, text="X axis min", relief=SUNKEN, anchor=W).grid(row=4, column=3, sticky=E+W)
        slide_x_max=Scale(frame7, from_=-99, to=100, variable=self.x_max, command=check_axis, resolution=1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_x_max.grid(row=6, column=0, padx=10, pady=10, rowspan=3, columnspan=3)
        slide_x_max.set(89)
        tk.Label(frame7, text="X axis max", relief=SUNKEN, anchor=W).grid(row=7, column=3, sticky=E+W)
        slide_y_min=Scale(frame7, from_=-100, to=9, variable=self.y_min, command=check_axis, resolution=1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_y_min.grid(row=9, column=0, padx=10, pady=10, rowspan=3, columnspan=3)
        slide_y_min.set(-50)
        tk.Label(frame7, text="Y axis min", relief=SUNKEN, anchor=W).grid(row=10, column=3, sticky=E+W)
        slide_y_max=Scale(frame7, from_=-59, to=10, variable=self.y_max, command=check_axis, resolution=1, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_y_max.grid(row=12, column=0, padx=10, pady=10, rowspan=3, columnspan=3)
        slide_y_max.set(0)
        tk.Label(frame7, text="Y axis max", relief=SUNKEN, anchor=W).grid(row=13, column=3, sticky=E+W)
        tk.Label(frame7, text="These options take effect with the next plot refresh", relief=SUNKEN, anchor=W).grid(row=20, column=0, columnspan=4, padx=20, pady=20, sticky=E+W)

        '''CONFIGURE THE TABS FOR PLOTTING'''
        plot_tabs = ttk.Notebook(self.master)
        plot_tabs.grid(padx=10, pady=10, row=0, column=4, columnspan=4)
        plot_tabs.columnconfigure((0, 1, 2, 3, 4, 5, 6, 7), weight=1)
        plot_tabs.rowconfigure((0, 1, 2, 3, 4, 5), weight=1)
        self.frame11 = Frame(plot_tabs, width=700, height=500)
        self.frame11.grid(row=0, column=2)
        self.frame11.columnconfigure((0, 1, 2, 3, 4, 5), weight=1)
        self.frame11.rowconfigure((0), weight=1)
        plot_tabs.add(self.frame11, text="Rectangular Plot")
        self.frame12 = Frame(plot_tabs, width=700, height=500)
        self.frame12.grid(row=0, column=2)
        self.frame12.columnconfigure((0, 1, 2, 3, 4, 5), weight=1)
        self.frame12.rowconfigure((0), weight=1)
        plot_tabs.add(self.frame12, text="Polar Plot")
        self.frame13 = Frame(plot_tabs, width=700, height=500)
        self.frame13.grid(row=0, column=2)
        self.frame13.columnconfigure((0, 1, 2, 3, 4, 5), weight=1)
        self.frame13.rowconfigure((0), weight=1)
        plot_tabs.add(self.frame13, text="FFT")
        self.frame14 = Frame(plot_tabs, width=700, height=500)
        self.frame14.grid(row=0, column=2)
        self.frame14.columnconfigure((0, 1, 2, 3, 4, 5), weight=1)
        self.frame14.rowconfigure((0), weight=1)
        plot_tabs.add(self.frame14, text="Signal Tracking")
        plot_tabs.select(0)
        
        def conf(event):
            plot_tabs.config(height=max(root.winfo_height()-100, 500), width=max(root.winfo_width()-450, 300))
        root.bind("<Configure>", conf)
        
        self.generate_Figures()
        '''START THE UPDATER'''
        self.updater()
        
    def updater(self):
        self.programBeam()
        try:
            self.plotData(1,1,0)
        except:
            print(1)
            self.programBeam()
        self.refresh_time = self.update_time.get()
        if self.refresh.get()==1:
            self.master.after(self.refresh_time, self.updater)

    def find_peak(self):
        self.programBeam()
        print("Begin Tracking Mode.  This will last about 20 seconds.")
        ADAR_set_RxPhase(spi, self.ADDR1, 1, self.max_PhDelta, 2.8125, self.RxPhase1, self.RxPhase2, self.RxPhase3, self.RxPhase4)
        ADAR_update_Rx(spi, self.ADDR1)
        ADAR_set_RxPhase(spi, self.ADDR2, 2, self.max_PhDelta, 2.8125, self.RxPhase5, self.RxPhase6, self.RxPhase7, self.RxPhase8)
        ADAR_update_Rx(spi, self.ADDR2)
        i=0
        #start=time.time()
        if self.mode_var.get() == 'Tracking':  # I realize this gets caught in a loop that you can't use the GUI during.  But I'm not having success with threading and TKinter....  Any ideas are welcome!
            #track_thread= Thread(target=self.track, args=(self.max_PhDelta,))
            #track_thread.start()
            #plot_thread = Thread(self.plotData, args=(0,0,1,))
            #plot_thread.start()
            #track_thread.join()
            for i in range(0,1500):
                self.track(self.max_PhDelta)
                if i%20==0:
                    self.plotData(0,0,1)
            self.mode_var.set('Beam Sweep')
        print("End of Tracking Operation. To do tracking again, select the Tracking mode from the pulldown menu.")

    def track(self, PhDelta):
        ADAR_set_RxPhase(spi, self.ADDR1, 1, PhDelta, 2.8125, self.RxPhase1, self.RxPhase2, self.RxPhase3, self.RxPhase4)
        ADAR_update_Rx(spi, self.ADDR1)
        ADAR_set_RxPhase(spi, self.ADDR2, 2, PhDelta, 2.8125, self.RxPhase5, self.RxPhase6, self.RxPhase7, self.RxPhase8)
        ADAR_update_Rx(spi, self.ADDR2)
        PeakValue_sum, PeakValue_delta, PeakValue_beam_phase, sum_chan, target_error = self.getData(1)
        error_thresh = 0
        if target_error > (-1*error_thresh):
            PhDelta = PhDelta - 2.8125
        elif target_error < (1*error_thresh):
            PhDelta = PhDelta + 2.8125
        SteerAngle = self.ConvertPhaseToSteerAngle(PhDelta)
        self.TrackArray.append(SteerAngle)
        self.max_PhDelta = PhDelta


    def RxPhaseUpdate(self, val):
        x=0
        #self.programBeam()
        #self.plotData()

    def generate_Figures(self):
        plt.clf()
        self.figure1 = plt.Figure(figsize=(4, 3), dpi=100)
        self.ax1 = self.figure1.add_subplot(3,1,(1,2))  # this plot is in the first and second cell, so has 2x the height
        self.ax2 = self.figure1.add_subplot(3,1,3)      # this plot is in the 3rd cell
        self.graph1 = FigureCanvasTkAgg(self.figure1, self.frame11)
        self.graph1.draw()
        self.graph1.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.graph1, self.frame11)
        self.toolbar.update()
        
        self.polar1 = plt.Figure(figsize=(1, 1), dpi=100)
        self.pol_ax1 = self.polar1.add_subplot(111, projection='polar')
        self.pol_graph1 = FigureCanvasTkAgg(self.polar1, self.frame12)
        self.polar1.subplots_adjust(0, 0, 1, 1)
        self.pol_graph1.draw()
        self.pol_graph1.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.pol_graph1, self.frame12)
        self.toolbar.update()
        
        self.figure3 = plt.Figure(figsize=(4, 3), dpi=100)
        self.ax3 = self.figure3.add_subplot(111)
        self.graph3 = FigureCanvasTkAgg(self.figure3, self.frame13)
        self.graph3.draw()
        self.graph3.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.graph3, self.frame13)
        self.toolbar.update()
        
        self.figure4 = plt.Figure(figsize=(4, 3), dpi=100)
        self.ax4 = self.figure4.add_subplot(111)
        self.graph4 = FigureCanvasTkAgg(self.figure4, self.frame14)
        self.graph4.draw()
        self.graph4.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.graph4, self.frame14)
        self.toolbar.update()
        
    def savePlot(self):
        self.saved_gain = self.ArrayGain
        self.saved_angle = self.ArrayAngle

    def clearPlot(self):
        self.saved_gain = []
        self.saved_angle = []
        
    def closeProgram(self):
        self.master.destroy()
        plt.close('all')
        sys.exit(0)
    
    def programTaper(self):
        self.RxGain1 = self.Rx1Gain_set.get()
        self.RxGain2 = self.Rx2Gain_set.get()
        self.RxGain3 = self.Rx3Gain_set.get()
        self.RxGain4 = self.Rx4Gain_set.get()
        ADAR_set_RxTaper(spi, self.ADDR1, self.RxGain2, self.RxGain1, self.RxGain4, self.RxGain3)
        if self.num_ADARs == 2:
            self.RxGain5 = self.Rx5Gain_set.get()
            self.RxGain6 = self.Rx6Gain_set.get()
            self.RxGain7 = self.Rx7Gain_set.get()
            self.RxGain8 = self.Rx8Gain_set.get()
            ADAR_set_RxTaper(spi, self.ADDR2, self.RxGain6, self.RxGain5, self.RxGain8, self.RxGain7)

    def programLO(self):
        self.SignalFreq = self.freq.get()*1e9
        self.LO_freq = int(self.SignalFreq - self.Mixer_LO)
        self.Rx_gain = int(self.RxGain.get())
        SDR_setLOandGain(self.sdr, self.num_Rx, self.LO_freq, self.Rx_gain)
        
    def ConvertPhaseToSteerAngle(self, PhDelta):
        # steering angle theta = arcsin(c*deltaphase/(2*pi*f*d)
        value1 = (self.c * np.radians(np.abs(PhDelta)))/(2*3.14159*(self.SignalFreq-self.bandwidth*1000000)*self.d)
        clamped_value1 = max(min(1, value1), -1)     #arcsin argument must be between 1 and -1, or numpy will throw a warning
        theta = np.degrees(np.arcsin(clamped_value1))
        if PhDelta>=0:
            SteerAngle = theta   # positive PhaseDelta covers 0deg to 90 deg
        else:
            SteerAngle = -theta   # negative phase delta covers 0 deg to -90 deg
        return SteerAngle
    
    def getData(self, Averages):
        total_sum=0
        total_delta=0
        total_beam_phase=0
        for count in range (0, Averages):
            data_raw=SDR_getData(self.sdr, self.num_Rx)
            data = data_raw
            chan1 = data[0]   # Rx1 data
            chan2 = data[1]   # Rx2 data
            sum_chan = chan1+chan2
            delta_chan = chan1-chan2
            NumSamples = len(sum_chan)               #number of samples
            #win = np.blackman(NumSamples)
            y_sum = sum_chan * 1
            y_delta = delta_chan * 1
            s_sum = np.fft.fftshift(y_sum)
            s_delta = np.fft.fftshift(y_delta)
            max_index = np.argmax(s_sum)
            total_beam_phase = total_beam_phase + (np.angle(s_sum[max_index]) - np.angle(s_delta[max_index]))
            s_mag_sum = np.abs(s_sum[max_index]) #* 2 / np.sum(win)
            s_mag_delta = np.abs(s_delta[max_index]) #* 2 / np.sum(win)
            s_dbfs_sum = 20*np.log10(np.max([s_mag_sum, 10**(-15)])/(2**12))        # make sure the log10 argument isn't zero (hence np.max)
            s_dbfs_delta = 20*np.log10(np.max([s_mag_delta, 10**(-15)])/(2**12))    # make sure the log10 argument isn't zero (hence np.max)
            total_sum=total_sum+(s_dbfs_sum)   # sum up all the loops, then we'll average
            total_delta=total_delta+(s_dbfs_delta)   # sum up all the loops, then we'll average
        PeakValue_sum = total_sum/Averages
        PeakValue_delta = total_delta/Averages
        PeakValue_beam_phase = total_beam_phase/Averages
        if np.sign(PeakValue_beam_phase)==-1:
            target_error = min(-0.01, (np.sign(PeakValue_beam_phase) * (PeakValue_sum - PeakValue_delta) + np.sign(PeakValue_beam_phase) * (PeakValue_sum + PeakValue_delta)/2) / (PeakValue_sum + PeakValue_delta))
        else:
            target_error = max(0.01, (np.sign(PeakValue_beam_phase) * (PeakValue_sum - PeakValue_delta) + np.sign(PeakValue_beam_phase) * (PeakValue_sum + PeakValue_delta)/2) / (PeakValue_sum + PeakValue_delta))
        return(PeakValue_sum, PeakValue_delta, PeakValue_beam_phase, sum_chan, target_error)

    def programBeam(self):
        self.programLO()
        self.programTaper()
        self.Averages = int(self.Avg.get())
        steer_res = self.res.get()
        phase_step_size = 360/(2**self.bits.get())
        self.bandwidth = self.BW.get()
        self.RxSignal_text.set(str("Signal Bandwidth = "+str(self.bandwidth)+" MHz"))
        self.MixerLO_text.set(str("Mixer LO Freq = "+str(int(self.Mixer_LO/1000000))+" MHz"))
        self.PlutoRxLO_text.set(str("Pluto Rx LO = "+str(int(self.LO_freq/1000000))+" MHz"))
        self.BeamCalc_text.set(str("Beam Calculated at "+str(int(self.SignalFreq/1000000-self.bandwidth))+" MHz"))
        self.AngleMeas_text.set(str("Beam Measured at "+str(int(self.SignalFreq/1000000))+" MHz"))

        SteerValues = np.arange(-90, 90+steer_res, steer_res)  # convert degrees to radians
        # Phase delta = 2*Pi*d*sin(theta)/lambda = 2*Pi*d*sin(theta)*f/c
        PhaseValues = np.degrees(2*3.14159*self.d*np.sin(np.radians(SteerValues))*self.SignalFreq/self.c)
        self.res_text.set(str("Phase Shift LSB = "+str(phase_step_size))+" deg")
        if self.res_bits.get()==1:
            phase_limit = int(225/phase_step_size)*phase_step_size+phase_step_size
            PhaseValues = np.arange(-phase_limit, phase_limit, phase_step_size)
        if self.mode_var.get() == 'Static Phase':
            PhaseValues = [self.RxPhaseDelta.get()]
        gain = []
        delta = []
        beam_phase = []
        angle = []
        diff_error = []
        self.max_gain = []
        max_signal = -1000000000
        max_angle = -90
        self.RxPhase1 = self.Rx1Phase_set.get()+self.Rx1_cal
        self.RxPhase2 = self.Rx2Phase_set.get()+self.Rx2_cal
        self.RxPhase3 = self.Rx3Phase_set.get()+self.Rx3_cal
        self.RxPhase4 = self.Rx4Phase_set.get()+self.Rx4_cal
        if self.num_ADARs == 2:
            self.RxPhase5 = self.Rx5Phase_set.get()+self.Rx5_cal
            self.RxPhase6 = self.Rx6Phase_set.get()+self.Rx6_cal
            self.RxPhase7 = self.Rx7Phase_set.get()+self.Rx7_cal
            self.RxPhase8 = self.Rx8Phase_set.get()+self.Rx8_cal
        for PhDelta in PhaseValues:
            ADAR_set_RxPhase(spi, self.ADDR1, 1, PhDelta, phase_step_size, self.RxPhase1, self.RxPhase2, self.RxPhase3, self.RxPhase4)
            ADAR_update_Rx(spi, self.ADDR1)
            if self.num_ADARs == 2:
                ADAR_set_RxPhase(spi, self.ADDR2, 2, PhDelta, phase_step_size, self.RxPhase5, self.RxPhase6, self.RxPhase7, self.RxPhase8)
                ADAR_update_Rx(spi, self.ADDR2)
            SteerAngle = self.ConvertPhaseToSteerAngle(PhDelta)
            PeakValue_sum, PeakValue_delta, PeakValue_beam_phase, sum_chan, target_error = self.getData(self.Averages)

            if PeakValue_sum>max_signal:    #for the largest value, save the data so we can plot it in the FFT window
                max_signal = PeakValue_sum
                max_angle = PeakValue_beam_phase
                self.max_PhDelta = PhDelta
                data_fft = sum_chan
            gain.append(PeakValue_sum)
            delta.append(PeakValue_delta)
            beam_phase.append(PeakValue_beam_phase)
            angle.append(SteerAngle)
            diff_error.append(target_error)

        # take the FFT of the raw data ("data_fft" which corresponded to the peak gain
        NumSamples = len(data_fft)          #number of samples
        win = np.blackman(NumSamples)
        y = data_fft * win
        sp = np.absolute(np.fft.fft(y))
        sp = sp[1:-1]
        sp = np.fft.fftshift(sp)
        s_mag = np.abs(sp) * 2 / np.sum(win)    # Scale FFT by window and /2 since we are using half the FFT spectrum
        s_mag = np.maximum(s_mag, 10**(-15))
        self.max_gain = 20*np.log10(s_mag/(2**12))     # Pluto is a 12 bit ADC, so use that to convert to dBFS
        ts = 1 / float(self.SampleRate)
        self.xf = np.fft.fftfreq(NumSamples, ts)
        self.xf = np.fft.fftshift(self.xf[1:-1])     # this is the x axis (freq in Hz) for our fft plot

        self.ArrayGain = gain
        self.ArrayDelta = delta
        self.ArrayBeamPhase = beam_phase
        self.ArrayAngle = angle
        self.ArrayError = diff_error
        self.peak_gain=max(self.ArrayGain)
        index_peak_gain = np.where(self.ArrayGain==self.peak_gain)
        index_peak_gain = index_peak_gain[0]
        self.max_angle=self.ArrayAngle[int(index_peak_gain[0])]
        
    def plotData(self, plot_gain, plot_fft, plot_tracking):
        # plot sum of both channels and subtraction of both channels

        if plot_gain == 1:
            x_axis_min=self.x_min.get()
            x_axis_max=self.x_max.get()
            y_axis_min=self.y_min.get()
            y_axis_max=self.y_max.get()
            self.ax1.cla()
            if self.show_sum.get()==1:
                self.ax1.plot(self.ArrayAngle, self.ArrayGain, '-o', ms=3, alpha=0.7, mfc='blue', color='blue')
            if self.show_delta.get()==1:
                self.ax1.plot(self.ArrayAngle, self.ArrayDelta, '-o', ms=3, alpha=0.7, mfc='red', color='red')
            self.ax1.plot(self.saved_angle, self.saved_gain, '-o', ms=1, alpha=0.5, mfc='green', color='green')
            self.ax1.set_title('Array Gain vs Steering Angle')
            #self.ax1.set_xlabel('Steering Angle (deg)')
            self.ax1.set_ylabel('Gain (dBFS)')
            if self.show_sum.get()==1 & self.show_delta.get()==1:
                self.ax1.legend(["Sum", "Delta"])
            elif self.show_sum.get()==1:
                self.ax1.legend(["Sum"])
            elif self.show_delta.get()==1:
                self.ax1.legend(["Delta"])
            else:
                self.ax1.legend([" "])
            self.ax1.set_xlim([x_axis_min, x_axis_max])
            self.ax1.set_ylim([y_axis_min, y_axis_max])
            self.ax1.grid()
            if self.PlotMax_set.get() == 1:
                self.ax1.axhline(y=self.peak_gain, color='blue', linestyle="--", alpha=0.3)
            if self.AngleMax_set.get() == 1:
                self.ax1.axvline(x=self.max_angle, color = 'red', linestyle=":", alpha=0.3)
            if self.HPBW_set.get() == 1:
                shade = collections.BrokenBarHCollection.span_where(np.clip(self.ArrayAngle, max_angle-40, max_angle+40), ymin=-60, ymax=self.peak_gain, where=self.ArrayGain>self.peak_gain-3, facecolor='green', alpha=0.1)
                self.ax1.add_collection(shade)
                
            # Plot phase delta and error function
            self.ax2.cla()
            if self.show_error.get()==1:
                self.ax2.plot(self.ArrayAngle, np.sign(self.ArrayBeamPhase), '-o', ms=3, alpha=0.7, mfc='blue', color='blue')
                self.ax2.plot(self.ArrayAngle, self.ArrayError, '-o', ms=3, alpha=0.7, mfc='red', color='red')
                self.ax2.legend(["Phase Delta", "Error Function"])
            self.ax2.set_xlabel('Steering Angle (deg)')
            self.ax2.set_ylabel('Error Function')
            self.ax2.set_xlim([x_axis_min, x_axis_max])
            self.ax2.set_ylim([-1.5, 1.5])
            self.ax2.grid()
        
            self.graph1.draw()
            
            #Polar Plot
            self.pol_ax1.cla()
            self.pol_ax1.plot(np.radians(self.ArrayAngle), self.ArrayGain, '-o', ms=5, alpha=0.7, mfc='blue', color='blue')
            self.pol_ax1.plot(np.radians(self.saved_angle), self.saved_gain, '-o', ms=1, alpha=0.5, mfc='green', color='green')
            self.pol_ax1.set_theta_offset(np.pi/2)  # rotate polar axis so that 0 deg is on top
            self.pol_ax1.set_theta_direction(-1)
            self.pol_ax1.set_thetamin(-89)
            self.pol_ax1.set_thetamax(89)
            self.pol_ax1.set_rmin(y_axis_min)
            self.pol_ax1.set_rmax(y_axis_max)
            self.pol_graph1.draw()
        
        # FFT Spectrum
        if plot_fft == 1:
            x_axis_min=self.x_min.get()
            x_axis_max=self.x_max.get()
            y_axis_min=self.y_min.get()
            y_axis_max=self.y_max.get()
            self.ax3.cla()
            self.ax3.plot(self.xf/1e6, self.max_gain)
            self.ax3.set_title('FFT at Peak Steering Angle')
            #self.ax3.set_xlabel('Freq (MHz)')
            self.ax3.set_xlabel(f"Freq - {int(self.SignalFreq/1e6)} MHz")
            self.ax3.set_ylabel('Gain (dBFS)')
            #self.ax3.set_xlim([x_axis_min, x_axis_max])
            self.ax3.set_ylim([-100, y_axis_max])
            self.ax3.grid()
            max_fft_gain=max(self.max_gain)
            index_max_fft_gain = np.where(self.max_gain==max_fft_gain)
            max_freq=self.xf[int(index_max_fft_gain[0])]/1e6
            if self.PlotMax_set.get() == 1:
                self.max_hold = max(self.max_hold, max_fft_gain)
                self.min_hold = min(self.min_hold, max_fft_gain)
                self.ax3.axhline(y=max_fft_gain, color='blue', linestyle="--", alpha=0.3)
                self.ax3.axhline(y=self.max_hold, color='green', linestyle="--", alpha=0.3)
                self.ax3.axhline(y=self.min_hold, color='red', linestyle="--", alpha=0.3)
            if self.AngleMax_set.get() == 1:
                self.ax3.axvline(x=max_freq, color = 'orange', linestyle=":", alpha=0.5)
            self.graph3.draw()

        # Monopulse Tracking Waterfall Plot
        if plot_tracking == 1:
            self.TrackArray = self.TrackArray[-1000:]  # just use the last elements
            self.ax4.cla()
            self.ax4.plot(self.TrackArray[-1000:], range(0,10000, 10), '-o', ms=3, alpha=0.7, mfc='blue')
            self.ax4.set_xlabel('Steering Angle (deg)')
            self.ax4.set_ylabel('time (ms)')
            self.ax4.set_xlim([-60, 60])
            #self.ax4.set_ylim([-1, 1])
            self.ax4.grid()
            self.graph4.draw()

root = Tk()
root.title("Phased Array Beamforming")
root.geometry("1200x600+0+0")
root.minsize(700,600)
app=App(root)
root.mainloop()
            