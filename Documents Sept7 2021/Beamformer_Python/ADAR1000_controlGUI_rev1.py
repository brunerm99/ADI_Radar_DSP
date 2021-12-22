#!/usr/bin/env python
#  Must use Python 3!!

import sys
import os
import time
import numpy as np
import spidev
from tkinter import *
import tkinter as tk
from tkinter import ttk
import tkinter.messagebox as mb
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.widgets import Cursor
import matplotlib.collections as collections

from ADAR_functions import *   #import the ADAR1000 functions (These all start with ADAR_xxxx)
from SDR_functions import *    #import the SDR functions (These all start with SDR_xxxx)

spi = spidev.SpiDev()
spi.open(0, 0)  # set bus = 0 and chip select (device) = 0
spi.max_speed_hz = 500000
spi.mode = 0

class App:  
    def __init__(self, master):      
        '''SET DEFAULT VALUES'''
        self.Taper = 0
        self.SymTaper = 0
        self.RxGain1 = 127
        self.RxGain2 = 127
        self.RxGain3 = 127
        self.RxGain4 = 127
        self.RxGain5 = 127
        self.RxGain6 = 127
        self.RxGain7 = 127
        self.RxGain8 = 127
        self.Rx1_cal = -19.6875   # you can put phase cal values here (to compensate for phase mismatches in the lines, etc.)
        self.Rx2_cal = 0
        self.Rx3_cal = -8.4375
        self.Rx4_cal = 16.8750
        self.Rx5_cal = 0
        self.Rx6_cal = 0
        self.Rx7_cal = 0
        self.Rx8_cal = -0
        # The ADAR1000 address is set by the address pins on the ADAR1000.  This is set by P10 on the eval board.
        # ADDR 00 (BEAM0, 0x00) is set by leaving all jumpers off of P10
        # ADDR 01 (BEAM1, 0x20) is set by jumpering pins 4 and 6 on P10
        # ADDR 10 (BEAM2, 0x40) is set by jumpering pins 3 and 5 on P10
        # ADDR 11 (BEAM3, 0x60) is set by jumpering both 4+6 and 3+5 on P10
        self.ADDR1=0x20         
        self.ADDR2=0x40         
        self.num_ADARs = 1      # Number of ADAR1000's connected -- this can be either 1 or 2. no other values are allowed
        self.num_Rx = 1         # Number of Rx channels (i.e. Pluto this must be 1, but AD9361 SOM this could be 1 or 2)
        self.sel_Tx = 0         # 0 = Rx mode, 1 = Tx mode
        '''Intialize the ADAR1000'''
        ADAR_init(spi, self.ADDR1)
        if self.num_ADARs==2:
            ADAR_init(spi, self.ADDR2)
        '''BUILD THE GUI: Master Frame'''
        self.master = master
        cntrl_tabs = ttk.Notebook(self.master, height=500, width=400)
        cntrl_tabs.grid(padx=10, pady=10, row=0, column=0, columnspan=4, sticky=N)
        cntrl_width=400
        cntrl_height=500
        frame2 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame3 = Frame(cntrl_tabs, width=cntrl_width, height=cntrl_height)
        frame2.grid(row=0, column=1)
        frame3.grid(row=0, column=1)
        cntrl_tabs.add(frame2, text="Gain")
        cntrl_tabs.add(frame3, text="Phase")
        '''Frame2:  Gain'''
        self.Rx1Gain_set = tk.IntVar()
        self.Rx2Gain_set = tk.IntVar()
        self.Rx3Gain_set = tk.IntVar()
        self.Rx4Gain_set = tk.IntVar()
        self.Rx5Gain_set = tk.IntVar()
        self.Rx6Gain_set = tk.IntVar()
        self.Rx7Gain_set = tk.IntVar()
        self.Rx8Gain_set = tk.IntVar()
        def programGains(var1):
            self.programTaper()
        slide_Rx1Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx1Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx2Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx2Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx3Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx3Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx4Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx4Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
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
        if self.num_ADARs == 2:
            slide_Rx5Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx5Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx6Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx6Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx7Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx7Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
            slide_Rx8Gain=Scale(frame2, from_=0, to=127, resolution=1, variable=self.Rx8Gain_set, command=programGains, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
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
        '''Frame3:  Phase'''
        self.Rx1Phase_set = tk.DoubleVar()
        self.Rx2Phase_set = tk.DoubleVar()
        self.Rx3Phase_set = tk.DoubleVar()
        self.Rx4Phase_set = tk.DoubleVar()
        self.Rx5Phase_set = tk.DoubleVar()
        self.Rx6Phase_set = tk.DoubleVar()
        self.Rx7Phase_set = tk.DoubleVar()
        self.Rx8Phase_set = tk.DoubleVar()
        def programPhase(var1):
            self.programBeam()
        slide_Rx1Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx1Phase_set, command=programPhase, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx2Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx2Phase_set, command=programPhase, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx3Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx3Phase_set, command=programPhase, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
        slide_Rx4Phase=Scale(frame3, from_=-180, to=180, resolution=2.8125, digits=7, variable=self.Rx4Phase_set, command=programPhase, troughcolor="LightYellow3", bd=2, orient=HORIZONTAL, relief=SUNKEN, length=200)
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
    
    def programTaper(self):
        self.RxGain1 = self.Rx1Gain_set.get()
        self.RxGain2 = self.Rx2Gain_set.get()
        self.RxGain3 = self.Rx3Gain_set.get()
        self.RxGain4 = self.Rx4Gain_set.get()
        print(self.ADDR1, self.RxGain1, self.RxGain2, self.RxGain3, self.RxGain4)
        ADAR_set_RxTaper(spi, self.ADDR1, self.RxGain1, self.RxGain2, self.RxGain3, self.RxGain4)
        ADAR_update_Rx(spi, self.ADDR1)
        if self.num_ADARs == 2:
            self.RxGain5 = self.Rx5Gain_set.get()
            self.RxGain6 = self.Rx6Gain_set.get()
            self.RxGain7 = self.Rx7Gain_set.get()
            self.RxGain8 = self.Rx8Gain_set.get()
            ADAR_set_RxTaper(spi, self.ADDR2, self.RxGain5, self.RxGain6, self.RxGain7, self.RxGain8)
            ADAR_update_Rx(spi, self.ADDR2)

    def programBeam(self):
        PhDelta = 0
        phase_step_size = 2.8125
        self.RxPhase1 = self.Rx1Phase_set.get()+self.Rx1_cal
        self.RxPhase2 = self.Rx2Phase_set.get()+self.Rx2_cal
        self.RxPhase3 = self.Rx3Phase_set.get()+self.Rx3_cal
        self.RxPhase4 = self.Rx4Phase_set.get()+self.Rx4_cal
        print(PhDelta, phase_step_size, self.RxPhase1, self.RxPhase2, self.RxPhase3, self.RxPhase4)
        if self.num_ADARs == 2:
            self.RxPhase5 = self.Rx5Phase_set.get()+self.Rx5_cal
            self.RxPhase6 = self.Rx6Phase_set.get()+self.Rx6_cal
            self.RxPhase7 = self.Rx7Phase_set.get()+self.Rx7_cal
            self.RxPhase8 = self.Rx8Phase_set.get()+self.Rx8_cal
        ADAR_set_RxPhase(spi, self.ADDR1, self.num_ADARs, PhDelta, phase_step_size, self.RxPhase1, self.RxPhase2, self.RxPhase3, self.RxPhase4)
        ADAR_update_Rx(spi, self.ADDR1)
        if self.num_ADARs == 2:
            ADAR_set_RxPhase(spi, self.ADDR2, self.num_ADARs, PhDelta, phase_step_size, self.RxPhase5, self.RxPhase6, self.RxPhase7, self.RxPhase8)
            ADAR_update_Rx(spi, self.ADDR2)

root = Tk()
root.title("Phased Array Beamforming")
root.geometry("1200x600+0+0")
root.minsize(700,600)
app=App(root)
root.mainloop()
            