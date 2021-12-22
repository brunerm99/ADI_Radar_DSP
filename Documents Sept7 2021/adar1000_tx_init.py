# this module will be imported in the into your flowgraph
# You can learn more about SPI and Rasp Pi at https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all#spi-on-pi

import time
import spidev

# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 0

# Enable SPI
spi = spidev.SpiDev()

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 500000
spi.mode = 0

# Rasp Pi Broadcom chip can only do 8 bit SPI writes.
# So the 24 bit SPI writes of the ADAR1000 need to be broken
# into 3 chunks.  Use the xfer2 command to keep CS low, until all 4 are there.

ADDR1 = 0x20
ADDR2 = 0x40

beamformers = [ADDR1, ADDR2]

spi.xfer2([0x00, 0x00, 0x81])  # resets the device.  Writing to ADDR0 of reg 0x00 also writes to all addresses on that CS line

for ADDR in beamformers:
    # Initialize the ADAR1000
    #spi.xfer2([ADDR, 0x00, 0x81])  # reset the device
    #spi.xfer2([ADDR, 0x00, 0x18])  # Sets SDO  pin to active (4 wire SPI)
    spi.xfer2([ADDR+0x04, 0x00, 0x55])  # Trims LDO to 1.8V
    spi.xfer2([ADDR, 0x38, 0x60])  # Bypasses beam and bias RAM (use SPI for gain/phase)
    spi.xfer2([ADDR, 0x2F, 0x7F])  # Enables all four Tx channels, the Tx Driver, Tx Vector Modulator and Tx VGA
    spi.xfer2([ADDR, 0x36, 0x16])  # Sets the TX VGA bias to [0010] and the TX vector modulator bias to [110]
    spi.xfer2([ADDR, 0x37, 0x06])  # Sets the Tx Driver bias to [110]
    #spi.xfer2([ADDR, 0x31, 0x42])  # Enables the whole Tx and enable TR_SPI
    time.sleep(0.1)

    spi.xfer2([ADDR, 0x31, 0xD0])  # Enables Tx and the switch, but puts part into Rx mode until we bias the PA's properly.  TR_SW_POS should be at 3.3V now
    spi.xfer2([ADDR, 0x30, 0x40])  # Sets bit 6 high (enables control of pins for external biasing)
    spi.xfer2([ADDR, 0x29, 0x39])  # Sets "on" PA gate bias to about -1.1V
    spi.xfer2([ADDR, 0x2A, 0x39])  # Sets "on" PA gate bias to about -1.1V
    spi.xfer2([ADDR, 0x2B, 0x39])  # Sets "on" PA gate bias to about -1.1V
    spi.xfer2([ADDR, 0x2C, 0x39])  # Sets "on" PA gate bias to about -1.1V
    spi.xfer2([ADDR, 0x46, 0x68])  # Sets "off" PA gate bias to about -2V
    spi.xfer2([ADDR, 0x47, 0x68])  # Sets "off" PA gate bias to about -2V
    spi.xfer2([ADDR, 0x48, 0x68])  # Sets "off" PA gate bias to about -2V
    spi.xfer2([ADDR, 0x49, 0x68])  # Sets "off" PA gate bias to about -2V
    spi.xfer2([ADDR, 0x31, 0x90])  # Changes Reg 0x31 to TX_EN=0.  PA_BIAS should be at -2V now.    

    # Write registers to set Rx1-4 to 45 deg and Max Gain
    spi.xfer2([ADDR, 0x1C, 0xFF])  # Sets Tx1 to max gain
    spi.xfer2([ADDR, 0x20, 0x36])  # Sets Tx1 I vector to positive and [10110]
    spi.xfer2([ADDR, 0x21, 0x36])  # Sets Tx1 Q vector to positive and [10110]
    spi.xfer2([ADDR, 0x1D, 0xFF])  # Sets Tx2 to max gain
    spi.xfer2([ADDR, 0x22, 0x36])  # Sets Tx2 I vector to positive and [10110]
    spi.xfer2([ADDR, 0x23, 0x36])  # Sets Tx2 Q vector to positive and [10110]
    spi.xfer2([ADDR, 0x1E, 0xFF])  # Sets Tx3 to max gain
    spi.xfer2([ADDR, 0x24, 0x36])  # Sets Tx3 I vector to positive and [10110]
    spi.xfer2([ADDR, 0x25, 0x36])  # Sets Tx3 Q vector to positive and [10110]
    spi.xfer2([ADDR, 0x1F, 0xFF])  # Sets Tx4 to max gain
    spi.xfer2([ADDR, 0x26, 0x36])  # Sets Tx4 I vector to positive and [10110]
    spi.xfer2([ADDR, 0x27, 0x36])  # Sets Tx4 Q vector to positive and [10110]
      
    spi.xfer2([ADDR, 0x28, 0x02])  # Loads Tx vectors from SPI.
    time.sleep(0.1)


#spi.xfer2([ADDR1, 0x31, 0xD2])  # Put part into Tx mode (bit 1=High).  TR_SW_POS should be 0V and PA bias should be -1.1V
#spi.xfer2([ADDR2, 0x31, 0xD2])  # Put part into Tx mode (bit 1=High).  TR_SW_POS should be 0V and PA bias should be -1.1V
#spi.xfer2([ADDR1, 0x31, 0x90])  # Put part into Rx mode (bit 1=High).  TR_SW_POS should be 3.3V and PA bias should be -2V
#spi.xfer2([ADDR2, 0x31, 0x90])  # Put part into Rx mode (bit 1=High).  TR_SW_POS should be 3.3V and PA bias should be -2V
    
    
# Read some registers (connect SDO to the Rpi cable)
#spi.xfer2([0x80+ADDR1, 0x03, 0x00])

# Power down the ADAR1000
#spi.xfer2([ADDR, 0x00, 0x81])  # reset the device
#spi.xfer2([ADDR, 0x00, 0x18])  # Sets SDO  pin to active (4 wire SPI)
#spi.xfer2([ADDR+0x04, 0x00, 0x55])  # Trims LDO to 1.8V


