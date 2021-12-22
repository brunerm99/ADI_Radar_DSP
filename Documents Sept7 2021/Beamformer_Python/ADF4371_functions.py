# ADF4371_functions.py 

def ADF4371_init(spi):
    # Initialize the ADF4371 and program RF16 port to 9.5 GHz
    spi.xfer2([0x00, 0x00, 0x18])  # 
    spi.xfer2([0x00, 0x01, 0x00])  # 
    spi.xfer2([0x00, 0x20, 0x14])  # 
    spi.xfer2([0x00, 0x00, 0x18])  # 
    spi.xfer2([0x00, 0x01, 0x00])  # 
    spi.xfer2([0x00, 0x04, 0x00])  # 
    spi.xfer2([0x00, 0x05, 0x00])  # 
    spi.xfer2([0x00, 0x11, 0x00])  #

    spi.xfer2([0x00, 0x12, 0x40])  # 
    spi.xfer2([0x00, 0x14, 0x00])  # 
    spi.xfer2([0x00, 0x15, 0x00])  # 
    spi.xfer2([0x00, 0x16, 0x00])  # 
    spi.xfer2([0x00, 0x17, 0x01])  # 
    spi.xfer2([0x00, 0x18, 0x00])  # 
    spi.xfer2([0x00, 0x19, 0x01])  # 
    spi.xfer2([0x00, 0x1A, 0x00])  # 
    spi.xfer2([0x00, 0x1B, 0x00])  # 
    spi.xfer2([0x00, 0x1C, 0x00])  # 
    spi.xfer2([0x00, 0x1D, 0x00])  # 
    spi.xfer2([0x00, 0x1E, 0x48])  #

    spi.xfer2([0x00, 0x1F, 0x01])  # 
    spi.xfer2([0x00, 0x20, 0x14])  # 
    spi.xfer2([0x00, 0x21, 0x00])  # 
    spi.xfer2([0x00, 0x22, 0x00])  # 
    spi.xfer2([0x00, 0x23, 0x00])  # 
    spi.xfer2([0x00, 0x24, 0x80])  # 
    spi.xfer2([0x00, 0x25, 0x0B])  # 
    spi.xfer2([0x00, 0x26, 0x22])  # 
    spi.xfer2([0x00, 0x27, 0xCD])  # 
    spi.xfer2([0x00, 0x28, 0x83])  # 
    spi.xfer2([0x00, 0x2A, 0x00])  # 
    spi.xfer2([0x00, 0x2B, 0x00])  #

    spi.xfer2([0x00, 0x2C, 0x44])  # 
    spi.xfer2([0x00, 0x2D, 0x11])  # 
    spi.xfer2([0x00, 0x2E, 0x12])  # 
    spi.xfer2([0x00, 0x2F, 0x94])  # 
    spi.xfer2([0x00, 0x30, 0x2A])  # 
    spi.xfer2([0x00, 0x31, 0x02])  # 
    spi.xfer2([0x00, 0x32, 0x04])  # 
    spi.xfer2([0x00, 0x33, 0x22])  # 
    spi.xfer2([0x00, 0x34, 0x85])  # 
    spi.xfer2([0x00, 0x35, 0xFA])  # 
    spi.xfer2([0x00, 0x36, 0x30])  #

    spi.xfer2([0x00, 0x37, 0x00])  # 
    spi.xfer2([0x00, 0x38, 0x00])  # 
    spi.xfer2([0x00, 0x39, 0x07])  # 
    spi.xfer2([0x00, 0x3A, 0x55])  # 
    spi.xfer2([0x00, 0x3D, 0x00])  # 
    spi.xfer2([0x00, 0x3E, 0x0C])  # 
    spi.xfer2([0x00, 0x3F, 0x80])  # 
    spi.xfer2([0x00, 0x40, 0x50])  # 
    spi.xfer2([0x00, 0x41, 0x28])  # 
    spi.xfer2([0x00, 0x42, 0x00])  # 
    spi.xfer2([0x00, 0x43, 0x00])  #

    spi.xfer2([0x00, 0x44, 0x00])  # 
    spi.xfer2([0x00, 0x45, 0x00])  # 
    spi.xfer2([0x00, 0x46, 0x00])  # 
    spi.xfer2([0x00, 0x47, 0xC0])  # 
    spi.xfer2([0x00, 0x52, 0xF4])  # 
    spi.xfer2([0x00, 0x6C, 0x00])  # 
    spi.xfer2([0x00, 0x70, 0xE3])  # 
    spi.xfer2([0x00, 0x71, 0x60])  # 
    spi.xfer2([0x00, 0x72, 0x32])  # 
    spi.xfer2([0x00, 0x73, 0x00])  # 
    spi.xfer2([0x00, 0x10, 0x2F])  # 

