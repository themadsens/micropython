

dpy = None
def open_dpy():
    from machine import Pin, SPI
    import ili9342c as ili9342c
    global dpy
    if dpy: 
        return dpy

    # initialize display
    try:
        spi = SPI(2, baudrate=60000000, sck=Pin(18), mosi=Pin(23))
        dpy = ili9342c.ILI9342C(spi, 320, 240,
                                reset       = Pin(33, Pin.OUT),
                                cs          = Pin(14, Pin.OUT),
                                dc          = Pin(27, Pin.OUT),
                                backlight   = Pin(32, Pin.OUT),
                                rotation    = 0,
                                buffer_size = 64*64*2)

        # enable display and clear screen
        dpy.init()
        dpy.fill(ili9342c.BLACK)
        return dpy

    finally:
        # shutdown spi
        if 'spi' in locals():
            spi.deinit()
    return dpy

def _init():
    from machine import Pin, Signal, I2C, UART

    global btn, i2c, gps1, gps2, led1, led2
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=42000)
    gps1 = UART(1, baudrate=9600, tx=17, rx=16)
    gps2 = UART(2, baudrate=9600, tx=5, rx=2)
    led1 = Signal(Pin(25, Pin.OUT))
    led2 = Signal(Pin(26, Pin.OUT))
    btn = [
        Pin(39, Pin.IN),
        Pin(38, Pin.IN),
        Pin(37, Pin.IN),
    ]

_init()
