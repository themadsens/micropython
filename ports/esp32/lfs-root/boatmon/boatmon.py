import uasyncio
import fm_board
import hw
import time
import machine
import ili9342c as dpydrv
import bmp280 as bmp
from ina219 import INA219
from micropyNMEA import MicropyNMEA
from font import vga2_16x32 as vga2, vga2_bold_16x16 as vga2sb
from micropython import const

LIGHTBLUE = const(0x38ff)
NAV = const(0)
BAT = const(1)
ENV = const(2)
SAT = const(3)

runCnt = 0
dpy, frame, CH, CHS, CW, DPYWID, DPYHEI = [None]*7
rtc = machine.RTC()
bmp280 = None
ina219 = None

gps = MicropyNMEA()
gpsIn = []
def handleGPS():
    global gpsIn, gps, ydnr
    if hw.gps96.any() <= 0:
        return

    chunk = hw.gps96.read()
    for b in chunk:
        gps.update(chr(b))
    lines = []
    try:
        lines = chunk.decode('ascii').split('\n')
    except:
        print("DECODE error", len(chunk), chunk)
    if len(lines) > 1:
        lines[0] = "".join(gpsIn) + lines[0]
        gpsIn.clear()
        while len(lines) > 1:
            line = lines.pop(0)
            if hw.btn[1]() == 0:
                print(": " + line)
            if gps.valid and (not ydnr.valid or time.ticks_ms() - ydnr.fix_time > 5000):
                hw.ydnr.write(line + "\n")
                if line[3:6] == "GLL": # $GPGLL Seems to be last in the batch
                    frame.updateFields(gps)

    if len(lines) > 0 and len(lines[0]) > 0:
        gpsIn.append(lines[0])

    if gps.timestamp != [0, 0, 0.0] and gps.date != (0, 0, 0):
        dt = rtc.datetime()
        DT = (gps.date[2] + 2000, gps.date[1], gps.date[0], dt[3]) + tuple(gps.timestamp[0:2]) + (int(gps.timestamp[2]),) + (dt[7],)
        if DT != dt:
            rtc.datetime(DT)

    return

ydnr = MicropyNMEA()
ydnrIn = []
def handleYDNR():
    global ydnrIn, ydnr
    if hw.ydnr.any() > 0:
        chunk = hw.ydnr.read()
        for b in chunk:
            ydnr.update(chr(b))

        lines = []
        try:
            lines = chunk.decode('ascii').split('\n')
        except:
            print("DECODE error", len(chunk), chunk)
        if len(lines) > 1:
            lines[0] = "".join(ydnrIn) + lines[0]
            ydnrIn.clear()
            while len(lines) > 1:
                line = lines.pop(0)
                if hw.btn[1]() == 0:
                    print("#%s%s%s" % ((ydnr.valid and "|" or "-"),
                                       (time.ticks_ms() - ydnr.fix_time < 5000 and "|" or "-"),
                                       line))
            if line[3:6] == "RMC" and ydnr.valid and time.ticks_ms() - ydnr.fix_time < 5000:
                frame.updateFields(ydnr)
        if len(lines) > 0 and len(lines[0]) > 0:
            ydnrIn.append(lines[0])

def handleI2C():
    if bmp280 and frame.curFrame == ENV:
        bmp280.normal_measure()
        frame.updateFields(None)
    elif ina219 and frame.curFrame == BAT:
        frame.updateFields(None)

lastCnt = 0
async def main_coro():
    global runCnt, lastCnt
    hw.gps96.read()
    hw.ydnr.read()
    while True:
        await uasyncio.sleep_ms(50)
        handleGPS()
        handleYDNR()
        if runCnt - lastCnt > 20:
            handleI2C()
            lastCnt = runCnt
        runCnt += 1
    return

sleeping = False
btnDown = [0,0,0]
def _btnChanged(state, btn):
    global sleeping, btnDown

    print("Button changed %s now %s frame %s" % (btn, state, frame.curFrame))
    if state == 0:
        btnDown[btn] = time.ticks_ms()
    if state == 0 and (btn == 0 or btn == 2):
        frame.shift(-1 if btn == 0 else 1)
    elif state == 1 and btn == 1 and time.ticks_ms() - btnDown[1] < 500:
        sleeping = not sleeping
        dpy.sleep_mode(sleeping)

    return

def start():
    print("-- BoatMon - start")
    for i in range(3):
        fm_board.addButtonHandler(i, _btnChanged)
    global dpy, frame, DPYWID, DPYHEI, CH, CHS, CW, bmp280, ina219
    try:
        bmp280 = bmp.BMP280(hw.i2c, use_case = bmp.BMP280_CASE_WEATHER)
        bmp280.oversample(bmp.BMP280_OS_HIGH)
        bmp280.temp_os = bmp.BMP280_TEMP_OS_2
        bmp280.press_os = bmp.BMP280_PRES_OS_2
        bmp280.standby = bmp.BMP280_STANDBY_250
        bmp280.iir = bmp.BMP280_IIR_FILTER_2
        bmp280.spi3w = bmp.BMP280_SPI3W_ON
        bmp280.power_mode = bmp.BMP280_POWER_NORMAL
    except OSError:
        print("Can not find bmp280 on i2c")
        bmp280 = None

    try:
        maxAmps = const(26.6667) # 50A / 75mV * 40mV
        shuntOhms = const(0.0015) # 75mV / 50A
        ina219 = INA219(shuntOhms, hw.i2c, max_expected_amps=maxAmps)
        ina219.configure(ina219.RANGE_16V, ina219.GAIN_1_40MV)
    except OSError:
        print("Can not find ina219 on i2c")
        ina219 = None

    dpy = hw.open_dpy()
    DPYWID = dpy.width()
    DPYHEI = dpy.height()
    CH = vga2.HEIGHT
    CHS = vga2sb.HEIGHT
    CW = vga2.WIDTH
    frame = Frame()
    uasyncio.create_task(main_coro())
    frame.draw()

frameTop = (b"Nav Bat Env St 12:42")
btnLine  = (b"    \x11          \x10    ")
class Frame:
    def __init__(self):
        self.curFrame = 0
        self.curLine = 0
        self.everyOther = False

    def shift(self, n):
        self.curFrame = (self.curFrame + n) % 4
        self.draw()

    def line(self, s, off = None):
        if off == None: off = CW // 2

        dpy.text(vga2, s, off, CH * self.curLine + CH*3//2, dpydrv.BLACK, dpydrv.WHITE)
        self.curLine += 1

    def field(self, s, y, x, off = None):
        if off == None: off = CW // 2 if y > 0 else 0
        if y > 0:
            dpy.text(vga2, s, off + CW * x, CH * (y-1) + CH*3//2, dpydrv.BLACK, dpydrv.WHITE)
        else:
            dpy.text(vga2, s, off + CW * x, 0, dpydrv.WHITE, dpydrv.BLUE)

    def drawNav(self):
        self.line(b"Sog/Cog:   . k    \xf8")
        self.line(b"Aws/Awa:   . m    \xf8")
        self.line(b"Aws24h:   . m   :  ")
        self.line(b"1h 4h:    . m   . m")
        self.line(b"Depth:    . m")
        X,Y = (DPYWID - 7*CW+10, DPYHEI - CH - CHS - 2)
        dpy.rect(X, Y, 7*CW-15, CH, dpydrv.BLACK)
        Y += CH
        dpy.line(X, Y-3, X + 7*CW-16, Y-CH+5, dpydrv.BLACK)

    def drawBat(self):
        self.line(b" -- Batteries --  ")
        self.line(b"House:    . V   . A")
        self.line(b"Start:  12.7V")
        self.line(b"Solar:  27.3V  3.1A")

    def drawEnv(self):
        self.line(b" -- Environment --  ")
        self.line(b"Air temp:      . \xf8C")
        self.line(b"Water temp:    . \xf8C")
        self.line(b"Barometer:     . Hp")

    def drawSat(self):
        self.line(b"        UT   :  :  ")
        self.line(b"        DoP        ")
        self.line(b"        Sat S/N #  ")
        self.line(b"        ___________")
        self.line(b"  \xf8  .      \xf8  .   ")

    def draw(self):
        self.curLine = 0
        dpy.fill(dpydrv.WHITE)
        dpy.fill_rect(0, 0, DPYWID, CH, dpydrv.BLUE)
        dpy.text(vga2, frameTop, 0, 0, dpydrv.WHITE, dpydrv.BLUE)
        dpy.text(vga2, frameTop[self.curFrame*4: self.curFrame*4 + 3], 
                 self.curFrame * CW * 4, 0, dpydrv.BLACK, LIGHTBLUE)
        H, M = time.localtime(time.time())[3:5]
        dpy.text(vga2, b"%02d:%02d" % (H, M), CW * 15, 0, dpydrv.WHITE, dpydrv.BLUE)
        dpy.text(vga2sb, btnLine, 0, DPYHEI - CHS, dpydrv.WHITE, dpydrv.BLUE)
        dpy.text(vga2sb, b" \xf0 ", DPYWID//2 - 3*CW//2, DPYHEI - CHS, dpydrv.WHITE, dpydrv.BLUE)

        if self.curFrame == NAV:
            self.drawNav()
        elif self.curFrame == BAT:
            self.drawBat()
        elif self.curFrame == ENV:
            self.drawEnv()
        elif self.curFrame == SAT:
            self.drawSat()
        return

    def updateFields(self, nmea):
        if nmea:
            DT = time.localtime()
            self.field("%02d:%02d" % (DT[3], DT[4]), 0, 15)
            sep = b":" if self.everyOther else b"\xf9"
            self.field(sep, 0, 17)
            self.everyOther = not self.everyOther
        if nmea and self.curFrame == NAV:
            self.field("%4.1f" % (nmea.speed[0]), 1, 9)
            self.field("%03d" % (nmea.course), 1, 15)
            if nmea.relative_wind_speed:
                self.field("%4.1f" % (nmea.relative_wind_speed), 2, 9)
                relspd = abs(nmea.relative_wind_speed)
                sign = " " if nmea.relative_wind_speed >= 0 else "-"
                self.field("%s%03d" % (sign, relspd), 2, 14)
            if nmea.depth_below_surface:
                self.field("%5.1f" % (nmea.depth_below_surface), 5, 7)

        elif ina219 and self.curFrame == BAT:
            self.field("%4.1f" % (ina219.voltage()), 2, 8)
            curr = ina219.current() / 1000 * 1.9
            self.field(("%5.2f" if curr < 9.9 else "%5.1f") % (curr,), 2, 13)
        elif bmp280 and self.curFrame == ENV:
            self.field("%4.1f" % (bmp280.temperature), 2, 13)
            self.field("%6.1f" % (bmp280.pressure / 100), 4, 11)
            if nmea and nmea.water_temperature:
                self.field("%4.1f" % (nmea.water_temperature), 3, 13)
        elif nmea and self.curFrame == SAT:
            self.field("%2d:%02d:%02d" % (nmea.timestamp[0], nmea.timestamp[1], nmea.timestamp[2]), 1, 11)
            if nmea.hdop >= 99:
                self.field("  ----  ", 2, 11)
            else:
                self.field("%4.1f %2dm" % (nmea.hdop, int(nmea.hdop * 5)), 2, 11)
            self.field("%02d" % (nmea.satellites_in_use), 3, 17)

            self.field("%2d" % (nmea.latitude[0]), 5, 0)
            self.field("%05.2f" % (nmea.latitude[1]), 5, 3)
            self.field("%1s" % (nmea.latitude[2]), 5, 8)
            self.field("%3d" % (nmea.longitude[0]), 5, 9)
            self.field("%05.2f" % (nmea.longitude[1]), 5, 13)
            self.field("%1s" % (nmea.longitude[2]), 5, 18)

            '''
            if nmea.satellite_data:
                buf = bm_utils.makeSatViewBM(nmea.satellite_data)
                dpy.blit_buffer(buf, 4, CH + 4, 128, 128)
            '''
        return

    def drawSatellites(self):
        return

    pass
