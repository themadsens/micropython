import uasyncio
import fm_board
import hw
import time
import machine
import ili9342c as dpydrv
from micropyGPS import MicropyGPS
from font import vga2_16x32 as vga2, vga2_bold_16x16 as vga2sb
from micropython import const

LIGHTBLUE = const(0x38ff)
NAV = const(0)
BAT = const(1)
ENV = const(2)
SAT = const(3)

testCnt = 0
dpy, frame, CH, CHS, CW, DPYWID, DPYHEI = [None]*7
gps = MicropyGPS()
uartIn = []
rtc = machine.RTC()

async def main_coro():
    global testCnt, uartIn, gps
    while True:
        await uasyncio.sleep_ms(50)
        if hw.gps1.any() > 0:
            chunk = hw.gps1.read()
            for b in chunk:
                gps.update(chr(b))
            if gps.valid:
                dt = rtc.datetime()
                DT = (gps.date[2] + 2000, gps.date[1], gps.date[0], dt[3]) + tuple(gps.timestamp[0:2]) + (int(gps.timestamp[2]),) + (dt[7],)
                if DT != dt:
                    rtc.datetime(DT)

            lines = []
            try:
                lines = chunk.decode('ascii').split('\n')
            except:
                print("DECODE error", len(chunk), chunk)
            if len(lines) > 1:
                lines[0] = "".join(uartIn) + lines[0]
                uartIn.clear()
                while len(lines) > 1:
                    line = lines.pop(0)
                    if hw.btn[1]() == 0:
                        print(": " + line)
                if line[0:6] == "$GPGLL": #Seems to be last in the batch
                    frame.updateFields(gps)
            if len(lines[0]) > 0:
                uartIn.append(lines[0])
        testCnt += 1
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
    global dpy, frame, DPYWID, DPYHEI, CH, CHS, CW
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
        self.line(b"Sog/Cog:  4.2k 190\xf8")
        self.line(b"Aws/Awa:  6.1m 234\xf8")
        self.line(b"Aws24h: 13.2m 12:34")
        self.line(b"1h 4h:  11.2m 12.3m")
        self.line(b"Depth:  12.3m")
        X,Y = (DPYWID - 7*CW+10, DPYHEI - CH - CHS - 2)
        dpy.rect(X, Y, 7*CW-15, CH, dpydrv.BLACK)
        Y += CH
        dpy.line(X, Y-3, X + 7*CW-16, Y-CH+5, dpydrv.BLACK)

    def drawBat(self):
        self.line(b" -- Batteries --  ")
        self.line(b"House:  12.5V 17.4A")
        self.line(b"Start:  12.7V")
        self.line(b"Solar:  27.3V  3.1A")

    def drawEnv(self):
        self.line(b" -- Environment --  ")
        self.line(b"Air temp:    22.3\xf8C")
        self.line(b"Water temp:  17.4\xf8C")
        self.line(b"Barometer:    943Hp")

    def drawSat(self):
        self.curLine = 4
        self.line(b"56\xf844.33N 13\xf844.33E")

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

    def updateFields(self, gps):
        DT = time.localtime()
        self.field("%02d:%02d" % (DT[3], DT[4]), 0, 15)
        if self.curFrame == NAV:
            self.field("%4.1f" % (gps.speed[0]), 1, 9)
            self.field("%03d" % (gps.course), 1, 15)
        elif self.curFrame == BAT:
            pass
        elif self.curFrame == ENV:
            pass
        elif self.curFrame == SAT:
            self.field("%2d" % (gps.latitude[0]), 5, 0)
            self.field("%05.2f" % (gps.latitude[1]), 5, 3)
            self.field("%1s" % (gps.latitude[2]), 5, 8)
            self.field("%3d" % (gps.longitude[0]), 5, 9)
            self.field("%05.2f" % (gps.longitude[1]), 5, 13)
            self.field("%1s" % (gps.longitude[2]), 5, 18)
