import uasyncio
import fm_board
import hw
import time
import ili9342c as dpydrv
from font import vga2_16x32 as vga2, vga2_bold_16x16 as vga2sb

testCnt = 0
dpy, frame, CH, CHS, CW, DPYWID, DPYHEI = [None]*7

async def main_coro():
    global testCnt
    while True:
        await uasyncio.sleep_ms(15)
        testCnt += 1
    return

def _btnChanged(state, n):
    # print("Button changed %s now %s" % (n, state))
    if state == 0 and (n == 0 or n == 2):
        frame.shift(-1 if n == 0 else 1)
    elif state == 0 and n == 1 and testCnt == 0:
        frame.draw()
        uasyncio.create_task(main_coro())

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

frameTop = (b"Nav Bat Env If 12:42")
btnLine  = (b"    \x11          \x10    ")
class Frame:
    def __init__(self):
        self.curFrame = 0
        self.curLine = 0

    def shift(self, n):
        self.curFrame = (self.curFrame + n) % 4
        self.draw()


    def line(self, s):
        dpy.text(vga2, s, 0, CH * self.curLine + CH, dpydrv.BLACK, dpydrv.WHITE)
        self.curLine += 1

    def drawNav(self):
        self.line(b"Lat:      56\xf844.33'N")
        self.line(b"Lon:      13\xf844.33'E")
        self.line(b"Sog/Cog:  4.2kt 190\xf8")
        self.line(b"Aws/Awa:  6.1ms 234\xf8")
        self.line(b"Aws24h: 13.2ms 12:34")
        self.line(b"Depth:  12.3m")
        X,Y = (DPYWID - 7*CW+10, DPYHEI - CH - CHS - 2)
        dpy.rect(X, Y, 7*CW-15, CH, dpydrv.BLACK)
        Y += CH
        dpy.line(X, Y-3, X + 7*CW-16, Y-CH+5, dpydrv.BLACK)

    def drawBat(self):
        self.line(b"")
        self.line(b"  -- Batteries --  ")
        self.line(b"House:   12.5V 17.4A")
        self.line(b"Start:   12.7V")
        self.line(b"Solar:   27.3V  3.1A")

    def drawEnv(self):
        self.line(b"")
        self.line(b" -- Environment --  ")
        self.line(b"Air temp:     22.3\xf8C")
        self.line(b"Water temp:   17.4\xf8C")
        self.line(b"Humidity:        47%")
        self.line(b"Barometer:     943Hp")

    def drawIf(self):
        self.line(b"If")

    def draw(self):
        self.curLine = 0
        dpy.fill(dpydrv.WHITE)
        dpy.fill_rect(0, 0, DPYWID, CH, dpydrv.BLUE)
        dpy.text(vga2, frameTop, 0, 0, dpydrv.WHITE, dpydrv.BLUE)
        dpy.text(vga2, frameTop[self.curFrame*4: self.curFrame*4 + 3], 
                 self.curFrame * CW * 4, 0, dpydrv.BLACK, dpydrv.BLUE)
        dpy.hline(self.curFrame * CW * 4, CH - 3, CW * 3, dpydrv.BLACK)
        dpy.hline(self.curFrame * CW * 4, CH - 2, CW * 3, dpydrv.BLACK)
        dpy.hline(self.curFrame * CW * 4, CH - 1, CW * 3, dpydrv.BLACK)
        H, M = time.localtime(time.time())[3:5]
        dpy.text(vga2, b"%02d:%02d" % (H, M), CW * 15, 0, dpydrv.WHITE, dpydrv.BLUE)
        dpy.text(vga2sb, btnLine, 0, DPYHEI - CHS, dpydrv.WHITE, dpydrv.BLUE)
        dpy.text(vga2sb, b" \xf0 ", DPYWID//2 - 3*CW//2, DPYHEI - CHS, dpydrv.WHITE, dpydrv.BLUE)

        if self.curFrame == 0:
            self.drawNav()
        elif self.curFrame == 1:
            self.drawBat()
        elif self.curFrame == 2:
            self.drawEnv()
        elif self.curFrame == 3:
            self.drawIf()

