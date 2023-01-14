'''
toasters.py

    An example using bitmap to draw sprites on the
    M5Stack Core Display.

    youtube video: https://youtu.be/0uWsjKQmCpU

    spritesheet from CircuitPython_Flying_Toasters
    https://learn.adafruit.com/circuitpython-sprite-animation-pendant-mario-clouds-flying-toasters
'''

import random
import uasyncio
import ili9342c
from . import t1,t2,t3,t4,t5

TOASTERS = [t1, t2, t3, t4]
TOAST = [t5]


class toast():
    '''
    toast class to keep track of a sprites locaton and step
    '''
    def __init__(self, sprites, x, y):
        self.sprites = sprites
        self.steps = len(sprites)
        self.x = x
        self.y = y
        self.step = random.randint(0, self.steps-1)
        self.speed = random.randint(2, 5)

    def move(self):
        if self.x <= 0:
            self.speed = random.randint(2, 5)
            self.x = 320-64

        self.step += 1
        self.step %= self.steps
        self.x -= self.speed


async def run_asynch(dpy):
    '''
    Draw and move sprite
    '''

    # create toast spites in random positions
    sprites = [
        toast(TOASTERS, 320-64, 0),
        toast(TOAST, 320-64*2, 80),
        toast(TOASTERS, 320-64*4, 160)]

    # move and draw sprites
    while True:
        for man in sprites:
            bitmap = man.sprites[man.step]
            dpy.fill_rect(
                man.x+bitmap.WIDTH-man.speed,
                man.y,
                man.speed,
                bitmap.HEIGHT,
                ili9342c.BLACK)

            man.move()

            if man.x > 0:
                dpy.bitmap(bitmap, man.x, man.y)
            else:
                dpy.fill_rect(
                    0,
                    man.y,
                    bitmap.WIDTH,
                    bitmap.HEIGHT,
                    ili9342c.BLACK)

            await uasyncio.sleep(0.05)


