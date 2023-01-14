import sys, os, machine, uos, io, esp, esp32  # noqa
from upysh import *  # noqa
from esp_util import *  # noqa
from os import path
import uasyncio
import hw

def toasters():
    hw.open_dpy()
    from toasters.toasters import run_asynch
    uasyncio.create_task(run_asynch(hw.dpy))
    return

def aclock():
    from aclock import aclock
    uasyncio.create_task(aclock())

def defbtn():
    for i in range(3):
        hw.btn[i].irq(lambda pin, i_=i: print("Button %d is %d" % (i_, pin())))
    return

def defuart():
    hw.gps1.irq(lambda: 42)
    return

def _main():
    #from inisetup import install_async_hook
    #install_async_hook()
    import fm_board

    esp32.set_event_poll_hook(fm_board.async_hook)

    esp32.set_readline_hooks(fm_board.rl_init, fm_board.rl_append)

    print("Imported sys,os,machine,io,esp,esp32 for you\n\n")

    if path.exists('boatmon'):
        from boatmon import boatmon
        boatmon.start()

    return

_main()
