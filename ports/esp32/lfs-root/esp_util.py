#----------------
# Shell utilities
#----------------
import esp32
import os
import gc
from collections import namedtuple


class tasks:
    def __repr__(self):
        print(self.__call__())
        return ""

    def __call__(self):
        return esp32.tasks()
tasks = tasks()

MALLOC_CAP_EXEC           = 1<<0  # Memory must be able to run executable code
MALLOC_CAP_32BIT          = 1<<1  # Memory must allow for aligned 32-bit data accesses
MALLOC_CAP_8BIT           = 1<<2  # Memory must allow for 8/16/...-bit data accesses
MALLOC_CAP_DMA            = 1<<3  # Memory must be able to accessed by DMA
MALLOC_CAP_SPIRAM         = 1<<10 # Memory must be in SPI RAM
MALLOC_CAP_INTERNAL       = 1<<11 # Memory must be internal;
MALLOC_CAP_DEFAULT        = 1<<12 # Memory can be returned in a non-capability-specific memory allocation
MALLOC_CAP_IRAM_8BIT      = 1<<13 # Memory must be in IRAM and allow unaligned access

Desc = namedtuple('Desc', ["mask", "desc"])
Info = namedtuple('Info', ["total", "free", "largest", "minfree"])

class Heap_info:
    def __init__(self, print_all = False):
        self.print_all = print_all

    def __repr__(self):
        self.__call__()
        return ""

    def __call__(self):
        desc = [
            Desc(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT, "internal"),
            Desc(MALLOC_CAP_EXEC | MALLOC_CAP_32BIT, "iram"),
            Desc(MALLOC_CAP_SPIRAM, "spi"),
            # Desc(MALLOC_CAP_DEFAULT, "heap-all"),
        ]

        print("MEM HEAP       SIZE       USED  PCT       FREE     LOWEST LARGESTBLK")
        for d in desc:
            infoList = esp32.idf_heap_info(d.mask) or [(0, 0, 0, 0)]

            if self.print_all:
                infoList = map(lambda i: Info(*i), infoList)
            else:
                infoList = [Info(*[(lambda i,x: sum(x) if i != 2 else max(x))(i, x) for i, x in enumerate(zip(*infoList))])]

            for info in infoList:
                used = info.total - info.free
                print("%-8s %10d %10d %3d%% %10d %10d %10d" %
                      (d.desc, info.total, used,
                       used * 100 / info.total if info.total > 0 else 0,
                       info.free, info.minfree, info.largest))

        s = os.statvfs('//')
        print('\nDisk: {0} KB'.format((s[0]*s[3])/1024))

        gc.collect()
        F = gc.mem_free()
        A = gc.mem_alloc()
        T = F+A
        P = '{0:.2f}%'.format(F/T*100)
        print('Mpy Mem: Total:{0} Free:{1} ({2})'.format(T, F, P))

    def eoc(self):
        return

heap_info = Heap_info()
heap_all = Heap_info(True)

__all__ = [
    "heap_info",
    "heap_all",
    "tasks"
]
