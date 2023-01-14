import os
import asyncio
import hw

_btnPrev = [1, 1, 1]
_asyncHooks = []
_btnHooks = [[], [], []]
async def loop_coro():
    for i in range(3):
        cur = hw.btn[i]()
        if cur != _btnPrev[i]:
            for handler in _btnHooks[i]: handler[i](cur, i)
            _btnPrev[i] = cur

    for handler in _asyncHooks:
        handler()

    await asyncio.sleep_ms(10)

def addAsyncJob(handler):
    _asyncHooks.append(handler)

def addButtonHandler(i, handler):
    _btnHooks[i].append(handler)

_btnTask = None
def poll_buttons(_):
    global _btnTask
    if _btnTask: return

    _btnTask = asyncio.run(loop_coro())
    return

def rl_init():
    try: 
        with open("history.txt", "r") as hist:
            return hist.read().splitlines()
    except:
        return []
    return

hist_max = 10000
def rl_append(line):
    if not line: return
    
    try:
        with open("history.txt", "a" ) as hist: hist.write(line + "\n")
        if os.stat("history.txt")[6] > hist_max + 5000:
            out, size = [], 0
            for line in open("history.txt", "r").splitlines().reverse():
                out.append(line)
                size += line.encode().len()
                if size > hist_max:
                    break
            with open("history.txt", "w" ) as hist: hist.write("\n".join(out.reverse()) + "\n")
    except:
        print("Can not append to history.txt")
        raise
    return
