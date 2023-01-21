import os
import uasyncio as asyncio
import hw

btnPrev = [1, 1, 1]
btnHooks = [[], [], []]
btnCount = 0
async def task_btns():
    global btnCount
    while True:
        for i in range(3):
            cur = hw.btn[i]()
            if cur != btnPrev[i]:
                for handler in btnHooks[i]: handler(cur, i)
                btnPrev[i] = cur
        btnCount += 1
        await asyncio.sleep_ms(15)
    return

def addButtonHandler(i, handler):
    btnHooks[i].append(handler)

btnTask = None
def poll_buttons():
    global btnTask
    if btnTask: return

    btnTask = asyncio.create_task(task_btns())
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
