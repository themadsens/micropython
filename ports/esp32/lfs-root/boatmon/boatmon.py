import uasyncio
import fm_board
import hw

testCnt = 0

async def main_coro():
    global testCnt
    while True:
        await uasyncio.sleep_ms(15)
        testCnt += 1

    return

def _btnChanged(state, n):
    print("Button changed %s now %s" % (n, state))

def start():
    print("-- BoatMon - start")
    for i in range(3):
        fm_board.addButtonHandler(i, _btnChanged)
    uasyncio.create_task(main_coro())
