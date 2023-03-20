import pyspacemouse
import time

success = pyspacemouse.open()
if success:
    cnt = 0
    while 1:
        state = pyspacemouse.read()
        cnt += 1
        if cnt % 100000 ==0:
            print(state.roll, state.pitch, state.yaw)