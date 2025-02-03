import sysv_ipc as ipc
from PIL import Image
import numpy as np
import cv2

path = "/usr/share/ue5camsim.data"
key = ipc.ftok(path, 10)
shm = ipc.SharedMemory(key, 0, 0)

shm.attach(0,0)


while True:
    buf = shm.read()
    id = int.from_bytes(buf[:8], 'little')
    size_x = int.from_bytes(buf[8:12], 'little')
    size_y = int.from_bytes(buf[12:16], 'little')

    print()
    print("=================")
    print("Message ID: ", id)
    print("Size x: ", size_x)
    print("Size y: ", size_y)
    print("=================")
    print()

    img = Image.frombytes('RGB', (size_x, size_y), buf[16:])
    frame = np.array(img)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

shm.detach()
pass
