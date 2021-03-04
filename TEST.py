from threading import Thread
from threading import Event
import time

def fun1(e):
    print("fun1 initialized")
    while (True):
        e.wait()
        e.clear()
        print("fun1 is running now")

def fun2():
    print("fun2 initialized")
    while (True):
        time.sleep(1)
        event.set()
        

event = Event()
thread1 = Thread(target=fun1, args = (event,))
thread2 = Thread(target=fun2,)

thread1.start()
thread2.start()
