from threading import Thread
import time

def threaded_stuff():
  cnt = 10	
  while cnt:
    print(cnt)
    cnt -= 1
    time.sleep(1)

if __name__ == "__main__":
  thread = Thread(target = threaded_stuff)
  thread.start()
  print("parallel")