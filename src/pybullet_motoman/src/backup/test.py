import threading
import time

exitFlag = 0

class cameraThread(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        print("start to print love")
        printLove(self.name, 20, self.counter)
        print("end printing love")

def printLove(threadName, counter, delay):
    while counter:
        try:
            time.sleep(0.000000000000000000000000000000000001)      
            print("love you" + str(counter))
            counter -= 1
        except KeyboardInterrupt:
            threadName.exit()

sensorAgent = cameraThread(1, "camera Thread", 0)
sensorAgent.start()

for i in range(10000000):
    print(i)

exitFlag = 1


# import threading
# import time

# exitFlag = 0

# class myThread (threading.Thread):
#    def __init__(self, threadID, name, counter):
#       threading.Thread.__init__(self)
#       self.threadID = threadID
#       self.name = name
#       self.counter = counter
#    def run(self):
#       print "Starting " + self.name
#       print_time(self.name, 5, self.counter)
#       print "Exiting " + self.name

# def print_time(threadName, counter, delay):
#    while counter:
#       if exitFlag:
#          threadName.exit()
#       time.sleep(delay)
#       print "%s: %s" % (threadName, time.ctime(time.time()))
#       counter -= 1

# # Create new threads
# thread1 = myThread(1, "Thread-1", 1)

# # Start new Threads
# thread1.start()

# exitFlag = 1

# print "Exiting Main Thread"