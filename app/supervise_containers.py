#coding:utf-8
#!/usr/bin/python
# Filename: global_variable.py

containerIdList = []
lastChangeTimeList = []

import threading  
from dockerops import stopContainer
import time
class abandoned_container(threading.Thread): #Find the abandoned container to remove
    def __init__(self):  
        threading.Thread.__init__(self)  
        
   
   
   
   
    def run(self):   
        while True:
            print "containerIdList length"
            print len(containerIdList)
            for i in xrange(len(containerIdList)):
                print containerIdList[i]
                print "   "
                print lastChangeTimeList[i]
                if time.time()-lastChangeTimeList[i] > 60:
                    stopContainer(containerIdList[i])
                    print containerIdList[i]
                    for j in range(i,len(containerIdList)-1):
                        containerIdList[j] = containerIdList[j+1]
                        lastChangeTimeList[j] = lastChangeTimeList[j+1]
                    del containerIdList[-1]
                    del lastChangeTimeList[-1]
            time.sleep(10)
            print "Stopping Stopping"
