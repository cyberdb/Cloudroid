#coding:utf-8
#!/usr/bin/python
# Filename: global_variable.py

import threading  
from dockerops import stopContainer
from dockerops import removeContainer
from dockerops import listContainner
import time
from app import db, models 
from models import Container

def reset_container_db():#Initialize the db with the information of running containers

    cons = Container.query.all()
    for j in cons:
        db.session.delete(j)
        db.session.commit() 
    containerList = listContainner()
    for i in xrange(len(containerList)):
        removeContainer(containerList[i]['Id'])
    return 'Container db has been initialized!'

mutex=threading.Lock() 
class abandoned_container(threading.Thread): #Find the abandoned container to remove
    def __init__(self):  
        threading.Thread.__init__(self)  
        
    def run(self):
        mutex.acquire()
        while True:
            
            Containers = models.Container.query.all() 
            for c in Containers:
                if time.time()-float(c.createdtime) > 600:
                    removeContainer(c.containerid)
                    print "Stopped %s"%c.containerid
            time.sleep(10)
            
        mutex.release()