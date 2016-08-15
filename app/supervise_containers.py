#coding:utf-8
#!/usr/bin/python
# Filename: global_variable.py

import threading  
from dockerops import stopContainer
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
        u = models.Container(containerid = str(containerList[i]['Id']), createdtime = str(time.time()))
        db.session.add(u)
        db.session.commit()
            
        
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
                print c.containerid
                print c.createdtime
                if time.time()-float(c.createdtime) > 60:
                    stopContainer(c.containerid)
                    print "Stopped %s"%c.containerid
                    db.session.delete(c)
                    db.session.commit()
            time.sleep(10)
            #print "Stopping Stopping"
            
        mutex.release()