#coding:utf-8
from app import db, models
from app.models import ServerIP
import socket


DOCKER_PORT = 'unix://var/run/docker.sock'
default_url = 'http://127.0.0.1:5002'
port = '5002'
downloadFileName = None


def ipaddr():
    hostname = socket.getfqdn(socket.gethostname())
    ipaddr = socket.gethostbyname(hostname)
    return ipaddr



def url():
    serverip = models.ServerIP.query.first()
    if serverip.serverip == None:
        get_url = default_url
    else:
        get_url = serverip.serverip+':'+port
            
    if get_url.startswith('http'):
        get_url = get_url
    else:
        get_url = 'http://'+get_url
    return get_url