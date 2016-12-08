#! /usr/bin/python
#coding=utf-8
# filename : rosdep.py
# author : Siteen
# update : 2016/12/7 

import os
import  xml.dom.minidom
import subprocess



startdir = '/catkin_install'
target = 'package.xml'
def scandir(startdir, target) :
    os.chdir(startdir)
    for obj in os.listdir(os.curdir) :
        if obj == target :
            document = xml.dom.minidom.parse(obj)
            content = document.documentElement
            tagname = content.getElementsByTagName('name')
            tagcontent = tagname[0]
            rosdepname = tagcontent.firstChild.data
            output = os.popen('rosdep install ' + rosdepname + ' -y')
            print output.read()
        if os.path.isdir(obj) :
            scandir(obj, target)
            os.chdir(os.pardir) #!!!

scandir(startdir,target)

