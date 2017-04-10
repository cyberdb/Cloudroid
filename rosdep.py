#! /usr/bin/python
#coding=utf-8
# filename : rosdep.py
# author : Siteen
# update : 2017/4/10
#This file is used for installation of ros package dependencies. These dependencies are listed in package.xml.

import os
import xml.dom.minidom
import subprocess
import shlex

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
            subprocess.call(shlex.split('bash /buildimages.sh '+rosdepname))
            #output = os.popen('/bin/bash ./buildimages.sh '+rosdepname)
            #print output.read()
        if os.path.isdir(obj) :
            scandir(obj, target)
            os.chdir(os.pardir) #!!!


scandir(startdir,target)
