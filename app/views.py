# coding:utf-8
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, micROS Team
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of micROS-drt nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#from flask import Flask, request, redirect, url_for
from flask import render_template, flash, redirect, session, url_for, request, g
from flask_login import login_user, logout_user, current_user, login_required
from flask import jsonify,send_from_directory,abort
from app import app, db, lm
#from app.models import User
from app.dockerops import *
from app.supervise import *
import os, sys
#import socket
from app.commonset import *

reload(sys)
sys.setdefaultencoding('utf-8')




@lm.user_loader
def load_user(uid):
    return User.query.get(int(uid))

@app.before_request
def before_request():
    g.user = current_user
    
@app.route('/')
@app.route('/index')
@login_required
def index():
    posts = [{ 'body': 'Welcome to Cloudroid! Please set your server IP first.' }]
    return render_template('index.html',posts=posts)


@app.route('/signup', methods=['GET', 'POST'])
def signup():
    from forms import SignupForm
   
    form = SignupForm()
    if form.validate_on_submit():
        user = User.query.filter_by(email=form.email.data.lower()).first()
        if user is not None:
            form.email.errors.append("The Email address is already taken.")
            return render_template('signup.html', form=form)

        newuser = User(form.firstname.data,form.lastname.data,form.email.data,form.password.data)
        db.session.add(newuser)
        db.session.commit()

        session['email'] = newuser.email
        return redirect(url_for('login'))
   
    return render_template('signup.html', form=form)


@app.route('/login', methods=['GET', 'POST'])
def login():
    if g.user is not None and g.user.is_authenticated:
        return redirect(url_for('index'))

    from app.forms import LoginForm

    form = LoginForm()
    if form.validate_on_submit():
        session['remember_me'] = form.remember_me.data
        user = User.query.filter_by(email=form.email.data.lower()).first()
        if user and user.check_password(form.password.data):
            session['email'] = form.email.data
            login_user(user,remember=session['remember_me'])
            return redirect(url_for('index'))
        else:
            return render_template('login.html',form=form,failed_auth=True)
             
    return render_template('login.html',form=form)

@app.route('/logout')
def logout():
    logout_user()
    return redirect(url_for('index'))


@app.route('/upload', methods=['GET', 'POST'])
@login_required
def upload():
    from app.forms import UploadForm
    
    form = UploadForm()
    if form.validate_on_submit():
        action_msg = uploadFile(form.ros_file.data, form.manifest_file.data, form.comments.data)
        action_list = action_msg.split(";")
        if len(action_list) != 2:
            action_error_msg = action_list[0]
        else:
            action_error_msg = action_list[0]
            proxy_name = action_list[1]
        url_base = url()
        succeed = (action_error_msg == "None")
        if succeed == True:
            return render_template('download.html',download_url = "download/"+proxy_name)
        else:
            return render_template('upload.html',form=form, action_error_msg = action_error_msg, succeed = succeed)   
         
    return render_template('upload.html',form=form, action_error_msg = None, succeed = False)

@app.route('/download/<string:proxy_name>', methods=['GET'])
def download(proxy_name):
    from app.forms import UploadForm
    
    form = UploadForm()
    proxy_name_zip = proxy_name + ".zip"
    path1 = app.root_path+'/download'
    path2 = os.path.join(path1, proxy_name_zip)
    if os.path.exists(path2):
        return send_from_directory(path1,proxy_name_zip,as_attachment=True)
    action_error_msg = downloadFileBuild(proxy_name)
    if None == action_error_msg:
        return send_from_directory(path1,proxy_name_zip,as_attachment=True)
    else:
        return render_template('upload.html',form=form, action_error_msg = action_error_msg, succeed = False)


# @app.route('/nodes', methods=['GET'])
# def nodes():
#     from app import db, models
#     nodes = models.Node.query.all()
#     result = []
#     part_line = {'nodename':'default','nodeip':'default'}
#     for i in nodes:
#         part_line['nodename'] = i.nodename
#         part_line['nodeip'] = i.nodeip
#         result.append(part_line)
#         part_line = {}
#     print result
#     return render_template('nodes.html', nodetable=result)


# @app.route('/addnode', methods=['GET', 'POST'])
# def addnode():
#     from app import db, models
#     from app.forms import NodeForm
#     form = NodeForm()
#     nodes = models.Node.query.all()
#     result = []
#     part_line = {'nodename':'default','nodeip':'default'}
#     if form.validate_on_submit():
#         exist_node = 0
#         for i in nodes:
#             if form.nodename.data == i.nodename:
#                 db.session.delete(i)
#                 db.session.commit()
#                 u = models.Node(nodename=form.nodename.data, nodeip=form.nodeip.data)
#                 db.session.add(u)
#                 db.session.commit()
#                 part_line['nodename'] = form.nodename.data
#                 part_line['nodeip'] = form.nodeip.data
#                 print form.nodeip.data
#                 print part_line['nodeip']
#                 result.append(part_line)
#                 part_line = {}
#                 exist_node = 1
#                 print result
#             else:
#                 part_line['nodename'] = i.nodename
#                 part_line['nodeip'] = i.nodeip
#                 result.append(part_line)
#                 part_line = {}
#         if exist_node == 0:
#             u = models.Node(nodename=form.nodename.data, nodeip=form.nodeip.data)
#             db.session.add(u)
#             db.session.commit()
#             part_line['nodename'] = form.nodename.data
#             part_line['nodeip'] = form.nodeip.data
#             result.append(part_line)
#             return render_template('nodes.html', nodetable=result)
#         return render_template('nodes.html', nodetable=result)
#     else:
#         return render_template('addnode.html', form = form)


# @app.route('/delnode/<string:nodename>', methods=['GET'])
# def delnode(nodename):
#     from app import db, models
#     nodes = models.Node.query.all()
#     result = []
#     part_line = {'nodename': 'default', 'nodeip': 'default'}
#     for i in nodes:
#         if i.nodename == nodename:
#             db.session.delete(i)
#             db.session.commit()
#             continue
#         part_line['nodename'] = i.nodename
#         part_line['nodeip'] = i.nodeip
#         result.append(part_line)
#         part_line = {}
#     return render_template('nodes.html', nodetable=result)



@app.route('/images', methods=['GET'])
def images():
    from app import db, models         
    images = models.Image.query.all()
    result = []
    part_line = {'imagename':'default','uploadname':'default','uploaduser':'default','comments':'default'}
    for i in images:
        part_line['imagename'] = i.imagename
        part_line['uploadname'] = i.uploadname
        part_line['uploaduser'] = i.uploaduser
        part_line['comments'] = i.comments
        result.append(part_line)
        part_line = {}
    return render_template('images.html',imagetables = result)
  

@app.route('/idetailed/<string:image_name>', methods=['GET'])
def idetailed(image_name):
    from app import db, models 
    
    image = models.Image.query.filter_by(imagename = image_name).first()
    
    return render_template('idetailed.html',imagename = image.imagename, uploadname = image.uploadname, uploaduser = image.uploaduser, uploadtime = image.uploadtime, subscribed_topics = StringToList(image.subscribed_topics), published_topics = StringToList(image.published_topics), advertised_services = StringToList(image.advertised_services), advertised_actions = StringToList(image.advertised_actions), comments = image.comments)



@app.route('/delete/<string:image_name>', methods=['GET'])
def delete(image_name):
    
    error_msg = deleteImage(image_name)
    
    return render_template('delete.html', imagename = image_name, error_msg = error_msg)
    

@app.route('/services', methods=['GET'])
def services():
    servicei = serviceinfo()
    return render_template('service.html', servicetables = servicei)


@app.route('/remove/<string:serviceid>', methods=['GET'])
def remove(serviceid):
    removeServices(serviceid)
    servicei = serviceinfo()
    return render_template('service.html', servicetables = servicei)


@app.route('/getinstance/<string:image_name>', methods=['GET'])
def get_instance(image_name):
    return 'ws://' + str(getServicePort(image_name))
    
      
@app.route('/ping/<string:service_id>', methods=['GET'])
def ping(service_id):
     
    from app import db, models
    from models import Service
    finding = Service.query.filter_by(serviceid=service_id).first()
    if finding is not None:
        image_name = finding.imagename
        uploadn = finding.uploadname
        usern = finding.username
        firstcreatetime = finding.firstcreatetime
        u = Service(serviceid = service_id, createdtime = str(time.time()), imagename = image_name, uploadname = uploadn, username = usern, firstcreatetime = firstcreatetime)
        db.session.add(u) 
        db.session.commit() 
        db.session.delete(finding)
        db.session.commit()
    else:
        return "The service "+service_id+" has been removed!"
    
    return "There are existing service:"+service_id
        
