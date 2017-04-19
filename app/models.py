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

from werkzeug import generate_password_hash, check_password_hash
from werkzeug import *
from app import db

class User(db.Model):
    __tablename__ = 'users'
    uid = db.Column(db.Integer, primary_key = True)
    firstname = db.Column(db.String(100))
    lastname = db.Column(db.String(100))
    email = db.Column(db.String(120), unique=True)
    passwdhash = db.Column(db.String(54))
     
    def __init__(self,firstname,lastname,email,password):
        self.firstname = firstname.title()
        self.lastname = lastname.title()
        self.email = email.lower()
        self.set_password(password)
    
    def is_authenticated(self):
        return True
 
    def is_active(self):
        return True

    def is_anonymous(self):
        return False
 
    def get_id(self):
        return unicode(str(self.uid))
       
    def set_password(self, password):
        self.passwdhash = generate_password_hash(password)
     
    def check_password(self, password):
        return check_password_hash(self.passwdhash, password)

class Image(db.Model):
    __tablename__ = 'images'
    uid = db.Column(db.Integer, primary_key = True)
    imagename = db.Column(db.String(100))
    uploadname = db.Column(db.String(100))
    comments = db.Column(db.String(100))
    uploadtime = db.Column(db.DateTime())
    uploaduser = db.Column(db.String(100))
    published_topics = db.Column(db.String(100))
    subscribed_topics = db.Column(db.String(100))
    advertised_services = db.Column(db.String(100))
    advertised_actions = db.Column(db.String(100))

    
class Service(db.Model):
    __tablename__ = 'services'
    uid = db.Column(db.Integer, primary_key = True)
    serviceid = db.Column(db.String(100))
    createdtime = db.Column(db.String(100))
    imagename =  db.Column(db.String(100))
    uploadname =  db.Column(db.String(100))
    username =  db.Column(db.String(100))
    firstcreatetime = db.Column(db.DateTime())

class ServerIP(db.Model):
    __tablename__ = 'serverip'
    uid = db.Column(db.Integer, primary_key = True)
    serverip = db.Column(db.String(100))
