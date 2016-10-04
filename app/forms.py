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

from flask.ext.wtf import Form
from wtforms import StringField, BooleanField, SelectField, TextAreaField, SubmitField, \
                    PasswordField, ValidationError, RadioField
from flask_wtf.file import FileField, FileAllowed, FileRequired
from wtforms.validators import DataRequired, Email, EqualTo
from app import db

class SignupForm(Form):
    firstname = StringField('First name', validators=[DataRequired("Please enter your first name.")])
    lastname = StringField('Last name', validators=[DataRequired("Please enter your last name.")])
    email = StringField('Email', validators=[DataRequired("Please enter your email address."),
                                             Email("Please enter your email address.")])
    password = PasswordField('Password', validators=[DataRequired("Please enter a password."),
                                                     EqualTo('confirm', message="Passwords must match")])
    confirm = PasswordField('Repeat Password')

class LoginForm(Form):
    email = StringField('Email', validators=[DataRequired("Please enter your email address."),
                                             Email("Please enter your email address.")])
    password = PasswordField('Password', validators=[DataRequired("Please enter a password.")])
    remember_me = BooleanField('remember_me', default=False)

class UploadForm(Form):
    do_action = StringField('Action')
    ros_file = FileField('Upload ROS File Name', validators=[FileRequired('The ROS package filename is required'), FileAllowed(['zip'], 'Zipped ROS installable file only!')])
    manifest_file = FileField('Upload Manifiest File Name', validators=[FileRequired('The mainifest filename is required'), FileAllowed(['json'], 'Json manifest file only!')])
    comments = StringField('Optional Comments')

class NodeForm(Form):
    nodename = StringField('Nodename', validators=[DataRequired("Please enter the Nodename.")])
    nodeip = StringField('Nodeip', validators=[DataRequired("Please enter the Nodeip.")])


class SetForm(Form):
    ip = StringField('IP', validators=[DataRequired("Please enter the IP of server.")])


        

