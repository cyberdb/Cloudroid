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
    
class SetForm(Form):
    ip = StringField('IP', validators=[DataRequired("Please enter the IP of server.")])
        
