from flask import Flask, request, redirect, url_for
from flask import render_template, flash, redirect, session, url_for, request, g
from flask_login import login_user, logout_user, current_user, login_required
from app import app, db, lm
from app.models import User
from app.dockerops import *

port = 9090

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
    posts = [{ 'body': 'Welcome to Robot Cloud!' }]
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
        action_error_msg = None
        param_do = form.do_action.data
        
        if (param_do == 'upload'):
            action_error_msg = uploadFile(form.ros_file.data, form.manifest_file.data, form.comments.data)
        
        succeed = (action_error_msg == None)
        return render_template('upload.html',form=form, action_error_msg = action_error_msg, succeed = succeed)    
         
    return render_template('upload.html',form=form, action_error_msg = None, succeed = False)

@app.route('/getinstance/<string:image_name>', methods=['GET'])
def get_instance(image_name):
    import socket
    
    hostname = socket.getfqdn(socket.gethostname(  ))
    ipaddr = socket.gethostbyname(hostname)
    return 'ws://' + ipaddr + ':' + str(getContainerPort(image_name, ''))
    
      

    
        

