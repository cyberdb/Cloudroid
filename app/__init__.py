from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_login import LoginManager
import logging
        
def setlogger():
    logging.basicConfig(level=logging.DEBUG,
                format='%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s',
                datefmt='%d %b %Y %H:%M:%S',
                filename='robotcloud.log',
                filemode='w')

    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    formatter = logging.Formatter(fmt='[%(asctime)s][%(levelname)s] %(message)s', datefmt='%H:%M:%S')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)

setlogger()

 
app = Flask(__name__)
app.config.from_object('config')
db = SQLAlchemy(app)
lm = LoginManager()
lm.init_app(app)
lm.login_view = 'login'

from supervise_containers import abandoned_container
from supervise_containers import reset_container_db
initCon = reset_container_db()
print initCon
      
      
container_dropping_thread = abandoned_container() 
container_dropping_thread.start() 


from app import views, models











