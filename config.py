import os
basedir = os.path.abspath(os.path.dirname(__file__))

CSRF_ENABLED = True
SECRET_KEY = 'robotcloud_secret_key'

SQLALCHEMY_DATABASE_URI = 'sqlite:///../database/robotcloud.db'
