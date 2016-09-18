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

import zipfile, os, shutil, json, time, logging
from docker import Client
from werkzeug import secure_filename
from app import db, models 
from app.models import *
from app.commonset import *
from datetime import datetime
from flask_login import current_user
from flask.templating import render_template
from flask.globals import session
from Tkinter import image_names


current_milli_time = lambda: int(round(time.time() * 1000))


class StreamLineBuildGenerator(object):
    def __init__(self, json_data):
        self.__dict__ = json.loads(json_data)

def downloadFileBuild(downloadFileName):
    images = models.Image.query.filter_by(imagename = downloadFileName).first()
    subscribed_topics = StringToList(images.subscribed_topics)
    published_topics = StringToList(images.published_topics)
    advertised_services = StringToList(images.advertised_services)
    image_name = images.imagename
    
    
    '''Generating client-side proxy'''
    client_path='./client'
    download_path = './app/download'
    try:
        if os.path.exists(client_path):
            shutil.rmtree(client_path)
        if os.path.exists(download_path):
            shutil.rmtree(download_path)
        os.mkdir(download_path)
        unzip_cmd = 'unzip cloudproxy.zip -d ' + client_path
        os.system(unzip_cmd)
        client_url = url()
        client_launch = render_template('client.launch', published_topics = subscribed_topics, subscribed_topics = published_topics, 
                                        advertised_services = advertised_services, url = client_url, image_id = image_name)
        with open("./client/cloudproxy/launch/client.launch", "wb") as fh:
            fh.write(client_launch)
        zip_cmd = 'zip -r ./client/' + image_name +".zip " + "./client/cloudproxy/"
        os.system(zip_cmd)
        os.system("mv ./client/" + image_name + ".zip ./app/download/")
        
    except Exception, e:
        error_string = 'Unable to generating client proxy for image {}. \nReason: {}'.format(image_name, str(e))
        logging.error(error_string)
        return error_string
    return None
            
    
        
def uploadFile(ros_file, manifest_file, comments):  
    upload_path='./upload'
    logging.info('Uploading %s to path %s', ros_file.filename, upload_path)
    
          
    '''The internal unique id of a uploaded file will be the mill time since 1970''' 
    image_name = str(current_milli_time()) 
    save_filename = image_name + '.zip'
    
    '''Save file to the upload directory. Replace filename with the internal unique id'''      
    try:
        if not os.path.exists(upload_path):
            os.mkdir(upload_path)
             
        ros_file.save(os.path.join(upload_path, save_filename))
    except Exception, e:
        error_string = 'Unable to save file {} to path {}. \nReason: {}'.format(save_filename, upload_path, str(e))
        logging.error(error_string)
        return error_string
    
    logging.info('Unzipping uploaded file %s', ros_file.filename)
    
    '''Unzip the uploaded file''' 
    temp_path='./temp'
    try:
        if os.path.exists(temp_path):
            shutil.rmtree(temp_path)
        
        unzip_cmd = 'unzip ' + os.path.join(upload_path, save_filename) + ' -d ' + temp_path
        os.system(unzip_cmd)
    except Exception, e:
        error_string = 'Unzip file {} to path {} failure. \nReason: {}'.format(save_filename, temp_path, str(e))
        logging.error(error_string)
        return error_string 
    
    manifest = json.load(manifest_file)
    published_topics = manifest.get('published_topics')
    subscribed_topics = manifest.get('subscribed_topics')
    advertised_services = manifest.get('advertised_services')
    advertised_actions = manifest.get('advertised_actions')   
    start_cmds = manifest.get('start_cmds')
    mem_limit = manifest.get('mem_limit')
    memswap_limit = manifest.get('memswap_limit')
    cpushares = manifest.get('cpushares')
    cpusetcpus = manifest.get('cpusetcpus')
    container_limits = {'memory':int(mem_limit), 'memswap':int(memswap_limit), 'cpushares':int(cpushares), 'cpusetcpus':cpusetcpus}
  
    if start_cmds == None :
        error_string = 'Manifest file {} does not contain start_cmds. Generate docker image failed'.format(manifest_file.getFileName())
        logging.error(error_string)
        return error_string
    rosentry = render_template('ros_entry.sh', start_cmds = start_cmds)
    with open("./temp/ros_entry.sh", "wb") as fh:
        fh.write(rosentry)
            
    '''Building the docker image''' 
    logging.info('Generating docker image with tag %s', image_name)
    try:
        docker_client = Client(base_url=DOCKER_PORT)
        generator = docker_client.build(path=".", rm = True, tag = image_name, container_limits = container_limits)
        
        '''Check any error by inspecting the output of build()''' 
        for line in generator:
            try:
                stream_line = StreamLineBuildGenerator(line)
                if hasattr(stream_line, "error"):
                    error_string = 'Unable to generating docker image with name {}. \nReason: {}'.format(image_name, stream_line.error)
                    logging.error(error_string)
                    return error_string
            except ValueError:
                ''' If we are not able to deserialize the received line as JSON object, just ignore it'''
                continue
    except Exception, e:
        error_string = 'Unable to generating docker image with name {}. \nReason: {}'.format(image_name, str(e))
        logging.error(error_string)
        return error_string
        
    shutil.rmtree(temp_path)
     
    '''Insert a new record to the image table in the database'''  
    image_record = Image(imagename = image_name, uploadname = ros_file.filename, comments = comments, uploadtime = datetime.now(), uploaduser = current_user.email, published_topics = ListToString(published_topics), subscribed_topics = ListToString(subscribed_topics), advertised_services = ListToString(advertised_services), advertised_actions = ListToString(advertised_actions))
    db.session.add(image_record)
    db.session.commit()
    
    logging.info('Uploading file %s to robotcloud successfully!', ros_file.filename)
    
    return "None;"+image_name

def getContainerPort(image_name, cmd):
    logging.info('Starting a new container with image %s', image_name)
    
    try:
        '''Create container with image_name, Rosbridge port 9090 is mapped into a random port on the host machine'''
        docker_client = Client(base_url=DOCKER_PORT)
        config=docker_client.create_host_config(port_bindings={9090: None})
        container = docker_client.create_container(image=image_name, ports = [9090], host_config = config)
      
        '''Start the newly created container'''     
        container_id = container.get('Id')
        docker_client.start(container=container_id, port_bindings={9090: None})     
        
        '''Inspect state to ensure the container has been started'''
        response = docker_client.inspect_container(container_id)
        if (response == None or response.get('State') == None or not (response.get('State')).get('Running')):
            logging.error('Container %s status inspect failure. Start failed', container_id)
            return
               
    except Exception, e:
        logging.error('Unable to start the container with image %s. \nReason: %s', image_name, str(e))
        return
    
    '''Get port number on the host machine'''   
    logging.info('Finished starting a new container with id %s', container_id)
    response = docker_client.port(container_id, 9090)
    host_port = None
    if (response != None):
        host_port = response[0].get('HostPort')

    logging.info('New container is started. Websocket port on the host machine is %s.', host_port)
    logging.info('Store the container infomation to the db')
    try:
        imageinfo = models.Image.query.filter_by(imagename = image_name).first()
        uploadn = imageinfo.uploadname
        usern = imageinfo.uploaduser
        container_record = Container(containerid = container_id, createdtime = str(time.time()), imagename = image_name, uploadname = uploadn, username = usern, firstcreatetime = datetime.now(), containerstopped = False)
        db.session.add(container_record)
        db.session.commit()
    except Exception, e:
        logging.error('Failed to store the container info to the db. \nReason: %s', str(e))
        return
    return host_port+" "+container_id

def containerinfo():
    logging.info('The query of containers info list')
    
    try:  
        containers = models.Container.query.all()
        result = []
        part_line = {'containerid':'default','imagename':'default','filename':'default','user':'default','createtime':'default','containerstopped':'Stop'}
        for i in containers:
            part_line['containerid'] = i.containerid[0:12]
            part_line['imagename'] = i.imagename
            part_line['filename'] = i.uploadname
            part_line['user'] = i.username
            part_line['createtime'] = i.firstcreatetime        
            if i.containerstopped:
                part_line['containerstopped'] = 'Start'
            else:
                part_line['containerstopped'] = 'Stop'
            result.append(part_line)
            part_line = {}
        
        return result
            
    except Exception, e:
        logging.error('Unable to list the containers info. \nReason: %s', str(e))
        return
    

def listContainner():
    logging.info('The query of existing containers')
    
    try:
        docker_client = Client(base_url=DOCKER_PORT)
        return docker_client.containers(quiet=True, all=True)#Only running containers are shown by default
    
    except Exception, e:
        logging.error('Unable to list the containers. \nReason: %s', str(e))
        return

def stopContainer(container_id):
    logging.info('Stopping the container %s', container_id)
    
    try:
        docker_client = Client(base_url=DOCKER_PORT)
        docker_client.stop(container = container_id)
        stop_con = models.Container.query.all()
        for i in stop_con:
            if i.containerid[0:12] == container_id:
                container_id = i.containerid
                createdtime = i.createdtime
                image_name = i.imagename
                uploadn = i.uploadname
                usern = i.username
                firstcreatetime = i.firstcreatetime
                container_record = Container(containerid = container_id, createdtime = createdtime, imagename = image_name, uploadname = uploadn, username = usern, firstcreatetime = firstcreatetime, containerstopped = True)
                db.session.add(container_record)
                db.session.commit()
                db.session.delete(i)
                db.session.commit()
                break

               
    except Exception, e:
        logging.error('Unable to stop the container %s. \nReason: %s', container_id, str(e))
        return
     
def startContainer(container_id):
    logging.info('Starting the container %s', container_id)
    
    try:
        docker_client = Client(base_url=DOCKER_PORT)
        docker_client.start(container = container_id)
        start_con = models.Container.query.all()
        for i in start_con:
            if i.containerid[0:12] == container_id:
                container_id = i.containerid
                createdtime = i.createdtime
                image_name = i.imagename
                uploadn = i.uploadname
                usern = i.username
                firstcreatetime = i.firstcreatetime
                container_record = Container(containerid = container_id, createdtime = createdtime, imagename = image_name, uploadname = uploadn, username = usern, firstcreatetime = firstcreatetime, containerstopped = False)
                db.session.add(container_record)
                db.session.commit()
                db.session.delete(i)
                db.session.commit()
                break

               
    except Exception, e:
        logging.error('Unable to stop the container %s. \nReason: %s', container_id, str(e))
        return         
     

def removeContainer(container_id):
    logging.info('Remove the container %s', container_id)
    
    try:
        docker_client = Client(base_url=DOCKER_PORT)
        docker_client.remove_container(container = container_id, force = True)
        remove_con = models.Container.query.all()
        for i in remove_con:
            if (i.containerid[0:12] == container_id)or(i.containerid == container_id):
                db.session.delete(i)
                db.session.commit()
                break
               
    except Exception, e:
        if str(e).find('No such container:'):
            remove_con = models.Container.query.all()
            for i in remove_con:
                if (i.containerid[0:12] == container_id)or(i.containerid == container_id):
                    db.session.delete(i)
                    db.session.commit()
                    break
        logging.error('Unable to remove the container %s. \nReason: %s', container_id, str(e))
        return

def deleteImage(image_name):
    logging.info('Delete the image %s', image_name)
    
    try:
        docker_client = Client(base_url=DOCKER_PORT)
        docker_client.remove_image(image = image_name, force = True)
        image = models.Image.query.filter_by(imagename = image_name).first()
        db.session.delete(image)
        db.session.commit() 
    
        
    except Exception, e:
        print str(e)
        if (str(e).find('No such image:')):
            image = models.Image.query.filter_by(imagename = image_name).first()
            db.session.delete(image)
            db.session.commit()
            error_string = 'Unable to delete the image {}. \nReason: {}. Delete the record'.format(image_name, str(e)) 
        else:
            error_string = 'Unable to delete the image {}. \nReason: {}'.format(image_name, str(e))
        logging.error(error_string)
        return error_string
    
    return None   
    
def ListToString(lista):
    if lista.__len__() == 0:
        stringa = "None"
        return stringa
    else:
        stringa = ""
        for i in range(0,lista.__len__()):
            if (lista[i].split("#")).__len__() <= 1:
                lista[i] = str(lista[i]+"#")
        stringa = stringa.join(lista)
        return stringa

def StringToList(stringa):
    if stringa == "None":
        lista = []
        return lista
    else:
        lista = stringa.split('#')
        lista.pop()
        return lista
    