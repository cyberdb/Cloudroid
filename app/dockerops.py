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
import docker
from app import db, models 
from app.models import *
from app.commonset import *
from datetime import datetime
from flask_login import current_user
from flask.templating import render_template
from flask.globals import session
from Tkinter import image_names


registry = '192.168.1.101:5000'

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
        docker_client = docker.Client(base_url = 'unix://var/run/docker.sock')
        registry_imagename = registry +'/'+ image_name
        generator = docker_client.build(path=".", rm = True, tag = registry_imagename, container_limits = container_limits)
        
        '''Check any error by inspecting the output of build()''' 
        for line in generator:
            try:
                stream_line = StreamLineBuildGenerator(line)
                if hasattr(stream_line, "error"):
                    error_string = 'Unable to generating docker image with name {}. \nReason: {}'.format(registry_imagename, stream_line.error)
                    logging.error(error_string)
                    return error_string
            except ValueError:
                ''' If we are not able to deserialize the received line as JSON object, just ignore it'''
                continue
        '''Push the images to private repository'''
        response_push = docker_client.push(registry_imagename, stream = True)
        '''Check any error by inspecting the output of push()'''
        for line in response_push:
            try:
                json_line = json.loads(line)
                if 'error' in json_line.keys():
                    error_string = 'Unable to push docker image with name {}. \nReason: {}'.format(registry_imagename, json_line['error'])
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


def getServicePort(image_name):
    logging.info('Starting a new services with image %s', image_name)
    
    try:
        client = docker.Client(base_url=DOCKER_PORT)
        image = registry+'/'+image_name
        com_cre_ser = 'sudo docker service create --replicas 1  --publish ' + ':9090 ' + image
        service_ps = os.popen(com_cre_ser).read().split('\n')
        service_id = service_ps[0]
        time.sleep(10)
        ser_ins = client.inspect_service(service_id)
        port = ser_ins['Endpoint']['Ports'][0]['PublishedPort']
        print port
        ser_ps_com = "sudo docker service ps " + service_id
        service_ps = os.popen(ser_ps_com).read().split('\n')
        if service_ps[1].split()[4] == 'Running':
            node = service_ps[1].split()[3]
        else:
            logging.error('Unable to create the service with image %s. \nReason: %s', service_ps[1].split()[6])
            return
        print node
        get_node = models.Node.query.filter_by(nodename = node).first()
        ip = get_node.nodeip
        '''
        if (node == 'micros-PowerEdge-R430'):
            ip = '192.168.1.121'
        elif (node == 'huben-PowerEdge-R430'):
            ip = '192.168.1.101'
        '''
    except Exception, e:
        logging.error('Unable to create the service with image %s. \nReason: %s', image_name, str(e))
        return
    logging.info('Store the service infomation to the db')
    try:
        imageinfo = models.Image.query.filter_by(imagename = image_name).first()
        uploadn = imageinfo.uploadname
        usern = imageinfo.uploaduser
        service_record = Service(serviceid = service_id, createdtime = str(time.time()), imagename = image_name, uploadname = uploadn, username = usern, firstcreatetime = datetime.now())
        db.session.add(service_record)
        db.session.commit()
    except Exception, e:
        logging.error('Failed to store the service info to the db. \nReason: %s', str(e))
        return

    return ip+':'+str(port)+" "+service_id


def serviceinfo():
    logging.info('The query of services info list')
    
    try:  
        services = models.Service.query.all()
        result = []
        part_line = {'serviceid':'default','imagename':'default','filename':'default','user':'default','createtime':'default'}
        for i in services:
            part_line['serviceid'] = i.serviceid
            part_line['imagename'] = i.imagename
            part_line['filename'] = i.uploadname
            part_line['user'] = i.username
            part_line['createtime'] = i.firstcreatetime
            result.append(part_line)
            part_line = {}
        
        return result
            
    except Exception, e:
        logging.error('Unable to list the services info. \nReason: %s', str(e))
        return
    


def removeServices(serviceid):
    logging.info('Remove the service %s', serviceid)
    
    try:
        docker_client = docker.Client(base_url=DOCKER_PORT)
        docker_client.remove_service(serviceid)
        remove_con = models.Service.query.all()
        for i in remove_con:
            if (i.serviceid == serviceid):
                db.session.delete(i)
                db.session.commit()
                break
               
    except Exception, e:
        logging.error('Unable to remove the service %s. \nReason: %s', serviceid, str(e))
        return


def deleteImage(image_name):
    logging.info('Delete the image %s', image_name)
    
    try:
        #docker_client = docker.Client(base_url=DOCKER_PORT)
        #docker_client.remove_image(image = image_name, force = True)
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
    