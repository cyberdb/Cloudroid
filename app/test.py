import zipfile, os, shutil, json, time, logging
from flask.templating import render_template

from datetime import datetime
from flask_login import current_user
from flask.templating import render_template
from flask.globals import session


manifest_file = '{\
   "subscribed_topics": [{"topic_name":"/tf","compression":"none"},{"topic_name":"/base_scan","compression":"compressed"}],\
   "published_topics": [{"topic_name":"/map","compression":"none"}],\
   "advertised_services": [],\
   "advertised_actions": [],\
   "start_cmds": ["rosrun gmapping slam_gmapping scan:=base_scan"],\
   "mem_limit": "1073741824",\
   "memswap_limit" : "1073741824",\
   "cpushares":"100",\
   "cpusetcpus":"0,1"\
}'
manifest = json.loads(manifest_file)
published_topics = manifest.get('published_topics')
print published_topics
subscribed_topics = manifest.get('subscribed_topics')
advertised_services = manifest.get('advertised_services')
advertised_actions = manifest.get('advertised_actions')   
start_cmds = manifest.get('start_cmds')
external_lib_paths = manifest.get('external_lib_paths', [])
mem_limit = manifest.get('mem_limit')
memswap_limit = manifest.get('memswap_limit')
cpushares = manifest.get('cpushares')
cpusetcpus = manifest.get('cpusetcpus')
container_limits = {'memory':int(mem_limit), 'memswap':int(memswap_limit), 'cpushares':int(cpushares), 'cpusetcpus':cpusetcpus}


#rosentry = render_template('/home/dev/ws/Cloudroid/app/ros_entry.sh', start_cmds = start_cmds, external_lib_paths=external_lib_paths)
#with open("./temp/ros_entry.sh", "wb") as fh:
#        fh.write(rosentry)
    
    












#compress_list = 

#topic_compress = render_template('compression.launch', compress_list = compress_list)
#with open("./temp/compression.launch", "wb") as fh:
#     fh.write(topic_compress)


