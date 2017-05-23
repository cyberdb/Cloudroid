import json, websocket, logging, threading, time, argparse
import message_conversion as msgconv
import roslib, rospy
import actionlib_msgs.msg
import uuid
from importlib import import_module
from rosservice import get_service_type, get_service_args, call_service
from rostopic import get_topic_type
import urllib2
from rospkg.common import ResourceNotFound
import unicodedata


def get_remote_topic_type(topic_name, url):
    while True:
        try:
            ws = websocket.create_connection(url)
            break
        except Exception, e:
            rospy.loginfo('Create connection to Rosbridge server %s failed, retrying. Reason: %s', url, str(e))
        
        time.sleep(2)        
               
    try:
        # get topic type
        ws.send(json.dumps({
            'op': 'call_service',
            'service': '/rosapi/topic_type',
            'args': [topic_name]
        }))
        x = json.loads(ws.recv())
        
        assert x['service'] == '/rosapi/topic_type'
        
        ws.close()
        
        if x['result']:
            return x['values']['type']    
        else:
            return ""   
    except Exception, e:
        rospy.logerr('Get the type of topic %s from Rosbridge server %s failed. Reason: %s', topic_name, url, str(e))
        ws.close()
        return ""


def get_remote_service_info(service_name, url):
    while True:
        try:
            ws = websocket.create_connection(url)
            break
        except Exception, e:
            rospy.loginfo('Create connection to Rosbridge server %s failed, retrying. Reason: %s', url, str(e))
        
        time.sleep(2)        
               
    try:
        # get topic type
        ws.send(json.dumps({
            'op': 'call_service',
            'service': '/rosapi/service_type',
            'args': [service_name]
        }))
        x = json.loads(ws.recv())
        assert x['service'] == '/rosapi/service_type'
        ws.close()

        if x['result']:
            return x['values']['type'] 
        else:
            return ""
    except Exception, e:
        rospy.logerr('Get the type of service %s from Rosbridge server %s failed. Reason: %s', service_name, url, str(e))
        ws.close()
        return ""

def wait_topic_ready(topic_name, url):
    remote_topic_type = ""
    while remote_topic_type == "": 
        remote_topic_type = get_remote_topic_type(topic_name, url)
	#print remote_topic_type+"  remote_topic_type"
        if (remote_topic_type == ""):
            rospy.loginfo("Failed to get the remote type of topic %s. Retrying...", topic_name)
        time.sleep(1)
    
    local_topic_type = (None, None, None)
    while local_topic_type[0] == None:
        local_topic_type = get_topic_type(topic_name)
	#print str(local_topic_type)+"  local_topic_type"
        if (local_topic_type[0] == None):
            rospy.loginfo("Failed to get the local type of topic %s. Retrying...", topic_name)
        time.sleep(1)
    
    if remote_topic_type == local_topic_type[0]:
	#print str(local_topic_type)+"  equal"
        return local_topic_type[0]
    else:
        return None

def wait_service_ready(service_name, url):
    remote_service_type = ""
    while remote_service_type == "": 
        remote_service_type = get_remote_service_info(service_name, url)
        if (remote_service_type == ""):
            rospy.loginfo("Failed to get the remote type of service %s. Retrying...", service_name)
        time.sleep(1)
    
    local_service_type = None
    while local_service_type == None:
        local_service_type = get_service_type(service_name)
        if (local_service_type == None):
            rospy.loginfo("Failed to get the local type of service %s. Retrying...", service_name)
        time.sleep(1)
    
    if remote_service_type != local_service_type:
        return None, None

    local_service_args = None
    while local_service_args == None:
        local_service_args = get_service_args(service_name)
        if (local_service_args == None):
            rospy.loginfo("Failed to get the arguments list of service %s. Retrying...", service_name)
        time.sleep(1)
    
    service_args = local_service_args.split()

    return local_service_type, service_args

    
class SubscribedTopicProxy(threading.Thread):
    def __init__(self, topic_name, url, queue_size, test = False):
        threading.Thread.__init__(self)
        self.topic_name = topic_name
        self.url = url
        self.queue_size = queue_size
        self.test = test       
    
    def run(self):
        self.topic_type = wait_topic_ready(self.topic_name, self.url)
	#print str(self.topic_type)+"  self.topic_type"
        if not self.topic_type:
            rospy.logerr('Type of topic %s are not equal in the remote and local sides', self.topic_name)
            return
        
        topic_type_module, topic_type_name = tuple(self.topic_type.split('/'))
        try:       
            roslib.load_manifest(topic_type_module)
            msg_module = import_module(topic_type_module + '.msg')
            self.rostype = getattr(msg_module, topic_type_name)
                
            if self.test:
                self.publisher = rospy.Publisher(self.topic_name + '_rb', self.rostype, queue_size = self.queue_size)
            else: 
                self.publisher = rospy.Publisher(self.topic_name, self.rostype, queue_size = self.queue_size)
                                      
            self.ws = websocket.WebSocketApp(self.url, on_message = self.on_message, on_error = self.on_error, on_close = self.on_close, on_open = self.on_open)
            rospy.loginfo('Create connection to Rosbridge server %s for subscribed topic %s successfully', self.url, self.topic_name)
            self.ws.run_forever()
        except ResourceNotFound, e:
            rospy.logerr('Proxy for subscribed topic %s init falied. Reason: Could not find the required resource: %s', self.topic_name, str(e))
        except Exception, e:
            rospy.logerr('Proxy for subscribed topic %s init falied. Reason: %s', self.topic_name, str(e)) 
        
    def on_message(self, ws, message):
        data = json.loads(message)

        rosmsg = self.rostype()
        if not data or data['op'] != 'publish' or data['topic'] != self.topic_name:
            rospy.logerr('Failed to handle message on subscribed topic %s [%s]', self.topic_name, data)
            return
        
        data.pop('_format', None)      
        try:
            msgconv.populate_instance(data['msg'], rosmsg)
            self.publisher.publish(rosmsg)
        except Exception, e:
            rospy.logerr('Failed to publish message on topic %s. Reason: %s', self.topic_name, str(e))
    
    def on_error(self, ws, error):
        rospy.logerr('Websocket connection error on topic %s', self.topic_name)
    
    def on_close(self, ws):
        rospy.loginfo('Websocket connection closed on topic %s', self.topic_name)
        
        try:
            self.ws.send(json.dumps({
                'op': 'unsubscribe',
                'topic': self.topic_name,
            }))
        except Exception, e:
            rospy.logerr('Failed to send %s unsubscription request. Reason: %s', self.topic_name, str(e))
        
        rospy.loginfo('Unsubscribed topic %s from Rosbridge server %s', self.topic_name, url)
    
    def on_open(self, ws):
        rospy.loginfo('Websocket connected for topic %s', self.topic_name)
        
        try:
            self.ws.send(json.dumps({
                'op': 'subscribe',
                'topic': self.topic_name,
            }))
        except Exception, e:
            rospy.logerr('Failed to send %s subscription request. Reason: %s', self.topic_name, str(e))
            
        rospy.loginfo('Subscribed topic %s from Rosbridge server %s', self.topic_name, self.url)

class CallServiceProxy(threading.Thread):
    def __init__(self, service_name, url, queue_size, test = False):
        threading.Thread.__init__(self)
        self.service_name = service_name
        self.url = url
        self.queue_size = queue_size
        self.test = test       
        self.lock = threading.RLock()
        self.event_queue = {}
    
    def run(self):
        self.service_type, self.service_args = wait_service_ready(self.service_name, self.url)
        if not self.service_type:
            rospy.logerr('Type of service %s are not equal in the remote and local sides', self.service_type)
            return
        
        service_type_module, service_type_name = tuple(self.service_type.split('/'))
        try:       
            roslib.load_manifest(service_type_module)
            msg_module = import_module(service_type_module + '.srv')
            self.srvtype = getattr(msg_module, service_type_name)
            
            if self.test:
                self.caller = rospy.Service(self.service_name + '_rb', self.srvtype, self.callback)#, self.queue_size)
            else: 
                self.caller = rospy.Service(self.service_name, self.srvtype, self.callback)#, self.queue_size)
                                      
            self.ws = websocket.WebSocketApp(self.url, on_message = self.on_message, on_error = self.on_error, on_close = self.on_close, on_open = self.on_open)
            rospy.loginfo('Create connection to Rosbridge server %s for calling service %s successfully', self.url, self.service_name)
            self.ws.run_forever()
        except ResourceNotFound, e:
            rospy.logerr('Proxy for service %s init falied. Reason: Could not find the required resource: %s', self.service_name, str(e))
        except Exception, e:
            rospy.logerr('Proxy for service %s init falied. Reason: %s', self.service_name, str(e)) 
        

    def callback(self, req):
        if not self.advertised:
            return None

        sleepEvent = threading.Event()
        sleepEvent.clear()
        call_id = str(uuid.uuid1()).encode('ascii')

        with self.lock:
        # need lock to protect
            self.event_queue[call_id] = {
                'result': None,
                'event': sleepEvent
            }
	
	args_lists = [ getattr(req, it) for it in self.service_args ]
	
        try:
            self.ws.send(json.dumps({
                'op': 'call_service',
                'id': call_id,
                'service': self.service_name,
                'args': args_lists
            }))
        except Exception, e:
            rospy.logerr('Failed to call service on topic %s with %s. Reason: %s', self.service_name, self.service_args, str(e))

        sleepEvent.wait()

        with self.lock:
        # need lock to protect
            ret = self.event_queue[call_id].get('result', {})
            self.event_queue.pop(call_id)

        return ret


    def on_message(self, ws, message):
        data = json.loads(message)
        if not data or data['op'] != 'service_response' or data['service'] != self.service_name:
            rospy.logerr('Failed to handle message on service type %s [%s]', self.service, data)
            return
	
        try:
            # need lock to protect
            call_id = data.get('id').encode('ascii')
            value = data.get('values')
	    value_unicode = {}
	    for key, one_value in value.items():
		value_unicode[unicodedata.normalize('NFKD',key).encode('ascii','ignore')] = one_value

            with self.lock:
                self.event_queue[call_id]['result'] = value_unicode
                self.event_queue[call_id]['event'].set()

        except Exception, e:
            rospy.logerr('Failed to publish message on topic %s. Reason: %s', self.service_name, str(e))
	
    
    def on_error(self, ws, error):
        rospy.logerr('Websocket connection error on service %s', self.service_name)
    
    def on_close(self, ws):
        rospy.loginfo('Websocket connection closed on service %s', self.service_name)
        
        self.caller.shutdown('The remote websocket is closed')
        # try:
        #     self.ws.send(json.dumps({
        #         'op': 'unsubscribe',
        #         'topic': self.topic_name,
        #     }))
        # except Exception, e:
        #     rospy.logerr('Failed to send %s unsubscription request. Reason: %s', self.topic_name, str(e))
        
        #rospy.loginfo('Unsubscribed topic %s from Rosbridge server %s', self.topic_name, url)
    
    def on_open(self, ws):
        self.advertised = True
        rospy.loginfo('Websocket connected for service %s', self.service_name)

        
class PublishedTopicProxy(threading.Thread):
    def __init__(self, topic_name, url, test = False):
        threading.Thread.__init__(self)
        self.topic_name = topic_name
        self.url = url
        self.rb_topic_name = topic_name
        if test:
            self.rb_topic_name = self.rb_topic_name + '_rb'
        
        self.advertised = False
                
    def run(self):
        self.topic_type = wait_topic_ready(self.topic_name, self.url)
        if not self.topic_type:
            rospy.logerr('Type of topic %s are not equal in the remote and local sides', self.topic_name)
            return
                       
        topic_type_module, topic_type_name = tuple(self.topic_type.split('/'))
        try:      
            roslib.load_manifest(topic_type_module)
            msg_module = import_module(topic_type_module + '.msg')
            self.rostype = getattr(msg_module, topic_type_name)
                 
            self.subscriber = rospy.Subscriber(self.topic_name, self.rostype, self.callback)
                                      
            self.ws = websocket.WebSocketApp(self.url, on_message = self.on_message, on_error = self.on_error, on_close = self.on_close, on_open = self.on_open)
            rospy.loginfo('Create connection to Rosbridge server for published topic %s successfully', self.topic_name)
            self.ws.run_forever()
        except ResourceNotFound, e:
            rospy.logerr('Could not find the required resource %s', str(e))
        except Exception, e:
            rospy.logerr('Proxy for published topic %s init falied. Reason: %s', self.topic_name, str(e))
    
    def callback(self, rosmsg):
        if not self.advertised:
            return
        
        data = msgconv.extract_values(rosmsg)
        try:
            self.ws.send(json.dumps({
                'op': 'publish',
                'topic': self.rb_topic_name,
                'msg': data,
            }))
        except Exception, e:
            rospy.logerr('Failed to publish message on topic %s with %s. Reason: %s', self.topic_name, data, str(e))
        
    def on_message(self, ws, message):
        data = json.loads(message)
        rospy.logerr('Unexpected message on published topic %s [%s]', self.topic_name, data)
                
    def on_error(self, ws, error):
        rospy.logerr('Websocket connection error on topic %s', self.topic_name)
    
    def on_close(self, ws):
        rospy.loginfo('Websocket connection closed on topic %s', self.topic_name)
        
        try:
            self.ws.send(json.dumps({
                'op': 'unadertise',
                'topic': self.rb_topic_name,
            }))
        except Exception, e:
            rospy.logerr('Failed to send %s unadvertising request. Reason: %s', self.topic_name, str(e))
            return
        
        rospy.loginfo('Unadvertised topic %s to Rosbridge server %s', self.topic_name, url)
            
    def on_open(self, ws):
        rospy.loginfo('Websocket connected for topic %s', self.topic_name)
        
        try:
            self.ws.send(json.dumps({
                'op': 'advertise',
                'topic': self.rb_topic_name,
                'type': self.topic_type,
            }))
        except Exception, e:
            rospy.logerr('Failed to send %s advertising request. Reason: %s', self.topic_name, str(e))
            return
        
        rospy.loginfo('Advertised topic %s to Rosbridge server %s', self.topic_name, self.url)
        self.advertised = True


class keep_container_live(threading.Thread): #Connect to server every 20 seconds to tell the server this client is alive 
    def __init__(self, flask_url):  
        threading.Thread.__init__(self)   
        self.flask_url = flask_url  
   
    def run(self):   
        while True:  
            try:
        	response = urllib2.Request(self.flask_url) 
		containerMsg = urllib2.urlopen(response).read() 
            	time.sleep(20)
		print containerMsg
	    except Exception, e:
                rospy.logerr('Failed to connect to PING. Reason: %s', str(e))


def start_proxies():  
    parser = argparse.ArgumentParser()
    
    parser.add_argument('url', help='The url address of robot cloud.')
    
    parser.add_argument('--services', '--srv', nargs='+', help='Services provided by ROSBridge server')
    parser.add_argument('--published_topics', '--pub', nargs='+', help='Topics published to ROSBridge server')
    parser.add_argument('--subscribed_topics', '--sub', nargs='+', help='Topics subscribed from ROSBrdge server')
    parser.add_argument('--actions', nargs='+', help='Actions provided by ROSBridge server')
    parser.add_argument('-q', '--queue_size', type=int, default=1000, help='ROS message queue size on each topic')
    parser.add_argument('-t', '--test', action='store_true', default=False, help='Use if server and client are using the same ROS master for testing. Client service and topic names will have _ws appended.')
    parser.add_argument('-i', '--image_id', help='Unique image id on the robot cloud.', default="")
    
    args = parser.parse_args(rospy.myargv()[1:])
     
    if not args.url.startswith('http'):
        args.url = 'http://' + args.url
    httpurl = args.url + '/getinstance/' + args.image_id
    try:
        req = urllib2.Request(httpurl)
        url_and_containerid = urllib2.urlopen(req).read()
	wsurl = url_and_containerid.split(" ")[0]
	containerid = url_and_containerid.split(" ")[1]
    except Exception, e:
        rospy.logerr('Failed to get websocket address for image %s from %s. Reason: %s', args.image_id, httpurl, str(e))
        return
    
    flask_url = args.url + '/ping/' + str(containerid)
    print "&&&&&"+flask_url+"%%%%%%"
    proxy = keep_container_live(flask_url)
    proxy.start()


    rospy.init_node('cloud_proxy', anonymous=True)
        
    for topic_name in args.subscribed_topics:
        proxy = SubscribedTopicProxy(topic_name, wsurl, args.queue_size, args.test)
        proxy.start()
    
    for topic_name in args.published_topics:
        proxy = PublishedTopicProxy(topic_name, wsurl, args.test)
        proxy.start()
    
    if args.services != None:  
	for service_name in args.services:
    		proxy = CallServiceProxy(service_name, wsurl, args.test)
	        proxy.start()

    rospy.spin()

    
