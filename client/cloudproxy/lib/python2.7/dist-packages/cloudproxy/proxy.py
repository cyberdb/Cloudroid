import json, websocket, logging, threading, time, argparse
import message_conversion as msgconv
import roslib, rospy
import actionlib_msgs.msg
from importlib import import_module
from rostopic import get_topic_type
import urllib2
from rospkg.common import ResourceNotFound

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

def wait_topic_ready(topic_name, url):
    remote_topic_type = ""
    while remote_topic_type == "": 
        remote_topic_type = get_remote_topic_type(topic_name, url)
        if (remote_topic_type == ""):
            rospy.loginfo("Failed to get the remote type of topic %s. Retrying...", topic_name)
        time.sleep(1)
    
    local_topic_type = None
    while local_topic_type == None:
        local_topic_type, _, _ = get_topic_type(topic_name)
        if (local_topic_type == None):
            rospy.loginfo("Failed to get the local type of topic %s. Retrying...", topic_name)
        time.sleep(1)
    
    if remote_topic_type == local_topic_type:
        return local_topic_type
    else:
        return None
    
class SubscribedTopicProxy(threading.Thread):
    def __init__(self, topic_name, url, queue_size, test = False):
        threading.Thread.__init__(self)
        self.topic_name = topic_name
        self.url = url
        self.queue_size = queue_size
        self.test = test       
    
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
        wsurl = urllib2.urlopen(req).read()  
    except Exception, e:
        rospy.logerr('Failed to get websocket address for image %s from %s. Reason: %s', args.image_id, httpurl, str(e))
        return
    
    rospy.init_node('cloud_proxy', anonymous=True)
        
    for topic_name in args.subscribed_topics:
        proxy = SubscribedTopicProxy(topic_name, wsurl, args.queue_size, args.test)
        proxy.start()
    
    for topic_name in args.published_topics:
        proxy = PublishedTopicProxy(topic_name, wsurl, args.test)
        proxy.start()
    
    rospy.spin()
    