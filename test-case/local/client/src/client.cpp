#include "ros/ros.h"
#include "std_msgs/String.h"
#include "time.h"
#include "server_test/testmsg.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
static int count = 0;
struct timeval start;

ros::Publisher pub1;

void callbackClient(const server_test::testmsg::ConstPtr& msg)
{
  ROS_INFO("Tester 1 heard: [%s] [%i]", msg->name.c_str(), msg->age);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener", ros::init_options::AnonymousName);

  ros::NodeHandle node;
  pub1 = node.advertise<std_msgs::String>("tester1", 1000);
  ros::Subscriber sub1 = node.subscribe("tester1reply", 10000, callbackClient);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pub1.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
