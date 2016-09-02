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

void callbackTester1(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Tester 1 heard: [%s]", msg->data.c_str());

  std::stringstream ss;
  ss << msg->data.c_str();

  server_test::testmsg replymsg;
  replymsg.name = ss.str();
  replymsg.age = 10;
  pub1.publish(replymsg);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle node;
  pub1 = node.advertise<server_test::testmsg>("tester1reply", 1000);
  ros::Subscriber sub1 = node.subscribe("tester1", 10000, callbackTester1);

  ros::spin();
  return 0;
}
