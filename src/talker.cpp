// talker.cpp
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include "ros_hello/DynamicConfig.h"

bool talker_test = false;

void callback(ros_hello::DynamicConfig &config, uint32_t level) {
  talker_test = config.talker_test;
}

// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// This tutorial demonstrates simple sending of messages over the ROS system.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker"); // name of the node
  ros::NodeHandle nh;  // node handle
  
  dynamic_reconfigure::Server<ros_hello::DynamicConfig> server;
  dynamic_reconfigure::Server<ros_hello::DynamicConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  // chatter - topic name, 100 - size of the message queue
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 100);  
  ros::Rate loop_rate(1); // 1Hz
  int count = 0;
  
  std::string hello_name;
  nh.param<std::string>("hello_name", hello_name, "world");
  
  while (ros::ok()) {  // return false if SIGINT is received (Ctrl-C)
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello " << hello_name << " " << count;
    if (talker_test) {
      ss << " TEST";
    }
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str()); // replacement for printf/cout
    chatter_pub.publish(msg);
    
    ros::spinOnce();
    
    loop_rate.sleep();
    count++;
  }
  return 0;
  
}
