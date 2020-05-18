// add_two_ints_server.cpp
#include "ros/ros.h"
#include "ros_hello/AddTwoInts.h"
#include "utils.h"

bool add(ros_hello::AddTwoInts::Request  &req,
         ros_hello::AddTwoInts::Response &res)
{
  res.sum = uadd((int)req.a, (int)req.b); // req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
