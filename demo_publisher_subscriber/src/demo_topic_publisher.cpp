#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdio.h>
int main(int argc, char **argv){

  ros::init(argc, argv, "demo_topic_publisher");
  ros::NodeHandle node_obj;
  ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 10);
  ros::Rate loop_rate(1);
  int number_count = 0;

  while (ros::ok())
  {

    std_msgs::Int32 msg;
    msg.data = number_count;
    ROS_INFO("%d", msg.data);
    number_publisher.publish(msg);
    ++number_count;
    number_count = number_count%20;

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;

}
