#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdio.h>

void number_callback_output(const std_msgs::Int32::ConstPtr& msg){

  printf("message from arduino is \n%d\n", msg->data);
  printf("\nuser's input is \n");

}


int main(int argc, char **argv){

  ros::init(argc, argv, "cp_1_subscriber");
  ros::NodeHandle node_obj;
  ros::Subscriber number_subscriber_output = node_obj.subscribe("/chatter", 10, number_callback_output);

  printf("\nuser's input is \n");

  ros::spin();
  return 0;

}
