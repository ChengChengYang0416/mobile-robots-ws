#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdio.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "cp_1_publisher");
  ros::NodeHandle node_obj;
  ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 10);
  ros::Rate loop_rate(100);
  int num = 0;

  while (ros::ok()){

    scanf("%d", &num);

    std_msgs::Int32 msg;
    msg.data = num;
    number_publisher.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;

}
