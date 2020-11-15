#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <stdio.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "cp3_start");
  ros::NodeHandle node_obj;
  ros::Publisher start_pub = node_obj.advertise<std_msgs::Int32>("/start_pub", 10);
  ros::Rate loop_rate(100);
  std_msgs::Int32 start;
  start.data = 0;

  while (ros::ok()){
    printf("Please enter 1 if you want to start. \n");
    printf("Enter 0 if you want to stop. \n");
    scanf("%d", &start.data);

    start_pub.publish(start);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
