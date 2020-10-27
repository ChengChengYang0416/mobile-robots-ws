#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <stdio.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "motors_cmd_pub");
  ros::NodeHandle node_obj;
  ros::Publisher motor_L_pub = node_obj.advertise<std_msgs::Int32>("/motor_L", 10);
  ros::Publisher motor_R_pub = node_obj.advertise<std_msgs::Int32>("/motor_R", 10);
  ros::Rate loop_rate(100);

  std_msgs::Int32 motor_L_cmd;
  std_msgs::Int32 motor_R_cmd;

  while (ros::ok()){
    printf("Please enter PWM commands for left and right wheels : ");
    scanf("%d %d", &motor_L_cmd.data, &motor_R_cmd.data);

    motor_L_pub.publish(motor_L_cmd);
    motor_R_pub.publish(motor_R_cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
