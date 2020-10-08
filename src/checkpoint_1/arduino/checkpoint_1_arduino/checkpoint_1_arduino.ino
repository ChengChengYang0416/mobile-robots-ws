#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;
int k;
int n;
int check = 0;

void messageCB(const std_msgs::Int32& toggle_msg){
  n = toggle_msg.data;
  k = n*2;
  check = 1;
}

std_msgs::Int32 str_msg;
std_msgs::Int32 str_msg2;
ros::Subscriber<std_msgs::Int32> sub("/numbers", &messageCB);
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher get_number("get_number", &str_msg2);


char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.advertise(get_number);
}

void loop()
{
  str_msg2.data = n;
  str_msg.data = k;
  get_number.publish(&str_msg2);
  if (check == 1){
    chatter.publish( &str_msg );
    check = 0;
  }
  
  nh.spinOnce();
  delay(1000);
}
