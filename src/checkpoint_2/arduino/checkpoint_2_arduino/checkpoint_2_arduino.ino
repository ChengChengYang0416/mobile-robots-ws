#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

/* variables used for data processing */
int output_data;
int received_data;
int check = 0;

/* subscriber call back function */
void messageCB(const std_msgs::Int32& _msg){
  received_data = _msg.data;
  output_data = received_data*2;
  check = 1;
}

/* publisher and subscriber */
ros::NodeHandle  nh;
std_msgs::Int32 str_msg;
ros::Subscriber<std_msgs::Int32> sub("/numbers", &messageCB);
ros::Publisher chatter("chatter", &str_msg);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop()
{

  str_msg.data = output_data;
  if (check == 1){
    chatter.publish( &str_msg );
    check = 0;
  }
  
  nh.spinOnce();
  delay(100);
}
