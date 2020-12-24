#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

int action = 0;		// 0 -> go straight, 1 -> turn left, 2 -> turn right
std_msgs::Int32 start;

/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
	int flags = fcntl(0, F_GETFL, 0);
	fcntl(0, F_SETFL, flags | O_NONBLOCK);

	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0) {
		perror("tcsetattr()");
	}
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0) {
		perror("tcsetattr ICANON");
	}
	if (read(0, &buf, 1) < 0) {
		//perror ("read()");
	}
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0) {
		perror ("tcsetattr ~ICANON");
	}
	return (buf);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    for(int i = 0; i < count; i = i + 30) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		if (i == 150 || i == 120){
			if (scan->ranges[i] > 5.0){
				ROS_INFO(": [%f, %s]", degree, "6.0");
			}else {
				ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
			}

			if (scan->ranges[i] < 0.5){
				start.data = 68;
			}else{
				start.data = 65;
			}
		}
		if (i == 210 || i == 240){
			if (scan->ranges[i] > 5.0){
				ROS_INFO(": [%f, %s]", degree, "6.0");
			}else {
				ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
			}

			if (scan->ranges[i] < 0.5){
				start.data = 67;
			}else{
				if (start.data != 68){
					start.data = 65;
				}
			}
		}
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cool_node");
	ros::NodeHandle node_obj;
	ros::Publisher command_pub = node_obj.advertise<std_msgs::Int32>("/command_pub", 10);
	ros::Subscriber sub = node_obj.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
	ros::Rate loop_rate(100);
	start.data = 65;

	while (ros::ok()) {
#if 0
		int c = getch();
		start.data = 0;
		if (c != EOF) {
			switch (c) {
			case 65:    // key up 65 (^)
				start.data = 65;
				break;

			case 66:    // key down (v)
				start.data = 66;
				break;

			case 67:	// key right (>)
				start.data = 67;
				break;

			case 68:	// key left (<)
				start.data = 68;
				break;

			case 63:
				return 0;
				break;
			}
		}
#endif

		command_pub.publish(start);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
