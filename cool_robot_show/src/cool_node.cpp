#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cool_node");
	ros::NodeHandle node_obj;
	ros::Publisher command_pub = node_obj.advertise<std_msgs::Int32>("/command_pub", 10);
	ros::Rate loop_rate(100);
	std_msgs::Int32 start;
	start.data = 0;

	while (ros::ok()) {
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

		command_pub.publish(start);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
