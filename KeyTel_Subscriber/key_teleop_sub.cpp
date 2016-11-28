#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

using namespace std;


ros::Publisher serial_pub;


void velCallback(const geometry_msgs::Twist& key_vel)
{
	ros::NodeHandle r;
	serial_pub = r.advertise<std_msgs::UInt16>("commands_serial", 1000);
	ros::Rate loop_rate(10);

	if (key_vel.linear.x == 0.8)
	{
		cout << "FORWARDS" << endl;

		std_msgs::UInt16 msg;
		msg.data = 1;

		serial_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	if (key_vel.linear.x == -0.5)
	{
		cout << "BACKWARDS" << endl;

		std_msgs::UInt16 msg;
		msg.data = 2;

		serial_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	if (key_vel.angular.z == 1.0)
	{
		cout << "LEFT" << endl;

		std_msgs::UInt16 msg;
		msg.data = 3;

		serial_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	if (key_vel.angular.z == -1.0)
	{
		cout << "RIGHT" << endl;

		std_msgs::UInt16 msg;
		msg.data = 4;

		serial_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_sub");
	ros::NodeHandle n;

	ros::Subscriber c_sub = n.subscribe("key_vel", 1000, velCallback);
	ros::spin();
	return 0;
}
