#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <sstream>

using namespace cv;
using namespace std;
using namespace ros;

void Callback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		imshow("sub image", cv_bridge::toCvShare(msg, "bgr8")->image);
		waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("couldnt convert from %s to bgr8", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	init(argc, argv, "image_sub");
	NodeHandle n;

	cout << "entered" << endl;

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("snake_robot_im", 1, Callback);

	spin();

	return 0;
}
