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

#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include <RaspiCamCV.h>

#include <sstream>

using namespace cv;
using namespace std;
using namespace ros;


int main(int argc, char **argv)
{
	// initialise ROS node
	init(argc, argv, "image_pub");
	NodeHandle n;

	// initialise pi camera
	raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
	Mat pi_image;

	// check camera will open
	if (!Camera.open())
	{
		cout << "ERROR opening camera" << endl;
	}

	// initialise image transport publisher to send images
	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("snake_robot_im", 1);

	while (1)
	{

		// set opencv image as image from camera
		Camera.grab();
		Camera.retrieve(pi_image);

		// display pi camera image
		//imshow("pi image", pi_image);

		// convert pi image to a message
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pi_image).toImageMsg();

		int i = 0;
		Rate loop_rate(5);
		while (i < 1)
		{
				// publish message
				pub.publish(msg);
				spinOnce();
				loop_rate.sleep();
				i++;
		}

		// exit program if ESC key is pressed
		if (waitKey(30) == 27)
		{
			Camera.release();
			break;
		}
	}

	return 0;
}
