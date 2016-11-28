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
#include <iostream>

using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
	// initialise pi camera
	raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
	Mat pi_image;

	// check camera will open
	if (!Camera.open())
	{
		cout << "ERROR opening camera" << endl;
	}

	while (1)
	{

		// set opencv image as image from camera
		Camera.grab();
		Camera.retrieve(pi_image);

		// display pi camera image
		//imshow("pi image", pi_image);

		// save image for streaming
		imwrite("/tmp/stream/pic.jpg", pi_image);

		// exit program if ESC key is pressed
		if (waitKey(30) == 27)
		{
			Camera.release();
			break;
		}
	}

	return 0;
}
