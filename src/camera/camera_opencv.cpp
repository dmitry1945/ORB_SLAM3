
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include<algorithm>
#include<fstream>
#include<iomanip>


#include "camera_opencv.h"


cv::VideoCapture camera_init(int cam_id, int width, int height)
{
	cv::VideoCapture cam(cam_id);

	if (!cam.set(cv::CAP_PROP_FRAME_WIDTH, width))
	{
		std::cout << "Invalid width! \n";
	}
	if (!cam.set(cv::CAP_PROP_FRAME_HEIGHT, height))
	{
		std::cout << "Invalid heigh! \n";
	}

	//if (!cam.set(cv::CAP_PROP_FPS, frame_fps))
	//{
	//	std::cout << "Invalid FPS! \n";
	//}
	if (!cam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')))
	{
		std::cout << "Invalid FPS! \n";
	}


	if (!cam.isOpened()) {
		std::cout << "Error: Stereo Cameras not found or there is some problem connecting them. Please check your cameras.\n";
		return cam;
	}
	return cam;
}