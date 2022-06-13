#ifndef camera_opencv_H_
#define camera_opencv_H_
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>

cv::VideoCapture camera_init(int cam_id, int width, int height);

#endif // camera_opencv_H_