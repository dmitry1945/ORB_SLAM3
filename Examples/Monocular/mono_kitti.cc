/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "camera_opencv.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
bool use_cam = false;


// OpenCV port of 'LAPM' algorithm (Nayar89)
double modifiedLaplacian(const cv::Mat& src)
{
    cv::Mat M = (cv::Mat_<double>(3, 1) << -1, 2, -1);
    cv::Mat G = cv::getGaussianKernel(3, -1, CV_64F);

    cv::Mat Lx;
    cv::sepFilter2D(src, Lx, CV_64F, M, G);

    cv::Mat Ly;
    cv::sepFilter2D(src, Ly, CV_64F, G, M);

    cv::Mat FM = cv::abs(Lx) + cv::abs(Ly);

    double focusMeasure = cv::mean(FM).val[0];
    return focusMeasure;
}
// OpenCV port of 'LAPV' algorithm (Pech2000)
double varianceOfLaplacian(const cv::Mat& src)
{
    cv::Mat lap;
    cv::Laplacian(src, lap, CV_64F);

    cv::Scalar mu, sigma;
    cv::meanStdDev(lap, mu, sigma);

    double focusMeasure = sigma.val[0] * sigma.val[0];
    return focusMeasure;
}

// OpenCV port of 'GLVN' algorithm (Santos97)
double normalizedGraylevelVariance(const cv::Mat& src)
{
    cv::Scalar mu, sigma;
    cv::meanStdDev(src, mu, sigma);

    double focusMeasure = (sigma.val[0] * sigma.val[0]) / mu.val[0];
    return focusMeasure;
}

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    std::cout << "vstrImageFilenames - " << vstrImageFilenames.size() << endl;
    std::cout << "vTimestamps - " << vTimestamps.size() << endl;

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    double t_resize = 0.f;
    double t_track = 0.f;
    cv::Rect pic1(0, 0, 640, 480);

    cv::VideoCapture cam;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    if (use_cam)
    {
        cam = camera_init(0, 1280, 720);
        cv::FileStorage fSettings("c:/Work/Espressif/Tech/Arrow/SFM/SW/SFM_SLAM/Examples/StereoCam/intrinsics_mono.yml", cv::FileStorage::READ);
        fSettings["cameraMatrix"] >> cameraMatrix;
        fSettings["distCoeffs"] >> distCoeffs;
        fSettings.release();
    }

    cv::Mat im;

    for(int ni=0; ni<nImages; ni++)
    {
        if (use_cam)
        {
            cv::Mat img_color;
            cv::Mat img_dist;
            cam >> img_color;
            cvtColor(img_color, img_dist, cv::COLOR_BGR2GRAY);

            //std::vector<cv::Mat> channels;
            //cv::Mat hsv;
            //cv::cvtColor(img_color, hsv, cv::COLOR_BGR2HSV);
            //cv::split(hsv, channels);
            //img_dist = channels[0];
            //cv::Canny(img_dist, img_dist, 30, 95);

            if (0) // check coeffs
            {
                std::cout << "start check" << std::endl;
                for (size_t y = 0; y < img_dist.rows; y++)
                {
                    for (size_t x = 0; x < img_dist.cols; x++)
                    {
                        img_dist.at<uint8_t>(y, x) = 0;
                        img_dist.at<uint8_t>(100, x) = 255/2;
                        img_dist.at<uint8_t>(img_dist.rows - 100, x) = 255/2;
                        img_dist.at<uint8_t>(img_dist.rows / 2, x) = 255/2;
                    }
                    img_dist.at<uint8_t>(y, 100) = 255/2;
                    img_dist.at<uint8_t>(y, img_dist.cols -100) = 255/2;
                    img_dist.at<uint8_t>(y, img_dist.cols/2) = 255/2;
                }
                std::cout << "show check"  << std::endl;
                cv::imshow("check ", img_dist);
                //distCoeffs *= 0;
                cv::undistort(img_dist, im, cameraMatrix, distCoeffs);
                im += img_dist;
                cv::imshow("Result  undistort", im);
                cv::waitKey(100);
                //std::cout << "Blur detection: " << varianceOfLaplacian(im) << ", modifiedLaplacian = " << modifiedLaplacian(im)  << std::endl;
                continue;
            }
            else
            {
                distCoeffs *= 0;
                cv::undistort(img_dist, im, cameraMatrix, distCoeffs);
                cv::imshow("Result  undistort", im);
                cv::waitKey(1);
                //TestTrack(im);
                //continue;

            }
            double blur_im = varianceOfLaplacian(im);
            std::cout << "blur_im = " << blur_im << std::endl;

            if (blur_im < 2000)
            {
                // skip image
                continue;
            }
        }
        else
        {        // Read image from file
            im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            //im = im(pic1);
            std::cout << "file - " << vstrImageFilenames[ni] << std::endl;

        }
		double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vector<ORB_SLAM3::IMU::Point>(), vstrImageFilenames[ni]);
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        int active_points = 0;
        int del_points = 0;
        for (size_t i = 0; i < SLAM.mpTracker->mCurrentFrame->mvpMapPoints.size(); i++)
        {
            if (SLAM.mpTracker->mCurrentFrame->mvpMapPoints[i] != NULL)
            {
                if (SLAM.mpTracker->mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
                {
                    active_points++;
                }
            }
            else
            {
                del_points++;
            }
        }
        //std::cout << "Frames: " << ORB_SLAM3::Frame::frame_count << std::endl;
        //std::cout << "Maps: " << SLAM.mpAtlas->CountMaps() << 
        //    ", KFs: " << SLAM.mpAtlas->KeyFramesInMap() <<
        //    ", MPs: " << SLAM.mpAtlas->MapPointsInMap() <<
        //    ", MPs in Frame: " << SLAM.mpTracker->mCurrentFrame->mvpMapPoints.size() << 
        //    ", active_points: " << active_points <<
        //    ", del_points: " << del_points << std::endl;

        //cv::imshow("Processing image: ", im);
        //cv::waitKey(1);

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    while (true)
    {
        cv::waitKey(10);
    }
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}


#include <string>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
namespace fs = std::filesystem;


void LoadImages(const string& strPathToSequence, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
    //std::string path = strPathToSequence;
    //auto entry = fs::directory_iterator(path);
    //int entry_count = 2;
    //while (entry != fs::directory_iterator())
    //{

    //    stringstream ss;
    //    ss << setfill('0') << setw(6) << entry_count;
    //    std::string filename = strPathToSequence + "/frame_" + ss.str() + "_0.png";
    //    vstrImageFilenames.push_back(filename);
    //    vTimestamps.push_back(0.1 * (double)entry_count);
    //    //std::cout << filename << ", count " << entry_count << std::endl;
    //    entry++;
    //    entry_count++;
    //    // if (entry_count > 1600) break;
    //}

    std::string path = strPathToSequence;
    auto entry = fs::directory_iterator(path);
    int entry_count = 0;
    while (entry != fs::directory_iterator())
    {
        std::string filename = entry->path().string();
        vstrImageFilenames.push_back(filename);
        vTimestamps.push_back(0.1 * (double)entry_count);
        //std::cout << filename << ", count " << entry_count << std::endl;
        entry++;
        entry_count++;
        // if (entry_count > 1600) break;
    }
    std::sort(vstrImageFilenames.begin(), vstrImageFilenames.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs < rhs;
        });
    //for (const auto& file : vstrImageFilenames) {
    //    std::cout << file << '\n';
    //}
    //std::cout << "Count - " << entry_count << std::endl;
}
