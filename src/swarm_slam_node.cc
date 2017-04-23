/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float64.h>
#include <ORB_SLAM2/intVector.h>
#include <ORB_SLAM2/floatVector.h>
#include <ORB_SLAM2/float32Vector.h>
#include <ORB_SLAM2/floatMat.h>
#include <ORB_SLAM2/frame.h>
#include <ORB_SLAM2/bow.h>
#include <ORB_SLAM2/descriptor.h>
#include <ORB_SLAM2/feature.h>
#include <ORB_SLAM2/point2D.h>
#include <ORB_SLAM2/keypoint.h>


#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"
#include"../../../include/Frame.h"
#include"../../../Thirdparty/DBoW2/DBoW2/BowVector.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM),isIni(false),countFrame(0){}

    void GrabImage_0(const sensor_msgs::ImageConstPtr& msg);

    void GrabImage_1(const sensor_msgs::ImageConstPtr& msg);

    void GrabImage_2(const sensor_msgs::ImageConstPtr& msg);

    void GrabImage_3(const sensor_msgs::ImageConstPtr& msg);

    void insertKeyFrame(const ORB_SLAM2::frame &frame_msg);

    void FrameinfoToFrame(const ORB_SLAM2::frame &frame_msg, ORB_SLAM2::Frame* CurrentFramePtr);

    ORB_SLAM2::System* mpSLAM;

    ros::Publisher pub_;
    ros::Publisher pub_a_;
    bool isIni;
    unsigned int countFrame;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multiMono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true,false,4);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub_0 = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage_0,&igb);
    ros::Subscriber sub_1 = nodeHandler.subscribe("/camera0/image_raw", 1, &ImageGrabber::GrabImage_1,&igb);
    ros::Subscriber sub_2 = nodeHandler.subscribe("/camera1/image_raw", 1, &ImageGrabber::GrabImage_2,&igb);
    ros::Subscriber sub_3 = nodeHandler.subscribe("/camera2/image_raw", 1, &ImageGrabber::GrabImage_3,&igb);
    //publish the assistant tracking frames
    igb.pub_ = nodeHandler.advertise<ORB_SLAM2::floatMat>("framepose",10);
    igb.pub_a_ = nodeHandler.advertise<ORB_SLAM2::frame>("frameinfo",10);
    // ros::Publisher  pub = nodeHandler.advertise<ORB_SLAM2::floatVector>("/framepose",10);

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage_0(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ORB_SLAM2::Frame* CurrentKeyFramePtr;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,0,cv_ptr->header.stamp.toSec(),CurrentKeyFramePtr);
    if(Tcw.cols != 0)
    {
        isIni = true;
        countFrame++;
        ORB_SLAM2::floatMat tcw_msg;
        ORB_SLAM2::floatVector v;
        v.floatVector.push_back(Tcw.at<float>(0,0));
        v.floatVector.push_back(Tcw.at<float>(1,0));
        v.floatVector.push_back(Tcw.at<float>(2,0));

        tcw_msg.floatMat.push_back(v);
        tcw_msg.floatMat.push_back(v);
        tcw_msg.floatMat.push_back(v);
        // std::cout << "publish the tcw_msg.data = " << Tcw.at<float>(0,0) << std::endl;
        // std::cout << "Tcw = " << Tcw.cols << " x " << Tcw.rows << std::endl;
        pub_.publish(tcw_msg);
    }

}

void ImageGrabber::GrabImage_1(const sensor_msgs::ImageConstPtr& msg)
{
    if(!isIni) return;
    if(countFrame < 50) return;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    ORB_SLAM2::Frame* CurrentKeyFramePtr;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,1,cv_ptr->header.stamp.toSec(),CurrentKeyFramePtr);

}

void ImageGrabber::GrabImage_2(const sensor_msgs::ImageConstPtr& msg)
{
    if(!isIni) return;
    if(countFrame < 100) return;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    ORB_SLAM2::Frame* CurrentKeyFramePtr;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,2,cv_ptr->header.stamp.toSec(),CurrentKeyFramePtr);
}


void ImageGrabber::GrabImage_3(const sensor_msgs::ImageConstPtr& msg)
{
    if(!isIni) return;
    if(countFrame < 150) return;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ORB_SLAM2::Frame* CurrentKeyFramePtr;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,3,cv_ptr->header.stamp.toSec(),CurrentKeyFramePtr);
}
