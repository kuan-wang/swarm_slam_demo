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
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void insertKeyFrame(const ORB_SLAM2::frame &frame_msg);

    void PublishFrameinfo(ORB_SLAM2::Frame* CurrentKeyFramePtr);

    void FrameinfoToFrame(ORB_SLAM2::frame &frame_msg);

    ORB_SLAM2::System* mpSLAM;

    ros::Publisher pub_;
    ros::Publisher pub_a_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "multiMono_a");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, false, 2);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera1/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub_a = nodeHandler.subscribe("/frameinfo0", 1, &ImageGrabber::insertKeyFrame,&igb);
    //assistant tracking thread
    //ros::Subscriber sub_a = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage_a,&igb);
    //publish the assistant tracking frames
    igb.pub_ = nodeHandler.advertise<ORB_SLAM2::floatMat>("framepose1",10);
    igb.pub_a_ = nodeHandler.advertise<ORB_SLAM2::frame>("frameinfo1",10);
    // ros::Publisher  pub = nodeHandler.advertise<ORB_SLAM2::floatVector>("/framepose",10);

    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
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
        if(CurrentKeyFramePtr) PublishFrameinfo(CurrentKeyFramePtr);
        pub_.publish(tcw_msg);
    }

}


void ImageGrabber::insertKeyFrame(const ORB_SLAM2::frame &frame_msg)
{
    // std::cout << "get the frame_msg" << '\n';
    // // mpSLAM->mpTracker_a->SetAssistant(false);
    // std::cout << "now the mCurrentFrame is" << '\n';
    // mpSLAM->mpTracker_a->mCurrentFrame.Show();
    // ORB_SLAM2::Frame* CurrentKeyFramePtr = new ORB_SLAM2::Frame(mpSLAM->mpTracker->mCurrentFrame);
    // FrameinfoToFrame(frame_msg,CurrentKeyFramePtr);
    // mpSLAM->mpTracker_a->mCurrentFrame = *CurrentKeyFramePtr;
    // std::cout << "afrer FrameinfoToFrame now the mCurrentFrame is" << '\n';
    // mpSLAM->mpTracker_a->mCurrentFrame.Show();
    // std::cout << "start track sharing" << '\n';
    // mpSLAM->mpTracker_a->TrackForNewFrameFromSharing();
    // std::cout << "finished track sharing" << '\n';

    // mpSLAM->mpTracker_a->SetAssistant(true);

}

void ImageGrabber::PublishFrameinfo(ORB_SLAM2::Frame* CurrentKeyFramePtr)
{
    std::cout << "mTcw = " << CurrentKeyFramePtr->mTcw << std::endl;

    ORB_SLAM2::frame frame_msg;
    frame_msg.mTimeStamp = CurrentKeyFramePtr->mTimeStamp;
    frame_msg.N = CurrentKeyFramePtr->N;

    for(std::vector<cv::KeyPoint>::const_iterator vit=CurrentKeyFramePtr->mvKeysUn.begin(), vend=CurrentKeyFramePtr->mvKeysUn.end(); vit!=vend; vit++)
    {
        ORB_SLAM2::keypoint keypoint_msg;
        keypoint_msg.x = vit->pt.x;
        keypoint_msg.y = vit->pt.y;
        keypoint_msg.angle = vit->angle;
        keypoint_msg.class_id = vit->class_id;
        keypoint_msg.octave = vit->octave;
        keypoint_msg.response = vit->response;
        keypoint_msg.size = vit->size;
        frame_msg.mvKeysUn.push_back(keypoint_msg);
    }

    for(DBoW2::BowVector::const_iterator vit=CurrentKeyFramePtr->mBowVec.begin(), vend=CurrentKeyFramePtr->mBowVec.end(); vit!=vend; vit++)
    {
        ORB_SLAM2::bow bow_msg;
        bow_msg.WordId = vit->first;
        bow_msg.WordValue = vit->second;
        frame_msg.mBowVec.push_back(bow_msg);
    }

    for(std::map<unsigned int,std::vector<unsigned int>>::const_iterator vit=CurrentKeyFramePtr->mFeatVec.begin(), vend=CurrentKeyFramePtr->mFeatVec.end(); vit!=vend; vit++)
    {
        ORB_SLAM2::feature feature_msg;
        feature_msg.NodeId = vit->first;
        feature_msg.i_feature = vit->second;
        frame_msg.mFeatVec.push_back(feature_msg);
    }

    for (int i = 0; i < CurrentKeyFramePtr->mDescriptors.rows; i++) {
        ORB_SLAM2::descriptor descriptor_msg;
        std::vector<uchar> v;
        for (int j = 0; j < CurrentKeyFramePtr->mDescriptors.cols; j++) {
            v.push_back(CurrentKeyFramePtr->mDescriptors.at<uchar>(i,j));
        }
        // std::cout << "mDescriptors[" << i << "]:"<< CurrentKeyFramePtr->mDescriptors.row(i).clone()<< std::endl;
        descriptor_msg.descriptor = v;
        frame_msg.mDescriptors.push_back(descriptor_msg);
    }

    std::vector<uchar> ucharv;
    for (size_t i = 0; i < CurrentKeyFramePtr->mvbOutlier.size(); i++) {
        if(CurrentKeyFramePtr->mvbOutlier[i]) ucharv.push_back(1);
        else ucharv.push_back(0);
    }
    frame_msg.mvbOutlier = ucharv;

    for (int i = 0; i < CurrentKeyFramePtr->mTcw.rows; i++) {
        ORB_SLAM2::float32Vector float32v_msg;
        for (int j = 0; j < CurrentKeyFramePtr->mTcw.cols; j++) {
            float32v_msg.float32Vector.push_back(CurrentKeyFramePtr->mTcw.at<float>(i,j));
        }
        frame_msg.mTcw.push_back(float32v_msg);
    }


    frame_msg.nNextId = CurrentKeyFramePtr->nNextId;
    frame_msg.mnId = CurrentKeyFramePtr->mnId;

    pub_a_.publish(frame_msg);

}
