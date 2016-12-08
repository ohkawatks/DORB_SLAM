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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include "ORBextractor.h"
#include "System.h"
#include "dorbslam/OrbDescriptor.h"
#include "dorbslam/KeyPoint.h"
#include "std_msgs/Int32.h"
#define IMAGE_TOPIC ("/camera/image_raw")
#define PUBLISH_TOPIC_NAME ("/orb_descriptor")
#define STATE_TOPIC ("/tracker_state")
using namespace std;


class ImageGrabber
{
public:
  //ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
  ImageGrabber(ros::NodeHandle nh,
               int nRGB, 
               int nFeatures, 
               float fScaleFactor, 
               int nLevels, 
               int fIniThFAST, 
               int fMinThFAST
               )
    :mbRGB(nRGB),
     mIniORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST),
     mORBextractorLeft(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST),
     msub_state(nh.subscribe(STATE_TOPIC, 1, &ImageGrabber::GrabState, this)),
     mpub(nh.advertise<dorbslam::OrbDescriptor>(PUBLISH_TOPIC_NAME, 1000)),
    mTrackerState(ORB_SLAM2::Tracking::SYSTEM_NOT_READY)//SYSTEM_NOT_READY
  {

  }
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
  void GrabState(const std_msgs::Int32ConstPtr& msg);
private:
  //ORB_SLAM2::System* mpSLAM;
  int mbRGB;
  ORB_SLAM2::ORBextractor mIniORBextractor;
  ORB_SLAM2::ORBextractor mORBextractorLeft;
  ros::Subscriber msub_state;
  ros::Publisher mpub;

  int mTrackerState;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_extractor");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
      cout << "Could not open the configuration file: \"" << argv[2] << "\"" << endl;
      return -1;
    }
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    
    ros::NodeHandle nodeHandler;
    int nRGB = fSettings["Camera.RGB"];
    ImageGrabber igb(nodeHandler, nRGB, nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);//&SLAM);

    ros::Subscriber sub = nodeHandler.subscribe(IMAGE_TOPIC, 1, &ImageGrabber::GrabImage,&igb);
    ros::spin();

    // Save camera trajectory
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

    // convert to grayscale image if necessary
    cv::Mat img = cv_ptr->image;
    if(img.channels()==3)
    {
      if(mbRGB)
        cvtColor(img,img,CV_RGB2GRAY);
      else
        cvtColor(img,img,CV_BGR2GRAY);
    }
    else if(img.channels()==4)
    {
        if(mbRGB)
            cvtColor(img,img,CV_RGBA2GRAY);
        else
            cvtColor(img,img,CV_BGRA2GRAY);
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    ORB_SLAM2::ORBextractor* extractor;

    if (mTrackerState < ORB_SLAM2::Tracking::OK){
      extractor = &mIniORBextractor;
    }
    else{
      extractor = &mORBextractorLeft;
    }
    //extract descriptors
    (*extractor)(img,cv::Mat(), keypoints, descriptors);    
    printf("rows:%d,cols%d\n",  descriptors.rows, descriptors.cols);

    
    dorbslam::OrbDescriptor desc;
    dorbslam::KeyPoint kp;
    dorbslam::ExtractorSettings settings;
    // copy descriptors to the message file
    for(unsigned int i = 0;i<keypoints.size();i++){
      kp.angle=keypoints[i].angle;
      kp.class_id=keypoints[i].class_id;
      kp.octave=keypoints[i].octave;
      kp.pt[0]=keypoints[i].pt.x;
      kp.pt[1]=keypoints[i].pt.y;
      kp.response=keypoints[i].response;
      kp.size=keypoints[i].size;
      
      for(int j =0;j<32;j++){
        kp.descriptor[j] = descriptors.data[i*32+j];
      }
      desc.KeyPoints.push_back(kp);
    }
    //mnScaleLevels = mpORBextractorLeft->GetLevels();
    desc.Settings.levels= extractor->GetLevels();
    //mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    desc.Settings.ScaleFactor = extractor->GetScaleFactor();
    //mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    desc.Settings.ScaleFactors = extractor->GetScaleFactors();
    ///mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    desc.Settings.InvScaleFactors = extractor->GetInverseScaleFactors();
    desc.Settings.LevelScaleSigma2 = extractor->GetScaleSigmaSquares();
    desc.Settings.InvLevelScaleSigmas2 = extractor->GetInverseScaleSigmaSquares();
    desc.Settings.ImageCols = img.cols;
    desc.Settings.ImageRows = img.rows;
    desc.header = cv_ptr->header;
    mpub.publish(desc);
    ////////////////////
    //mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}

void ImageGrabber::GrabState(const std_msgs::Int32ConstPtr& msg){
  this->mTrackerState = msg->data;
}
