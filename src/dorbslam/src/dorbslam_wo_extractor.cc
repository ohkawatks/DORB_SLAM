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
#include "dorbslam/OrbDescriptor.h"
#include "dorbslam/KeyPoint.h"
#include "dorbslam/TrackingState.h"
#include "std_msgs/Int32.h"
#include <opencv2/core/core.hpp>
#include "dorbslam/TimeStamp.h"
#include"System.h"
#ifdef ENABLE_PERFORM
#include "measurmentManager.h"
#endif

#ifdef ENABLE_PERFORM
extern measurmentManager *g_measurmentServer;
extern char* __progname;
#endif


#define DESCRIPTOR_TOPIC_NAME ("/orb_descriptor")
#define STATE_TOPIC_NAME ("/tracker_state")
using namespace std;

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM2::System* pSLAM,
               ros::NodeHandle nh
               ):mpSLAM(pSLAM),
#ifdef PUBLISH_TIME_STAMP
                 pub_start_stamp_(nh.advertise<dorbslam::TimeStamp>("/time_stamp/dorbsla_wo_extractor/begin", 100)),
                 pub_end_stamp_(nh.advertise<dorbslam::TimeStamp>("/time_stamp/dorbsla_wo_extractor/end", 100)),
#endif
                 mpub(nh.advertise<dorbslam::TrackingState>(STATE_TOPIC_NAME, 1000))
  {}


  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
  void GrabDescriptor(const dorbslam::OrbDescriptorConstPtr& msg);

private:
  ORB_SLAM2::System* mpSLAM;
  ros::Publisher mpub;
#ifdef PUBLISH_TIME_STAMP
  ros::Publisher pub_start_stamp_;
  ros::Publisher pub_end_stamp_;
#endif

};

int main(int argc, char **argv)
{
#ifdef ENABLE_PERFORM
   unsigned char measument_path[256];
#endif

    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
#ifdef ENABLE_PERFORM
    g_measurmentServer = new measurmentManager();
    memset( (char*)measument_path,0x00, sizeof(measument_path));
    sprintf( (char*)measument_path,"/%s/measurment",__progname);
#endif

    // Create SLAM system. 
    // It initializes all system threads and gets ready to process frames.
    ros::NodeHandle nodeHandler;
#ifdef ENABLE_PERFORM
    ros::ServiceServer server = nodeHandler.advertiseService( (char*)measument_path, 
                                                              &measurmentManager::service, g_measurmentServer);
#endif

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM, nodeHandler);


    //    ros::Subscriber sub = nodeHandler.subscribe("/orb_descriptor",
    //                                                1,
    //                                                &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe(DESCRIPTOR_TOPIC_NAME,
                                                1,
                                                &ImageGrabber::GrabDescriptor,&igb);
#ifdef ENABLE_PERFORM   
    g_measurmentServer->processThroughputEntry(0,0,"ImageGrabber::GrabDescriptor");
    g_measurmentServer->processThroughputEntry(0,1,"ImageGrabber::GrabImage");
#endif

    while ( ros::ok() ){
      ros::spinOnce();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    time_t rawtime;
    struct tm * timeinfo;
    char fname[255];

    time (&rawtime);
    timeinfo = localtime(&rawtime);
#ifdef ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT
    strftime(fname, 80,"KeyFrameTrajectory_div3_%Y%m%d-%I%M%S.txt",timeinfo);
#else
    strftime(fname, 80,"KeyFrameTrajectory_div1_%Y%m%d-%I%M%S.txt",timeinfo);
#endif

    SLAM.SaveKeyFrameTrajectoryTUM(fname);

#ifdef ENABLE_PERFORM
    g_measurmentServer->processThroughputDelete(0,0);
    g_measurmentServer->processThroughputDelete(0,1);
#endif
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabDescriptor(const dorbslam::OrbDescriptorConstPtr& msg)
{
	dorbslam::TrackingState tracking_state;
#ifdef ENABLE_PERFORM
	g_measurmentServer->processThroughputThreadStart(0,0,msg->header.stamp.toSec());
#endif

#ifdef PUBLISH_TIME_STAMP
      {
        dorbslam::TimeStamp stamp;
        ros::WallTime wallstamp = ros::WallTime::now();
        stamp.wall_stamp.sec = wallstamp.sec;
        stamp.wall_stamp.nsec = wallstamp.nsec;
        stamp.header.stamp = msg->header.stamp;
        pub_start_stamp_.publish(stamp);
      }
#endif

    // Copy the ros image message to cv::Mat.
    mpSLAM->TrackMonocular(msg->KeyPoints,
                           msg->Settings,
                           msg->header.stamp.toSec()
                           );

    int state = mpSLAM->getTrackerState();
    tracking_state.header = msg->header;
    tracking_state.data = state;
    mpub.publish(tracking_state);
#ifdef PUBLISH_TIME_STAMP
      {
        dorbslam::TimeStamp stamp;
        ros::WallTime wallstamp = ros::WallTime::now();
        stamp.wall_stamp.sec = wallstamp.sec;
        stamp.wall_stamp.nsec = wallstamp.nsec;
        stamp.header.stamp = msg->header.stamp;
        pub_end_stamp_.publish(stamp);
      }
#endif

#ifdef ENABLE_PERFORM
    g_measurmentServer->processThroughputThreadEnd(0,0,msg->header.stamp.toSec());
#endif

}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
#ifdef ENABLE_PERFORM
    g_measurmentServer->processThroughputThreadStart(0,1,msg->header.stamp.toSec());
#endif


    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

#ifdef ENABLE_PERFORM
    g_measurmentServer->processThroughputThreadEnd(0,1,msg->header.stamp.toSec());
#endif
}

