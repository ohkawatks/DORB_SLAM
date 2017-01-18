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

#include<opencv2/core/core.hpp>

#include"System.h"
#ifdef ENABLE_PERFORM   
#include "measurmentManager.h"
#endif

#ifdef ENABLE_PERFORM   
extern measurmentManager *	g_measurmentServer;
extern char* __progname;
#endif
	
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
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
    ros::NodeHandle nodeHandler;

#ifdef ENABLE_PERFORM   
    g_measurmentServer = new measurmentManager();
        
	memset( (char*)measument_path,0x00, sizeof(measument_path));
	sprintf( (char*)measument_path,"/%s/measurment",__progname);

    ros::ServiceServer server = nodeHandler.advertiseService( (char*)measument_path, 
																					&measurmentManager::service, g_measurmentServer);
#endif

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

#ifdef ENABLE_PERFORM   
  	g_measurmentServer->processThroughputEntry(0,0,"ImageGrabber::GrabDescriptor");	
#endif

	while ( ros::ok() ){
		ros::spinOnce();
	}   

#ifdef ENABLE_PERFORM   
  	g_measurmentServer->processThroughputDelete(0,0);	
#endif

    // Stop all threads
    SLAM.Shutdown();
    
        time_t rawtime;
    struct tm * timeinfo;
    char fname[255];

    time (&rawtime);
    timeinfo = localtime(&rawtime);
#ifdef ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT
    strftime(fname, 80,"KeyFrameTrajectory_div2_%Y%m%d-%I%M%S.txt",timeinfo);
#else
    strftime(fname, 80,"KeyFrameTrajectory_%Y%m%d-%I%M%S.txt",timeinfo);
#endif
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(fname);

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{

#ifdef ENABLE_PERFORM   
	g_measurmentServer->processThroughputThreadStart(0,0,msg->header.stamp.toSec());
#endif

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
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

#ifdef ENABLE_PERFORM   
	g_measurmentServer->processThroughputThreadEnd(0,0,msg->header.stamp.toSec());
#endif
}


