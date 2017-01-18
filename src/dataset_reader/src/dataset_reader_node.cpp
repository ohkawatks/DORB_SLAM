#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#ifdef ENABLE_PERFORM   
#include "measurmentManager.h"
#endif

#define PUBLISH_TOPIC_NAME "/camera/image_raw"

#ifdef ENABLE_PERFORM   
measurmentManager *	g_measurmentServer;
extern char* __progname;
#endif

using namespace std;

class DatasetReader{
public:
  DatasetReader():it_(n_)
  {
    pub_img_= it_.advertise(PUBLISH_TOPIC_NAME, 1000);
  }

  int run(const std::string& dataPath){
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    dataPath_ = dataPath;


    if (LoadImages(dataPath_, 
                   vstrImageFilenames,
                   vTimestamps )
        ){
      ROS_ERROR("data path [%s] open failed.", dataPath.c_str());
      return 1;
    };

    int nImages = vstrImageFilenames.size();


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    double stamp = 0.0;
    ros::Time begin = ros::Time::now();

#ifdef ENABLE_PERFORM   
	 g_measurmentServer->processThroughputEntry(0,0,"dataset_reader::main");	
#endif

    for(int cnt = 0; cnt < nImages;cnt++){
      if (!ros::ok())
        break;
      stamp = 0;

#ifdef ENABLE_PERFORM   
  	   g_measurmentServer->processThroughputThreadStart(0,0,stamp);	
#endif

      // Read image from file
      im = cv::imread(dataPath_+"/"+vstrImageFilenames[cnt],CV_LOAD_IMAGE_UNCHANGED);

      if(im.empty()){
        cerr << endl << "Failed to load image at: " << vstrImageFilenames[cnt] << endl;
          return 1;
      }
      stamp = publish(cnt, im); 
#ifdef ENABLE_PERFORM   
      g_measurmentServer->processThroughputThreadEnd(0,0,stamp);	
#endif

      ROS_INFO("Frame %d[%f] published.", cnt,  vTimestamps[cnt]);
      ros::spinOnce();

#ifdef ENABLE_PERFORM   
		ros::Duration(0.1).sleep();
#else
		ros::Duration sleeptime = ros::Duration(vTimestamps[cnt+1]) 
			- (ros::Time::now() - begin);

		if (sleeptime.toSec() > 0)
 			sleeptime.sleep();
#endif
    }

#ifdef ENABLE_PERFORM   
	while ( ros::ok() ){
		ros::spinOnce();
	}   
	g_measurmentServer->processThroughputDelete(0,0);	
#endif

    return 0;
  }


private:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_img_;

  std::string dataPath_;

  virtual int LoadImages(const string &dataPath,
                          vector<string>& vstrImageFilenames,
                          vector<double>& vTimestamps
                          )=0; 

  double publish(int counter, const cv::Mat img){
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.seq = counter;
    
    header.stamp = ros::Time::now();

    if(img.channels()==1)
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
    else if(img.channels()==3)    
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);      
    else if(img.channels()==4)    
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, img);   
    img_bridge.toImageMsg(img_msg);
    pub_img_.publish(img_msg); 
	return header.stamp.toSec();
  }
};

