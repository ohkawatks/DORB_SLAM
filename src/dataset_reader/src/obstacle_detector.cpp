
////////////////////////////////////////////////////////////
// インクルードファイル
////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obstacle_detector.h"
//#define DEBUG_TIME 1

int m_tracker_init=0;
using namespace std;

////////////////////////////////////////////////////////////
// 定数定義
////////////////////////////////////////////////////////////

#define USB_CAM "/camera/depth/image_raw"
#define USB_CAM_TEMP "/camera/depth/image_temp"
#define TS_DEPTH_IMAGE (0.1)
/*
enum {
  DETECT_IMAGE_WIDTH = 640,
  DETECT_IMAGE_HEIGHT = 480,
};
*/


const int TH_DETECT_COUNT = 10;

/** 
 * コンストラクタ
 * 
 */
ObstacleDetect::ObstacleDetect():
  it_(nh_),
  result_pub_(nh_.advertise<std_msgs::Bool>(PUBLISH_TOPIC_NAME, 1000)),
  image_sub_(it_.subscribe(USB_CAM_TEMP, 1, &ObstacleDetect::imageCB, this)),
  image_sub_temp_(it_.subscribe(USB_CAM, 1, &ObstacleDetect::imageCBTemp, this)),
  pub_img_temp_(it_.advertise(USB_CAM_TEMP, 1000)),
  detect_count(0)
{
  nh_.getParam("/tsukisoi/params/S1",   this->TH_DIST_OBSTACLE);
  this->TH_DIST_OBSTACLE *= 1000; // m -> mm
  double ST1;
  nh_.getParam("/tsukisoi/params/ST1", ST1 );
  this->TH_DETECT_COUNT =  (int)floor(ST1 / TS_DEPTH_IMAGE) ;
  result.data = false;
  sub_state = nh_.subscribe("/tsukisoi_robot/state", 
                              10,
                              &ObstacleDetect::stateCB, 
                              this
                              );
}

ObstacleDetect::~ObstacleDetect() 
{
}

/** 
 * 距離画像のxスキャンに対するクラスタリング関数
 * 
 * @param img 距離画像 
 * @param cl クラスタリング結果
 */
void ObstacleDetect::clustering(
                                cv::Mat& img,  
                                std::vector<std::vector<cv::Vec2d> >& cl
                                ){
  const int scan_y = 240;
  const double TH_DIST = 70; // 7cm
  int y_offs = img.size().width*scan_y;
  int x_offs = img.size().width/4; //左右1/5は評価しない
  std::vector<cv::Vec2d> c;
  cv::Vec2d prev;
  const int MIN_CLUSTER_SIZE = 7;

  // clustering
  for(int i=x_offs;i<img.size().width - x_offs;i++){
    double a = i; double r = img.at<uint16_t>(scan_y, i);
    if (r == 0){
      if(c.size() != 0){
        if (c.size() > MIN_CLUSTER_SIZE)
          cl.push_back(c);
        c.clear();
      }
      continue;
    }
    if(c.size() == 0){
      prev[1] = r;
      c.push_back(prev);
      continue;
    }
    if(fabs(r-prev[1]) < TH_DIST){
      prev[0] = a; prev[1] = r;
      c.push_back(prev);
    }
    else{
      if (c.size() > MIN_CLUSTER_SIZE)
        cl.push_back(c);
      c.clear();
    }
  }
  if(c.size() != 0){
    if (c.size() > MIN_CLUSTER_SIZE)
      cl.push_back(c);
  }
 }

int ObstacleDetect::detect(cv::Mat& img){
  //printf("%d, %d\n", img.size().width, img.size().height);

  std::vector<std::vector<cv::Vec2d> > cl;
  clustering(img, cl);
  ROS_INFO("%d cluster found.", int(cl.size()));
  std::vector<std::vector<cv::Vec2d> >::const_iterator itr;
  for (itr = cl.begin(); itr != cl.end(); itr++){
    double sum = 0;
    for(int i = 0;i < itr->size(); i++){
      sum += (*itr)[i][1];
    }
    double ave = sum / (*itr).size();
    ROS_INFO("TH;%f,  dist: %f", this->TH_DIST_OBSTACLE, ave );
    if (this->TH_DIST_OBSTACLE > ave){
      return true;
    }
  }
  return false;
}
/** 
 * ROSによる画像入力のためのコールバック関数
 * 
 * @param msg 画像が含まれたROSメッセージ
 */

void ObstacleDetect::imageCBTemp( const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg);
  pub_img_temp_.publish(cv_ptr->toImageMsg());
}
 
void ObstacleDetect::imageCB( const sensor_msgs::ImageConstPtr& msg)
{
        
  int i, j;
  cv_bridge::CvImagePtr cv_ptr;
  //result.data = false;
  // CALIBのタイミングで障害物距離パラメータを再読み込み
  if (state_.main_state == "MAIN_STATE_CALIB"){
    nh_.getParam("/tsukisoi/params/S1",   this->TH_DIST_OBSTACLE);
    this->TH_DIST_OBSTACLE *= 1000.0;
    double ST1;
    nh_.getParam("/tsukisoi/params/ST1", ST1 );
    this->TH_DETECT_COUNT = (int)floor(ST1 / TS_DEPTH_IMAGE) ;
  }

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  // 探索関数の呼び出し
  
  detect_count = detect(cv_ptr->image) ? detect_count + 1 : 0;

  if (detect_count > TH_DETECT_COUNT){
    ROS_INFO("detect_count: %d\n",detect_count );
    result.data = true;
  }
  else{
    result.data = false;
  }
  result_pub_.publish(result);
}

/** 
 * ROSによるステート情報取得のためのコールバック関数
 * 
 * @param msg ステートメッセージ
 */
void ObstacleDetect::stateCB(const tsukisoi::StateConstPtr& msg){
    state_ = *msg;
  };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detector");
  ObstacleDetect fd;
  std::string path = ros::package::getPath("obstacle_detection");
  std::string sound_command = "play " + path + "/sound/notify.mp3";
  //ros::spin();
  ros::Rate loop_rate(10);
  int play_sound_flag;
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
    if (fd.result.data == true && play_sound_flag){
      system(sound_command.c_str());
      play_sound_flag=0;
    }
    else{
      play_sound_flag=1;
    }
  }

  return(0);
}

