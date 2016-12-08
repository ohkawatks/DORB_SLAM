# DORB_SLAM
Distributed ORB SLAM

#1 Prerequisites
We have tested the library in and **14.04** + ROS Indigo. Only Monocular camera dataset is supported for now.


## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:

    sudo apt-get install libblas-dev
    sudo apt-get install liblapack-dev

## DBoW2 and g2o (Included in Thirdparty folder under src/dorbslam)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

#2 Build
 copy src/dorbslam and src/dataset_reader to your ros working path.Build with the command 
 
    catkin_make

#3 Running Node
##3.1 Running Non-Distributed node
Execute following command to Run orbslam node,

        rosrun dorbslam dorbslam_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

Start data stream by the data set reader (or camera node)

        rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
        rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 

##3.2 Running Distribution type 1 (ORB extracter is distrbuted)

- Run by Launch file
    Execute following command 
   
         roslaunch dorbslam dorbslam_div1.launch vocabulary:="FULL_PATH_TO_VOCAbulary_FILE" setting:="FULL_PATH_TO_SETTING_FILE"

    Start data stream by the data set reader (or camera node)

          rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
          rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 


- Run Each nodes
    Execute following command to Run core node,

        rosrun dorbslam dorbslam_node_div1 PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Execute following command to Run Orb extractor node.
  
        rosrun dorbslam orb_extracter_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Start data stream by the data set reader (or camera node)

          rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
          rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 



##3.3 Running Distribution type 2 (bundle adjustment is distrbuted)

- Run by Launch file
    Execute following command

          roslaunch dorbslam dorbslam_div2.launch vocabulary:="FULL_PATH_TO_VOCAbulary_FILE" setting:="FULL_PATH_TO_SETTING_FILE"

    Start data stream by the data set reader 

          rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
          rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 


- Run Each nodes
    Execute following command to Run core node,

          rosrun dorbslam dorbslam_node_div2 PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Execute following command to bunle adjustment node.

          rosrun dorbslam bundle_adjustment_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Start data stream by the data set reader (or camera node)

          rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
          rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 



##3.4 Running Distribution type 3 node (ORB extractor and local bundle adjustment is distrbuted)

- Run by Launch file
    Execute following command 
    
          roslaunch dorbslam dorbslam_div3.launch vocabulary:="FULL_PATH_TO_VOCAbulary_FILE" setting:="FULL_PATH_TO_SETTING_FILE"

    Start data stream by the data set reader (or camera node)
    
          rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
          rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 


- Run Each nodes
    Execute following command to Run core node,

          rosrun dorbslam dorbslam_node_div3 PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Execute following command to Run Orb extractor node.
    
          rosrun dorbslam orb_extracter_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Execute following command to Run bundle adjustment node.

          rosrun dorbslam bundle_adjustment_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

    Start data stream by the data set reader (or camera node)

          rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
          rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 


#4 Disable GUI
If you'd like to disable gui, define "_\_DISABLE_GUI" deffinition flag in the CMakefile.

- For the div1 and non distributed node, add following line to CMake

        target_compile_definitions(${PROJECT_NAME} PUBLIC__DISABLE_GUI=1)

- For the div2 and div3 change the line 

        target_compile_definitions(${PROJECT_NAME}_div2 PUBLIC ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1)

    into 
    
        target_compile_definitions(${PROJECT_NAME}_div2 PUBLIC ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1 PUBLIC__DISABLE_GUI=1)

