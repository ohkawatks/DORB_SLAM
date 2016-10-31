# DORB_SLAM
Distributed ORB SLAM

#1 Prerequisites
We have tested the library in and **14.04** + ROS Indigo. Only Monocular camera dataset is supported for now.


## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

## DBoW2 and g2o (Included in Thirdparty folder under src/orbslam)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

#2 Build
 copy src/orbslam and src/dataset_reader to your ros working path.Build with the command 
  catkin_make

#3 Running Non-Distributed node
Execute following command to Run orbslam node,
  rosrun orbslam orbslam_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

Start data stream by the data set reader 
 rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
 rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 

#4 Running Distribution type 1 node (ORB extracter is distrbuted)
Execute following command to Run core node,
  rosrun orbslam orbslam_node_div1 PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

Execute following command to Run Orb extractor node.
  rosrun orbslam orb_extracter_node PATH_TO_VOCABULARY PATH PATH_TO_SETTINGS_FILE

Start data stream by the data set reader 
 rosrun dataset_reader kitti_reader_node PATH_TO_SEQUENCE 
 rosrun dataset_reader tum_reader_node PATH_TO_SEQUENCE 


