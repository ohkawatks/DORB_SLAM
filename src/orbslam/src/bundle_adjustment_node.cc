#include "ros/ros.h"
#include "Optimizer.h"
#include "Converter.h"
#include "orbslam/BundleAdjustment.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


// Eigen::Matrix<double,3,1> toVector3d(const std::vector<double>& vec){
//     Eigen::Matrix<double,3,1> v;
//     v << vec[0], vec[1], vec[2];
//     return v;
// }


void initOptimizerFromMsg(g2o::SparseOptimizer& optimizer, 
                          const orbslam::BundleAdjustment::Request  &req, 
                          vector<g2o::EdgeSE3ProjectXYZ*>& vpEdges){
  const float thHuberMono = sqrt(5.991);
  // copy request data to optimizzer buffer
  for(size_t i= 0;i<req.localgraph.vertices_kf.size();i++){
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    {

      Eigen::Matrix<double,3,3> R;
      Eigen::Matrix<double,3,1> t;
      const boost::array<double, 16ul> T = req.localgraph.vertices_kf[i].pose;
      R << T[0], T[1], T[2], 
        T[4], T[5], T[6],
        T[8], T[9], T[10];
      t <<T[3], T[7], T[11];
      //std::cout << R << std::endl << std::endl;
      vSE3->setEstimate(g2o::SE3Quat(R,t));
    }
    vSE3->setId(req.localgraph.vertices_kf[i].id);
    vSE3->setFixed(req.localgraph.vertices_kf[i].isFixed);
    optimizer.addVertex(vSE3);
  }

  for(size_t i= 0;i<req.localgraph.vertices_mp.size();i++){
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();

    {
      Eigen::Matrix<double,3,1> v;
      const boost::array<double, 3ul> vec = req.localgraph.vertices_mp[i].pos;
      v << vec[0], vec[1], vec[2];
      //std::cout << "after:"<< v <<std::endl << std::endl;
      vPoint->setEstimate(v);
    }

    vPoint->setId(req.localgraph.vertices_mp[i].id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
  }
  //  unsigned long maxKFid = 0;

  //  if(pbStopFlag)
  //    if(*pbStopFlag)
  //      return;

  for(size_t i= 0;i<req.localgraph.edges.size();i++){  
    const orbslam::Edge& edge = req.localgraph.edges[i];
    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edge.mpid)));
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edge.kfid)));
    Eigen::Matrix<double,2,1> obs;
    obs << edge.x, edge.y;

    e->setMeasurement(obs);
    e->setInformation(Eigen::Matrix2d::Identity()*edge.invsigma2);

    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuberMono);

    e->fx = edge.fx;
    e->fy = edge.fy;
    e->cx = edge.cx;
    e->cy = edge.cy;

    optimizer.addEdge(e);
    vpEdges.push_back(e);
  }
}

void packToMsg(const g2o::SparseOptimizer& optimizer, 
               const orbslam::BundleAdjustment::Request& req,
               orbslam::BundleAdjustment::Response& res, 
               vector<g2o::EdgeSE3ProjectXYZ*>& vpEdges)
{
  // keyframe
  res.localgraph.vertices_kf.resize(req.localgraph.vertices_kf.size());
  for(size_t i = 0; i<req.localgraph.vertices_kf.size();i++){
    const orbslam::VertexKF& v = req.localgraph.vertices_kf[i];
    orbslam::VertexKF& v_out = res.localgraph.vertices_kf[i];    
    const g2o::VertexSE3Expmap* vSE3 = static_cast<const g2o::VertexSE3Expmap*>(optimizer.vertex(v.id));
    g2o::SE3Quat SE3quat = vSE3->estimate();

    cv::Mat pose = ORB_SLAM2::Converter::toCvMat(SE3quat);
    //std::cout << "pack2msgcv"<<std::endl<< pose << std::endl << std::endl;
    //v_out.pose.resize(12);
    for(size_t i=0;i<16;i++)  v_out.pose[i] = pose.at<float>(i);
    
    v_out.id = v.id;
    v_out.isFixed = v.isFixed;
  }

  // mappoint
  res.localgraph.vertices_mp.resize(req.localgraph.vertices_mp.size());
  for(size_t i = 0; i<req.localgraph.vertices_mp.size();i++){
    orbslam::VertexMP v = req.localgraph.vertices_mp[i];
    orbslam::VertexMP& v_out = res.localgraph.vertices_mp[i];    
    //    cv::Mat pos = pMP->GetWorldPos();
    const g2o::VertexSBAPointXYZ* vPoint = static_cast<const g2o::VertexSBAPointXYZ*>(optimizer.vertex(v.id));

    cv::Mat pos = ORB_SLAM2::Converter::toCvMat(vPoint->estimate());
    //    v_out.pos.reserve(3);
    for(size_t i =0;i<3;i++) v_out.pos[i] = pos.at<float>(i);
    v_out.id = v.id;
  }  

  // edges(to delete)
  // Check inlier observations       
  res.localgraph.edges.reserve(req.localgraph.edges.size());
  for(size_t i=0; i<vpEdges.size();i++){
    g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];
    if(req.localgraph.edges[i].isBadMp) continue;
    //if(req.localgraph.vertices_mp[i].isBad) continue;
    //if(req.localgraph.vertices_mp[req.localgraph.edges[i].mpid].isBad) continue;
    if(e->chi2()>5.991 || !e->isDepthPositive())
      {
        orbslam::Edge edge;
        edge.mpid = res.localgraph.edges[i].mpid; // mappoint id
        edge.kfid = res.localgraph.edges[i].kfid; // kfid
        res.localgraph.edges.push_back(edge);
        //        printf("to delete\n");
      }
  }
  //  printf("done\n");
}



bool bundle_adjustment(orbslam::BundleAdjustment::Request  &req,
         orbslam::BundleAdjustment::Response &res)
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  ROS_INFO("BUNDLE1");
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  vector<g2o::EdgeSE3ProjectXYZ*> vpEdges;
  optimizer.setAlgorithm(solver);

  initOptimizerFromMsg(optimizer, req, vpEdges);
  ROS_INFO("BUNDLE2");
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inlier observations
  for(size_t i=0; i<vpEdges.size();i++){
    g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];
    //    if(req.localgraph.vertices_mp[req.localgraph.edges[i].mpid].isBad) continue;
    if(req.localgraph.edges[i].isBadMp) continue;
    if(e->chi2()>5.991 || !e->isDepthPositive())
      {
        e->setLevel(1);
      }
    e->setRobustKernel(0);
  }
  ROS_INFO("BUNDLE3");
    //  bool bDoMore= true;

  // if(pbStopFlag)
  //   if(*pbStopFlag)
  //     bDoMore = false;

  //  if(bDoMore)
  //    {

  // Check inlier observations
  // Optimize again without the outliers
  optimizer.initializeOptimization(0);
  optimizer.optimize(10);
  packToMsg(optimizer, req, res, vpEdges);

  ROS_INFO("BUNDLE:request_kf:%d mp:%d edge:%d", 
           (int)req.localgraph.vertices_kf.size(),
           (int)req.localgraph.vertices_mp.size(),
           (int)req.localgraph.edges.size());
  ROS_INFO("BUNDLE:response_kf:%d mp:%d edge:%d", 
           (int)res.localgraph.vertices_kf.size(),
           (int)res.localgraph.vertices_mp.size(),
           (int)res.localgraph.edges.size());
  
  return true;
 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bundle_adjustment_node");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("bundle_adjustment", 
                                                    bundle_adjustment);
  ROS_INFO("Ready to bundle adjustment node.");
  ros::spin();

  return 0;
}
