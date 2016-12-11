#include "ros/ros.h"
#include "Optimizer.h"
#include "Converter.h"
#include "dorbslam/BundleAdjustment.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"



void initOptimizerFromMsg(g2o::SparseOptimizer& optimizer, 
                          const dorbslam::BundleAdjustment::Request  &req, 
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

      vPoint->setEstimate(v);
    }

    vPoint->setId(req.localgraph.vertices_mp[i].id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
  }



  for(size_t i= 0;i<req.localgraph.edges.size();i++){  
    const dorbslam::Edge& edge = req.localgraph.edges[i];
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
               const dorbslam::BundleAdjustment::Request& req,
               dorbslam::BundleAdjustment::Response& res, 
               vector<g2o::EdgeSE3ProjectXYZ*>& vpEdges)
{
  // keyframe
  res.localgraph.vertices_kf.resize(req.localgraph.vertices_kf.size());
  for(size_t i = 0; i<req.localgraph.vertices_kf.size();i++){
    const dorbslam::VertexKF& v = req.localgraph.vertices_kf[i];
    dorbslam::VertexKF& v_out = res.localgraph.vertices_kf[i];    
    const g2o::VertexSE3Expmap* vSE3 = static_cast<const g2o::VertexSE3Expmap*>(optimizer.vertex(v.id));
    g2o::SE3Quat SE3quat = vSE3->estimate();

    cv::Mat pose = ORB_SLAM2::Converter::toCvMat(SE3quat);

    //v_out.pose.resize(12);
    for(size_t i=0;i<16;i++)  v_out.pose[i] = pose.at<float>(i);
    
    v_out.id = v.id;
    v_out.isFixed = v.isFixed;
  }

  // mappoint
  res.localgraph.vertices_mp.resize(req.localgraph.vertices_mp.size());
  for(size_t i = 0; i<req.localgraph.vertices_mp.size();i++){
    dorbslam::VertexMP v = req.localgraph.vertices_mp[i];
    dorbslam::VertexMP& v_out = res.localgraph.vertices_mp[i];    

    const g2o::VertexSBAPointXYZ* vPoint = static_cast<const g2o::VertexSBAPointXYZ*>(optimizer.vertex(v.id));

    cv::Mat pos = ORB_SLAM2::Converter::toCvMat(vPoint->estimate());
    for(size_t i =0;i<3;i++) v_out.pos[i] = pos.at<float>(i);
    v_out.id = v.id;
  }  

  // edges(to delete)
  // Check inlier observations       
  res.localgraph.edges.reserve(req.localgraph.edges.size());
  for(size_t i=0; i<vpEdges.size();i++){
    g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];
    if(req.localgraph.edges[i].isBadMp) continue;

    if(e->chi2()>5.991 || !e->isDepthPositive())
      {
        dorbslam::Edge edge;
        edge.mpid = req.localgraph.edges[i].mpid; // mappoint id
        edge.kfid = req.localgraph.edges[i].kfid; // kfid
        res.localgraph.edges.push_back(edge);
      }
  }
}



bool bundle_adjustment(dorbslam::BundleAdjustment::Request  &req,
         dorbslam::BundleAdjustment::Response &res)
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  vector<g2o::EdgeSE3ProjectXYZ*> vpEdges;
  optimizer.setAlgorithm(solver);

  initOptimizerFromMsg(optimizer, req, vpEdges);
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inlier observations
  for(size_t i=0; i<vpEdges.size();i++){
    g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];

    if(req.localgraph.edges[i].isBadMp) continue;
    if(e->chi2()>5.991 || !e->isDepthPositive())
      {
        e->setLevel(1);
      }
    e->setRobustKernel(0);
  }

  // Check inlier observations
  // Optimize again without the outliers
  optimizer.initializeOptimization(0);
  optimizer.optimize(10);
  packToMsg(optimizer, req, res, vpEdges);
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
