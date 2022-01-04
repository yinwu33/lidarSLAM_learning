#include <gaussian_newton.h>
#include <readfile.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "timer.hpp"

#include <ceres/ceres.h>

#include <Eigen/Core>

// TODO:ceres

struct CostFunction {
  CostFunction(const Eigen::Vector3d& measurement) {
    Z_ = PoseToTrans(measurement);
  }

  /**
   * @brief
   *
   * @tparam T
   * @param xi Vertex i double[3]
   * @param xj Vertex j double[3]
   * @param residual double[2]
   * @return true
   * @return false
   */
  template<typename T>
  bool operator() (const T* const xi_x, const T* const xi_y, const T* const xi_theta, const T* const xj_x, const T* const xj_y, const T* const xj_theta, T* residual) const {
    T xix = xi_x[0];
    T xiy = xi_y[0];
    T xitheta = xi_theta[0];
    T xjx = xj_x[0];
    T xjy = xj_y[0];
    T xjtheta = xj_theta[0];

    Eigen::Matrix<T, 3, 3> Xi;
    Eigen::Matrix<T, 3, 3> Xj; 

    Xi << cos(xitheta), -sin(xitheta), xix,
      sin(xitheta), cos(xitheta), xiy,
      T(0), T(0), T(1);
    Xj << cos(xjtheta), -sin(xjtheta), xjx,
      sin(xjtheta), cos(xjtheta), xjy,
      T(0), T(0), T(1);

    Eigen::Matrix<T, 3, 3> E = Z_.cast<T>().inverse() * Xi.inverse() * Xj;

    Eigen::Matrix<T, 3, 1> e;
    e(0) = E(0, 2);
    e(1) = E(1, 2);
    e(2) = atan2(E(1, 0), E(0, 0));

    residual[0] = e.block(0, 0, 2, 1).norm();
    residual[1] = e[2];
    // residual[0] = (0);
  // residual[1] = (0);
  return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d measurement) {
    return (new ceres::AutoDiffCostFunction<CostFunction, 2, 1, 1, 1, 1, 1, 1>(new CostFunction(measurement)));
  }

  //位姿-->转换矩阵
  Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x) const
  {
    Eigen::Matrix3d trans;
    trans << cos(x(2)), -sin(x(2)), x(0),
      sin(x(2)), cos(x(2)), x(1),
      0, 0, 1;

    return trans;
  }


  //转换矩阵－－＞位姿
  Eigen::Vector3d TransToPose(Eigen::Matrix3d trans) const
  {
    Eigen::Vector3d pose;
    pose(0) = trans(0, 2);
    pose(1) = trans(1, 2);
    pose(2) = atan2(trans(1, 0), trans(0, 0));

    return pose;
  }

private:
  Eigen::Matrix3d Z_;
};

//for visual
void PublishGraphForVisulization(ros::Publisher* pub,
  std::vector<Eigen::Vector3d>& Vertexs,
  std::vector<Edge>& Edges,
  int color = 0)
{
  visualization_msgs::MarkerArray marray;

  //point--red
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "ls-slam";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;

  if (color == 0)
  {
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
  }
  else
  {
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
  }

  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  //linear--blue
  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;

  if (color == 0)
  {
    edge.color.r = 0.0;
    edge.color.g = 0.0;
    edge.color.b = 1.0;
  }
  else
  {
    edge.color.r = 1.0;
    edge.color.g = 0.0;
    edge.color.b = 1.0;
  }
  edge.color.a = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint id = 0;

  //加入节点
  for (uint i = 0; i < Vertexs.size(); i++)
  {
    m.id = id;
    m.pose.position.x = Vertexs[i](0);
    m.pose.position.y = Vertexs[i](1);
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;
  }

  //加入边
  for (int i = 0; i < Edges.size();i++)
  {
    Edge tmpEdge = Edges[i];
    edge.points.clear();

    geometry_msgs::Point p;
    p.x = Vertexs[tmpEdge.xi](0);
    p.y = Vertexs[tmpEdge.xi](1);
    edge.points.push_back(p);

    p.x = Vertexs[tmpEdge.xj](0);
    p.y = Vertexs[tmpEdge.xj](1);
    edge.points.push_back(p);
    edge.id = id;

    marray.markers.push_back(visualization_msgs::Marker(edge));
    id++;
  }

  pub->publish(marray);
}



int main(int argc, char** argv)
{
  Timer timer;
  double t = .0;
  ros::init(argc, argv, "ls_slam");

  ros::NodeHandle nodeHandle, nh_private("~");

  std::string fileName = nh_private.param<std::string>("fileName", "test_quadrat");
  int method = nh_private.param<int>("method", 0);

  // beforeGraph
  ros::Publisher beforeGraphPub, afterGraphPub;
  beforeGraphPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("beforePoseGraph", 1, true);
  afterGraphPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("afterPoseGraph", 1, true);

  // std::string fileName = "intel";
  // std::string fileName = "killian";
  std::string VertexPath = "/home/ubuntu/workspace/lesson_6/LSSLAMProject/src/ls_slam/data/" + fileName + "-v.dat";
  std::string EdgePath = "/home/ubuntu/workspace/lesson_6/LSSLAMProject/src/ls_slam/data/" + fileName + "-e.dat";

  //    std::string VertexPath = "/home/eventec/LSSLAMProject/src/ls_slam/data/intel-v.dat";
  //    std::string EdgePath = "/home/eventec/LSSLAMProject/src/ls_slam/data/intel-e.dat";

  std::vector<Eigen::Vector3d> Vertexs;
  std::vector<Edge> Edges;

  timer.Start();
  ReadVertexInformation(VertexPath, Vertexs);
  ReadEdgesInformation(EdgePath, Edges);
  double t_read = timer.Stop();

  PublishGraphForVisulization(&beforeGraphPub,
    Vertexs,
    Edges);

  // convert vector to array
  double Vertex_Array[Vertexs.size() * 3];
  for (int i = 0; i < Vertexs.size(); ++i) {
    Vertex_Array[i * 3] = Vertexs[i][0];
    Vertex_Array[i * 3 + 1] = Vertexs[i][1];
    Vertex_Array[i * 3 + 2] = Vertexs[i][2];
  }

  // ! Ceres Part
  ceres::Problem problem;

  double initError = ComputeError(Vertexs, Edges);
  std::cout << "initError:" << initError << std::endl;

  int maxIteration = 100;
  double epsilon = 1e-4;
  double lastError = -1;

  double t_solve = .0;
  double t_update = .0;
  int count = 0;

  // ! ceres part
  for (int i = 0; i < Edges.size(); ++i) {

    ceres::CostFunction* cost_function = CostFunction::Create(Edges[i].measurement);
    problem.AddResidualBlock(
      cost_function, nullptr,
      &Vertexs[Edges[i].xi][0],
      &Vertexs[Edges[i].xi][1],
      &Vertexs[Edges[i].xi][2],
      &Vertexs[Edges[i].xj][0],
      &Vertexs[Edges[i].xj][1],
      &Vertexs[Edges[i].xj][2]
    );

  }

  ceres::Solver::Options options;
  options.max_num_iterations = 100;

  if (method == 0) {
    options.linear_solver_type = ceres::DENSE_QR;
  }
  else if (method == 1) {
    options.linear_solver_type = ceres::SPARSE_SCHUR;
  }

  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  double finalError = ComputeError(Vertexs, Edges);

  std::cout << "\n\n--------------------------------\nFinalError:" << finalError << std::endl;
  PublishGraphForVisulization(&afterGraphPub,
    Vertexs,
    Edges, 1);

  ros::spin();



  return 0;
}




