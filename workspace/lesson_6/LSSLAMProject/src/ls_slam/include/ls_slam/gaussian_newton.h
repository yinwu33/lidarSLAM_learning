#ifndef GAUSSIAN_NEWTON_H
#define GAUSSIAN_NEWTON_H

#include <vector>
#include <eigen3/Eigen/Core>

typedef struct edge
{
  int xi, xj;
  Eigen::Vector3d measurement;
  Eigen::Matrix3d infoMatrix;
}Edge;


Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
  std::vector<Edge>& Edges, int method = 0);

double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
  std::vector<Edge>& Edges);


#endif
