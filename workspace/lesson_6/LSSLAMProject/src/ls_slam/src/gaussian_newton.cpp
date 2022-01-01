#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SparseCholesky>

#include <iostream>


//位姿-->转换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)), -sin(x(2)), x(0),
        sin(x(2)), cos(x(2)), x(1),
        0, 0, 1;

    return trans;
}


//转换矩阵－－＞位姿
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0, 2);
    pose(1) = trans(1, 2);
    pose(2) = atan2(trans(1, 0), trans(0, 0));

    return pose;
}

//计算整个pose-graph的误差
double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
    std::vector<Edge>& Edges)
{
    double sumError = 0;
    for (int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() * Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sumError += ei.transpose() * infoMatrix * ei;
    }
    return sumError;
}


/**
 * @brief CalcJacobianAndError
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bi    相对于xj的Jacobian矩阵
 */
void CalcJacobianAndError(Eigen::Vector3d xi, Eigen::Vector3d xj, Eigen::Vector3d z,
    Eigen::Vector3d& ei, Eigen::Matrix3d& Ai, Eigen::Matrix3d& Bi)
{
    //TODO--Start
    Eigen::Matrix3d Xi = PoseToTrans(xi);
    Eigen::Matrix3d Xj = PoseToTrans(xj);
    Eigen::Matrix3d Z = PoseToTrans(z);

    Eigen::Matrix2d Ri = Xi.block(0, 0, 2, 2);
    Eigen::Matrix2d Rj = Xj.block(0, 0, 2, 2);
    Eigen::Matrix2d Rij = Z.block(0, 0, 2, 2);

    Eigen::Vector2d ti = xi.block(0, 0, 2, 1);
    Eigen::Vector2d tj = xj.block(0, 0, 2, 1);
    Eigen::Vector2d tij = z.block(0, 0, 2, 1);

    ei = TransToPose(Z * (Xi.inverse() * Xj));

    Eigen::Matrix2d dRi_dtheta_T;
    double cos_theta = Ri(0, 0);
    double sin_theta = Ri(1, 0);
    dRi_dtheta_T << -sin_theta, cos_theta,
        -cos_theta, -sin_theta;

    Ai.block(0, 0, 2, 2) = -Rij.transpose() * Ri.transpose();
    Ai.block(0, 2, 2, 1) = Rij.transpose() * dRi_dtheta_T * (tj - ti);
    Ai(2, 2) = -1;

    Bi.block(0, 0, 2, 2) = -Ai.block(0, 0, 2, 2);
    Bi(2, 2) = 1;
    //TODO--end
}

/**
 * @brief LinearizeAndSolve
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
    std::vector<Edge>& Edges)
{
    //申请内存
    Eigen::MatrixXd H(Vertexs.size() * 3, Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    //固定第一帧
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0, 0, 3, 3) += I;

    //构造H矩阵　＆ b向量
    for (int i = 0; i < Edges.size();i++)
    {
        //提取信息
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        //计算误差和对应的Jacobian
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bi;
        CalcJacobianAndError(xi, xj, z, ei, Ai, Bi);

        //TODO--Start
        Eigen::Matrix3d Hii = Ai.transpose() * tmpEdge.infoMatrix * Ai;
        Eigen::Matrix3d Hij = Ai.transpose() * tmpEdge.infoMatrix * Bi;
        Eigen::Matrix3d Hjj = Bi.transpose() * tmpEdge.infoMatrix * Bi;

        Eigen::Vector3d bi = Ai.transpose() * tmpEdge.infoMatrix * ei;
        Eigen::Vector3d bj = Bi.transpose() * tmpEdge.infoMatrix * ei;

        H.block<3, 3>(tmpEdge.xi * 3, tmpEdge.xi * 3) += Hii;
        H.block<3, 3>(tmpEdge.xi * 3, tmpEdge.xj * 3) += Hij;
        H.block<3, 3>(tmpEdge.xj * 3, tmpEdge.xi * 3) += Hij.transpose();
        H.block<3, 3>(tmpEdge.xj * 3, tmpEdge.xj * 3) += Hjj;

        b.block<3, 1>(tmpEdge.xi * 3, 0) += bi;
        b.block<3, 1>(tmpEdge.xj * 3, 0) += bj;
        //TODO--End
    }

    //求解
    Eigen::VectorXd dx;

    //TODO--Start
    dx.resize(Vertexs.size() * 3);

    // * method 1 inverse
    dx =  -H.inverse() * b;

    // * method 2 cholesky
    // dx = H.ldlt().solve(-b);

    // * method 3
    // Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    // Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(H_sparse);
    // dx = solver.solve(-b);

    // * method 4
    //TODO-End

    return dx;
}











