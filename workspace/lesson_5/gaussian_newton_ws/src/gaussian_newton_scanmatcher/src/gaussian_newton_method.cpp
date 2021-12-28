#include <map.h>
#include "gaussian_newton_method.h"

const double GN_PI = 3.1415926;

//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    if (angle > GN_PI)
        angle -= 2 * GN_PI;
    else if (angle < -GN_PI)
        angle += 2 * GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T << cos(vec(2)), -sin(vec(2)), vec(0),
        sin(vec(2)), cos(vec(2)), vec(1),
        0, 0, 1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt, Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0), pt(1), 1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0), tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
    std::vector<Eigen::Vector2d> laser_pts,
    double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for (int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i], Trans);

        int cell_x, cell_y;
        cell_x = MAP_GXWX(map, tmp_pt(0));
        cell_y = MAP_GYWY(map, tmp_pt(1));

        map->cells[MAP_INDEX(map, cell_x, cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map, 0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示市场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, Eigen::Vector2d& coords)
{

    Eigen::Vector3d ans;
    //TODO
    double x = coords[0];
    double y = coords[1];
    // int ind_x = MAP_GXWX(map, x);
    // int ind_y = MAP_GYWY(map, y);
    int ind_x = floor((x - map->origin_x) / map->resolution) + map->size_x / 2;
    int ind_y = floor((y - map->origin_y) / map->resolution) + map->size_y / 2;

    if (!MAP_VALID(map, ind_x, ind_y) or !MAP_VALID(map, ind_x + 1, ind_y + 1)) {
        std::cout << "WARN, coords outside map" << std::endl;
        return ans;
    }

    double z1 = map->cells[MAP_INDEX(map, ind_x, ind_y)].score;
    double z2 = map->cells[MAP_INDEX(map, ind_x + 1, ind_y)].score;
    double z3 = map->cells[MAP_INDEX(map, ind_x, ind_y + 1)].score;
    double z4 = map->cells[MAP_INDEX(map, ind_x + 1, ind_y + 1)].score;

    double x0 = MAP_WXGX(map, ind_x);
    double x1 = MAP_WXGX(map, ind_x + 1);
    double y0 = MAP_WYGY(map, ind_y);
    double y1 = MAP_WYGY(map, ind_y + 1);

    ans[0] = (x - x1) / (x0 - x1) * (y
        - y1) / (y0 - y1) * z1
        + (x - x0) / (x1 - x0) * (y
            - y1) / (y0 - y1) * z2
        + (x - x0) / (x1 - x0) * (y
            - y0) / (y1 - y0) * z3
        + (x - x1) / (x0 - x1) * (y
            - y0) / (y1 - y0) * z4;
    ans[1] = 1 / (x0 - x1) * (y - y1)
        / (y0 - y1) * z1
        + 1 / (x1 - x0) * (y - y1)
        / (y0 - y1) * z2
        + 1 / (x1 - x0) * (y - y0)
        / (y1 - y0) * z3
        + 1 / (x0 - x1) * (y - y0)
        / (y1 - y0) * z4;
    ans[2] = (x - x1) / (x0 - x1)
        * 1 / (y0 - y1) * z1
        + (x - x0) / (x1 - x0)
        * 1 / (y0 - y1) * z2
        + (x - x0) / (x1 - x0)
        * 1 / (y1 - y0) * z3
        + (x - x1) / (x0 - x1)
        * 1 / (y1 - y0) * z4;
    //END OF TODO
    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
    std::vector<Eigen::Vector2d>& laser_pts,
    Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO
    for (size_t i = 0; i < laser_pts.size(); ++i) {
        Eigen::Vector2d point = laser_pts[i]; // x
        Eigen::Matrix3d transform_matrix = GN_V2T(now_pose);
        Eigen::Vector2d transformed_point = GN_TransPoint(point, transform_matrix);

        Eigen::Vector3d ans = InterpMapValueWithDerivatives(map, transformed_point);

        double px = point[0];
        double py = point[1];
        double theta = now_pose[2];

        double f = 1 - ans[0];

        Eigen::Matrix<double, 2, 3> dS_dT;
        dS_dT << 1, 0, -sin(theta) * px - cos(theta) * py,
            0, 1, cos(theta)* px - sin(theta) * py;

        Eigen::Vector2d dM_dx_dy;
        dM_dx_dy << ans[1], ans[2];

        Eigen::Vector3d J = (dM_dx_dy.transpose() * dS_dT).transpose();

        H += J * J.transpose();
        b += J * f;
    }

    //END OF TODO
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t* map, Eigen::Vector3d& init_pose, std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;

    for (int i = 0; i < maxIteration;i++)
    {
        //TODO
        Eigen::Vector3d b;
        Eigen::Matrix3d H;
        ComputeHessianAndb(map, now_pose, laser_pts, H, b);

        // Eigen::Vector3d delta_T = H.colPivHouseholderQr().solve(b);
        Eigen::EigenSolver<Eigen::Matrix3d> solver(H);
        Eigen::Vector3cd H_eigen_values = solver.eigenvalues();
        double lambda_min = std::min({H_eigen_values[0].real(), H_eigen_values[1].real(), H_eigen_values[2].real()});

        if (lambda_min <= 0) {
        std::cout << "lambda: " << lambda_min << std::endl;
            H += (-lambda_min + 1) * Eigen::Matrix3d::Identity();
        }
        Eigen::Vector3d delta_T = H.inverse() * b;

        now_pose += delta_T;

        //END OF TODO
    }
    init_pose = now_pose;

}
