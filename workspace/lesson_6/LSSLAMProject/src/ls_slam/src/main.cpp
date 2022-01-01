#include <gaussian_newton.h>
#include <readfile.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "timer.hpp"


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

    ros::NodeHandle nodeHandle;

    // beforeGraph
    ros::Publisher beforeGraphPub, afterGraphPub;
    beforeGraphPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("beforePoseGraph", 1, true);
    afterGraphPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("afterPoseGraph", 1, true);

    // std::string fileName = "intel";
    // std::string fileName = "killian";
    std::string fileName = "test_quadrat";
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

    double initError = ComputeError(Vertexs, Edges);
    std::cout << "initError:" << initError << std::endl;

    int maxIteration = 100;
    double epsilon = 1e-4;
    double lastError = -1;

    double t_solve = .0;
    double t_update = .0;

    for (int i = 0; i < maxIteration;i++)
    {
        std::cout << "======Iterations:" << i << "======" << std::endl;
        timer.Start();
        Eigen::VectorXd dx = LinearizeAndSolve(Vertexs, Edges);
        t = timer.Stop();
        t_solve += t;
        std::cout << "Duration for optimazation: " << t << std::endl;

        //进行更新
        //TODO--Start
        timer.Start();
        for (int j = 0; j < Vertexs.size(); ++j) {
            Vertexs[j] += dx.block<3, 1>(j * 3, 0);
        }
        t = timer.Stop();
        t_update += t;
        std::cout << "Duration for update: " << t << std::endl;

        //TODO--End

        double maxError = -1;
        for (int k = 0; k < 3 * Vertexs.size();k++)
        {
            if (maxError < std::fabs(dx(k)))
            {
                maxError = std::fabs(dx(k));
            }
        }
        if (maxError > lastError) {
            std::cout << "! Error increase:, current error: " << maxError << std::endl;
            lastError = maxError;
        }
        if (maxError < epsilon)
            break;
    }


    double finalError = ComputeError(Vertexs, Edges);

    std::cout << "========================\nFinalError:" << finalError << std::endl;

    t_solve /= maxIteration;
    t_update /= maxIteration;

    std::cout << "Duration for reading: " << t_read << std::endl;
    std::cout << "Average Optimazation Time: " << t_solve << std::endl;
    std::cout << "Average Update Time: " << t_update << std::endl;

    PublishGraphForVisulization(&afterGraphPub,
        Vertexs,
        Edges, 1);

    ros::spin();



    return 0;
}




