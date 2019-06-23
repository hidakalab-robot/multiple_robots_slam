#ifndef MULTI_EXPLORATION
#define MULTI_EXPLORATION

/** 
 * このクラスに内包したいシミュレータ
 * 
 * @param 地図
 * @param 各ロボットの位置姿勢
 * @param 分岐領域の座標
 * @param 分岐領域を発見したロボット
 * @param 未探査領域の座標
 * @return 分岐を発見したロボットが探査を行うべきか
*/
//dynamicReconfigureでいろいろ軸を動かしたりできるように
//可変パラメータ
/*
ロボットの台数
各ロボットの位置姿勢
分岐領域の個数
分岐領域の座標
未探査領域の個数
未探査領域の座標

 */
//評価などの関数は外部のを参照できるようにする
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <exploration/common_lib.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <exploration/multi_exploration_simulatorConfig.h>

class MultiExplorationSimulator
{
private:
    //パラメータをvectorで保管する
    ros::NodeHandle nh;

    int OLD_ROBOT_NUMBER;
    int OLD_BRANCH_NUMBER;
    int OLD_FRONTIER_NUMBER;
    
    std::string GLOBAL_FRAME;

    geometry_msgs::PoseArray robotPoses;
    visualization_msgs::Marker branchCoordinates;
    visualization_msgs::Marker frontierCoordinates;

    CommonLib::pubStruct<geometry_msgs::PoseArray> poses_;
    CommonLib::pubStruct<visualization_msgs::Marker> branches_;
    CommonLib::pubStruct<visualization_msgs::Marker> frontiers_;

public:
    MultiExplorationSimulator();
    void callback(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level);
    // void initialize(void);
};

MultiExplorationSimulator::MultiExplorationSimulator()
    :nh("~")
    ,poses_("pose_array",1,true)
    ,branches_("branch_array",1,true)
    ,frontiers_("frontier_array",1,true){

    nh.param<std::string>("global_frame",GLOBAL_FRAME,"map");
    robotPoses.header.frame_id = branchCoordinates.header.frame_id = frontierCoordinates.header.frame_id = GLOBAL_FRAME;

    double BRANCH_SCALE,FRONTIER_SCALE;
    nh.param<double>("branch_scale", BRANCH_SCALE, 0.5);
    nh.param<double>("frontier_scale", FRONTIER_SCALE, 0.5);

    // for branch parameter
    branchCoordinates.ns = "branch_array";
    branchCoordinates.scale.x = branchCoordinates.scale.y = branchCoordinates.scale.z = BRANCH_SCALE;
    branchCoordinates.color.r = 1.0f;
    branchCoordinates.color.g = 1.0f;
    branchCoordinates.color.b = 0.0f;
    branchCoordinates.color.a = 1.0f;

    // for frontier parameter
    frontierCoordinates.ns = "frontier_array";
    frontierCoordinates.scale.x = frontierCoordinates.scale.y = frontierCoordinates.scale.z = FRONTIER_SCALE;
    frontierCoordinates.color.r = 0.0f;
    frontierCoordinates.color.g = 1.0f;
    frontierCoordinates.color.b = 1.0f;
    frontierCoordinates.color.a = 1.0f;

    // common parameter
    branchCoordinates.pose.orientation.w = frontierCoordinates.pose.orientation.w = 1.0;
    branchCoordinates.type = frontierCoordinates.type = visualization_msgs::Marker::CUBE_LIST;
    branchCoordinates.action = frontierCoordinates.action = visualization_msgs::Marker::ADD;
    branchCoordinates.lifetime = frontierCoordinates.lifetime = ros::Duration(0);
    branchCoordinates.id = frontierCoordinates.id =  0;
}

void MultiExplorationSimulator::callback(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level) {
    // resize array
    // int ROBOT_NUMBER = cfg.robot_number;
    // int BRANCH_NUMBER = cfg.branch_number;
    // int FRONTIER_NUMBER = cfg.frontier_number;
    robotPoses.poses.resize(cfg.robot_number);
    branchCoordinates.points.resize(cfg.branch_number);
    frontierCoordinates.points.resize(cfg.frontier_number);

    // // delete no　use parameter // no debug 
    // for(int i=cfg.robot_number+1;i<=OLD_ROBOT_NUMBER;++i) nh.deleteParam("robot"+std::to_string(i));
    // for(int i=cfg.branch_number+1;i<=OLD_BRANCH_NUMBER;++i) nh.deleteParam("branch"+std::to_string(i));
    // for(int i=cfg.frontier_number+1;i<=OLD_FRONTIER_NUMBER;++i) nh.deleteParam("frontier"+std::to_string(i));

    // read parameter
    for(int i=1;i<=cfg.robot_number;++i){
        double x,y,yaw;
        nh.param<double>("robots/robot"+std::to_string(i)+"/robot"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("robots/robot"+std::to_string(i)+"/robot"+std::to_string(i)+"_y",y,0.0);
        nh.param<double>("robots/robot"+std::to_string(i)+"/robot"+std::to_string(i)+"_yaw",yaw,0.0);
        robotPoses.poses[i] = CommonLib::msgPose(CommonLib::msgPoint(x,y),CommonLib::yawToQ(yaw));
    }

    for(int i=1;i<=cfg.branch_number;++i){
        double x,y;
        nh.param<double>("branches/branch"+std::to_string(i)+"/branch"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("branches/branch"+std::to_string(i)+"/branch"+std::to_string(i)+"_y",y,0.0);
        branchCoordinates.points[i] = CommonLib::msgPoint(x,y);
    }

    for(int i=1;i<=cfg.frontier_number;++i){
        double x,y;
        nh.param<double>("frontiers/frontier"+std::to_string(i)+"/frontier"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("frontiers/frontier"+std::to_string(i)+"/frontier"+std::to_string(i)+"_y",y,0.0);
        frontierCoordinates.points[i] = CommonLib::msgPoint(x,y);
    }

    // publish for rviz
    poses_.pub.publish(robotPoses);
    branches_.pub.publish(branchCoordinates);
    frontiers_.pub.publish(frontierCoordinates);

    // update OLD NUMBERS
    OLD_ROBOT_NUMBER = cfg.robot_number;
    OLD_BRANCH_NUMBER = cfg.branch_number;
    OLD_FRONTIER_NUMBER = cfg.frontier_number;

}

// void MultiExplorationSimulator::initialize(void){
//     // resize array
//     int ROBOT_NUMBER, BRANCH_NUMBER, FRONTIER_NUMBER;
//     nh.param<int>("robot_number",ROBOT_NUMBER,0);
//     nh.param<int>("branch_number",BRANCH_NUMBER,0);
//     nh.param<int>("frontier_number",FRONTIER_NUMBER,0);

//     robotPoses.poses.resize(ROBOT_NUMBER);
//     branchCoordinates.points.resize(BRANCH_NUMBER);
//     frontierCoordinates.points.resize(FRONTIER_NUMBER);

//     // delete no　use parameter
//     for(int i=ROBOT_NUMBER+1;i<=OLD_ROBOT_NUMBER;++i) nh.deleteParam("robot"+std::to_string(i));
//     for(int i=BRANCH_NUMBER+1;i<=OLD_BRANCH_NUMBER;++i) nh.deleteParam("branch"+std::to_string(i));
//     for(int i=FRONTIER_NUMBER+1;i<=OLD_FRONTIER_NUMBER;++i) nh.deleteParam("frontier"+std::to_string(i));

//     // read parameter
//     for(int i=1;i<=ROBOT_NUMBER;++i){
//         double x,y,yaw;
//         nh.param<double>("robot"+std::to_string(i)+"/x",x,0.0);
//         nh.param<double>("robot"+std::to_string(i)+"/y",y,0.0);
//         nh.param<double>("robot"+std::to_string(i)+"/yaw",yaw,0.0);
//         robotPoses.poses[i] = CommonLib::msgPose(CommonLib::msgPoint(x,y),CommonLib::yawToQ(yaw));
//     }

//     for(int i=1;i<=BRANCH_NUMBER;++i){
//         double x,y;
//         nh.param<double>("branch"+std::to_string(i)+"/x",x,0.0);
//         nh.param<double>("branch"+std::to_string(i)+"/y",y,0.0);
//         branchCoordinates.points[i] = CommonLib::msgPoint(x,y);
//     }

//     for(int i=1;i<=FRONTIER_NUMBER;++i){
//         double x,y;
//         nh.param<double>("frontier"+std::to_string(i)+"/x",x,0.0);
//         nh.param<double>("frontier"+std::to_string(i)+"/y",y,0.0);
//         frontierCoordinates.points[i] = CommonLib::msgPoint(x,y);
//     }

//     // publish for rviz
//     poses_.pub.publish(robotPoses);
//     branches_.pub.publish(branchCoordinates);
//     frontiers_.pub.publish(frontierCoordinates);

//     // update OLD NUMBERS
//     OLD_ROBOT_NUMBER = ROBOT_NUMBER;
//     OLD_BRANCH_NUMBER = BRANCH_NUMBER;
//     OLD_FRONTIER_NUMBER = FRONTIER_NUMBER;
// }


#endif // MULTI_EXPLORATION