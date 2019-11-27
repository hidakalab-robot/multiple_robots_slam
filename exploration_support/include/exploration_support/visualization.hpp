#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <exploration_libraly/construct.hpp>
#include <exploration_libraly/struct.hpp>
#include <exploration_msgs/FrontierArray.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <actionlib_msgs/GoalStatusArray.h>
#include <visualization_msgs/Marker.h>

namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;

class Visualization
{
private:
    // static parameters
    std::string INIT_FRAME_ID;
    double POSE_PUBLISH_RATE;
    double GOAL_PUBLISH_RATE;
    double BRANCH_PUBLISH_RATE;
    double FRONTIER_PUBLISH_RATE;
    double USEFUL_FRONTIER_PUBLISH_RATE;
    double ROAD_PUBLISH_RATE;
    
    // variables
    // pose
    ExStc::subStructSimple pose_;
    ExStc::pubStruct<nav_msgs::Path> posePath_;   
    nav_msgs::Path pp_;

    // goal
    ExStc::subStructSimple goal_;
    ExStc::pubStruct<visualization_msgs::Marker> goalMarker_;
    visualization_msgs::Marker gm_;
    ExStc::subStructSimple goSt_;
    std::mutex gmMutex_;

    // branch
    ExStc::subStructSimple branch_;
    ExStc::pubStruct<visualization_msgs::Marker> branchMarker_;
    visualization_msgs::Marker bm_;

    // frontier
    ExStc::subStructSimple frontier_;
    ExStc::pubStruct<visualization_msgs::Marker> frontierMarker_;
    visualization_msgs::Marker fm_;

    // useful frontier
    ExStc::subStructSimple useFro_;
    ExStc::pubStruct<visualization_msgs::Marker> useFroMarker_;
    visualization_msgs::Marker ufm_;

    // road
    ExStc::subStructSimple road_;
    ExStc::pubStruct<visualization_msgs::Marker> roadMarker_;
    visualization_msgs::Marker rm_;
    std::mutex rmMutex_;
    
    // functions
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCB(const geometry_msgs::PointStamped::ConstPtr& msg);
    void goalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void branchCB(const exploration_msgs::PointArray::ConstPtr& msg);
    void frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg);
    void useFroCB(const exploration_msgs::FrontierArray::ConstPtr& msg);
    void roadCB(const geometry_msgs::PointStamped::ConstPtr& msg);
    void posePathPublisher(void);
    void goalMarkerPublisher(void);
    void branchMarkerPublisher(void);
    void frontierMarkerPublisher(void);
    void useFroMarkerPublisher(void);
    void roadMarkerPublisher(void);
    void loadParams(void);

public:
    Visualization();
    void multiThreadMain(void);
};

Visualization::Visualization()
    :pose_("pose",1,&Visualization::poseCB, this)
    ,posePath_("visualization/pose", 1)
    ,goal_("goal",1,&Visualization::goalCB, this)
    ,goalMarker_("visualization/goal", 1, true)
    ,goSt_("move_base/status",1,&Visualization::goalStatusCB, this)
    ,branch_("branch",1,&Visualization::branchCB, this)
    ,branchMarker_("visualization/branch", 1, true)
    ,frontier_("frontier",1,&Visualization::frontierCB, this)
    ,frontierMarker_("visualization/frontier", 1, true)
    ,useFro_("useful_frontier",1,&Visualization::useFroCB, this)
    ,useFroMarker_("visualization/useful_frontier", 1, true)
    ,road_("road",1,&Visualization::roadCB, this)
    ,roadMarker_("visualization/road", 1, true){
    gm_ = ExCos::msgCubeListMarker(INIT_FRAME_ID,0.5,1.0,0.0,1.0);
    bm_ = ExCos::msgCubeListMarker(INIT_FRAME_ID,0.5,1.0,1.0,0.0);
    fm_ = ExCos::msgCubeListMarker(INIT_FRAME_ID,0.5,0.0,1.0,1.0);
    ufm_ = ExCos::msgCubeListMarker(INIT_FRAME_ID,0.5,1.0,0.5,0.5);
    rm_ = ExCos::msgCubeListMarker(INIT_FRAME_ID,0.5,0.5,0.5,1.0);
}

void Visualization::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pp_.poses.push_back(*msg);
    pp_.header.frame_id = msg->header.frame_id;
    pp_.header.stamp = ros::Time::now();
}

// goalに着いたのを検知してマーカーを消したい
void Visualization::goalCB(const geometry_msgs::PointStamped::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(gmMutex_);
    gm_.points = ExCos::oneFactorVector(msg->point);
    gm_.header.frame_id = msg->header.frame_id;
    gm_.header.stamp = ros::Time::now();
}

void Visualization::goalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(!msg->status_list.empty() && (msg->status_list.back().status > 2)){
        std::lock_guard<std::mutex> lock(gmMutex_);
        gm_.points = std::vector<geometry_msgs::Point>();
        gm_.header.stamp = ros::Time::now();
    }
    else{
        std::lock_guard<std::mutex> lock(rmMutex_);
        rm_.points = std::vector<geometry_msgs::Point>();
        rm_.header.stamp = ros::Time::now();
    }
}

void Visualization::branchCB(const exploration_msgs::PointArray::ConstPtr& msg){
    bm_.points = msg->points;
    bm_.header.frame_id = msg->header.frame_id;
    bm_.header.stamp = ros::Time::now();
}

void Visualization::frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg){
    auto replace = [&msg]{
        std::vector<geometry_msgs::Point> p;
        p.reserve(msg->frontiers.size());
        for(auto&& f : msg->frontiers) p.emplace_back(f.point); 
        return p;
    };
    fm_.points = replace();
    fm_.header.frame_id = msg->header.frame_id;
    fm_.header.stamp = ros::Time::now();
}

void Visualization::useFroCB(const exploration_msgs::FrontierArray::ConstPtr& msg){
    auto replace = [&msg]{
        std::vector<geometry_msgs::Point> p;
        p.reserve(msg->frontiers.size());
        for(auto&& f : msg->frontiers) p.emplace_back(f.point); 
        return p;
    };
    ufm_.points = replace();
    ufm_.header.frame_id = msg->header.frame_id;
    ufm_.header.stamp = ros::Time::now();
}

// 本当に空になってるか確認
void Visualization::roadCB(const geometry_msgs::PointStamped::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(rmMutex_);
    rm_.points = msg->header.frame_id != "" ? ExCos::oneFactorVector(msg->point) : std::vector<geometry_msgs::Point>();
    rm_.header.frame_id = msg->header.frame_id != "" ? msg->header.frame_id : INIT_FRAME_ID;
    rm_.header.stamp = ros::Time::now();
}

void Visualization::posePathPublisher(void){
    ros::Rate rate(POSE_PUBLISH_RATE);
    while(ros::ok()){
        posePath_.pub.publish(pp_);
        rate.sleep();
    }
}

void Visualization::goalMarkerPublisher(void){
    ros::Rate rate(GOAL_PUBLISH_RATE);
    while(ros::ok()){
        goalMarker_.pub.publish(gm_);
        rate.sleep();
    }
}

void Visualization::branchMarkerPublisher(void){
    ros::Rate rate(BRANCH_PUBLISH_RATE);
    while(ros::ok()){
        branchMarker_.pub.publish(bm_);
        rate.sleep();
    }
}

void Visualization::frontierMarkerPublisher(void){
    ros::Rate rate(FRONTIER_PUBLISH_RATE);
    while(ros::ok()){
        frontierMarker_.pub.publish(fm_);
        rate.sleep();
    }
}

void Visualization::useFroMarkerPublisher(void){
    ros::Rate rate(USEFUL_FRONTIER_PUBLISH_RATE);
    while(ros::ok()){
        useFroMarker_.pub.publish(ufm_);
        rate.sleep();
    }
}

void Visualization::roadMarkerPublisher(void){
    ros::Rate rate(ROAD_PUBLISH_RATE);
    while(ros::ok()){
        roadMarker_.pub.publish(rm_);
        rate.sleep();
    }
}

void Visualization::multiThreadMain(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread ppThread([this]() { posePathPublisher(); });
    std::thread gmThread([this]() { goalMarkerPublisher(); });
    std::thread bmThread([this]() { branchMarkerPublisher(); });
    std::thread fmThread([this]() { frontierMarkerPublisher(); });
    std::thread ufmThread([this]() { useFroMarkerPublisher(); });
    std::thread rmThread([this]() { roadMarkerPublisher(); });
    ros::spin();
    ppThread.join();//スレッドの終了を待つ
    gmThread.join();
    bmThread.join();
    fmThread.join();
    ufmThread.join();
    rmThread.join();
    ROS_INFO_STREAM("end main loop\n");
}

void Visualization::loadParams(void){
    ros::NodeHandle nh("~");
    // static parameters
    nh.param<std::string>("init_frame_id", INIT_FRAME_ID, "robot1/map");
    nh.param<double>("pose_publish_rate", POSE_PUBLISH_RATE, 10.0);
    nh.param<double>("goal_publish_rate", GOAL_PUBLISH_RATE, 10.0);
    nh.param<double>("branch_publish_rate", BRANCH_PUBLISH_RATE, 10.0);
    nh.param<double>("frontier_publish_rate", FRONTIER_PUBLISH_RATE, 10.0);
    nh.param<double>("useful_frontier_publish_rate", USEFUL_FRONTIER_PUBLISH_RATE, 10.0);
    nh.param<double>("road_publish_rate", ROAD_PUBLISH_RATE, 10.0);
}

#endif //VISUALIZATION_HPP