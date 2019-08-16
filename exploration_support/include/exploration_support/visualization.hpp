#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

//topicの情報をrvizで表示するためのクラス
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/construct.hpp>
#include <thread>
#include <nav_msgs/Path.h>
#include <exploration_msgs/FrontierArray.h>

// goal_array -> frontier, branch

class Visualization
{
private:
    double POSE_PUBLISH_RATE;
    double GOAL_PUBLISH_RATE;
    double BRANCH_PUBLISH_RATE;
    double FRONTIER_PUBLISH_RATE;
    std::string MAP_FRAME_ID;

    //pose
    ExpLib::Struct::subStructSimple pose_;
    ExpLib::Struct::pubStruct<nav_msgs::Path> posePath_;   
    nav_msgs::Path pp_;

    //goal
    ExpLib::Struct::subStructSimple goal_;
    ExpLib::Struct::pubStruct<visualization_msgs::Marker> goalMarker_;
    visualization_msgs::Marker gm_;

    // branch
    ExpLib::Struct::subStructSimple branch_;
    ExpLib::Struct::pubStruct<visualization_msgs::Marker> branchMarker_;
    visualization_msgs::Marker bm_;

    // frontier
    ExpLib::Struct::subStructSimple frontier_;
    ExpLib::Struct::pubStruct<visualization_msgs::Marker> frontierMarker_;
    visualization_msgs::Marker fm_;
    
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCB(const geometry_msgs::PointStamped::ConstPtr& msg);
    void branchCB(const exploration_msgs::PointArray::ConstPtr& msg);
    void frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg);

    void posePathPublisher(void);
    void goalMarkerPublisher(void);
    void branchMarkerPublisher(void);
    void frontierMarkerPublisher(void);

public:
    Visualization();
    void multiThreadMain(void);
};

Visualization::Visualization()
    :pose_("pose",1,&Visualization::poseCB, this)
    ,posePath_("visualization/pose", 1)
    ,goal_("goal",1,&Visualization::goalCB, this)
    ,goalMarker_("visualization/goal", 1)
    ,branch_("branch",1,&Visualization::branchCB, this)
    ,branchMarker_("visualization/branch", 1)
    ,frontier_("frontier",1,&Visualization::frontierCB, this)
    ,frontierMarker_("visualization/frontier", 1){

    ros::NodeHandle p("~");
    p.param<double>("pose_publish_rate", POSE_PUBLISH_RATE, 10.0);
    p.param<double>("goal_publish_rate", GOAL_PUBLISH_RATE, 10.0);
    p.param<double>("branch_publish_rate", BRANCH_PUBLISH_RATE, 10.0);
    p.param<double>("frontier_publish_rate", FRONTIER_PUBLISH_RATE, 10.0);
    
    std::string INIT_FRAME_ID = "robot1/map";

    //goalMarker
    gm_.header.frame_id = INIT_FRAME_ID;
    gm_.pose.orientation.w = 1.0;
    gm_.scale.x = gm_.scale.y = gm_.scale.z = 0.5;
    gm_.pose.position.z = 0;
    gm_.type = visualization_msgs::Marker::CUBE;
    gm_.action = visualization_msgs::Marker::ADD;
    gm_.lifetime = ros::Duration(0);
    gm_.color.r = 1.0f;
    gm_.color.g = 0.0f;
    gm_.color.b = 1.0f;
    gm_.color.a = 1.0f;

    //branchMarker
    bm_.header.frame_id = INIT_FRAME_ID;
    bm_.pose.orientation.w = 1.0;
    bm_.scale.x = bm_.scale.y = bm_.scale.z = 0.5;
    bm_.type = visualization_msgs::Marker::CUBE_LIST;
    bm_.action = visualization_msgs::Marker::ADD;
    bm_.lifetime = ros::Duration(0);
    bm_.color.r = 1.0f;
    bm_.color.g = 1.0f;
    bm_.color.b = 0.0f;
    bm_.color.a = 1.0f;

    //frontierMarker
    fm_.header.frame_id = INIT_FRAME_ID;
    fm_.pose.orientation.w = 1.0;
    fm_.scale.x = fm_.scale.y = fm_.scale.z = 0.5;
    fm_.type = visualization_msgs::Marker::CUBE_LIST;
    fm_.action = visualization_msgs::Marker::ADD;
    fm_.lifetime = ros::Duration(0);
    fm_.color.r = 0.0f;
    fm_.color.g = 1.0f;
    fm_.color.b = 1.0f;
    fm_.color.a = 1.0f;
}

void Visualization::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pp_.poses.push_back(*msg);
    pp_.header.frame_id = msg->header.frame_id;
    pp_.header.stamp = ros::Time::now();
}

void Visualization::goalCB(const geometry_msgs::PointStamped::ConstPtr& msg){
    gm_.pose.position = msg->point;
    gm_.header.frame_id = msg->header.frame_id;
    gm_.header.stamp = ros::Time::now();
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

void Visualization::multiThreadMain(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread ppThread([this]() { posePathPublisher(); });
    std::thread gmThread([this]() { goalMarkerPublisher(); });
    std::thread bmThread([this]() { branchMarkerPublisher(); });
    std::thread fmThread([this]() { frontierMarkerPublisher(); });
    ros::spin();
    ppThread.join();//スレッドの終了を待つ
    gmThread.join();
    bmThread.join();
    fmThread.join();
    ROS_INFO_STREAM("end main loop\n");
}

#endif //VISUALIZATION_HPP