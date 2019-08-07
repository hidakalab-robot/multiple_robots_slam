#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

//topicの情報をrvizで表示するためのクラス
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/constructor.hpp>
#include <thread>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

class Visualization
{
private:
    double POSE_PUBLISH_RATE;
    double GOAL_PUBLISH_RATE;
    double GOALARRAY_PUBLISH_RATE;
    std::string MAP_FRAME_ID;

    //pose
    ExpLib::subStructSimple pose_;
    // ExpLib::pubStruct<visualization_msgs::Marker> poseMarker_;
    ExpLib::pubStruct<nav_msgs::Path> posePath_;   
    // visualization_msgs::Marker pm;
    nav_msgs::Path pp;

    //goal
    ExpLib::subStructSimple goal_;
    ExpLib::pubStruct<visualization_msgs::Marker> goalMarker_;
    visualization_msgs::Marker gm;

    //goalList
    ExpLib::subStructSimple goalArray_;
    ExpLib::pubStruct<visualization_msgs::Marker> goalArrayMarker_;
    visualization_msgs::Marker gam;

    tf::TransformListener listener;
    
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCB(const geometry_msgs::PointStamped::ConstPtr& msg);
    void goalArrayCB(const exploration_msgs::PointArray::ConstPtr& msg);

    void poseMarkerPublisher(void);
    void goalMarkerPublisher(void);
    void goalArrayMarkerPublisher(void);

public:
    Visualization();
    void multiThreadMain(void);
};

Visualization::Visualization()
    :pose_("pose",1,&Visualization::poseCB, this)
    // ,poseMarker_("visualization", 1)
    ,posePath_("visualization/pose", 1)
    ,goal_("goal",1,&Visualization::goalCB, this)
    ,goalMarker_("visualization/goal", 1)
    ,goalArray_("goal_array",1,&Visualization::goalArrayCB, this)
    ,goalArrayMarker_("visualization/goal_array", 1){

    ros::NodeHandle p("~");
    p.param<double>("pose_publish_rate", POSE_PUBLISH_RATE, 30.0);
    p.param<double>("goal_publish_rate", GOAL_PUBLISH_RATE, 30.0);
    p.param<double>("goalarray_publish_rate", GOALARRAY_PUBLISH_RATE, 30.0);
    std::string MAP_FRAME_ID;
    p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    //poseMarker
    // p.param<double>("line_width", pm.scale.x, 0.1);
    // pm.header.frame_id = MAP_FRAME_ID;
    // pm.pose.orientation.w = 1.0;
    // pm.type = visualization_msgs::Marker::LINE_STRIP;
    // pm.action = visualization_msgs::Marker::ADD;
    // pm.lifetime = ros::Duration(0);
    // pm.ns = "pose";
    // pm.id = 0;
    // pm.color.r = 0.0f;
    // pm.color.g = 0.0f;
    // pm.color.b = 1.0f;
    // pm.color.a = 1.0f;
    //goalMarker
    double goalScale;
    p.param<double>("goal_scale", goalScale, 0.5);
    gm.header.frame_id = MAP_FRAME_ID;
    gm.pose.orientation.w = 1.0;
    gm.scale.x = gm.scale.y = gm.scale.z = goalScale;
    gm.pose.position.z = 0;
    gm.type = visualization_msgs::Marker::CUBE;
    gm.action = visualization_msgs::Marker::ADD;
    gm.lifetime = ros::Duration(0);
    gm.ns = "goal";
    gm.id = 0;
    gm.color.r = 1.0f;
    gm.color.g = 0.0f;
    gm.color.b = 1.0f;
    gm.color.a = 1.0f;
    //goalArrayMarker
    double goalArrayScale;
    p.param<double>("goal_array_scale", goalArrayScale, 0.3);
    gam.header.frame_id = MAP_FRAME_ID;
    gam.pose.orientation.w = 1.0;
    gam.scale.x = gam.scale.y = gam.scale.z = goalArrayScale;
    gam.type = visualization_msgs::Marker::CUBE_LIST;
    gam.action = visualization_msgs::Marker::ADD;
    gam.lifetime = ros::Duration(0);
    gam.ns = "goalArray";
    gam.id = 0;
    gam.color.r = 1.0f;
    gam.color.g = 1.0f;
    gam.color.b = 0.0f;
    gam.color.a = 1.0f;
}

void Visualization::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // pm.points.push_back(ExpLib::msgPoint(msg -> pose.position.x,msg -> pose.position.y,msg -> pose.position.z));
    // pm.header.frame_id = msg->header.frame_id;
    // pm.header.stamp = ros::Time::now();
    pp.poses.push_back(*msg);
    pp.header.frame_id = msg->header.frame_id;
    pp.header.stamp = ros::Time::now();
}

void Visualization::goalCB(const geometry_msgs::PointStamped::ConstPtr& msg){
    gm.pose.position = msg->point;
    gm.header.frame_id = msg->header.frame_id;
    gm.header.stamp = ros::Time::now();
}

void Visualization::goalArrayCB(const exploration_msgs::PointArray::ConstPtr& msg){
    gam.points = msg->points;
    gam.header.frame_id = msg->header.frame_id;
    gam.header.stamp = ros::Time::now();
}

void Visualization::poseMarkerPublisher(void){
    ros::Rate rate(POSE_PUBLISH_RATE);
    while(ros::ok()){
        // poseMarker_.pub.publish(pm);
        posePath_.pub.publish(pp);
        rate.sleep();
    }
}

void Visualization::goalMarkerPublisher(void){
    ros::Rate rate(GOAL_PUBLISH_RATE);
    while(ros::ok()){
        goalMarker_.pub.publish(gm);
        rate.sleep();
    }
}

void Visualization::goalArrayMarkerPublisher(void){
    ros::Rate rate(GOALARRAY_PUBLISH_RATE);
    while(ros::ok()){
        goalArrayMarker_.pub.publish(gam);
        rate.sleep();
    }
}

void Visualization::multiThreadMain(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread pmThread([this]() { poseMarkerPublisher(); });
    std::thread gmThread([this]() { goalMarkerPublisher(); });
    std::thread gamThread([this]() { goalArrayMarkerPublisher(); });
    ros::spin();
    pmThread.join();//スレッドの終了を待つ
    gmThread.join();
    gamThread.join();
    ROS_INFO_STREAM("end main loop\n");
}

#endif //VISUALIZATION_HPP