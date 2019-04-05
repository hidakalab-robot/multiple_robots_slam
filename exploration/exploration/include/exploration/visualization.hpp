#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

//topicの情報をrvizで表示するためのクラス
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <exploration/common_lib.hpp>

class Visualization
{
private:
    //共通変数
    ros::NodeHandle p;

    //pose
    CommonLib::subStructSimple pose_;
    CommonLib::pubStruct<visualization_msgs::Marker> poseMarker_;    
    visualization_msgs::Marker pm;

    //goal
    CommonLib::subStructSimple goal_;
    CommonLib::pubStruct<visualization_msgs::Marker> goalMarker_;
    visualization_msgs::Marker gm;

    //goalList
    CommonLib::subStructSimple goalArray_;
    CommonLib::pubStruct<visualization_msgs::Marker> goalArrayMarker_;
    visualization_msgs::Marker glm;
    
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCB(const geometry_msgs::PointStamped::ConstPtr& msg);
    void goalArrayCB(const exploration_msgs::PointArray::ConstPtr& msg);

public:
    Visualization();
};

Visualization::Visualization()
    :p("~")
    ,pose_("pose",1,&Visualization::poseCB, this)
    ,poseMarker_("visualization", 1)
    ,goal_("goal",1,&Visualization::goalCB, this)
    ,goalMarker_("visualization", 1)
    ,goalArray_("goal_Array",1,&Visualization::goalArrayCB, this)
    ,goalArrayMarker_("visualization", 1){

    std::string MAP_FRAME_ID;
    p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    //poseMarker
    p.param<double>("line_width", pm.scale.x, 0.1);
    pm.header.frame_id = MAP_FRAME_ID;
    pm.pose.orientation.w = 1.0;
    pm.type = visualization_msgs::Marker::LINE_STRIP;
    pm.action = visualization_msgs::Marker::ADD;
    pm.lifetime = ros::Duration(0);
    pm.ns = "pose";
    pm.id = 0;
    pm.color.r = 0.0f;
    pm.color.g = 0.0f;
    pm.color.b = 1.0f;
    pm.color.a = 1.0f;
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
    glm.header.frame_id = MAP_FRAME_ID;
    glm.pose.orientation.w = 1.0;
    glm.scale.x = glm.scale.y = glm.scale.z = goalArrayScale;
    glm.type = visualization_msgs::Marker::CUBE_LIST;
    glm.action = visualization_msgs::Marker::ADD;
    glm.lifetime = ros::Duration(0);
    glm.ns = "goalArray";
    glm.id = 0;
    glm.color.r = 1.0f;
    glm.color.g = 1.0f;
    glm.color.b = 0.0f;
    glm.color.a = 1.0f;
}

void Visualization::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pm.points.push_back(CommonLib::msgPoint(msg -> pose.position.x,msg -> pose.position.y,msg -> pose.position.z));
    pm.header.stamp = ros::Time::now();
    poseMarker_.pub.publish(pm);
}

void Visualization::goalCB(const geometry_msgs::PointStamped::ConstPtr& msg){
    gm.pose.position.x = msg -> point.x;
    gm.pose.position.y = msg -> point.y;
    gm.header.stamp = ros::Time::now();
    goalMarker_.pub.publish(gm);
}

void Visualization::goalArrayCB(const exploration_msgs::PointArray::ConstPtr& msg){
    glm.points = msg -> points;
    glm.header.stamp = ros::Time::now();
    goalArrayMarker_.pub.publish(glm);
}

#endif //VISUALIZATION_HPP