#ifndef VISUALIZATION_H
#define VISUALIZATION_H

//topicの情報をrvizで表示するためのクラス
#include <ros/ros.h>
#include <ros/callback_queue.h>
//#include <exploration_msgs/PointArray.h>
//#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/ToGoal.h>
#include <exploration_msgs/MoveAngle.h>
#include <exploration_msgs/Goal.h>
#include <exploration_msgs/GoalList.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>

class Visualization
{
private:
    //共通変数
    ros::NodeHandle p;
    std::string mapFrameId;

    //goal
    ros::NodeHandle sg;
    ros::Subscriber subGoal;
    ros::CallbackQueue qGoal;
    //std::string goalTopic;
    //geometry_msgs::PointStamped goalData;
    exploration_msgs::Goal goalData;
    visualization_msgs::Marker goalMarker;
    ros::NodeHandle pgm;
    ros::Publisher pubGoalMarker;
    //std::string goalMarkerTopic;
    

    //goalList
    ros::NodeHandle sgl;
    ros::Subscriber subGoalList;
    ros::CallbackQueue qGoalList;
    //std::string goalListTopic;
    //exploration_msgs::PointArray goalListData;
    exploration_msgs::GoalList goalListData;
    visualization_msgs::Marker goalListMarker;
    ros::NodeHandle pglm;
    ros::Publisher pubGoalListMarker;
    //std::string goalListMarkerTopic;
    
    //std_msgs::ColorRGBA goalListColor;

    //toGoal
    ros::NodeHandle stg;
    ros::Subscriber subToGoal;
    ros::CallbackQueue qToGoal;
    //std::string toGoalTopic;
    exploration_msgs::ToGoal toGoalData;
    visualization_msgs::Marker toGoalMarker;
    ros::NodeHandle ptgm;
    ros::Publisher pubToGoalMarker;
    //std::string toGoalMarkerTopic;
    
    //moveAngle
    ros::NodeHandle sma;
    ros::Subscriber subMoveAngle;
    ros::CallbackQueue qMoveAngle;
    //std::string moveAngleTopic;
    exploration_msgs::MoveAngle moveAngleData;
    visualization_msgs::Marker moveAngleMarker;
    ros::NodeHandle pmam;
    ros::Publisher pubMoveAngleMarker;
    //std::string moveAngleMarkerTopic;
    
    //pose
    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    //std::string poseTopic;
    geometry_msgs::PoseStamped poseData;
    visualization_msgs::Marker poseMarker;
    std::string explorationMethod;
    ros::NodeHandle ppm;
    ros::Publisher pubPoseMarker;
    //std::string poseMarkerTopic;
    std_msgs::ColorRGBA poseColor;

    bool inputGoal;
    bool inputGoalList;
    bool inputToGoal;
    bool inputMoveAngle;
    bool inputPose;

    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    //void goalCB(const geometry_msgs::PointStamped::ConstPtr& msg);
    //void goalListCB(const exploration_msgs::PointArray::ConstPtr& msg);
    void goalCB(const exploration_msgs::Goal::ConstPtr& msg);
    void goalListCB(const exploration_msgs::GoalList::ConstPtr& msg);
    void toGoalCB(const exploration_msgs::ToGoal::ConstPtr& msg);
    void moveAngleCB(const exploration_msgs::MoveAngle::ConstPtr& msg);

    double qToYaw(geometry_msgs::Quaternion q);

public:
    Visualization();
    ~Visualization(){};

    void poseMarkerInitialize(void);
    void publishPoseMarker(void);

    void goalMarkerInitialize(void);
    void publishGoalMarker(void);

    void goalListMarkerInitialize(void);
    void publishGoalListMarker(void);

    void toGoalMarkerInitialize(void);
    void publishToGoalMarker(void);

    void moveAngleMarkerInitialize(void);
    void publishMoveAngleMarker(void);
};

Visualization::Visualization():p("~"){
    p.param<std::string>("map_frame_id", mapFrameId, "map");
}

double Visualization::qToYaw(geometry_msgs::Quaternion q){
    tf::Quaternion temp(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(temp).getRPY(roll,pitch,yaw);
    return yaw;
}

void Visualization::poseMarkerInitialize(void){
    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&Visualization::poseCB, this);

    pubPoseMarker = ppm.advertise<visualization_msgs::Marker>("pose_marker", 1);

    // p.param<std::string>("pose_topic", poseTopic, "pose");
    // sp.setCallbackQueue(&qPose);
    // subPose = sp.subscribe(poseTopic,1,&Visualization::poseCB, this);

    // p.param<std::string>("pose_marker_topic", poseMarkerTopic, "pose_marker");
    // pubPoseMarker = ppm.advertise<visualization_msgs::Marker>(poseMarkerTopic, 1);
    double lineWidth;
    p.param<double>("line_width", lineWidth, 0.1);

    poseMarker.header.frame_id = mapFrameId;
    poseMarker.pose.orientation.w = 1.0;
    poseMarker.scale.x = lineWidth;
    poseMarker.type = visualization_msgs::Marker::LINE_STRIP;
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.lifetime = ros::Duration(0);
    poseMarker.ns = "pose";
    poseMarker.id = 0;

    poseColor.a = 1.0f;

    inputPose = false;
}

void Visualization::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	poseData = *msg;
    inputPose = true;
}

void Visualization::publishPoseMarker(void){
    qPose.callOne(ros::WallDuration(0.5));

    if(inputPose){
        inputPose = false;
    }
    else{
        return;
    }

    p.param<std::string>("exploration_method", explorationMethod, "SENSOR");
    
    if(explorationMethod == "SENSOR"){
        poseColor.r = 0.0f;
        poseColor.g = 1.0f;
        poseColor.b = 1.0f;
    }
    else {
        poseColor.r = 0.0f;
        poseColor.g = 0.0f;
        poseColor.b = 1.0f;
    }

    geometry_msgs::Point input;

    input.x = poseData.pose.position.x;
    input.y = poseData.pose.position.y;
    input.z = poseData.pose.position.z;

    poseMarker.points.push_back(input);
    poseMarker.colors.push_back(poseColor);

    poseMarker.header.stamp = ros::Time::now();

    pubPoseMarker.publish(poseMarker);
}


void Visualization::goalMarkerInitialize(void){
    sg.setCallbackQueue(&qGoal);
    subGoal = sg.subscribe("goal",1,&Visualization::goalCB, this);

    pubGoalMarker = pgm.advertise<visualization_msgs::Marker>("goal_marker", 1);

    // p.param<std::string>("goal_topic", goalTopic, "goal");
    // sg.setCallbackQueue(&qGoal);
    // subGoal = sg.subscribe(goalTopic,1,&Visualization::goalCB, this);

    // p.param<std::string>("goal_marker_topic", goalMarkerTopic, "goal_marker");
    // pubGoalMarker = pgm.advertise<visualization_msgs::Marker>(goalMarkerTopic, 1);
    double goalSize;
    p.param<double>("goal_size", goalSize, 0.5);

    goalMarker.header.frame_id = mapFrameId;
    goalMarker.pose.orientation.w = 1.0;
    goalMarker.scale.x = goalSize;
    goalMarker.scale.y = goalSize;
    goalMarker.scale.z = goalSize;
    goalMarker.pose.position.z = 0;
    goalMarker.type = visualization_msgs::Marker::CUBE;
    goalMarker.action = visualization_msgs::Marker::ADD;
    goalMarker.lifetime = ros::Duration(0);
    goalMarker.ns = "goal";
    goalMarker.id = 0;

    goalMarker.color.r = 1.0f;
    goalMarker.color.g = 0.0f;
    goalMarker.color.b = 1.0f;
    goalMarker.color.a = 1.0f;

    inputGoal = false;
}

// void Visualization::goalCB(const geometry_msgs::PointStamped::ConstPtr& msg){
void Visualization::goalCB(const exploration_msgs::Goal::ConstPtr& msg){
	goalData = *msg;
    inputGoal = true;
}

void Visualization::publishGoalMarker(void){
    qGoal.callOne(ros::WallDuration(0.5));

    if(inputGoal){
        inputGoal = false;
    }
    else{
        return;
    }

    // goalMarker.pose.position.x = goalData.point.x;
    // goalMarker.pose.position.y = goalData.point.y;
    goalMarker.pose.position.x = goalData.global.x;
    goalMarker.pose.position.y = goalData.global.y;

    goalMarker.header.stamp = ros::Time::now();

    pubGoalMarker.publish(goalMarker);
}

void Visualization::goalListMarkerInitialize(void){
    sgl.setCallbackQueue(&qGoalList);
    subGoalList = sgl.subscribe("goal_list",1,&Visualization::goalListCB, this);

    pubGoalListMarker = pglm.advertise<visualization_msgs::Marker>("goal_list_marker", 1);

    // p.param<std::string>("goal_list_topic", goalListTopic, "goal_list");
    // sgl.setCallbackQueue(&qGoalList);
    // subGoalList = sgl.subscribe(goalListTopic,1,&Visualization::goalListCB, this);

    // p.param<std::string>("goal_list_marker_topic", goalListMarkerTopic, "goal_list_marker");
    // pubGoalListMarker = pglm.advertise<visualization_msgs::Marker>(goalListMarkerTopic, 1);
    double goalListSize;
    p.param<double>("goal_list_size", goalListSize, 0.3);

    goalListMarker.header.frame_id = mapFrameId;
    goalListMarker.pose.orientation.w = 1.0;
    goalListMarker.scale.x = goalListSize;
    goalListMarker.scale.y = goalListSize;
    goalListMarker.scale.z = goalListSize;
    goalListMarker.type = visualization_msgs::Marker::CUBE_LIST;
    goalListMarker.action = visualization_msgs::Marker::ADD;
    goalListMarker.lifetime = ros::Duration(0);
    goalListMarker.ns = "goalList";
    goalListMarker.id = 0;

    goalListMarker.color.r = 1.0f;
    goalListMarker.color.g = 1.0f;
    goalListMarker.color.b = 0.0f;
    goalListMarker.color.a = 1.0f;

    inputGoalList = false;
}

//void Visualization::goalListCB(const exploration_msgs::PointArray::ConstPtr& msg){
void Visualization::goalListCB(const exploration_msgs::GoalList::ConstPtr& msg){
	goalListData = *msg;
    inputGoalList = true;
}

void Visualization::publishGoalListMarker(void){
    qGoalList.callOne(ros::WallDuration(0.5));

    if(inputGoalList){
        inputGoalList = false;
    }
    else{
        return;
    }

    //goalListMarker.points = goalListData.points;
    goalListMarker.points = goalListData.global;
    goalListMarker.header.stamp = ros::Time::now();

    pubGoalListMarker.publish(goalListMarker);
}


void Visualization::toGoalMarkerInitialize(void){
    stg.setCallbackQueue(&qToGoal);
    subToGoal = stg.subscribe("to_goal",1,&Visualization::toGoalCB, this);

    pubToGoalMarker = ptgm.advertise<visualization_msgs::Marker>("to_goal_marker", 1);

    // p.param<std::string>("to_goal_topic", toGoalTopic, "to_goal");
    // stg.setCallbackQueue(&qToGoal);
    // subToGoal = stg.subscribe(toGoalTopic,1,&Visualization::toGoalCB, this);

    // p.param<std::string>("to_goal_marker_topic", toGoalMarkerTopic, "to_goal_marker");
    // pubToGoalMarker = ptgm.advertise<visualization_msgs::Marker>(toGoalMarkerTopic, 1);
    double toGoalSize;
    p.param<double>("to_goal_size", toGoalSize, 0.1);

    toGoalMarker.header.frame_id = mapFrameId;
    toGoalMarker.pose.orientation.w = 1.0;
    toGoalMarker.scale.x = toGoalSize;
    toGoalMarker.type = visualization_msgs::Marker::LINE_STRIP;
    toGoalMarker.action = visualization_msgs::Marker::ADD;
    toGoalMarker.lifetime = ros::Duration(0);
    toGoalMarker.ns = "toGoal";
    toGoalMarker.id = 0;

    toGoalMarker.color.r = 1.0f;
    toGoalMarker.color.g = 0.0f;
    toGoalMarker.color.b = 0.0f;
    toGoalMarker.color.a = 1.0f;

    inputToGoal = false;
}

void Visualization::toGoalCB(const exploration_msgs::ToGoal::ConstPtr& msg){
	toGoalData = *msg;
    inputToGoal = true;
}

void Visualization::publishToGoalMarker(void){
    qToGoal.callOne(ros::WallDuration(0.5));

    if(inputToGoal){
        inputToGoal = false;
    }
    else{
        return;
    }

    std::vector<geometry_msgs::Point> input;
    input.resize(2);

    input[0].x = toGoalData.pose.position.x;
    input[0].y = toGoalData.pose.position.y;

    input[1] = toGoalData.goal;

    toGoalMarker.points = input;

    toGoalMarker.header.stamp = ros::Time::now();

    pubToGoalMarker.publish(toGoalMarker);
}


void Visualization::moveAngleMarkerInitialize(void){
    sma.setCallbackQueue(&qMoveAngle);
    subMoveAngle = sma.subscribe("move_angle",1,&Visualization::moveAngleCB, this);

    pubMoveAngleMarker = pmam.advertise<visualization_msgs::Marker>("move_angle_marker", 1);

    // p.param<std::string>("move_angle_topic", moveAngleTopic, "move_angle");
    // sma.setCallbackQueue(&qMoveAngle);
    // subMoveAngle = sma.subscribe(moveAngleTopic,1,&Visualization::moveAngleCB, this);

    // p.param<std::string>("move_angle_marker_topic", moveAngleMarkerTopic, "move_angle_marker");
    // pubMoveAngleMarker = pmam.advertise<visualization_msgs::Marker>(moveAngleMarkerTopic, 1);
    double moveAngleSizeX;
    double moveAngleSizeY;
    
    p.param<double>("move_angle_size_x", moveAngleSizeX, 0.1);
    p.param<double>("move_angle_size_y", moveAngleSizeY, 0.5);

    moveAngleMarker.header.frame_id = mapFrameId;
    moveAngleMarker.pose.orientation.w = 1.0;
    moveAngleMarker.scale.x = moveAngleSizeX;
    moveAngleMarker.scale.y = moveAngleSizeY;
    moveAngleMarker.type = visualization_msgs::Marker::ARROW;
    moveAngleMarker.action = visualization_msgs::Marker::ADD;
    moveAngleMarker.lifetime = ros::Duration(0);
    moveAngleMarker.ns = "moveAngle";
    moveAngleMarker.id = 0;

    moveAngleMarker.color.r = 0.0f;
    moveAngleMarker.color.g = 1.0f;
    moveAngleMarker.color.b = 0.0f;
    moveAngleMarker.color.a = 1.0f;

    inputMoveAngle = false;
}

void Visualization::moveAngleCB(const exploration_msgs::MoveAngle::ConstPtr& msg){
	moveAngleData = *msg;
    inputMoveAngle = true;
}

void Visualization::publishMoveAngleMarker(void){
    qMoveAngle.callOne(ros::WallDuration(0.5));

    if(inputMoveAngle){
        inputMoveAngle = false;
    }
    else{
        return;
    }

    double moveAngleLength;
    p.param<double>("move_angle_length", moveAngleLength, 0.3);

    std::vector<geometry_msgs::Point> input;
    input.resize(2);

    input[0].x = moveAngleData.pose.position.x;
    input[0].y = moveAngleData.pose.position.y;

    double tempX,tempY;

    //ローカル座標系での矢印の終点座標
    tempX = moveAngleLength * cos(moveAngleData.local_angle);
    tempY = moveAngleLength * sin(moveAngleData.local_angle);

    //double yaw = 2*asin(moveAngleData.pose.orientation.z);
    double yaw = qToYaw(moveAngleData.pose.orientation);

    input[1].x = input[0].x + tempX * cos(yaw) - tempY * sin(yaw);
    input[1].y = input[0].y + tempX * sin(yaw) + tempY * cos(yaw);

    moveAngleMarker.points = input;

    moveAngleMarker.header.stamp = ros::Time::now();

    pubMoveAngleMarker.publish(moveAngleMarker);
}

#endif //VISUALIZATION_H