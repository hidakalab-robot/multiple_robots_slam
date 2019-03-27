#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

//topicの情報をrvizで表示するためのクラス
#include <ros/ros.h>
#include <ros/callback_queue.h>
//#include <exploration_msgs/ToGoal.h>
//#include <exploration_msgs/MoveAngle.h>
#include <exploration_msgs/Goal.h>
#include <exploration_msgs/GoalList.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>

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
    exploration_msgs::Goal goalData;
    visualization_msgs::Marker goalMarker;
    ros::NodeHandle pgm;
    ros::Publisher pubGoalMarker;
    //goalDelete
    ros::NodeHandle sgd;
    ros::Subscriber subGoalDel;
    ros::CallbackQueue qGoalDel;
    visualization_msgs::Marker goalDel;
    ros::NodeHandle pgd;
    ros::Publisher pubGoalDel;

    //goalList
    ros::NodeHandle sgl;
    ros::Subscriber subGoalList;
    ros::CallbackQueue qGoalList;
    exploration_msgs::GoalList goalListData;
    visualization_msgs::Marker goalListMarker;
    ros::NodeHandle pglm;
    ros::Publisher pubGoalListMarker;
    //goalListDelete
    ros::NodeHandle sgld;
    ros::Subscriber subGoalListDel;
    ros::CallbackQueue qGoalListDel;
    visualization_msgs::Marker goalListDel;
    ros::NodeHandle pgld;
    ros::Publisher pubGoalListDel;

    //toGoal
    // ros::NodeHandle stg;
    // ros::Subscriber subToGoal;
    // ros::CallbackQueue qToGoal;
    // exploration_msgs::ToGoal toGoalData;
    // visualization_msgs::Marker toGoalMarker;
    // ros::NodeHandle ptgm;
    // ros::Publisher pubToGoalMarker;
    //toGoalDelete
    // ros::NodeHandle stgd;
    // ros::Subscriber subToGoalDel;
    // ros::CallbackQueue qToGoalDel;
    // visualization_msgs::Marker toGoalDel;
    // ros::NodeHandle ptgd;
    // ros::Publisher pubToGoalDel;
    
    //moveAngle
    // ros::NodeHandle sma;
    // ros::Subscriber subMoveAngle;
    // ros::CallbackQueue qMoveAngle;
    // exploration_msgs::MoveAngle moveAngleData;
    // visualization_msgs::Marker moveAngleMarker;
    // ros::NodeHandle pmam;
    // ros::Publisher pubMoveAngleMarker;
    
    //pose
    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    geometry_msgs::PoseStamped poseData;
    visualization_msgs::Marker poseMarker;
    std::string explorationMethod;
    ros::NodeHandle ppm;
    ros::Publisher pubPoseMarker;
    std_msgs::ColorRGBA poseColor;

    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCB(const exploration_msgs::Goal::ConstPtr& msg);
    void goalListCB(const exploration_msgs::GoalList::ConstPtr& msg);
    //void toGoalCB(const exploration_msgs::ToGoal::ConstPtr& msg);
    //void moveAngleCB(const exploration_msgs::MoveAngle::ConstPtr& msg);

    void goalDelCB(const std_msgs::Empty::ConstPtr& msg);
    void goalListDelCB(const std_msgs::Empty::ConstPtr& msg);
    //void toGoalDelCB(const std_msgs::Empty::ConstPtr& msg);

    double qToYaw(const geometry_msgs::Quaternion& q);

public:
    Visualization();
    //~Visualization(){};

    void poseMarkerInitialize(void);
    void publishPoseMarker(void);

    void goalMarkerInitialize(void);
    void publishGoalMarker(void);
    void goalDeleteInitialize(void);
    void publishGoalDelete(void);

    void goalListMarkerInitialize(void);
    void publishGoalListMarker(void);
    void goalListDeleteInitialize(void);
    void publishGoalListDelete(void);

    // void toGoalMarkerInitialize(void);
    // void publishToGoalMarker(void);
    // void toGoalDeleteInitialize(void);
    // void publishToGoalDelete(void);

    // void moveAngleMarkerInitialize(void);
    // void publishMoveAngleMarker(void);
};

Visualization::Visualization():p("~"){
    p.param<std::string>("map_frame_id", mapFrameId, "map");
}

double Visualization::qToYaw(const geometry_msgs::Quaternion& q){
    tf::Quaternion temp(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(temp).getRPY(roll,pitch,yaw);
    return yaw;
}

void Visualization::poseMarkerInitialize(void){
    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&Visualization::poseCB, this);

    pubPoseMarker = ppm.advertise<visualization_msgs::Marker>("pose_marker", 1,true);

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
}

void Visualization::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
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
    input.x = msg -> pose.position.x;
    input.y = msg -> pose.position.y;
    input.z = msg -> pose.position.z;

    poseMarker.points.push_back(input);
    poseMarker.colors.push_back(poseColor);
    poseMarker.header.stamp = ros::Time::now();

    pubPoseMarker.publish(poseMarker);
}

void Visualization::publishPoseMarker(void){
    qPose.callOne(ros::WallDuration(0.5));
}


void Visualization::goalMarkerInitialize(void){
    sg.setCallbackQueue(&qGoal);
    subGoal = sg.subscribe("goal",1,&Visualization::goalCB, this);

    pubGoalMarker = pgm.advertise<visualization_msgs::Marker>("goal_marker", 1,true);

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

    //inputGoal = false;
}

void Visualization::goalCB(const exploration_msgs::Goal::ConstPtr& msg){
    goalMarker.pose.position.x = msg -> global.x;
    goalMarker.pose.position.y = msg -> global.y;
    goalMarker.header.stamp = ros::Time::now();

    pubGoalMarker.publish(goalMarker);
}

void Visualization::publishGoalMarker(void){
    qGoal.callOne(ros::WallDuration(0.5));
}

void Visualization::goalDeleteInitialize(void){
    sgd.setCallbackQueue(&qGoalDel);
    subGoalDel = sgd.subscribe("goal/delete",1,&Visualization::goalDelCB, this);

    pubGoalDel = pgd.advertise<visualization_msgs::Marker>("goal_marker", 1);

    goalDel.header.frame_id = mapFrameId;
    goalDel.action = visualization_msgs::Marker::DELETE;
    goalDel.ns = "goal";
    goalDel.id = 0;
}

void Visualization::goalDelCB(const std_msgs::Empty::ConstPtr& msg){
    pubGoalDel.publish(goalDel);
}

void Visualization::publishGoalDelete(void){
    qGoalDel.callOne(ros::WallDuration(0.2));
}

void Visualization::goalListMarkerInitialize(void){
    sgl.setCallbackQueue(&qGoalList);
    subGoalList = sgl.subscribe("goal_list",1,&Visualization::goalListCB, this);

    pubGoalListMarker = pglm.advertise<visualization_msgs::Marker>("goal_list_marker", 1, true);

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
}

void Visualization::goalListCB(const exploration_msgs::GoalList::ConstPtr& msg){
    goalListMarker.points = msg -> global;
    goalListMarker.header.stamp = ros::Time::now();

    pubGoalListMarker.publish(goalListMarker);
}

void Visualization::publishGoalListMarker(void){
    qGoalList.callOne(ros::WallDuration(0.5));
}

void Visualization::goalListDeleteInitialize(void){
    sgld.setCallbackQueue(&qGoalListDel);
    subGoalListDel = sgld.subscribe("goal_list/delete",1,&Visualization::goalListDelCB, this);

    pubGoalListDel = pgld.advertise<visualization_msgs::Marker>("goal_list_marker", 1);

    goalListDel.header.frame_id = mapFrameId;
    goalListDel.action = visualization_msgs::Marker::DELETE;
    goalListDel.ns = "goalList";
    goalListDel.id = 0;
}

void Visualization::goalListDelCB(const std_msgs::Empty::ConstPtr& msg){
    pubGoalListDel.publish(goalListDel);
}

void Visualization::publishGoalListDelete(void){
    qGoalListDel.callOne(ros::WallDuration(0.2));
}

// void Visualization::toGoalMarkerInitialize(void){
//     stg.setCallbackQueue(&qToGoal);
//     subToGoal = stg.subscribe("to_goal",1,&Visualization::toGoalCB, this);
//     pubToGoalMarker = ptgm.advertise<visualization_msgs::Marker>("to_goal_marker", 1, true);

//     double toGoalSize;
//     p.param<double>("to_goal_size", toGoalSize, 0.1);

//     toGoalMarker.header.frame_id = mapFrameId;
//     toGoalMarker.pose.orientation.w = 1.0;
//     toGoalMarker.scale.x = toGoalSize;
//     toGoalMarker.type = visualization_msgs::Marker::LINE_STRIP;
//     toGoalMarker.action = visualization_msgs::Marker::ADD;
//     toGoalMarker.lifetime = ros::Duration(0);
//     toGoalMarker.ns = "toGoal";
//     toGoalMarker.id = 0;
//     toGoalMarker.color.r = 1.0f;
//     toGoalMarker.color.g = 0.0f;
//     toGoalMarker.color.b = 0.0f;
//     toGoalMarker.color.a = 1.0f;
// }

// void Visualization::toGoalCB(const exploration_msgs::ToGoal::ConstPtr& msg){
//     std::vector<geometry_msgs::Point> input;
//     input.resize(2);

//     input[0].x = msg -> pose.position.x;
//     input[0].y = msg -> pose.position.y;
//     input[1] = msg -> goal;

//     toGoalMarker.points = input;
//     toGoalMarker.header.stamp = ros::Time::now();

//     pubToGoalMarker.publish(toGoalMarker);
// }

// void Visualization::publishToGoalMarker(void){
//     qToGoal.callOne(ros::WallDuration(0.5));
// }

// void Visualization::toGoalDeleteInitialize(void){
//     stgd.setCallbackQueue(&qToGoalDel);
//     subToGoalDel = stgd.subscribe("to_goal/delete",1,&Visualization::toGoalDelCB, this);

//     pubToGoalDel = ptgd.advertise<visualization_msgs::Marker>("to_goal_marker", 1, true);

//     toGoalDel.header.frame_id = mapFrameId;
//     toGoalDel.action = visualization_msgs::Marker::DELETE;
//     toGoalDel.ns = "toGoal";
//     toGoalDel.id = 0;
// }

// void Visualization::toGoalDelCB(const std_msgs::Empty::ConstPtr& msg){
//     pubToGoalDel.publish(toGoalDel);
// }

// void Visualization::publishToGoalDelete(void){
//     qToGoalDel.callOne(ros::WallDuration(0.2));
// }

// void Visualization::moveAngleMarkerInitialize(void){
//     sma.setCallbackQueue(&qMoveAngle);
//     subMoveAngle = sma.subscribe("move_angle",1,&Visualization::moveAngleCB, this);

//     pubMoveAngleMarker = pmam.advertise<visualization_msgs::Marker>("move_angle_marker", 1, true);

//     double moveAngleSizeX;
//     double moveAngleSizeY;
    
//     p.param<double>("move_angle_size_x", moveAngleSizeX, 0.1);
//     p.param<double>("move_angle_size_y", moveAngleSizeY, 0.5);

//     moveAngleMarker.header.frame_id = mapFrameId;
//     moveAngleMarker.pose.orientation.w = 1.0;
//     moveAngleMarker.scale.x = moveAngleSizeX;
//     moveAngleMarker.scale.y = moveAngleSizeY;
//     moveAngleMarker.type = visualization_msgs::Marker::ARROW;
//     moveAngleMarker.action = visualization_msgs::Marker::ADD;
//     moveAngleMarker.lifetime = ros::Duration(0);
//     moveAngleMarker.ns = "moveAngle";
//     moveAngleMarker.id = 0;
//     moveAngleMarker.color.r = 0.0f;
//     moveAngleMarker.color.g = 1.0f;
//     moveAngleMarker.color.b = 0.0f;
//     moveAngleMarker.color.a = 1.0f;
// }

// void Visualization::moveAngleCB(const exploration_msgs::MoveAngle::ConstPtr& msg){
//     double moveAngleLength;
//     p.param<double>("move_angle_length", moveAngleLength, 0.3);

//     std::vector<geometry_msgs::Point> input;
//     input.resize(2);

//     input[0].x = msg -> pose.position.x;
//     input[0].y = msg -> pose.position.y;

//     double tempX,tempY;

//     //ローカル座標系での矢印の終点座標
//     tempX = moveAngleLength * cos(msg -> local_angle);
//     tempY = moveAngleLength * sin(msg -> local_angle);

//     double yaw = qToYaw(msg -> pose.orientation);

//     input[1].x = input[0].x + tempX * cos(yaw) - tempY * sin(yaw);
//     input[1].y = input[0].y + tempX * sin(yaw) + tempY * cos(yaw);

//     moveAngleMarker.points = input;
//     moveAngleMarker.header.stamp = ros::Time::now();

//     pubMoveAngleMarker.publish(moveAngleMarker);
// }

// void Visualization::publishMoveAngleMarker(void){
//     qMoveAngle.callOne(ros::WallDuration(0.5));
// }

#endif //VISUALIZATION_H