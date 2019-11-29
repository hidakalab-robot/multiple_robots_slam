#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <exploration_libraly/construct.h>
#include <exploration_libraly/struct.h>
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

#endif //VISUALIZATION_HPP