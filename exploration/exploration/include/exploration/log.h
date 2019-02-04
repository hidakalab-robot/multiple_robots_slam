#ifndef LOG_H
#define LOG_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

class Log
{
private:
    ros::NodeHandle p;
    //std::string poseTopic;
    //std::string poseLogTopic;

    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    geometry_msgs::PoseStamped poseData;

    ros::NodeHandle ppl;
	ros::Publisher pubPoseLog;
    geometry_msgs::PoseArray poseLog;

    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    Log();
    ~Log(){};

    void poseLogInitialize(void);
    void publishPoseLog(void);
};

Log::Log(){
}

void Log::poseLogInitialize(void){
    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&Log::poseCB,this);

	pubPoseLog = ppl.advertise<geometry_msgs::PoseArray>("pose_log", 1);

    // p.param<std::string>("pose_topic", poseTopic, "pose");
    // sp.setCallbackQueue(&qPose);
    // subPose = sp.subscribe(poseTopic,1,&Log::poseCB,this);

	// p.param<std::string>("pose_log_topic", poseLogTopic, "pose_log");
	// pubPoseLog = ppl.advertise<geometry_msgs::PoseArray>(poseLogTopic, 1);
}

void Log::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

void Log::publishPoseLog(void){
    qPose.callOne(ros::WallDuration(1));

    poseLog.poses.push_back(poseData.pose);
    poseLog.header = poseData.header;
    poseLog.header.stamp = ros::Time::now();

    pubPoseLog.publish(poseLog);
}

#endif //LOG_H