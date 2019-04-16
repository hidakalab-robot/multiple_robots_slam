#ifndef LOG_HPP
#define LOG_HPP

#include <ros/ros.h>
#include <exploration/common_lib.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_msgs/PoseStampedArray.h>

class Log
{
private:
    CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;
    CommonLib::pubStruct<exploration_msgs::PoseStampedArray> poseArray_;

    exploration_msgs::PoseStampedArray poseLog;
public:
    Log():pose_("pose",1),poseArray_("pose_log",1){};

    void publishPoseLog(void){
        if(!pose_.q.callOne(ros::WallDuration(1))){
            poseLog.header = pose_.data.header;
            poseLog.header.stamp = ros::Time::now();
            poseLog.poses.push_back(pose_.data);
        }
        poseArray_.pub.publish(poseLog);
    };
};

#endif //LOG_HPP