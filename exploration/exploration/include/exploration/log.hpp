#ifndef LOG_HPP
#define LOG_HPP

#include <ros/ros.h>
#include <exploration/common_lib.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

class Log
{
private:
    CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;
    CommonLib::pubStruct<geometry_msgs::PoseArray> poseArray_;

    geometry_msgs::PoseArray poseLog;
public:
    Log():pose_("pose",1),poseArray_("pose_log",1){};

    void publishPoseLog(void){
        pose_.q.callOne(ros::WallDuration(1));
        if(pose_.data.header.stamp > poseLog.header.stamp){
            poseLog.header = pose_.data.header;
            poseLog.header.stamp = ros::Time::now();
            poseLog.poses.push_back(pose_.data.pose);
        }
        poseArray_.pub.publish(poseLog);
    };
};

#endif //LOG_HPP