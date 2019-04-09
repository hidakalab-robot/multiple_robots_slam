//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration/common_lib.hpp>
#include <std_msgs/Bool.h>

class ExplorationManager
{
private:
    double END_AREA;
    double TOLERANCE;

    CommonLib::subStructSimple map_;
    CommonLib::pubStruct<std_msgs::Bool> end_;

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //マップの面積を計算して終了条件と比較
        int freeSpace = 0;
        for(const auto& m : msg->data){
            if(m == 0) ++freeSpace;
        }
        msg->info.resolution * msg->info.resolution * freeSpace >= END_AREA*TOLERANCE ? end_.pub.publish(CommonLib::msgBool(true)) : end_.pub.publish(CommonLib::msgBool(false));
        // if(msg->info.resolution * msg->info.resolution * freeSpace >= END_AREA*TOLERANCE) end_.pub.publish(CommonLib::msgBool(true));
        // else end_.pub.publish(CommonLib::msgBool(false));
    };
public:
    ExplorationManager():map_("map", 1,&ExplorationManager::mapCB, this),end_("end",1,true){
        ros::NodeHandle p("~");
        p.param<double>("end_area",END_AREA,46.7*14-9.5*10-((4.1+2.7+7.5)*10-2.7*5.8)-8.0*10-7.5*10);//276.46
        p.param<double>("tolerance",TOLERANCE,0.8);
    };
};

#endif //EXPLORATION_MANAGER_HPP