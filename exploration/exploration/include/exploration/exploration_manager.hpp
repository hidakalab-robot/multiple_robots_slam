//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration/common_lib.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

class ExplorationManager
{
private:
    double END_AREA;
    double TOLERANCE;

    CommonLib::subStructSimple map_;
    CommonLib::pubStruct<std_msgs::Bool> end_;
    CommonLib::pubStruct<std_msgs::Float64> area_;

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //マップの面積を計算して終了条件と比較
        int freeSpace = 0;
        for(const auto& m : msg->data){
            if(m == 0) ++freeSpace;
        }
        double area = msg->info.resolution * msg->info.resolution * freeSpace;
        area >= END_AREA*TOLERANCE ? end_.pub.publish(CommonLib::msgBool(true)) : end_.pub.publish(CommonLib::msgBool(false));
        area_.pub.publish(CommonLib::msgDouble(std::move(area)));
    };
public:
    ExplorationManager():map_("map", 1,&ExplorationManager::mapCB, this),end_("end",1,true),area_("end/area",1,true){
        ros::NodeHandle p("~");
        p.param<double>("end_area",END_AREA,46.7*14-9.5*10-((4.1+2.7+7.5)*10-2.7*5.8)-8.0*10-7.5*10-0.9*10);//267.46
        END_AREA *= 1.5;
        p.param<double>("tolerance",TOLERANCE,0.9);
    };
};

#endif //EXPLORATION_MANAGER_HPP