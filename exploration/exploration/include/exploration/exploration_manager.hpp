//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration/common_lib.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <exploration/frontier_search.hpp>

class ExplorationManager
{
private:
    int END_CONDITION;
    double END_AREA;
    int END_FRONTIER;

    CommonLib::subStructSimple map_;
    CommonLib::pubStruct<std_msgs::Bool> end_;
    CommonLib::pubStruct<std_msgs::Float64> area_;
    CommonLib::pubStruct<std_msgs::Int32> frontier_;

    bool calcArea(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //マップの面積を計算して終了条件と比較
        int freeSpace = 0;
        for(const auto& m : msg->data){
            if(m == 0) ++freeSpace;
        }
        double area = msg->info.resolution * msg->info.resolution * freeSpace;
        area_.pub.publish(CommonLib::msgDouble(std::move(area)));
        return area >= END_AREA ? true : false;// true: end, false: continue
    }

    bool detectFrontier(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        static FrontierSearch fs;
        int frontier = fs.frontierDetection<int>(*msg);
        frontier_.pub.publish(CommonLib::msgInt(std::move(frontier)));
        return frontier < 0 || END_FRONTIER < frontier ? false : true;// true: end, false: continue
    }

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        bool end;
        switch (END_CONDITION){
            case 0:
                end = calcArea(msg);
                break;
            case 1:
                end = detectFrontier(msg);
                break;
        }
        end ? end_.pub.publish(CommonLib::msgBool(true)) : end_.pub.publish(CommonLib::msgBool(false));
    };
public:
    ExplorationManager():map_("map", 1,&ExplorationManager::mapCB, this),end_("end",1,true),area_("end/area",1,true),frontier_("end/frontier",1,true){
        ros::NodeHandle p("~");
        p.param<int>("end_condition",END_CONDITION,0); // 0:area, 1:frontier
        p.param<int>("end_frontier",END_FRONTIER,0);
        p.param<double>("end_area",END_AREA,46.7*14-9.5*10-((4.1+2.7+7.5)*10-2.7*5.8)-8.0*10-7.5*10-0.9*10);//267.46
        double AREA_RATE,TOLERANCE;
        p.param<double>("area_rate",AREA_RATE,1.0);
        p.param<double>("tolerance",TOLERANCE,0.9);
        END_AREA *= AREA_RATE * TOLERANCE;
    };
};

#endif //EXPLORATION_MANAGER_HPP