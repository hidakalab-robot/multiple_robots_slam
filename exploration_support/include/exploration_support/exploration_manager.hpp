//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/constructor.hpp>
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
    double END_TIME;

    ros::Time startTime;

    ExpLib::subStructSimple map_;
    ExpLib::pubStruct<std_msgs::Bool> end_;
    ExpLib::pubStruct<std_msgs::Float64> area_;
    ExpLib::pubStruct<std_msgs::Int32> frontier_;
    ExpLib::pubStruct<std_msgs::Float64> elapsed_;

    bool calcArea(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //マップの面積を計算して終了条件と比較
        int freeSpace = 0;
        for(const auto& m : msg->data){
            if(m == 0) ++freeSpace;
        }
        double area = msg->info.resolution * msg->info.resolution * freeSpace;
        area_.pub.publish(ExpLib::msgDouble(area));
        return area >= END_AREA ? true : false;// true: end, false: continue
    }

    bool detectFrontier(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        static FrontierSearch fs;
        int frontier = fs.frontierDetection<int>(*msg);
        frontier_.pub.publish(ExpLib::msgInt(frontier));
        return frontier < 0 || END_FRONTIER < frontier ? false : true;// true: end, false: continue
    }

    bool timer(void){
        double elapsedTime = ros::Duration(ros::Time::now()-startTime).toSec();
        elapsed_.pub.publish(ExpLib::msgDouble(elapsedTime));
        return elapsedTime > END_TIME ? true : false;
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
            case 2:
                end = timer();
                break;
            default:
                ROS_WARN_STREAM("end_condition is invalid !!");
                break;
        }
        end ? end_.pub.publish(ExpLib::msgBool(true)) : end_.pub.publish(ExpLib::msgBool(false));
    };
public:
    ExplorationManager():map_("map", 1,&ExplorationManager::mapCB, this),end_("end",1,true),area_("end/area",1,true),frontier_("end/frontier",1,true),elapsed_("end/elapsed_time",1,true){
        ros::NodeHandle p("~");
        p.param<int>("end_condition",END_CONDITION,0); // 0:area, 1:frontier, 2:timer
        p.param<int>("end_frontier",END_FRONTIER,0);
        p.param<double>("end_time",END_TIME,1200);// second
        p.param<double>("end_area",END_AREA,46.7*14-9.5*10-((4.1+2.7+7.5)*10-2.7*5.8)-8.0*10-7.5*10-0.9*10);//267.46
        double AREA_RATE,TOLERANCE;
        p.param<double>("area_rate",AREA_RATE,1.0);
        p.param<double>("tolerance",TOLERANCE,0.9);
        END_AREA *= AREA_RATE * TOLERANCE;
        usleep(2e5);//timeがsim_timeに合うのを待つ
        startTime = ros::Time::now();
    };
};

#endif //EXPLORATION_MANAGER_HPP