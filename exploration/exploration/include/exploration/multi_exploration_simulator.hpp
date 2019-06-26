#ifndef MULTI_EXPLORATION
#define MULTI_EXPLORATION

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <exploration/common_lib.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/multi_exploration_simulatorConfig.h>
#include <fstream>

// 終了時にパラメータの設定を保存したい
// 毎回設定値を保存しておいて終了のときにそれを書き出す
// 起動時に前回のパラメータがあればそれを読み込む

class MultiExplorationSimulator
{
private:
    ros::NodeHandle nh;
    std::string PARAMETER_FILE_PATH;
    bool OUTPUT_PARAMETERS;

    int ROBOT_NUMBER;
    int BRANCH_NUMBER;
    int FRONTIER_NUMBER;    
    
    geometry_msgs::PoseArray robotPoses;
    visualization_msgs::Marker branchCoordinates;
    visualization_msgs::Marker frontierCoordinates;

    CommonLib::pubStruct<geometry_msgs::PoseArray> poses_;
    CommonLib::pubStruct<visualization_msgs::Marker> branches_;
    CommonLib::pubStruct<visualization_msgs::Marker> frontiers_;

public:
    MultiExplorationSimulator();
    void callback(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level);
    void updateParameters(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn);
    void writeParameters(void);
};

MultiExplorationSimulator::MultiExplorationSimulator()
    :nh("~")
    ,poses_("pose_array",1,true)
    ,branches_("branch_array",1,true)
    ,frontiers_("frontier_array",1,true){

    nh.param<std::string>("parameter_file_path",PARAMETER_FILE_PATH,"simulator_last_parameters.yaml");
    nh.param<bool>("output_parameters",OUTPUT_PARAMETERS,true);

    std::string MAP_FRAME_ID;
    nh.param<std::string>("map_frame_id",MAP_FRAME_ID,"map");
    robotPoses.header.frame_id = branchCoordinates.header.frame_id = frontierCoordinates.header.frame_id = MAP_FRAME_ID;

    double BRANCH_SCALE,FRONTIER_SCALE;
    nh.param<double>("branch_scale", BRANCH_SCALE, 0.5);
    nh.param<double>("frontier_scale", FRONTIER_SCALE, 0.5);

    // for branch parameter
    branchCoordinates.ns = "branch_array";
    branchCoordinates.scale.x = branchCoordinates.scale.y = branchCoordinates.scale.z = BRANCH_SCALE;
    branchCoordinates.color.r = 1.0f;
    branchCoordinates.color.g = 1.0f;
    branchCoordinates.color.b = 0.0f;
    branchCoordinates.color.a = 1.0f;

    // for frontier parameter
    frontierCoordinates.ns = "frontier_array";
    frontierCoordinates.scale.x = frontierCoordinates.scale.y = frontierCoordinates.scale.z = FRONTIER_SCALE;
    frontierCoordinates.color.r = 0.0f;
    frontierCoordinates.color.g = 1.0f;
    frontierCoordinates.color.b = 1.0f;
    frontierCoordinates.color.a = 1.0f;

    // common parameter
    branchCoordinates.pose.orientation.w = frontierCoordinates.pose.orientation.w = 1.0;
    branchCoordinates.type = frontierCoordinates.type = visualization_msgs::Marker::CUBE_LIST;
    branchCoordinates.action = frontierCoordinates.action = visualization_msgs::Marker::ADD;
    branchCoordinates.lifetime = frontierCoordinates.lifetime = ros::Duration(0);
    branchCoordinates.id = frontierCoordinates.id =  0;
}

void MultiExplorationSimulator::callback(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level){
    // resize array
    ROBOT_NUMBER = cfg.robot_number;
    BRANCH_NUMBER = cfg.branch_number;
    FRONTIER_NUMBER = cfg.frontier_number;

    robotPoses.poses.resize(ROBOT_NUMBER);
    branchCoordinates.points.resize(BRANCH_NUMBER);
    frontierCoordinates.points.resize(FRONTIER_NUMBER);
}

void MultiExplorationSimulator::updateParameters(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn){
    // update robot parameters
    for(int i=1;i<=ROBOT_NUMBER;++i){
        double x,y,yaw;
        nh.param<double>("robot"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("robot"+std::to_string(i)+"_y",y,0.0);
        nh.param<double>("robot"+std::to_string(i)+"_yaw",yaw,0.0);
        robotPoses.poses[i-1] = CommonLib::msgPose(CommonLib::msgPoint(x,y),CommonLib::yawToQ(yaw*M_PI/180));
    }

    // update branch parameters
    for(int i=1;i<=BRANCH_NUMBER;++i){
        double x,y;
        nh.param<double>("branch"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("branch"+std::to_string(i)+"_y",y,0.0);
        branchCoordinates.points[i-1] = CommonLib::msgPoint(x,y);
    }

    // update frontier parameters
    for(int i=1;i<=FRONTIER_NUMBER;++i){
        double x,y;
        nh.param<double>("frontier"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("frontier"+std::to_string(i)+"_y",y,0.0);
        frontierCoordinates.points[i-1] = CommonLib::msgPoint(x,y);
    }

    // publish for rviz
    poses_.pub.publish(robotPoses);
    branches_.pub.publish(branchCoordinates);
    frontiers_.pub.publish(frontierCoordinates);

    // call planning function
    fn(robotPoses.poses,branchCoordinates.points,frontierCoordinates.points);
}

void MultiExplorationSimulator::writeParameters(void){
    //保存している最新のパラメータをyamlに書き出す
    if(!OUTPUT_PARAMETERS) return;

    std::cout << "writing last parameters ... ..." << std::endl;
    std::ofstream ofs(PARAMETER_FILE_PATH);
    
    if(ofs) std::cout << "file open succeeded" << std::endl;
    else {
        std::cout << "file open failed" << std::endl;
        return;
    }

    ofs << "robot_number: " << robotPoses.poses.size() << std::endl;
    ofs << "branch_number: " << branchCoordinates.points.size() << std::endl;
    ofs << "frontier_number: " << frontierCoordinates.points.size() << std::endl;

    for(int i=0,ie=robotPoses.poses.size();i!=ie;++i){
        ofs << "robot" << i+1 << "_x: " << robotPoses.poses[i].position.x << std::endl;
        ofs << "robot" << i+1 << "_y: " << robotPoses.poses[i].position.y << std::endl;
        ofs << "robot" << i+1 << "_yaw: " << CommonLib::qToYaw(robotPoses.poses[i].orientation)*180/M_PI << std::endl;
    }
    for(int i=0,ie=branchCoordinates.points.size();i!=ie;++i){
        ofs << "branch" << i+1 << "_x: " << branchCoordinates.points[i].x << std::endl;
        ofs << "branch" << i+1 << "_y: " << branchCoordinates.points[i].y << std::endl;
    }
    for(int i=0,ie=frontierCoordinates.points.size();i!=ie;++i){
        ofs << "frontier" << i+1 << "_x: " << frontierCoordinates.points[i].x << std::endl;
        ofs << "frontier" << i+1 << "_y: " << frontierCoordinates.points[i].y << std::endl;
    }
}

#endif // MULTI_EXPLORATION