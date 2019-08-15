#ifndef MULTI_EXPLORATION
#define MULTI_EXPLORATION

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/constructor.hpp>
#include <exploration_libraly/convert.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/multi_exploration_simulatorConfig.h>
#include <fstream>

// 終了時にパラメータの設定を保存したい
// 毎回設定値を保存しておいて終了のときにそれを書き出す
// 起動時に前回のパラメータがあればそれを読み込む

class MultiExplorationSimulator
{
private:
    ros::NodeHandle nh_;
    std::string PARAMETER_FILE_PATH;
    bool OUTPUT_PARAMETERS;

    int ROBOT_NUMBER;
    int BRANCH_NUMBER;
    int FRONTIER_NUMBER;    
    
    geometry_msgs::PoseArray robotPoses_;
    visualization_msgs::Marker branchCoordinates_;
    visualization_msgs::Marker frontierCoordinates_;

    ExpLib::pubStruct<geometry_msgs::PoseArray> poses_;
    ExpLib::pubStruct<visualization_msgs::Marker> branches_;
    ExpLib::pubStruct<visualization_msgs::Marker> frontiers_;

public:
    MultiExplorationSimulator();
    void callback(exploration_support::multi_exploration_simulatorConfig &cfg, uint32_t level);
    void updateParameters(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn);
    void writeParameters(void);
};

MultiExplorationSimulator::MultiExplorationSimulator()
    :nh_("~")
    ,poses_("pose_array",1,true)
    ,branches_("branch_array",1,true)
    ,frontiers_("frontier_array",1,true){

    nh_.param<std::string>("parameter_file_path",PARAMETER_FILE_PATH,"simulator_last_parameters.yaml");
    nh_.param<bool>("output_parameters",OUTPUT_PARAMETERS,true);

    std::string MAP_FRAME_ID;
    nh_.param<std::string>("map_frame_id",MAP_FRAME_ID,"map");
    robotPoses_.header.frame_id = branchCoordinates_.header.frame_id = frontierCoordinates_.header.frame_id = MAP_FRAME_ID;

    double BRANCH_SCALE,FRONTIER_SCALE;
    nh_.param<double>("branch_scale", BRANCH_SCALE, 0.5);
    nh_.param<double>("frontier_scale", FRONTIER_SCALE, 0.5);

    // for branch parameter
    branchCoordinates_.ns = "branch_array";
    branchCoordinates_.scale.x = branchCoordinates_.scale.y = branchCoordinates_.scale.z = BRANCH_SCALE;
    branchCoordinates_.color.r = 1.0f;
    branchCoordinates_.color.g = 1.0f;
    branchCoordinates_.color.b = 0.0f;
    branchCoordinates_.color.a = 1.0f;

    // for frontier parameter
    frontierCoordinates_.ns = "frontier_array";
    frontierCoordinates_.scale.x = frontierCoordinates_.scale.y = frontierCoordinates_.scale.z = FRONTIER_SCALE;
    frontierCoordinates_.color.r = 0.0f;
    frontierCoordinates_.color.g = 1.0f;
    frontierCoordinates_.color.b = 1.0f;
    frontierCoordinates_.color.a = 1.0f;

    // common parameter
    branchCoordinates_.pose.orientation.w = frontierCoordinates_.pose.orientation.w = 1.0;
    branchCoordinates_.type = frontierCoordinates_.type = visualization_msgs::Marker::CUBE_LIST;
    branchCoordinates_.action = frontierCoordinates_.action = visualization_msgs::Marker::ADD;
    branchCoordinates_.lifetime = frontierCoordinates_.lifetime = ros::Duration(0);
    branchCoordinates_.id = frontierCoordinates_.id =  0;
}

void MultiExplorationSimulator::callback(exploration_support::multi_exploration_simulatorConfig &cfg, uint32_t level){
    // resize array
    ROBOT_NUMBER = cfg.robot_number;
    BRANCH_NUMBER = cfg.branch_number;
    FRONTIER_NUMBER = cfg.frontier_number;

    robotPoses_.poses.resize(ROBOT_NUMBER);
    branchCoordinates_.points.resize(BRANCH_NUMBER);
    frontierCoordinates_.points.resize(FRONTIER_NUMBER);
}

void MultiExplorationSimulator::updateParameters(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn){
    // update robot parameters
    for(int i=1;i<=ROBOT_NUMBER;++i){
        double x,y,yaw;
        nh_.param<double>("robot"+std::to_string(i)+"_x",x,0.0);
        nh_.param<double>("robot"+std::to_string(i)+"_y",y,0.0);
        nh_.param<double>("robot"+std::to_string(i)+"_yaw",yaw,0.0);
        robotPoses_.poses[i-1] = ExpLib::msgPose(ExpLib::msgPoint(x,y),ExpLib::yawToQ(yaw*M_PI/180));
    }

    // update branch parameters
    for(int i=1;i<=BRANCH_NUMBER;++i){
        double x,y;
        nh_.param<double>("branch"+std::to_string(i)+"_x",x,0.0);
        nh_.param<double>("branch"+std::to_string(i)+"_y",y,0.0);
        branchCoordinates_.points[i-1] = ExpLib::msgPoint(x,y);
    }

    // update frontier parameters
    for(int i=1;i<=FRONTIER_NUMBER;++i){
        double x,y;
        nh_.param<double>("frontier"+std::to_string(i)+"_x",x,0.0);
        nh_.param<double>("frontier"+std::to_string(i)+"_y",y,0.0);
        frontierCoordinates_.points[i-1] = ExpLib::msgPoint(x,y);
    }

    // publish for rviz
    poses_.pub.publish(robotPoses_);
    branches_.pub.publish(branchCoordinates_);
    frontiers_.pub.publish(frontierCoordinates_);

    // call planning function
    fn(robotPoses_.poses,branchCoordinates_.points,frontierCoordinates_.points);
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

    ofs << "robot_number: " << robotPoses_.poses.size() << std::endl;
    ofs << "branch_number: " << branchCoordinates_.points.size() << std::endl;
    ofs << "frontier_number: " << frontierCoordinates_.points.size() << std::endl;

    for(int i=0,ie=robotPoses_.poses.size();i!=ie;++i){
        ofs << "robot" << i+1 << "_x: " << robotPoses_.poses[i].position.x << std::endl;
        ofs << "robot" << i+1 << "_y: " << robotPoses_.poses[i].position.y << std::endl;
        ofs << "robot" << i+1 << "_yaw: " << ExpLib::qToYaw(robotPoses_.poses[i].orientation)*180/M_PI << std::endl;
    }
    for(int i=0,ie=branchCoordinates_.points.size();i!=ie;++i){
        ofs << "branch" << i+1 << "_x: " << branchCoordinates_.points[i].x << std::endl;
        ofs << "branch" << i+1 << "_y: " << branchCoordinates_.points[i].y << std::endl;
    }
    for(int i=0,ie=frontierCoordinates_.points.size();i!=ie;++i){
        ofs << "frontier" << i+1 << "_x: " << frontierCoordinates_.points[i].x << std::endl;
        ofs << "frontier" << i+1 << "_y: " << frontierCoordinates_.points[i].y << std::endl;
    }
}

#endif // MULTI_EXPLORATION