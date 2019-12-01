#include <exploration/multi_exploration_simulator.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/convert.h>
#include <fstream>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <exploration/multi_exploration_simulatorConfig.h>
#include <exploration_libraly/struct.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

namespace ExCos = ExpLib::Construct;
namespace ExCov = ExpLib::Convert;
namespace ExStc = ExpLib::Struct;

MultiExplorationSimulator::MultiExplorationSimulator()
    :poses_(new ExStc::pubStruct<geometry_msgs::PoseArray>("pose_array",1,true))
    ,branches_(new ExStc::pubStruct<visualization_msgs::Marker>("branch_array",1,true))
    ,frontiers_(new ExStc::pubStruct<visualization_msgs::Marker>("frontier_array",1,true))
    ,drs_(new dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig>(ros::NodeHandle("~/mulsim"))){
    loadParams();
    drs_->setCallback(boost::bind(&MultiExplorationSimulator::dynamicParamsCB,this, _1, _2));
    robotPoses_->header.frame_id = MAP_FRAME_ID;
    *branchCoordinates_ = ExCos::msgCubeListMarker(MAP_FRAME_ID,BRANCH_SCALE,1.0,1.0,0.0);
    *frontierCoordinates_ = ExCos::msgCubeListMarker(MAP_FRAME_ID,FRONTIER_SCALE,0.0,1.0,1.0);
}

MultiExplorationSimulator::~MultiExplorationSimulator(){
    if(OUTPUT_MULSIM_PARAMETERS) outputParams();
}

void MultiExplorationSimulator::updateParams(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn){
    // resize array
    robotPoses_->poses.resize(ROBOT_NUMBER);
    branchCoordinates_->points.resize(BRANCH_NUMBER);
    frontierCoordinates_->points.resize(FRONTIER_NUMBER);

    ros::NodeHandle nh("~/mulsim");

    // update robot parameters
    for(int i=1;i<=ROBOT_NUMBER;++i){
        double x,y,yaw;
        nh.param<double>("robot"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("robot"+std::to_string(i)+"_y",y,0.0);
        nh.param<double>("robot"+std::to_string(i)+"_yaw",yaw,0.0);
        robotPoses_->poses[i-1] = ExCos::msgPose(ExCos::msgPoint(x,y),ExCov::yawToQ(yaw*M_PI/180));
    }

    // update branch parameters
    for(int i=1;i<=BRANCH_NUMBER;++i){
        double x,y;
        nh.param<double>("branch"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("branch"+std::to_string(i)+"_y",y,0.0);
        branchCoordinates_->points[i-1] = ExCos::msgPoint(x,y);
    }

    // update frontier parameters
    for(int i=1;i<=FRONTIER_NUMBER;++i){
        double x,y;
        nh.param<double>("frontier"+std::to_string(i)+"_x",x,0.0);
        nh.param<double>("frontier"+std::to_string(i)+"_y",y,0.0);
        frontierCoordinates_->points[i-1] = ExCos::msgPoint(x,y);
    }

    // publish for rviz
    poses_->pub.publish(*robotPoses_);
    branches_->pub.publish(*branchCoordinates_);
    frontiers_->pub.publish(*frontierCoordinates_);

    // call planning function
    fn(robotPoses_->poses,branchCoordinates_->points,frontierCoordinates_->points);
}

void MultiExplorationSimulator::loadParams(void){
    ros::NodeHandle nh("~/mulsim");
    // dynamic parameters
    nh.param<int>("robot_number",ROBOT_NUMBER,2);
    nh.param<int>("branch_number",BRANCH_NUMBER,1);
    nh.param<int>("frontier_number",FRONTIER_NUMBER,1);
    // static parameters
    nh.param<std::string>("map_frame_id",MAP_FRAME_ID,"map");
    nh.param<double>("branch_scale", BRANCH_SCALE, 0.5);
    nh.param<double>("frontier_scale", FRONTIER_SCALE, 0.5);
    nh.param<std::string>("mulsim_parameter_file_path",MULSIM_PARAMETER_FILE_PATH,"mulsim_last_parameters.yaml");
    nh.param<bool>("output_mulsim_parameters",OUTPUT_MULSIM_PARAMETERS,true);
}

void MultiExplorationSimulator::dynamicParamsCB(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level){
    ROBOT_NUMBER = cfg.robot_number;
    BRANCH_NUMBER = cfg.branch_number;
    FRONTIER_NUMBER = cfg.frontier_number;
}

void MultiExplorationSimulator::outputParams(void){
    //保存している最新のパラメータをyamlに書き出す
    std::cout << "writing mulsim last parameters ... ..." << std::endl;
    std::ofstream ofs(MULSIM_PARAMETER_FILE_PATH);
    
    if(ofs) std::cout << "mulsim param file open succeeded" << std::endl;
    else {
        std::cout << "mulsim param file open failed" << std::endl;
        return;
    }

    ofs << "robot_number: " << robotPoses_->poses.size() << std::endl;
    ofs << "branch_number: " << branchCoordinates_->points.size() << std::endl;
    ofs << "frontier_number: " << frontierCoordinates_->points.size() << std::endl;

    for(int i=0,ie=robotPoses_->poses.size();i!=ie;++i){
        ofs << "robot" << i+1 << "_x: " << robotPoses_->poses[i].position.x << std::endl;
        ofs << "robot" << i+1 << "_y: " << robotPoses_->poses[i].position.y << std::endl;
        ofs << "robot" << i+1 << "_yaw: " << ExCov::qToYaw(robotPoses_->poses[i].orientation)*180/M_PI << std::endl;
    }
    for(int i=0,ie=branchCoordinates_->points.size();i!=ie;++i){
        ofs << "branch" << i+1 << "_x: " << branchCoordinates_->points[i].x << std::endl;
        ofs << "branch" << i+1 << "_y: " << branchCoordinates_->points[i].y << std::endl;
    }
    for(int i=0,ie=frontierCoordinates_->points.size();i!=ie;++i){
        ofs << "frontier" << i+1 << "_x: " << frontierCoordinates_->points[i].x << std::endl;
        ofs << "frontier" << i+1 << "_y: " << frontierCoordinates_->points[i].y << std::endl;
    }
}