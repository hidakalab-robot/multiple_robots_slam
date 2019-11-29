#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ros/ros.h>
#include <exploration_libraly/struct.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <exploration_libraly/path_planning.h>
#include <navfn/navfn_ros.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/movement_parameter_reconfigureConfig.h>

//センサーデータを受け取った後にロボットの動作を決定する
//障害物回避を含む

/*
movement tutorial

In source file

    #include <exploration/movement.hpp>

        Movement mv;

    if you want to move to goal with move_base
        mv.moveToGoal(goal);//goal type == geometry_msgs::Point

    if you want to move forward
        mv.moveToForward();

    if you want to one rotations
        mv.oneRotation();
*/

namespace ExStc = ExpLib::Struct;

class Movement 
{
private:
    // dynamic parameters
    double FORWARD_VELOCITY;
    double ROTATION_VELOCITY;
    double CURVE_GAIN;
    bool USE_ANGLE_BIAS;
    double ANGLE_BIAS;
    double COSTMAP_MARGIN;
    int ESC_MAP_DIV_X;
    int ESC_MAP_DIV_Y;
    double ESC_MAP_WIDTH;
    double ESC_MAP_HEIGHT;
    double ROTATION_TOLERANCE;
    double GOAL_RESET_RATE;
    int PATH_BACK_INTERVAL;
    int RESET_GOAL_PATH_LIMIT;
    double RESET_GOAL_PATH_RATE;
    double BACK_VELOCITY;
    double BACK_TIME;
    double ROAD_CENTER_THRESHOLD;
    double ROAD_THRESHOLD;
    double ROAD_CENTER_GAIN;
    double FORWARD_ANGLE;
    double VFH_FAR_RANGE_THRESHOLD;
    double VFH_NEAR_RANGE_THRESHOLD;
    double VFH_RATE_THRESHOLD;
    double FAR_AVOIDANCE_GAIN;
    double NEAR_AVOIDANCE_GAIN;
    double EMERGENCY_THRESHOLD;
    double EMERGENCY_DIFF_THRESHOLD;
    double EMERGENCY_AVOIDANCE_GAIN;
    bool APPROACH_WALL;
    double WALL_FORWARD_ANGLE;
    double WALL_RATE_THRESHOLD;
    double WALL_DISTANCE_UPPER_THRESHOLD;
    double WALL_DISTANCE_LOWER_THRESHOLD;

    // static parameters
    std::string MOVEBASE_NAME;
    std::string MOVEMENT_PARAMETER_FILE_PATH;
    bool OUTPUT_MOVEMENT_PARAMETERS;

    // variables
    ExStc::subStruct<sensor_msgs::LaserScan> scan_;
    ExStc::subStruct<geometry_msgs::PoseStamped> pose_;
    ExStc::subStruct<kobuki_msgs::BumperEvent> bumper_;
    ExStc::subStruct<nav_msgs::OccupancyGrid> gCostmap_;
    ExStc::pubStruct<geometry_msgs::Twist> velocity_;
    ExStc::pubStruct<geometry_msgs::PointStamped> goal_;
    ExStc::pubStruct<geometry_msgs::PointStamped> road_;
    ExpLib::PathPlanning<navfn::NavfnROS> pp_;
    dynamic_reconfigure::Server<exploration::movement_parameter_reconfigureConfig> drs_;
    double previousOrientation_;

    // functions
    bool lookupCostmap(void);
    bool lookupCostmap(const geometry_msgs::PoseStamped& goal);
    bool lookupCostmap(const geometry_msgs::PoseStamped& goal, const nav_msgs::OccupancyGrid& cmap);
    void escapeFromCostmap(const geometry_msgs::PoseStamped& pose);
    void rotationFromTo(const geometry_msgs::Quaternion& from, const geometry_msgs::Quaternion& to);
    bool resetGoal(geometry_msgs::PoseStamped& goal);
    bool bumperCollision(const kobuki_msgs::BumperEvent& bumper);
    bool roadCenterDetection(const sensor_msgs::LaserScan& scan);
    bool VFHMove(const sensor_msgs::LaserScan& scan, double angle=0);
    bool emergencyAvoidance(const sensor_msgs::LaserScan& scan);
    bool forwardWallDetection(const sensor_msgs::LaserScan& scan, double& angle);
    double sideSpaceDetection(const sensor_msgs::LaserScan& scan, int plus, int minus);
    geometry_msgs::Twist velocityGenerator(double theta, double v, double gain);
    void loadParams(void);
    void dynamicParamsCB(exploration::movement_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    Movement();
    ~Movement();
    void moveToGoal(geometry_msgs::PointStamped goal);
    void moveToForward(void);
    void oneRotation(void);
};

#endif //MOVEMENT_H