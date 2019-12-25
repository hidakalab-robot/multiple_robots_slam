#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <memory>

// 前方宣言

/// my packages
namespace ExpLib{
    template <typename T>
    class PathPlanning;
    namespace Struct{
        template<typename T>
        struct pubStruct;
        template<typename T>
        struct subStruct;
    }
}
namespace exploration_msgs{
    template <class ContainerAllocator>
    struct AvoidanceStatus_;
    typedef ::exploration_msgs::AvoidanceStatus_<std::allocator<void>> AvoidanceStatus;
}
namespace exploration{
    class movement_parameter_reconfigureConfig;
}
/// ros
namespace dynamic_reconfigure{
    template <class ConfigType>
    class Server;
}
namespace navfn{
    class NavfnROS;
}
/// rosmsgs
namespace geometry_msgs{
    template <class ContainerAllocator>
    struct Point_;
    typedef ::geometry_msgs::Point_<std::allocator<void>> Point;
    template <class ContainerAllocator>
    struct PointStamped_;
    typedef ::geometry_msgs::PointStamped_<std::allocator<void>> PointStamped;
    template <class ContainerAllocator>
    struct PoseStamped_;
    typedef ::geometry_msgs::PoseStamped_<std::allocator<void>> PoseStamped;
    template <class ContainerAllocator>
    struct Quaternion_;
    typedef ::geometry_msgs::Quaternion_<std::allocator<void>> Quaternion;
    template <class ContainerAllocator>
    struct Twist_;
    typedef ::geometry_msgs::Twist_<std::allocator<void>> Twist;
}
namespace kobuki_msgs{
    template <class ContainerAllocator>
    struct BumperEvent_;
    typedef ::kobuki_msgs::BumperEvent_<std::allocator<void>> BumperEvent;
}
namespace nav_msgs{
    template <class ContainerAllocator>
    struct OccupancyGrid_;
    typedef ::nav_msgs::OccupancyGrid_<std::allocator<void>> OccupancyGrid;
}
namespace sensor_msgs{
    template <class ContainerAllocator>
    struct LaserScan_;
    typedef ::sensor_msgs::LaserScan_<std::allocator<void>> LaserScan;
}
// 前方宣言ここまで

//センサーデータを受け取った後にロボットの動作を決定する
//障害物回避を含む
namespace ExStc = ExpLib::Struct;

class Movement {
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
        bool CALC_RANGE_COS;
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
        std::unique_ptr<ExStc::subStruct<sensor_msgs::LaserScan>> scan_;
        std::unique_ptr<ExStc::subStruct<geometry_msgs::PoseStamped>> pose_;
        std::unique_ptr<ExStc::subStruct<kobuki_msgs::BumperEvent>> bumper_;
        std::unique_ptr<ExStc::subStruct<nav_msgs::OccupancyGrid>> gCostmap_;
        std::unique_ptr<ExStc::pubStruct<geometry_msgs::Twist>> velocity_;
        std::unique_ptr<ExStc::pubStruct<geometry_msgs::PointStamped>> goal_;
        std::unique_ptr<ExStc::pubStruct<geometry_msgs::PointStamped>> road_;
        std::unique_ptr<ExStc::pubStruct<exploration_msgs::AvoidanceStatus>> avoStatus_;
        std::unique_ptr<ExpLib::PathPlanning<navfn::NavfnROS>> pp_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration::movement_parameter_reconfigureConfig>> drs_;
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
        void publishMovementStatus(const std::string& status);
        void loadParams(void);
        void dynamicParamsCB(exploration::movement_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        Movement();
        ~Movement();
        void moveToGoal(geometry_msgs::PointStamped goal, bool sleep=false);
        void moveToForward(void);
        void oneRotation(void);
};

#endif //MOVEMENT_H