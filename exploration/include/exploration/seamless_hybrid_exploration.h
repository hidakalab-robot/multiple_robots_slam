#ifndef SEAMLESS_HYBRID_EXPLORATION_H
#define SEAMLESS_HYBRID_EXPLORATION_H

#include <exploration/sensor_based_exploration.h>

// 前方宣言

/// my packages
namespace ExpLib{
    template <typename T>
    class PathPlanning;
    namespace Struct{
        struct listStruct;
        template<typename T>
        struct pubStruct;
        template<typename T>
        struct subStruct;
    }
}
namespace exploration_msgs{
    template <class ContainerAllocator>
    struct BranchArray_;
    typedef ::exploration_msgs::BranchArray_<std::allocator<void>> BranchArray;
    template <class ContainerAllocator>
    struct FrontierArray_;
    typedef ::exploration_msgs::FrontierArray_<std::allocator<void>> FrontierArray;
    template <class ContainerAllocator>
    struct RobotInfoArray_;
    typedef ::exploration_msgs::RobotInfoArray_<std::allocator<void>> RobotInfoArray;
}
namespace exploration{
    class seamless_hybrid_exploration_parameter_reconfigureConfig;
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
    struct Pose_;
    typedef ::geometry_msgs::Pose_<std::allocator<void>> Pose;
    template <class ContainerAllocator>
    struct PoseStamped_;
    typedef ::geometry_msgs::PoseStamped_<std::allocator<void>> PoseStamped;
}
// 前方宣言ここまで

namespace ExStc = ExpLib::Struct;

class SeamlessHybridExploration :public SensorBasedExploration{
    private:
        // dynamic parameters
        double DISTANCE_WEIGHT;
        double DIRECTION_WEIGHT;
        double OTHER_ROBOT_WEIGHT;
        double DUPLICATE_COEFF;
        double ON_MAP_COEFF;
        double LAST_GOAL_WEIGHT;

        // static parameters
        std::string ROBOT_NAME;
        std::string SHE_PARAMETER_FILE_PATH;
        bool OUTPUT_SHE_PARAMETERS;

        // struct
        struct maxValue;
        struct preCalcResult;

        // variables
        std::unique_ptr<ExStc::subStruct<exploration_msgs::RobotInfoArray>> robotArray_;
        std::unique_ptr<ExStc::subStruct<exploration_msgs::FrontierArray>> frontier_;
        std::unique_ptr<ExpLib::PathPlanning<navfn::NavfnROS>> pp_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration::seamless_hybrid_exploration_parameter_reconfigureConfig>> drs_;
        std::unique_ptr<exploration_msgs::BranchArray> ba_;
        std::unique_ptr<exploration_msgs::FrontierArray> fa_;
        std::unique_ptr<exploration_msgs::RobotInfoArray> ria_;
        std::vector<preCalcResult> ownPreCalc_;
        std::vector<preCalcResult> otherPreCalc_;
        std::unique_ptr<geometry_msgs::PoseStamped> ps_;
        std::unique_ptr<maxValue> mVal_;

        // functions
        bool decideGoal(geometry_msgs::PointStamped& goal);
        // bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExStc::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
        bool decideGoal(geometry_msgs::PointStamped& goal, const exploration_msgs::BranchArray& ls, const geometry_msgs::PoseStamped& pose);
        // bool filter(std::vector<ExStc::listStruct>& ls, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria);
        bool filter(exploration_msgs::BranchArray& ba, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria);
        // void preCalc(const std::vector<ExStc::listStruct>& ls, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose);
        void preCalc(const exploration_msgs::BranchArray& ba, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose);
        void loadParams(void);
        void dynamicParamsCB(exploration::seamless_hybrid_exploration_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        SeamlessHybridExploration();
        ~SeamlessHybridExploration();
        bool forwardTargetDetection(void);
        bool getGoalAF(geometry_msgs::PointStamped& goal);
        void simBridge(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f);
};

#endif // SEAMLESS_HYBRID_EXPLORATION_H