#ifndef FRONTIER_BASED_EXPLORATION_H
#define FRONTIER_BASED_EXPLORATION_H

#include <memory>
#include <vector>

// 前方宣言

/// my packages
namespace ExpLib{
    namespace Struct{
        template<typename T>
        struct pubStruct;
        template<typename T>
        struct subStruct;
    }
}
namespace exploration_msgs{
    template <class ContainerAllocator>
    struct Frontier_;
    typedef ::exploration_msgs::Frontier_<std::allocator<void>> Frontier;
    template <class ContainerAllocator>
    struct FrontierArray_;
    typedef ::exploration_msgs::FrontierArray_<std::allocator<void>> FrontierArray;
}
namespace exploration{
    class frontier_based_exploration_parameter_reconfigureConfig;
}
/// ros
namespace dynamic_reconfigure{
    template <class ConfigType>
    class Server;
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
}
// 前方宣言 ここまで

namespace ExStc = ExpLib::Struct;

class FrontierBasedExploration{
    private:
        // dynamic parameters
        double DISTANCE_WEIGHT;
        double DIRECTION_WEIGHT;
        bool LAST_GOAL_EFFECT;
        double LAST_GOAL_TOLERANCE;

        // static parameters
        std::string FBE_PARAMETER_FILE_PATH;
        bool OUTPUT_FBE_PARAMETERS;

        // variables
        std::unique_ptr<ExStc::subStruct<exploration_msgs::FrontierArray>> frontier_;
        std::unique_ptr<ExStc::subStruct<geometry_msgs::PoseStamped>> pose_;
        std::unique_ptr<ExStc::pubStruct<geometry_msgs::PointStamped>> goal_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration::frontier_based_exploration_parameter_reconfigureConfig>> drs_;
        std::unique_ptr<geometry_msgs::Point> lastGoal_;

        // functions
        bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<exploration_msgs::Frontier>& frontiers, const geometry_msgs::PoseStamped& pose);
        void loadParams(void);
        void dynamicParamsCB(exploration::frontier_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        FrontierBasedExploration();
        ~FrontierBasedExploration();
        bool getGoal(geometry_msgs::PointStamped& goal);
};

#endif // FRONTIER_BASED_EXPLORATION_H