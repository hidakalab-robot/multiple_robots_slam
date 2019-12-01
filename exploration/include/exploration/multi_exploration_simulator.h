#ifndef MULTI_EXPLORATION_SIMULATOR_H
#define MULTI_EXPLORATION_SIMULATOR_H

#include <memory>
#include <vector>

// 前方宣言
namespace ExpLib{
    namespace Struct{
        template<typename T>
        struct subStruct;
        template<typename T>
        struct pubStruct;
    }
}
namespace geometry_msgs{
    template <class ContainerAllocator>
    struct Point_;
    typedef ::geometry_msgs::Point_<std::allocator<void>> Point;
    template <class ContainerAllocator>
    struct Pose_;
    typedef ::geometry_msgs::Pose_<std::allocator<void>> Pose;
    template <class ContainerAllocator>
    struct PoseArray_;
    typedef ::geometry_msgs::PoseArray_<std::allocator<void>> PoseArray;
}
namespace exploration{
    class multi_exploration_simulatorConfig;
}
namespace dynamic_reconfigure{
    template <class ConfigType>
    class Server;
}
namespace visualization_msgs{
    template <class ContainerAllocator>
    struct Marker_;
    typedef ::visualization_msgs::Marker_<std::allocator<void>> Marker;
}
// 前方宣言ここまで

namespace ExStc = ExpLib::Struct;

class MultiExplorationSimulator{
    private:
        // dynamic parameters
        int ROBOT_NUMBER;
        int BRANCH_NUMBER;
        int FRONTIER_NUMBER;

        // static parameters
        std::string MAP_FRAME_ID;
        double BRANCH_SCALE;
        double FRONTIER_SCALE;
        std::string MULSIM_PARAMETER_FILE_PATH;
        bool OUTPUT_MULSIM_PARAMETERS;

        // variables
        std::unique_ptr<ExStc::pubStruct<geometry_msgs::PoseArray>> poses_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> branches_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> frontiers_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig>> drs_;
        std::unique_ptr<geometry_msgs::PoseArray> robotPoses_;
        std::unique_ptr<visualization_msgs::Marker> branchCoordinates_;
        std::unique_ptr<visualization_msgs::Marker> frontierCoordinates_;

        // functions
        void loadParams(void);
        void dynamicParamsCB(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        MultiExplorationSimulator();
        ~MultiExplorationSimulator();
        void updateParams(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn);
};

#endif // MULTI_EXPLORATION_SIMULATOR_H