#ifndef FRONTIER_DETECTION_H
#define FRONTIER_DETECTION_H

#include <memory>
#include <vector>

// 前方宣言

namespace boost{
    template<class T> 
    class shared_ptr;
}
/// my packages
namespace ExpLib{
    namespace Struct{
        template<typename T>
        struct pubStruct;
        struct subStructSimple;
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
namespace exploration_support{
    class frontier_detection_parameter_reconfigureConfig;
}
/// ros
namespace dynamic_reconfigure{
    template <class ConfigType>
    class Server;
}
/// rosmsgs
namespace nav_msgs{
    template <class ContainerAllocator>
    struct OccupancyGrid_;
    typedef ::nav_msgs::OccupancyGrid_<std::allocator<void>> OccupancyGrid;
    typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid const> OccupancyGridConstPtr;
}
namespace sensor_msgs{
    template <class ContainerAllocator>
    struct PointCloud2_;
    typedef ::sensor_msgs::PointCloud2_<std::allocator<void>> PointCloud2;
}
// 前方宣言ここまで

namespace ExStc = ExpLib::Struct;

class FrontierDetection{
    private:
        // dynamic parameters
        double CLUSTER_TOLERANCE;
        int MIN_CLUSTER_SIZE;
        int MAX_CLUSTER_SIZE;
        float FILTER_SQUARE_DIAMETER;

        // static parameters
        std::string FRONTIER_PARAMETER_FILE_PATH;
        bool OUTPUT_FRONTIER_PARAMETERS;

        // struct
        struct mapStruct;
        struct clusterStruct;

        // variables
        std::unique_ptr<ExStc::subStructSimple> map_;
        std::unique_ptr<ExStc::pubStruct<exploration_msgs::FrontierArray>> frontier_;
        std::unique_ptr<ExStc::pubStruct<sensor_msgs::PointCloud2>> horizon_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration_support::frontier_detection_parameter_reconfigureConfig>> drs_;

        // functions
        void mapCB(const nav_msgs::OccupancyGridConstPtr& msg);
        void horizonDetection(mapStruct& map);
        clusterStruct clusterDetection(const mapStruct& map);
        void obstacleFilter(mapStruct& map,clusterStruct& cs);
        void publishHorizon(const clusterStruct& cs, const std::string& frameId);
        void publishFrontier(const std::vector<exploration_msgs::Frontier>& frontiers, const std::string& frameId);
        void loadParams(void);
        void dynamicParamsCB(exploration_support::frontier_detection_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        FrontierDetection();
        ~FrontierDetection();
};

#endif //FRONTIER_DETECTION_HPP