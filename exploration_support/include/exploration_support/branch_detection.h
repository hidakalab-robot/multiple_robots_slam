#ifndef BRANCH_DETECTION_H
#define BRANCH_DETECTION_H

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
        template<typename T>
        struct subStruct;
        struct subStructSimple;
    }
}
namespace exploration_msgs{
    template <class ContainerAllocator>
    struct Branch_;
    typedef ::exploration_msgs::Branch_<std::allocator<void>> Branch;
    template <class ContainerAllocator>
    struct BranchArray_;
    typedef ::exploration_msgs::BranchArray_<std::allocator<void>> BranchArray;
    template <class ContainerAllocator>
    struct PointArray_;
    typedef ::exploration_msgs::PointArray_<std::allocator<void>> PointArray;
}
namespace exploration_support{
    class branch_detection_parameter_reconfigureConfig;
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
    struct PoseStamped_;
    typedef ::geometry_msgs::PoseStamped_<std::allocator<void>> PoseStamped;     
}
namespace nav_msgs{
    template <class ContainerAllocator>
    struct OccupancyGrid_;
    typedef ::nav_msgs::OccupancyGrid_<std::allocator<void>> OccupancyGrid;
    template <class ContainerAllocator>
    struct Path_;
    typedef ::nav_msgs::Path_<std::allocator<void>> Path;
}
namespace sensor_msgs{
    template <class ContainerAllocator>
    struct LaserScan_;
    typedef ::sensor_msgs::LaserScan_<std::allocator<void>> LaserScan;
    typedef boost::shared_ptr< ::sensor_msgs::LaserScan const> LaserScanConstPtr;
}
// 前方宣言ここまで

namespace ExStc = ExpLib::Struct;

class BranchDetection{
    private:
        // dynamic parameters
        double OBSTACLE_CHECK_ANGLE;
        double OBSTACLE_RANGE_THRESHOLD;
        double BRANCH_RANGE_MIN;
        double BRANCH_RANGE_MAX;
        double BRANCH_DIFF_X_MIN;
        double BRANCH_DIFF_X_MAX;
        double BRANCH_DIFF_Y_MIN;
        double BRANCH_DIFF_Y_MAX;
        bool SCAN_FILTER;
        int SCAN_FILTER_ORDER;
        bool BRANCH_FILTER;
        int BRANCH_FILTER_ORDER;
        double BRANCH_FILTER_TOLERANCE;
        bool DUPLICATE_DETECTION;
        double DUPLICATE_TOLERANCE;
        double LOG_CURRENT_TIME;//if 30 -> 30秒前までのログで重複検出
        double NEWER_DUPLICATION_THRESHOLD;//最近通った場所の重複とみなす時間の上限,時間の仕様はLOG_NEWER_LIMITと同じ
        bool ON_MAP_BRANCH_DETECTION;
        double OMB_MAP_WINDOW_X;
        double OMB_MAP_WINDOW_Y;
        double ON_MAP_BRANCH_RATE;
        

        // static parameters
        std::string BRANCH_PARAMETER_FILE_PATH;
        bool OUTPUT_BRANCH_PARAMETERS;

        // variables
        std::unique_ptr<ExStc::subStructSimple> scan_;
        std::unique_ptr<ExStc::subStruct<geometry_msgs::PoseStamped>> pose_;
        std::unique_ptr<ExStc::subStruct<nav_msgs::Path>> poseLog_;
        std::unique_ptr<ExStc::subStruct<nav_msgs::OccupancyGrid>> map_;
        // std::unique_ptr<ExStc::pubStruct<exploration_msgs::PointArray>> branch_;
        std::unique_ptr<ExStc::pubStruct<exploration_msgs::BranchArray>> branch_;
        std::unique_ptr<ExStc::pubStruct<sensor_msgs::LaserScan>> filteredScan_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration_support::branch_detection_parameter_reconfigureConfig>> drs_;

        // functions
        void scanCB(const sensor_msgs::LaserScanConstPtr& msg);
        sensor_msgs::LaserScan scanFilter(const sensor_msgs::LaserScan& scan);
        // void branchFilter(std::vector<geometry_msgs::Point>& branches);
        void branchFilter(std::vector<exploration_msgs::Branch>& branches);
        void duplicateBranchDetection(std::vector<exploration_msgs::Branch>& branches);
        void onMapBranchDetection(std::vector<exploration_msgs::Branch>& branches);
        // void publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId);
        void publishBranch(const std::vector<exploration_msgs::Branch>& branches, const std::string& frameId);
        void loadParams(void);
        void dynamicParamsCB(exploration_support::branch_detection_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        BranchDetection();
        ~BranchDetection();
};

#endif // BRANCH_DETECTION_H