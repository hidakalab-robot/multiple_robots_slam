//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_H
#define EXPLORATION_MANAGER_H

#include <memory>

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
    struct FrontierArray_;
    typedef ::exploration_msgs::FrontierArray_<std::allocator<void>> FrontierArray;
    typedef boost::shared_ptr< ::exploration_msgs::FrontierArray const> FrontierArrayConstPtr;
}
namespace exploration_support{
    class exploration_manager_parameter_reconfigureConfig;
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
namespace std_msgs{
    template <class ContainerAllocator>
    struct Bool_;
    typedef ::std_msgs::Bool_<std::allocator<void>> Bool;
    template <class ContainerAllocator>
    struct Float64_;
    typedef ::std_msgs::Float64_<std::allocator<void>> Float64;
    template <class ContainerAllocator>
    struct Int32_;
    typedef ::std_msgs::Int32_<std::allocator<void>> Int32;
}
// 前方宣言ここまで

namespace ExStc = ExpLib::Struct;

class ExplorationManager{
    private:
        // dynamic parameters
        double END_AREA;
        int END_FRONTIER;
        double END_TIME;
        double END_AREA_DIFF;
        double END_AREA_DIFF_INTERVAL;

        // static parameters
        std::string EXMNG_PARAMETER_FILE_PATH;
        bool OUTPUT_EXMNG_PARAMETERS;

        // variables
        std::unique_ptr<ExStc::subStructSimple> map_;
        std::unique_ptr<ExStc::subStructSimple> frontier_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Bool>> areaEnd_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Bool>> frontierEnd_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Bool>> timerEnd_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Bool>> areaDiffEnd_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Float64>> areaVal_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Int32>> frontierVal_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Float64>> timerVal_;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Float64>> areaDiffVal_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration_support::exploration_manager_parameter_reconfigureConfig>> drs_;
        double area_;

        // functions
        void mapCB(const nav_msgs::OccupancyGridConstPtr& msg);
        void frontierCB(const exploration_msgs::FrontierArrayConstPtr& msg);
        void timer(void);
        void areaDiff(void);
        void loadParams(void);
        void dynamicParamsCB(exploration_support::exploration_manager_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        ExplorationManager();
        ~ExplorationManager();
        void multiThreadMain(void);
};

#endif //EXPLORATION_MANAGER_H