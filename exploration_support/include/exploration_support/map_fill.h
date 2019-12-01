#ifndef MAP_FILL_H
#define MAP_FILL_H

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
namespace exploration_support{
    class map_fill_parameter_reconfigureConfig;
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
// 前方宣言ここまで

namespace ExStc = ExpLib::Struct;

class MapFill{
    private:
        // dynamic parameters
        double FILL_SIZE_MIN;
        double FILL_SIZE_MAX;

        // static parameters
        std::string FILL_PARAMETER_FILE_PATH;
        bool OUTPUT_FILL_PARAMETERS;

        // variables
        std::unique_ptr<ExStc::subStructSimple> map_;
        std::unique_ptr<ExStc::pubStruct<nav_msgs::OccupancyGrid>> fillMap_;
        std::unique_ptr<dynamic_reconfigure::Server<exploration_support::map_fill_parameter_reconfigureConfig>> drs_;

        // functions
        void mapCB(const nav_msgs::OccupancyGridConstPtr& msg);
        void loadParams(void);
        void dynamicParamsCB(exploration_support::map_fill_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        MapFill();
        ~MapFill();
};

#endif //MAP_FILL_HPP