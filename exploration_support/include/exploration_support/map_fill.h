#ifndef MAP_FILL_H
#define MAP_FILL_H

#include <exploration_libraly/struct.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/map_fill_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;

class MapFill
{
private:
    // dynamic parameters
    double FILL_SIZE_MIN;
    double FILL_SIZE_MAX;

    // static parameters
    std::string FILL_PARAMETER_FILE_PATH;
    bool OUTPUT_FILL_PARAMETERS;

    // variables
    ExStc::subStructSimple map_;
    ExStc::pubStruct<nav_msgs::OccupancyGrid> fillMap_;
    dynamic_reconfigure::Server<exploration_support::map_fill_parameter_reconfigureConfig> drs_;

    // functions
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void loadParams(void);
    void dynamicParamsCB(exploration_support::map_fill_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    MapFill();
    ~MapFill();
};

#endif //MAP_FILL_HPP