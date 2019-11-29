#ifndef FRONTIER_DETECTION_H
#define FRONTIER_DETECTION_H

#include <exploration_libraly/construct.h>
#include <exploration_libraly/struct.h>
#include <exploration_msgs/FrontierArray.h>
#include <Eigen/Geometry>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <exploration_libraly/utility.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/frontier_detection_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;
namespace ExUtl = ExpLib::Utility;
namespace ExCos = ExpLib::Construct;

class FrontierDetection
{
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
    struct mapStruct{
        nav_msgs::MapMetaData info;
        std::vector<std::vector<int8_t>> source;
        std::vector<std::vector<int8_t>> horizon;
        std::vector<std::vector<int8_t>> frontierMap;
        mapStruct(const nav_msgs::OccupancyGrid& m);
    };
    struct clusterStruct{
        std::vector<Eigen::Vector2i> index;
        std::vector<int> isObstacle;
        std::vector<double> areas;
        std::vector<Eigen::Vector2d> variances;
        std::vector<double> covariance;
        std::vector<pcl::PointIndices> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

        clusterStruct();
        clusterStruct(const clusterStruct& cs);
        void reserve(int size);
    };

    // variables
    ExStc::subStructSimple map_;
    ExStc::pubStruct<exploration_msgs::FrontierArray> frontier_;
    ExStc::pubStruct<sensor_msgs::PointCloud2> horizon_;
    dynamic_reconfigure::Server<exploration_support::frontier_detection_parameter_reconfigureConfig> drs_;

    // functions
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
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