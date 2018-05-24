#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <limits>

#include <new_exploration_programs/segmented_cloud.h>
#include <new_exploration_programs/matching_info.h>


#include <visualization_msgs/MarkerArray.h>


class CentroidMatching
{
private:
  //new_exploration_programs::segmented_cloud pre_source_cloud;

public:
  CentroidMatching(){};
  ~CentroidMatching(){};

  //void matching_calc(void);
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "centroid_matching");
	CentroidMatching cm;


  return 0;
}
