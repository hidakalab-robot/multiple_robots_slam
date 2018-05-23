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

#include <visualization_msgs/MarkerArray.h>

class FeatureMatching
{
private:
  ros::NodeHandle ssc;
	ros::NodeHandle smc;

  ros::NodeHandle psc;
	ros::NodeHandle pmc;

  ros::NodeHandle prc;


  ros::Subscriber sc_sub;
  ros::Subscriber mc_sub;

  ros::Publisher sc_pub1;
	ros::Publisher sc_pub2;
	ros::Publisher sc_pub3;
	ros::Publisher sc_pub4;

  ros::Publisher mc_pub1;
	ros::Publisher mc_pub2;
	ros::Publisher mc_pub3;
	ros::Publisher mc_pub4;

  ros::Publisher rsc_pub;
  ros::Publisher rmc_pub;
  ros::Publisher rsm_pub;

  ros::Publisher origm_pub;
  ros::Publisher origs_pub;

  new_exploration_programs::segmented_cloud source_cloud;
  new_exploration_programs::segmented_cloud master_cloud;


  std::vector<Eigen::Vector2i> matching_list_m;

  visualization_msgs::MarkerArray matching_line_list_m;

  float shift_position;
  float matching_threshold;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr master_cloud_for_shift;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_master_cloud_for_shift;
  sensor_msgs::PointCloud2 shift_master_cloud;
  sensor_msgs::PointCloud2 shift_orig_master_cloud;

public:
  ros::CallbackQueue sc_queue;
  ros::CallbackQueue mc_queue;
  bool input_master;
  bool input_source;
  bool changed_master;
  bool matching;
  FeatureMatching();

  virtual ~FeatureMatching(){};

  void input_sourcecloud(const new_exploration_programs::segmented_cloud::ConstPtr& sc_msg);
  void input_mastercloud(const new_exploration_programs::segmented_cloud::ConstPtr& mc_msg);
  void mastercloud_test(void);
  void sourcecloud_test(void);
  virtual void matching_calc(void);
  void show_matching(void);
  void publish_registeredcloud(void);
  void publish_matchingline(void);
  void publish_orig(void);
  void shift_mcloud(void);
  void store(void);
};
