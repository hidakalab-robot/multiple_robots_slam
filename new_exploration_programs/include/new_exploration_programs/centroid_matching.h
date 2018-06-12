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
#include <pcl/registration/icp.h>

/*マッチングリストを受信して、その受信を使ってセンサデータをくっつける*/

class CentroidMatching
{
private:
  ros::NodeHandle smi;
  ros::NodeHandle pmi;

  ros::Subscriber smi_sub;

  ros::Publisher pmc_pub;

  ros::Publisher mids_pub;
  ros::Publisher midm_pub;

  new_exploration_programs::matching_info info;

  sensor_msgs::PointCloud2 centroid_merged_cloud_r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_merged_cloud;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_source_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr match_source_cloud;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr indi_merged_cloud_m;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr indi_source_cloud_m;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpout_cloud;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr for_merge_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr for_icpmerged_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr match_merged_cloud;

  std::vector<float> angle_m;

  //Eigen::Matrix4f icp_matrix;

  Eigen::Matrix3f icp_rot_matrix;
  Eigen::Vector3f icp_tra_vector;

  Eigen::Vector3f offset;//重心を原点にするときの移動

  Eigen::Matrix3f rot;//一次回転
  Eigen::Vector3f trans;//一次平行移動


public:
  ros::CallbackQueue mi_queue;//マッチング情報を受けとる
  bool input_info;
  bool one_matching;
  bool no_matching;

  CentroidMatching();

  ~CentroidMatching(){};

  void input_matchinginfo(const new_exploration_programs::matching_info::ConstPtr& mi_msg);
  void publish_mergedcloud(void);
  void centroid_vector(void);
  void moving_cloud(void);
  void merging_cloud(void);
  void if_onematching(void);
  //void if_nomatching(void);
  bool is_merged_empty(void);
  void nan_check(pcl::PointCloud<pcl::PointXYZRGB>& target_cloud);
  void icp_estimate(int merge_num, int source_num, std::vector<Eigen::Matrix4f>& all_icp_matrix);
  void independ_matchingcloud(int merged_num, int source_num);
  void final_transform(void);
  void middle_publish(void);
  void calc_rotra(std::vector<Eigen::Matrix4f>& all_icp_matrix);
  void icp4allcluster(void);
  void calc_Vangle(int merged_num, int source_num);
  void moving2center(int merged_num, int source_num);
  void nonicp_estimate(void);
  float angle_decision(int list_num, float angle);
  void nonicp_transform(float angle,int merged_num,int source_num);
  void icp_transform(void);
};
