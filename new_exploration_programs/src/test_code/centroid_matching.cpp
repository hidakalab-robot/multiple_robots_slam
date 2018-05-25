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

/*マッチングリストを受信して、その受信を使ってセンサデータをくっつける*/

class CentroidMatching
{
private:
  ros::NodeHandle smi;
  ros::NodeHandle pmi;

  ros::Subscriber smi_sub;

  ros::Publisher pmc_pub;

  new_exploration_programs::matching_info info;

  sensor_msgs::PointCloud2 centroid_merged_cloud_r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_merged_cloud;


public:
  ros::CallbackQueue mi_queue;//マッチング情報を受けとる
  bool input_info;

  CentroidMatching()
  :centroid_merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    smi.setCallbackQueue(&mi_queue);
    smi_sub = smi.subscribe("/pointcloud_matching/matching_info",1,&CentroidMatching::input_matchinginfo,this);
    pmc_pub = pmi.advertise<sensor_msgs::PointCloud2>("centroid_matching/merged_cloud", 1);
    input_info = false;
  };
  ~CentroidMatching(){};

  void input_matchinginfo(const new_exploration_programs::matching_info::ConstPtr& mi_msg);
  void publish_mergedcloud(void);

};


void CentroidMatching::input_matchinginfo(const new_exploration_programs::matching_info::ConstPtr& mi_msg)
{
  info = *mi_msg;
  input_info = true;
  std::cout << "input_matchinginfo" << '\n';
}

void CentroidMatching::publish_mergedcloud(void)
{
  pcl::toROSMsg (*centroid_merged_cloud, centroid_merged_cloud_r);
  pmc_pub.publish(centroid_merged_cloud_r);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "centroid_matching");
	CentroidMatching cm;

  while(ros::ok())
  {
    cm.mi_queue.callOne(ros::WallDuration(1));
    if(cm.input_info)
    {
      cm.publish_mergedcloud();
    }
    else
    {
      std::cout << "not_input_matchinginfo" << '\n';
    }
  }


  return 0;
}
