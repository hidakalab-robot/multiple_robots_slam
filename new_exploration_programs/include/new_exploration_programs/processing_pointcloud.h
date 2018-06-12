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
#include <geometry_msgs/Point.h>

#include <std_msgs/Header.h>


//#include <Eigen/Dense>

class ProcessingPointCloud
{
private:
	ros::NodeHandle ppcs;
	ros::NodeHandle ppcs2;

	ros::NodeHandle rtabs;
	ros::NodeHandle rtabs2;

	ros::NodeHandle ppcp;

	ros::Subscriber pc_sub;
	ros::Subscriber pc_sub2;

	ros::Subscriber rtab_sub;
	ros::Subscriber rtab_sub2;

	ros::Publisher pc_pub1;
	ros::Publisher pc_pub2;
	ros::Publisher pc_pub3;
	ros::Publisher pc_pub4;
	ros::Publisher pc_pub5;
	ros::Publisher seg_pub;
	ros::Publisher seg_pub2;

	ros::Publisher loc_pub;

	float camera_position_y;//カメラの高さ
	float ground_position_y;//どのくらいの高さまで床とするか

	float cloud_position;//表示の時それぞれの点群をどれくらい離すか

	float nan_c;

	/*ポイントクラウドの変数定義*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxeled_cloud;
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr deleted_ground_cloud;
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr for_detect_ground_cloud;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr for_view_ground_cloud;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;//何か探索用にツリーを作る

	std::vector<pcl::PointIndices> cluster_indices_m;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud;

	sensor_msgs::PointCloud2 orig_cloud;
	sensor_msgs::PointCloud2 vox_cloud;
	sensor_msgs::PointCloud2 gro_cloud;
	sensor_msgs::PointCloud2 del_cloud;
	sensor_msgs::PointCloud2 clu_cloud;

	new_exploration_programs::index clu_index;
	new_exploration_programs::f_vector clu_feature;
	new_exploration_programs::segmented_cloud source_cloud;
	geometry_msgs::Point clu_centroid;

	std_msgs::Header row_header;

public:
	ros::CallbackQueue pc_queue;
	ros::CallbackQueue pc_queue2;

	ros::CallbackQueue rtab_queue;
	ros::CallbackQueue rtab_queue2;

	bool input;

	bool input_m;
	bool input_o;

	ProcessingPointCloud();
	virtual ~ProcessingPointCloud(){};

	void input_source_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
	void input_merged_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);

	void input_rtabcloudM(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
	void input_rtabcloudO(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);

	void apply_voxelgrid(void);
	void delete_ground(void);
	void euclidean_clustering(void);
	void publish_pointcloud(void);
	void feature_extraction(void);
	void publish_source_segmented(void);
	void publish_merged_segmented(void);
	bool is_empty(void);
	void publish_empty_merged(void);
	void test_cloud(void);
	void naninf(void);

	void publish_localmap(void);
};
