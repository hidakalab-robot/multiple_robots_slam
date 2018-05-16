#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <limits>

#include <new_exploration_programs/segmented_cloud.h>

//#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>

//#include <tf/transform_listener.h>

//#include <pcl_ros/transforms.h>

//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <geometry_msgs/TransformStamped.h>

#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>

class ScanMatching
{
private:
  ros::NodeHandle psm;

	ros::NodeHandle ssd;
  ros::NodeHandle sod;

  ros::Subscriber sd_sub;
  ros::Subscriber od_sub;

  ros::Publisher mc_pub;

  //ros::Publisher t1_pub;
  //ros::Publisher t2_pub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pre_input_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud;

  nav_msgs::Odometry input_odom;
  nav_msgs::Odometry pre_input_odom;

  nav_msgs::Odometry estimate_odom;

  sensor_msgs::PointCloud2 merged_cloud_r;

  //tf::TransformListener tflistener;
  //geometry_msgs::TransformStamped transform;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr input_tf_cloud;
  //sensor_msgs::PointCloud2 input_msg;
  //sensor_msgs::PointCloud2 output_msg;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr tfin_cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr tfout_cloud;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr icpout_cloud;

  pcl::VoxelGrid<pcl::PointXYZ> vg;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr del_unval_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr del_ceiling_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_input_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr re_voxeled_input_cloud;

  float camera_position_y;
  float ceiling_position_y;
  float floor_position_y;
  float per_range_z;

  float x;
  float z;
  float move_x;
  float move_z;
  float move_s;

public:
  ros::CallbackQueue sd_queue;
  ros::CallbackQueue od_queue;
  bool input_c;
  bool input_o;

  ScanMatching()
  :input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //pre_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //input_tf_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //tfin_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //tfout_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  icpout_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //del_unval_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  //merged_cloud2(new pcl::PointCloud<pcl::PointXYZ>)
  //del_ceiling_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  voxeled_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  re_voxeled_input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  {
    ssd.setCallbackQueue(&sd_queue);
    sd_sub = ssd.subscribe("/camera/depth_registered/points",1,&ScanMatching::input_pointcloud,this);
    sod.setCallbackQueue(&od_queue);
    od_sub = sod.subscribe("/odom",1,&ScanMatching::input_odometry,this);
    input_c = false;
    input_o = false;
    mc_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/merged_cloud", 1);

    //t1_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/test_cloud1", 1);
    //t2_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/test_cloud2", 1);

    //transform.header.frame_id = "odom";
    //transform.child_frame_id = "camera_rgb_optical_frame";

    //vg.setLeafSize (0.1f, 0.1f, 0.1f);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.10);
    // Set the maximum number of iterations (criterion 1)
    //icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    //icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);

    camera_position_y = 0.41;
    ceiling_position_y = 2.3;
    floor_position_y = 0.3;
    per_range_z = 6.0;
  };
  ~ScanMatching(){};

  void input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
  void input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg);
  void voxeling(void);
  void remove_unrelialbe(void);
  void convert3dto2d(void);
  void odomove_cloud(void);
  void icp_test(void);
  void myICP(void);
  void publish_mergedcloud(void);
};

void ScanMatching::input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
  //input_msg = *pc_msg;
	pcl::fromROSMsg (*pc_msg, *input_cloud);
	std::cout << "input_pointcloud" << std::endl;
	input_c = true;
}

void ScanMatching::input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg)
{
  input_odom = *od_msg;
  std::cout << "input_odometry" << std::endl;
  input_o = true;
}

void ScanMatching::voxeling(void)
{
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.setInputCloud (input_cloud);
	//vg.filter (*del_unval_cloud);
  vg.filter (*voxeled_input_cloud);
}

void ScanMatching::remove_unrelialbe(void)
{
  // float camera_position_y = 0.41;
  // float ceiling_position_y = 2.3;
  // float floor_position_y = 0.3;
  // float per_range_z = 6.0;
  //float nan_c = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for(int i=0;i<voxeled_input_cloud->points.size();i++)
	{
		if(voxeled_input_cloud->points[i].y > camera_position_y - ceiling_position_y && voxeled_input_cloud->points[i].y < camera_position_y - floor_position_y) //&& voxeled_input_cloud->points[i].z < per_range_z)
		{
      remove_cloud->points.push_back(voxeled_input_cloud->points[i]);
		}
	}
  *voxeled_input_cloud = *remove_cloud;
  //vg.setLeafSize (0.1f, 0.1f, 0.1f);
  //vg.setInputCloud (del_unval_cloud);
	//vg.filter (*del_unval_cloud);
  //vg.filter (*merged_cloud);
  //*merged_cloud = *del_unval_cloud;
  voxeled_input_cloud -> width = voxeled_input_cloud -> points.size();
  voxeled_input_cloud -> height = 1;
  voxeled_input_cloud -> is_dense = false;
}

void ScanMatching::convert3dto2d(void)
{
  for(int i=0;i<voxeled_input_cloud->points.size();i++)
	{
    voxeled_input_cloud->points[i].y = camera_position_y;
	}

  vg.setLeafSize (0.2f, 0.2f, 0.2f);
  vg.setInputCloud (voxeled_input_cloud);
  vg.filter (*re_voxeled_input_cloud);

  *voxeled_input_cloud = *re_voxeled_input_cloud;
}

void ScanMatching::odomove_cloud(void)
{
  move_x = -input_odom.pose.pose.position.y;
  move_z = input_odom.pose.pose.position.x;
  move_s = 2*asin(input_odom.pose.pose.orientation.z);

  // float move_x = input_odom.pose.pose.position.x;
  // float move_y = input_odom.pose.pose.position.y;
  // float move_s = 2*asin(input_odom.pose.pose.orientation.z);

  //std::cout << "move_x: " << move_x;
  //std::cout << ", move_z: " << move_z;
  //std::cout << ", move_s: " << move_s*180/M_PI << '\n';

  for(int i=0;i<voxeled_input_cloud->points.size();i++)
  {
    x = voxeled_input_cloud->points[i].x;
    z = voxeled_input_cloud->points[i].z;

    x = x*cos(move_s)-z*sin(move_s);
    z = x*sin(move_s)+z*cos(move_s);

    voxeled_input_cloud->points[i].x = x + move_x;
    voxeled_input_cloud->points[i].z = z + move_z;
  }
}

void ScanMatching::icp_test(void)
{
  if(merged_cloud->points.size()!=0)
  {
    std::cout << "icp" << '\n';
    //IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    //icp.setInputSource (del_unval_cloud);
    icp.setInputSource (voxeled_input_cloud);
    icp.setInputTarget (merged_cloud);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    //icp.setMaxCorrespondenceDistance (0.10);
    // Set the maximum number of iterations (criterion 1)
    //icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    //icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);
    // Perform the alignment
    icp.align (*icpout_cloud);
    //icp.computeTransformation
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();
    transformation_matrix = icp.getFinalTransformation ().cast<float>();
    std::cout << transformation_matrix << '\n';
    /*
    R
    matrix (0, 0), matrix (0, 1), matrix (0, 2);
    matrix (1, 0), matrix (1, 1), matrix (1, 2)
    matrix (2, 0), matrix (2, 1), matrix (2, 2);

    T
    matrix (0, 3), matrix (1, 3), matrix (2, 3)
    */
  }
  else
  {
    *icpout_cloud = *voxeled_input_cloud;
  }

  *merged_cloud += *icpout_cloud;
}


void ScanMatching::myICP(void)
{
  //voxeled_input_cloud

  /*init_move*/
  Eigen::Matrix3f R_estimate;
  Eigen::Vector3f T_estimate;
  Eigen::Vector3f point_vector;
  Eigen::Vector3f move_point_vector;

  for (int i=0;i<voxeled_input_cloud->points.size();i++)
  {
    point_vector << voxeled_input_cloud->points[i].x,voxeled_input_cloud->points[i].y,voxeled_input_cloud->points[i].z;
    move_point_vector = R_estimate*point_vector + T_estimate;
    voxeled_input_cloud->points[i].x = move_point_vector(0);
    voxeled_input_cloud->points[i].y = move_point_vector(1);
    voxeled_input_cloud->points[i].z = move_point_vector(2);
  }

}


void ScanMatching::publish_mergedcloud(void)
{
  //*merged_cloud = *voxeled_input_cloud;
  pcl::toROSMsg (*merged_cloud, merged_cloud_r);
  merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
  mc_pub.publish(merged_cloud_r);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_matching");
	ScanMatching sm;

  //ros::Rate rate(1);

  while(ros::ok()){
    sm.sd_queue.callOne(ros::WallDuration(1));
    sm.od_queue.callOne(ros::WallDuration(1));
    if(sm.input_c && sm.input_o)
    {
      sm.voxeling();
      sm.remove_unrelialbe();
      sm.convert3dto2d();
      sm.odomove_cloud();
      sm.icp_test();
      sm.publish_mergedcloud();
    }
    else
    {
      std::cout << '\n' << "not input" << '\n';
    }
    sm.input_c = false;
    sm.input_o = false;
    //rate.sleep();
  }
	return 0;
}
