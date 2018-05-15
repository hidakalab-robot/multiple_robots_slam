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

#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/registration/icp.h>

class ScanMatching
{
private:
  ros::NodeHandle psm;

	ros::NodeHandle ssd;
  ros::NodeHandle sod;

  ros::Subscriber sd_sub;
  ros::Subscriber od_sub;

  ros::Publisher mc_pub;

  ros::Publisher t1_pub;
  ros::Publisher t2_pub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pre_input_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud;

  nav_msgs::Odometry input_odom;
  //nav_msgs::Odometry pre_input_odom;
  sensor_msgs::PointCloud2 merged_cloud_r;

  tf::TransformListener tflistener;
  geometry_msgs::TransformStamped transform;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_tf_cloud;
  sensor_msgs::PointCloud2 input_msg;
  sensor_msgs::PointCloud2 output_msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr tfin_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tfout_cloud;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr icpout_cloud;

  pcl::VoxelGrid<pcl::PointXYZ> vg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr del_unval_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr del_ceiling_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud2;

public:
  ros::CallbackQueue sd_queue;
  ros::CallbackQueue od_queue;
  bool input_c;
  bool input_o;

  ScanMatching()
  :input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //pre_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  input_tf_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  tfin_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  tfout_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  icpout_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  del_unval_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  //merged_cloud2(new pcl::PointCloud<pcl::PointXYZ>)
  //del_ceiling_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  {
    ssd.setCallbackQueue(&sd_queue);
    sd_sub = ssd.subscribe("/camera/depth_registered/points",1,&ScanMatching::input_pointcloud,this);
    sod.setCallbackQueue(&od_queue);
    od_sub = sod.subscribe("/odom",1,&ScanMatching::input_odometry,this);
    input_c = false;
    input_o = false;
    mc_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/merged_cloud", 1);

    t1_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/test_cloud1", 1);
    t2_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/test_cloud2", 1);

    transform.header.frame_id = "odom";
    transform.child_frame_id = "camera_rgb_optical_frame";
  };
  ~ScanMatching(){};

  void input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
  void input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg);
  void merge_cloud_test(void);
  void publish_mergedcloud(void);
  void tf_test(void);
  void icp_test(void);
  void del_unknown_val(void);
  void print44Matrix (const Eigen::Matrix4d& matrix);
  void del_floor(void);
  void remove_unrelialbe(void);
  void odomove_cloud(void);
};

void ScanMatching::input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
  input_msg = *pc_msg;
	pcl::fromROSMsg (*pc_msg, *input_cloud);
	//pcl::toROSMsg (*input_cloud, orig_cloud);
	std::cout << "input_pointcloud" << std::endl;
	input_c = true;
}

void ScanMatching::input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg)
{
  input_odom = *od_msg;
  std::cout << "input_odometry" << std::endl;
  input_o = true;
}

void ScanMatching::merge_cloud_test(void)
{
  //pcl::fromROSMsg (output_msg, *input_cloud);

  float x,y,z;
  float move_x = -input_odom.pose.pose.position.y;
  float move_z = input_odom.pose.pose.position.x;
  float move_s = 2*asin(input_odom.pose.pose.orientation.z);

  // float move_x = input_odom.pose.pose.position.x;
  // float move_y = input_odom.pose.pose.position.y;
  // float move_s = 2*asin(input_odom.pose.pose.orientation.z);

  std::cout << "move_x: " << move_x;
  std::cout << ", move_z: " << move_z;
  std::cout << ", move_s: " << move_s*180/M_PI << '\n';

  for(int i=0;i<input_cloud->points.size();i++)
  {
    //std::cout << i << '\n';
    x = input_cloud->points[i].x;// + move_x;
    //std::cout << "2" << '\n';
    z = input_cloud->points[i].z;// + move_z;
    //std::cout << "3" << '\n';
    x = x*cos(move_s)-z*sin(move_s);
    //std::cout << "4" << '\n';
    z = x*sin(move_s)+z*cos(move_s);
    //std::cout << "5" << '\n';
    input_cloud->points[i].x = x + move_x;
    input_cloud->points[i].z = z + move_z;
  }


  // for(int i=0;i<input_cloud->points.size();i++)
  // {
  //   //std::cout << i << '\n';
  //   x = input_cloud->points[i].x;// + move_x;
  //   //std::cout << "2" << '\n';
  //   y = input_cloud->points[i].y;// + move_z;
  //   //std::cout << "3" << '\n';
  //   x = x*cos(move_s)-y*sin(move_s);
  //   //std::cout << "4" << '\n';
  //   y = x*sin(move_s)+y*cos(move_s);
  //   //std::cout << "5" << '\n';
  //   input_cloud->points[i].x = x + move_x;
  //   input_cloud->points[i].y = y + move_y;
  // }

  *merged_cloud += *input_cloud;

  //if(pre_input_cloud.points.size() != 0)
  //{
  //  merged_cloud +=
  //}
  //pre_input_cloud = input_cloud;
  //pre_input_odom = input_odom;
}

void ScanMatching::tf_test(void)
{
  // //pcl_ros::transformPointCloud("odom",input_msg,output_msg,tflistener);
  // tf2::doTransform (input_msg, output_msg, transform);
  // //t1_pub.publish(input_msg);
  // //t2_pub.publish(output_msg);
  // pcl::fromROSMsg (input_msg, *tfin_cloud);
  // pcl::fromROSMsg (output_msg, *tfout_cloud);
  //
  // for(int i=0;i<tfin_cloud->points.size();i++)
  // {
  //   std::cout << tfin_cloud->points[i].y<< ", " << tfout_cloud->points[i].y <<  '\n';
  //   if(!ros::ok())
  //     break;
  // }
}

void print44Matrix (Eigen::Matrix4d& matrix)
{
  //printf ("Rotation matrix :\n");
  //printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  //printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  //printf ("Translation vector :\n");
//  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void ScanMatching::icp_test(void)
{
  //icp.computeTransformation(icpout_cloud,guess)
  if(merged_cloud->points.size()!=0)
  {
    std::cout << "icp" << '\n';
    //IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource (del_unval_cloud);
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
    //Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    //transformation_matrix = icp.getFinalTransformation ().cast<double>();
    //print44Matrix (transformation_matrix);
    //std::cout << icp.getFinalTransformation() << '\n';
  }
  else
  {
    *icpout_cloud = *del_unval_cloud;
  }


  *merged_cloud += *icpout_cloud;
}

void ScanMatching::publish_mergedcloud(void)
{
  //pcl::toROSMsg (*merged_cloud, merged_cloud_r);
  pcl::toROSMsg (*merged_cloud, merged_cloud_r);
  merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
  mc_pub.publish(merged_cloud_r);
}

void ScanMatching::del_unknown_val(void)
{
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.setInputCloud (input_cloud);
	vg.filter (*del_unval_cloud);
}

void ScanMatching::del_floor(void)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr detect_floor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr detect_ceiling_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*del_unval_cloud, *detect_floor_cloud);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);//RANSACの繰り返し回数
  seg.setDistanceThreshold (0.1);//モデルとどのくらい離れていてもいいか???謎
  seg.setAxis(Eigen::Vector3f (0.0,1.0,0.0));//法線ベクトル
  seg.setEpsAngle(15.0f * (M_PI/180.0f));//許容出来る平面の傾きラジアン

  float camera_position_y = 0.41;
  float floor_position_y = 0.3;
  //float ceiling_position_y = 1.8;
  float nan_c = std::numeric_limits<float>::quiet_NaN();

	for(int i=0;i<detect_floor_cloud->points.size();i++)
	{
		if(detect_floor_cloud->points[i].y < camera_position_y - floor_position_y)
		{
			detect_floor_cloud->points[i].x = nan_c;
		  detect_floor_cloud->points[i].y = nan_c;
			detect_floor_cloud->points[i].z = nan_c;
		}
	}

	 seg.setInputCloud (detect_floor_cloud);
	 seg.segment (*inliers, *coefficients);

	 if (inliers->indices.size () == 0)
	 {
		 std::cout << "Could not estimate floor" << std::endl;
	 }
   else
   {
     extract.setInputCloud (del_unval_cloud);
  	 extract.setIndices (inliers);
  	 extract.setNegative (true);//true:平面を削除、false:平面以外削除
  	 extract.filter (*del_unval_cloud);
   }


  //  pcl::copyPointCloud(*del_unval_cloud, *detect_ceiling_cloud);
  //  for(int i=0;i<detect_ceiling_cloud->points.size();i++)
 	// {
 	// 	if(detect_ceiling_cloud->points[i].y > camera_position_y - ceiling_position_y)
 	// 	{
 	// 		detect_ceiling_cloud->points[i].x = nan_c;
 	// 	  detect_ceiling_cloud->points[i].y = nan_c;
 	// 		detect_ceiling_cloud->points[i].z = nan_c;
 	// 	}
 	// }
  //
 	//  seg.setInputCloud (detect_ceiling_cloud);
 	//  seg.segment (*inliers, *coefficients);
  //
 	//  if (inliers->indices.size () == 0)
 	//  {
 	// 	 std::cout << "Could not estimate ceiling" << std::endl;
 	//  }
  // else
  // {
  //   extract.setInputCloud (del_unval_cloud);
  //  	extract.setIndices (inliers);
  //  	extract.setNegative (true);//true:平面を削除、false:平面以外削除
  //  	extract.filter (*del_unval_cloud);
  // }


}

void ScanMatching::remove_unrelialbe(void)
{
  float camera_position_y = 0.41;
  float ceiling_position_y = 2.3;
  float floor_position_y = 0.3;
  float per_range_z = 6.0;
  float nan_c = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZ>::Ptr del_ceiling_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for(int i=0;i<del_unval_cloud->points.size();i++)
	{
		if(del_unval_cloud->points[i].y > camera_position_y - ceiling_position_y && del_unval_cloud->points[i].y < camera_position_y - floor_position_y && del_unval_cloud->points[i].z < per_range_z)
		{
			//del_unval_cloud->points[i].x = nan_c;
		  //del_unval_cloud->points[i].y = nan_c;
			//del_unval_cloud->points[i].z = nan_c;
      del_ceiling_cloud->points.push_back(del_unval_cloud->points[i]);
		}
	}
  *del_unval_cloud = *del_ceiling_cloud;
  //vg.setLeafSize (0.1f, 0.1f, 0.1f);
  //vg.setInputCloud (del_unval_cloud);
	//vg.filter (*del_unval_cloud);
  //vg.filter (*merged_cloud);
  //*merged_cloud = *del_unval_cloud;
  // std::cout << del_unval_cloud -> points.size() << '\n';
  // std::cout << del_unval_cloud -> width << '\n';
  // std::cout << del_unval_cloud -> height << '\n';
  // std::cout << del_unval_cloud -> is_dense << '\n';
  del_unval_cloud -> width = del_unval_cloud -> points.size();
  del_unval_cloud -> height = 1;
  del_unval_cloud -> is_dense = false;
}

void ScanMatching::odomove_cloud(void)
{
  float x,y,z;
  float move_x = -input_odom.pose.pose.position.y;
  float move_z = input_odom.pose.pose.position.x;
  float move_s = 2*asin(input_odom.pose.pose.orientation.z);

  // float move_x = input_odom.pose.pose.position.x;
  // float move_y = input_odom.pose.pose.position.y;
  // float move_s = 2*asin(input_odom.pose.pose.orientation.z);

  //std::cout << "move_x: " << move_x;
  //std::cout << ", move_z: " << move_z;
  //std::cout << ", move_s: " << move_s*180/M_PI << '\n';

  for(int i=0;i<del_unval_cloud->points.size();i++)
  {
    //std::cout << i << '\n';
    x = del_unval_cloud->points[i].x;// + move_x;
    //std::cout << "2" << '\n';
    z = del_unval_cloud->points[i].z;// + move_z;
    //std::cout << "3" << '\n';
    x = x*cos(move_s)-z*sin(move_s);
    //std::cout << "4" << '\n';
    z = x*sin(move_s)+z*cos(move_s);
    //std::cout << "5" << '\n';
    del_unval_cloud->points[i].x = x + move_x;
    del_unval_cloud->points[i].z = z + move_z;
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_matching");
	ScanMatching sm;

  ros::Rate rate(1);

  while(ros::ok()){
    sm.sd_queue.callOne(ros::WallDuration(1));
    sm.od_queue.callOne(ros::WallDuration(1));
    if(sm.input_c && sm.input_o)
    {
      //sm.tf_test();
      //sm.merge_cloud_test();
      sm.del_unknown_val();
      sm.remove_unrelialbe();
      sm.odomove_cloud();
      //sm.del_floor();
      sm.icp_test();
      sm.publish_mergedcloud();

    }
    else
    {
      std::cout << '\n' << "not input" << '\n';
    }
    sm.input_c = false;
    sm.input_o = false;
    rate.sleep();
  }
	return 0;
}
