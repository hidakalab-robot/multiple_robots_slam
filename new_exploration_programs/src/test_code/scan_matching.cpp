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
  tfout_cloud(new pcl::PointCloud<pcl::PointXYZ>)
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

void ScanMatching::publish_mergedcloud(void)
{
  pcl::toROSMsg (*merged_cloud, merged_cloud_r);
  merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
  mc_pub.publish(merged_cloud_r);
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
      sm.merge_cloud_test();
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
