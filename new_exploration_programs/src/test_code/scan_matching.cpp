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


  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud;

  nav_msgs::Odometry input_odom;
  nav_msgs::Odometry pre_input_odom;


  sensor_msgs::PointCloud2 merged_cloud_r;

  pcl::VoxelGrid<pcl::PointXYZ> vg;


  pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_input_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr clone_input_cloud;

  std::vector<int> index_list_m;


  float camera_position_y;
  float ceiling_position_y;
  float floor_position_y;
  float per_range_z;

  //float x;
  //float z;
  float ave_sqrt_distance;
  float min_cost;

  float dx;
  float dz;
  float dth;

  float min_dx;
  float min_dz;
  float min_dth;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;

  Eigen::Matrix3f R_estimate;
  Eigen::Vector3f T_estimate;

public:
  ros::CallbackQueue sd_queue;
  ros::CallbackQueue od_queue;
  bool input_c;
  bool input_o;

  ScanMatching()
  :input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  voxeled_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  clone_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  kdtree(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    ssd.setCallbackQueue(&sd_queue);
    sd_sub = ssd.subscribe("/camera/depth_registered/points",1,&ScanMatching::input_pointcloud,this);
    sod.setCallbackQueue(&od_queue);
    od_sub = sod.subscribe("/odom",1,&ScanMatching::input_odometry,this);
    input_c = false;
    input_o = false;
    mc_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/merged_cloud", 1);

    camera_position_y = 0.41;
    ceiling_position_y = 2.3;
    floor_position_y = 0.3;
    per_range_z = 6.0;

    R_estimate = Eigen::Matrix3f::Identity();
    T_estimate = Eigen::Vector3f::Zero();
  };
  ~ScanMatching(){};

  void input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
  void input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg);
  void voxeling(void);
  void remove_unrelialbe(void);
  void icp_main(void);
  void icp_alignment(const float dx_f, const float dz_f, const float dth_f);
  void icp_correspondence(void);
  void icp_optimizer(void);
  float icp_cost(const float dx_c, const float dz_c, const float dth_c);
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
  vg.filter (*voxeled_input_cloud);

  *input_cloud = *voxeled_input_cloud;
}

void ScanMatching::remove_unrelialbe(void)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for(int i=0;i<input_cloud->points.size();i++)
	{
		if(input_cloud->points[i].y > camera_position_y - ceiling_position_y && input_cloud->points[i].y < camera_position_y - floor_position_y) //&& input_cloud->points[i].z < per_range_z)
		{
      remove_cloud->points.push_back(input_cloud->points[i]);
		}
	}
  *input_cloud = *remove_cloud;

  input_cloud -> width = input_cloud -> points.size();
  input_cloud -> height = 1;
  input_cloud -> is_dense = false;
}


void ScanMatching::icp_main(void)
{
  if(merged_cloud->points.size() > 0)
  {
    const float fin_threshold = 10;


    dx = -input_odom.pose.pose.position.y + pre_input_odom.pose.pose.position.y;
    dz = input_odom.pose.pose.position.x - pre_input_odom.pose.pose.position.x;
    dth = 2*asin(input_odom.pose.pose.orientation.z*pre_input_odom.pose.pose.orientation.w - pre_input_odom.pose.pose.orientation.z*input_odom.pose.pose.orientation.w);

    if(input_odom.pose.pose.orientation.z*pre_input_odom.pose.pose.orientation.z < 0)
    {
      dth *= -1;
    }

    icp_alignment(dx,dz,dth);

    while(true)
    {
      icp_correspondence();//点の対応付け
      icp_optimizer();
      icp_alignment(min_dx,min_dz,min_dth);
      if(min_cost>fin_threshold)
      {
        break;
      }
    }
  }
  else
  {
    *clone_input_cloud = *input_cloud;
  }

  pre_input_odom = input_odom;
  *merged_cloud += *clone_input_cloud;
}

void ScanMatching::icp_alignment(const float dx_f, const float dz_f, const float dth_f)
{

  Eigen::Vector3f move_point_vector;
  Eigen::Vector3f point_vector;

  Eigen::Matrix3f L_R_estimate;
  Eigen::Vector3f L_T_estimate;

  Eigen::Matrix3f R0_estimate;
  Eigen::Vector3f T0_estimate;

  *clone_input_cloud = *input_cloud;

  R0_estimate << cos(-dth_f),0,sin(-dth_f),0,1,0,-sin(-dth_f),0,cos(-dth_f);
  T0_estimate << dx_f,0,dz_f;

  L_R_estimate = R0_estimate * R_estimate;
  L_T_estimate = T0_estimate + T_estimate;

  for (int i=0;i<clone_input_cloud->points.size();i++)
  {
    point_vector << clone_input_cloud->points[i].x,clone_input_cloud->points[i].y,clone_input_cloud->points[i].z;
    move_point_vector = L_R_estimate*point_vector + L_T_estimate;
    clone_input_cloud->points[i].x = move_point_vector(0);
    clone_input_cloud->points[i].y = move_point_vector(1);
    clone_input_cloud->points[i].z = move_point_vector(2);
  }

}

void ScanMatching::icp_correspondence(void)//２つの点群の点を対応付ける
{
  kdtree->setInputCloud (merged_cloud);
  kdtree->setEpsilon(0.10);

  pcl::PointXYZ searchPoint;
  std::vector<int> v_nearest_point_index(1);
  int nearest_point_index;
  std::vector<int> index_list;//ここに点の対応関係が入ってる//メンバーにする

  std::vector<float> v_point_distance(1);
  float point_distance;
  std::vector<float> distance_list;//これは初回の距離を計算しているだけ


  /*最近傍点を検索して対応付け*/
  for(int i=0;i<clone_input_cloud->points.size();i++)
  {
    searchPoint.x = clone_input_cloud->points[i].x;
    searchPoint.y = clone_input_cloud->points[i].y;
    searchPoint.z = clone_input_cloud->points[i].z;

    if ( kdtree->nearestKSearch (searchPoint, 1, v_nearest_point_index, v_point_distance) > 0 )
    {
      nearest_point_index = v_nearest_point_index[0];
      point_distance = v_point_distance[0];
      index_list.push_back(nearest_point_index);
      distance_list.push_back(point_distance);
    }
    else
    {
        index_list.push_back(-1);
        distance_list.push_back(-1);
    }
  }

  /*初回のコストを計算*/

  int count = 0;
  /*対応点間の距離を計算*/
  for(int i=0;i<distance_list.size();i++)
  {
    if(distance_list[i] >= 0)
    {
      ave_sqrt_distance += distance_list[i];
      count++;
    }
  }

  index_list_m = index_list;

  min_cost = ave_sqrt_distance / (float)count;

}

void ScanMatching::icp_optimizer(void)
{
  float cost;

  while(true)//
  {
    /*dx,dz,dthを変化させる*/
    dx *= 1;
    dz *= 1;
    dth *= 1;

    cost = icp_cost(dx,dz,dth);

    if(cost < min_cost)
    {
      //costの変化量を計算してもいいかも
      min_cost = cost;
      min_dx = dx;
      min_dz = dz;
      min_dth = dth;
    }

    if(true/*min_costが変化しなくなったら*/)
    {
      break;
    }
  }
}


float ScanMatching::icp_cost(const float dx_c, const float dz_c, const float dth_c)
{
  /*とりあえず位置を変える*/
  icp_alignment(dx_c,dz_c,dth_c);

  /*コストを計算*/

  float diff_x;
  float diff_y;
  float diff_z;
  int count = 0;
  /*対応点間の距離を計算*/
  for(int i=0;i<index_list_m.size();i++)
  {
    if(index_list_m[i] >= 0)
    {
      diff_x = clone_input_cloud->points[i].x - merged_cloud->points[index_list_m[i]].x;
      diff_y = clone_input_cloud->points[i].y - merged_cloud->points[index_list_m[i]].y;
      diff_x = clone_input_cloud->points[i].z - merged_cloud->points[index_list_m[i]].z;
      ave_sqrt_distance += pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2);
      count++;
    }
  }

  return ave_sqrt_distance / (float)count;
}


void ScanMatching::publish_mergedcloud(void)
{
  //*merged_cloud = *voxeled_input_cloud;
  pcl::toROSMsg (*merged_cloud, merged_cloud_r);
  merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
  //merged_cloud_r.header.frame_id = "odom";
  mc_pub.publish(merged_cloud_r);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_matching");
	ScanMatching sm;

  ros::Rate rate(2);

  while(ros::ok()){
    sm.sd_queue.callOne(ros::WallDuration(1));
    sm.od_queue.callOne(ros::WallDuration(1));
    if(sm.input_c && sm.input_o)
    {
      sm.voxeling();
      sm.remove_unrelialbe();
      sm.icp_main();

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
