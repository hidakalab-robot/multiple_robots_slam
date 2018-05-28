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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr match_source_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr match_merged_cloud;

  std::vector<float> angle_m;


public:
  ros::CallbackQueue mi_queue;//マッチング情報を受けとる
  bool input_info;
  bool one_matching;
  bool no_matching;

  CentroidMatching()
  :centroid_merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
  //match_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  match_source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    smi.setCallbackQueue(&mi_queue);
    smi_sub = smi.subscribe("/pointcloud_matching/matching_info",1,&CentroidMatching::input_matchinginfo,this);
    pmc_pub = pmi.advertise<sensor_msgs::PointCloud2>("centroid_matching/merged_cloud", 1);
    input_info = false;
    one_matching = false;
    no_matching = false;
  };
  ~CentroidMatching(){};

  void input_matchinginfo(const new_exploration_programs::matching_info::ConstPtr& mi_msg);
  void publish_mergedcloud(void);
  void centroid_vector(void);
  void moving_cloud(void);
  void merging_cloud(void);
  void if_onematching(void);
  //void if_nomatching(void);
  bool is_merged_empty(void);
  void nan_check(void);
};


void CentroidMatching::input_matchinginfo(const new_exploration_programs::matching_info::ConstPtr& mi_msg)
{
  info = *mi_msg;
  input_info = true;

  pcl::fromROSMsg (info.source_cloud.vox_cloud, *match_source_cloud);
  std::cout << "200" << '\n';
  pcl::fromROSMsg (info.merged_cloud.orig_cloud, *centroid_merged_cloud);
  std::cout << "201" << '\n';
  //pcl::fromROSMsg (info.merged_cloud.vox_cloud,, *match_merged_cloud);

  std::cout << "input_matchinginfo" << '\n';

  std::cout << "info.matching_list.size() << " << info.matching_list.size() << '\n';

  if(info.matching_list.size() == 1)
  {
    one_matching = true;
  }
  else if(info.matching_list.size() == 0)
  {
    no_matching = true;
  }
}

void CentroidMatching::centroid_vector(void)
{
  std::vector<Eigen::Vector3f> v_m;
  std::vector<Eigen::Vector3f> v_s;

  Eigen::Vector3f mc_vector;
  Eigen::Vector3f sc_vector;

  std::vector<float> angle;
  float th;



  /*マッチングした個数-1個のベクトルを計算*/
  for(int i=0;i<info.matching_list.size()-1;i++)
  {
    mc_vector[0] = info.merged_cloud.clu_centroids[i+1].x - info.merged_cloud.clu_centroids[0].x;
    mc_vector[1] = info.merged_cloud.clu_centroids[i+1].y - info.merged_cloud.clu_centroids[0].y;
    mc_vector[2] = info.merged_cloud.clu_centroids[i+1].z - info.merged_cloud.clu_centroids[0].z;
    v_m.push_back(mc_vector);

    sc_vector[0] = info.source_cloud.clu_centroids[i+1].x - info.source_cloud.clu_centroids[0].x;
    sc_vector[1] = info.source_cloud.clu_centroids[i+1].y - info.source_cloud.clu_centroids[0].y;
    sc_vector[2] = info.source_cloud.clu_centroids[i+1].z - info.source_cloud.clu_centroids[0].z;
    v_s.push_back(sc_vector);



    /*回転の向きの正負を決める必要がある*/
    th = acos(mc_vector.dot(sc_vector)/(mc_vector.norm()*sc_vector.norm()));

    if(sc_vector[0] < mc_vector[0])
    {
      th *= -1;
    }

    angle.push_back(th);
  }

  angle_m = angle;

  std::cout << "fin << centroid_vector" << '\n';
}

void CentroidMatching::if_onematching(void)
{

  /*ここは重心がひとつのときの回転を推定するための関数です*/


  std::cout << "fin << if_onematching" << '\n';

}


void CentroidMatching::moving_cloud(void)
{
  Eigen::Vector3f trans;


  /*重心の移動を算出(x,zのみ)*/
  trans[0] = info.source_cloud.clu_centroids[0].x - info.merged_cloud.clu_centroids[0].x;
  trans[1] = 0.0;
  trans[2] = info.source_cloud.clu_centroids[0].z - info.merged_cloud.clu_centroids[0].z;

  /*動かす奴*/

  /*並進 trans*/
  /*回転 angle_m*/

  float rad;

  for(int i=0;i<angle_m.size();i++)
  {
    rad += angle_m[i];
  }

  rad /= (float)angle_m.size();



  Eigen::Vector3f point;
  Eigen::Vector3f a_point;
  Eigen::Matrix3f rot;


  rot << cos(-rad),0,sin(-rad),0,1,0,-sin(-rad),0,cos(-rad);


  for(int i=0;i<match_source_cloud->points.size();i++)
  {
    point << match_source_cloud->points[i].x,match_source_cloud->points[i].y,match_source_cloud->points[i].z;
    //a_point = rot * point + trans;
    a_point = rot * (point - trans);
    match_source_cloud->points[i].x = a_point(0);
    match_source_cloud->points[i].y = a_point(1);
    match_source_cloud->points[i].z = a_point(2);
  }

  std::cout << "fin << moving_cloud" << '\n';

}

void CentroidMatching::merging_cloud(void)
{
  *centroid_merged_cloud += *match_source_cloud;


  std::cout << "fin << merging_cloud" << '\n';
}

bool CentroidMatching::is_merged_empty(void)
{
  if(info.matching_list.size() > 0)
  {
    //std::cout << "matching_list is empty" << '\n';
    return false;
  }
  else
  {
    std::cout << "matching_list is empty" << '\n';
    return true;
  }
}

void CentroidMatching::publish_mergedcloud(void)
{
  centroid_merged_cloud -> width = centroid_merged_cloud -> points.size();
  centroid_merged_cloud -> height = 1;
  centroid_merged_cloud -> is_dense = false;

  //std::cout << "isdence_b" << (int)centroid_merged_cloud -> is_dense << '\n';

  pcl::toROSMsg (*centroid_merged_cloud, centroid_merged_cloud_r);

  centroid_merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
  //std::cout << "isdence_a" << (int)centroid_merged_cloud_r.is_dense << '\n';
  //centroid_merged_cloud_r.is_dense = false;

  //std::cout << "isdence_aa" << (int)centroid_merged_cloud_r.is_dense << '\n';


  pmc_pub.publish(centroid_merged_cloud_r);
}

void CentroidMatching::nan_check(void)
{
  int s_nancount = 0;
  int m_nancount = 0;

  //centroid_merged_cloud += *match_source_cloud;

  for(int i=0;i<centroid_merged_cloud->points.size();i++)
  {
    if(isnan(centroid_merged_cloud->points[i].x) || isnan(centroid_merged_cloud->points[i].y) || isnan(centroid_merged_cloud->points[i].z))
    {
      s_nancount++;
      //std::cout << "find_nan " << nancount << '\n';
    }
  }
  if(s_nancount > 0)
  {
    std::cout << "find_m_nan << " << s_nancount << '\n';
  }
  else
  {
    std::cout << "no_m_nan" << '\n';
  }


  for(int i=0;i<match_source_cloud->points.size();i++)
  {
    if(isnan(match_source_cloud->points[i].x) || isnan(match_source_cloud->points[i].y) || isnan(match_source_cloud->points[i].z))
    {
      m_nancount++;
    }
  }
  if(m_nancount > 0)
  {
    std::cout << "find_s_nan << " << m_nancount << '\n';
  }
  else
  {
    std::cout << "no_s_nan" << '\n';
  }

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
      if(cm.is_merged_empty())
      {
        cm.nan_check();
        cm.merging_cloud();
        cm.nan_check();
        cm.publish_mergedcloud();
      }
      else
      {
        if(cm.one_matching)
        {
          cm.if_onematching();//回転を推定
        }
        else if(!cm.no_matching)
        {
          /*マッチングがひとつの時ようのやつも作る*/
          cm.centroid_vector();//回転を推定
        }

        if(!cm.no_matching)
        {
          cm.moving_cloud();//実際に点群に操作を行う
          cm.nan_check();
          cm.merging_cloud();//点群を合成する
          cm.nan_check();
          cm.publish_mergedcloud();//合成した点群を出力
        }
      }

    }
    else
    {
      std::cout << "not_input_matchinginfo" << '\n';
    }
    cm.input_info =false;
    cm.one_matching = false;
    cm.no_matching = false;
  }


  return 0;
}
