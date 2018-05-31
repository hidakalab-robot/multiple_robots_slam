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

  new_exploration_programs::matching_info info;

  sensor_msgs::PointCloud2 centroid_merged_cloud_r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_merged_cloud;

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

  CentroidMatching()
  :centroid_merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
  //match_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  match_source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
  indi_merged_cloud_m(new pcl::PointCloud<pcl::PointXYZRGB>),
  indi_source_cloud_m(new pcl::PointCloud<pcl::PointXYZRGB>),
  icpout_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
  for_merge_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
  for_icpmerged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    smi.setCallbackQueue(&mi_queue);
    smi_sub = smi.subscribe("/pointcloud_matching/matching_info",1,&CentroidMatching::input_matchinginfo,this);
    pmc_pub = pmi.advertise<sensor_msgs::PointCloud2>("centroid_matching/merged_cloud", 1);
    input_info = false;
    one_matching = false;
    no_matching = false;
    //icp_matrix = Eigen::Matrix4f::Identity ();
    icp_rot_matrix = Eigen::Matrix3f::Identity ();
    icp_tra_vector = Eigen::Vector3f::Zero ();
    offset = Eigen::Vector3f::Zero ();
    rot = Eigen::Matrix3f::Identity ();
    trans = Eigen::Vector3f::Zero ();
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
  void icp_estimate(int merge_num, int source_num);
  void independ_matchingcloud(int merged_num, int source_num);
  void final_transform(void);
};


void CentroidMatching::input_matchinginfo(const new_exploration_programs::matching_info::ConstPtr& mi_msg)
{
  info = *mi_msg;
  input_info = true;

  //pcl::fromROSMsg (info.source_cloud.vox_cloud, *match_source_cloud);
  pcl::fromROSMsg (info.source_cloud.clu_cloud, *match_source_cloud);
  pcl::fromROSMsg (info.source_cloud.vox_cloud, *for_merge_cloud);
  std::cout << "200" << '\n';
  pcl::fromROSMsg (info.merged_cloud.orig_cloud, *centroid_merged_cloud);
  pcl::fromROSMsg (info.merged_cloud.clu_cloud, *for_icpmerged_cloud);
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
    /*これ違うのでは*/

    // mc_vector[0] = info.merged_cloud.clu_centroids[i+1].x - info.merged_cloud.clu_centroids[0].x;
    // mc_vector[1] = info.merged_cloud.clu_centroids[i+1].y - info.merged_cloud.clu_centroids[0].y;
    // mc_vector[2] = info.merged_cloud.clu_centroids[i+1].z - info.merged_cloud.clu_centroids[0].z;
    // v_m.push_back(mc_vector);
    //
    // sc_vector[0] = info.source_cloud.clu_centroids[i+1].x - info.source_cloud.clu_centroids[0].x;
    // sc_vector[1] = info.source_cloud.clu_centroids[i+1].y - info.source_cloud.clu_centroids[0].y;
    // sc_vector[2] = info.source_cloud.clu_centroids[i+1].z - info.source_cloud.clu_centroids[0].z;
    // v_s.push_back(sc_vector);

    /*多分これであってます*/

    //info. matching_list[0].merged_num;//これマージドの0番目の番号
    // info.matching_list[0].source_num;//これソースの0番目の番号
    //
    // info.matching_list[i].merged_num;//比較その他のマージド番号
    // info.matching_list[i].source_num;//比較その他のソース番号

    mc_vector[0] = info.merged_cloud.clu_centroids[info.matching_list[i+1].merged_num].x - info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].x;
    mc_vector[1] = info.merged_cloud.clu_centroids[info.matching_list[i+1].merged_num].y - info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].y;
    mc_vector[2] = info.merged_cloud.clu_centroids[info.matching_list[i+1].merged_num].z - info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].z;
    v_m.push_back(mc_vector);

    sc_vector[0] = info.source_cloud.clu_centroids[info.matching_list[i+1].source_num].x - info.source_cloud.clu_centroids[info.matching_list[0].source_num].x;
    sc_vector[1] = info.source_cloud.clu_centroids[info.matching_list[i+1].source_num].y - info.source_cloud.clu_centroids[info.matching_list[0].source_num].y;
    sc_vector[2] = info.source_cloud.clu_centroids[info.matching_list[i+1].source_num].z - info.source_cloud.clu_centroids[info.matching_list[0].source_num].z;
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
  std::vector<float> angle;
  float th;

  if(angle_m.size() > 0)
  {
    th = angle_m[0];
  }
  else
  {
    th = 0;
  }

  angle.push_back(th);

  angle_m = angle;

  std::cout << "fin << if_onematching" << '\n';

}


void CentroidMatching::moving_cloud(void)
{
  // Eigen::Vector3f trans;


  /*重心の移動を算出(x,zのみ)*//*これ違うかもしれない*/
  // trans[0] = info.source_cloud.clu_centroids[0].x - info.merged_cloud.clu_centroids[0].x;
  // trans[1] = 0.0;
  // trans[2] = info.source_cloud.clu_centroids[0].z - info.merged_cloud.clu_centroids[0].z;

  //info. matching_list[0].merged_num;//これマージドの0番目の番号
  // info.matching_list[0].source_num;//これソースの0番目の番号
  //
  // info.matching_list[i].merged_num;//比較その他のマージド番号
  // info.matching_list[i].source_num;//比較その他のソース番号

  trans[0] = info.source_cloud.clu_centroids[info.matching_list[0].source_num].x - info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].x;
  trans[1] = 0.0;
  trans[2] = info.source_cloud.clu_centroids[info.matching_list[0].source_num].z - info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].z;


  /*動かす奴*/

  /*並進 trans*/
  /*回転 angle_m*/

  float rad;

  for(int i=0;i<angle_m.size();i++)
  {
    rad += angle_m[i];
  }

  rad /= (float)angle_m.size();//これっぽい//解決しました



  Eigen::Vector3f point;
  Eigen::Vector3f a_point;
  //Eigen::Matrix3f rot;

  rot << cos(-rad),0,sin(-rad),0,1,0,-sin(-rad),0,cos(-rad);

  /*回転するまえに回転中心が原点になるように移動しないと*/
  //Eigen::Vector3f offset;
  //offset << info.merged_cloud.clu_centroids[0].x,0,info.merged_cloud.clu_centroids[0].z;
  offset << info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].x,info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].y,info.merged_cloud.clu_centroids[info.matching_list[0].merged_num].z;

  for(int i=0;i<match_source_cloud->points.size();i++)
  {
    point << match_source_cloud->points[i].x,match_source_cloud->points[i].y,match_source_cloud->points[i].z;
    //a_point = rot * point + trans;
    // a_point = (rot * (point - offset - trans)) + offset;
    a_point = (rot * (point - offset - trans));//icpやる前提の重心と大体の回転だけ合わせるやつ
    match_source_cloud->points[i].x = a_point(0);
    match_source_cloud->points[i].y = a_point(1);
    match_source_cloud->points[i].z = a_point(2);
  }

  std::cout << "fin << moving_cloud" << '\n';

  icp_estimate(info.matching_list[0].merged_num,info.matching_list[0].source_num);//icpで微調整
  final_transform();

}


void CentroidMatching::icp_estimate(int merged_num, int source_num)
{
  /*ここは重心を原点にして２つの点群を合わせたあとに回転を推定する関数です*/

  /*最初にicpを行うクラスタを独立した点群にする*/
  //indi_merged_cloud; ここに格納します
  //indi_source_cloud; ここに格納します

  /*icpで変化させてもひとつのクラスタについてのみなので全体の合わせは別でやる*/

  /*ソースクラスタが1つだった場合とそれ以外の場合で処理を分ける*/

  /*クラスタが一つでもクラスタに属してない点があるので抽出が必要でした*/

  /*ソースの方は事前に移動しているので抽出だけです*/

  std::cout << "extract cluster for ICP" << '\n';
  independ_matchingcloud(merged_num,source_num);


  std::cout << "icp_setup" << '\n';
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  // Set the input source and target
  //icp.setInputSource (del_unval_cloud);
  icp.setInputSource (indi_source_cloud_m);
  icp.setInputTarget (indi_merged_cloud_m);
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);


  std::cout << "icp_start" << '\n';
  icp.align (*icpout_cloud);

  Eigen::Matrix4f icp_matrix = Eigen::Matrix4f::Identity ();
  icp_matrix = icp.getFinalTransformation ().cast<float>();

  icp_rot_matrix << icp_matrix(0,0),icp_matrix(0.1),icp_matrix(0.2),icp_matrix(1,0),icp_matrix(1,1),icp_matrix(1,2),icp_matrix(2,0),icp_matrix(2,1),icp_matrix(2,2);
  icp_tra_vector << icp_matrix(0,3),icp_matrix(1,3),icp_matrix(2,3);
      /*
      R
      matrix (0, 0), matrix (0, 1), matrix (0, 2);
      matrix (1, 0), matrix (1, 1), matrix (1, 2)
      matrix (2, 0), matrix (2, 1), matrix (2, 2);

      T
      matrix (0, 3), matrix (1, 3), matrix (2, 3)
      */
}

void CentroidMatching::independ_matchingcloud(int merged_num, int source_num)
{
  /*引数　抽出対象のマージクラスタ番号, ソースクラスタ番号, マージドで抽出する必要があるか(false=重心移動だけ), ソースで抽出する必要があるか*/
  /*ここはicpのためにマッチングしたクラスタを独立した点群にする関数です*/
  /*またマージ側の重心が点群の原点になるようにします*/

  //それぞれ以下に格納します
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr indi_merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr indi_source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  /*ここにインデックスが入ってます*/
  // info.source_cloud.clu_indices[source_num].index;
  // info.merged_cloud.clu_indices[merged_num].index;

  /*これがクラスタリングした点群です*/
  //for_icpmerged_cloud
  //match_source_cloud



  /*マージド抽出*/
  for(int i=0;i<info.merged_cloud.clu_indices[merged_num].index.size();i++)
  {
    indi_merged_cloud -> points.push_back(for_icpmerged_cloud -> points[info.merged_cloud.clu_indices[merged_num].index[i]]);
    indi_merged_cloud -> points[i].x -= offset[0];
    indi_merged_cloud -> points[i].y -= offset[1];
    indi_merged_cloud -> points[i].z -= offset[2];
  }

  /*ソース抽出*/
  for(int i=0;i<info.source_cloud.clu_indices[source_num].index.size();i++)
  {
    indi_source_cloud -> points.push_back(match_source_cloud -> points[info.source_cloud.clu_indices[source_num].index[i]]);
  }

  /*抽出したあとに点群の設定*/
  indi_merged_cloud -> width = indi_merged_cloud -> points.size();
  indi_merged_cloud -> height = 1;
  indi_merged_cloud -> is_dense = false;

  indi_source_cloud -> width = indi_source_cloud -> points.size();
  indi_source_cloud -> height = 1;
  indi_source_cloud -> is_dense = false;


  indi_merged_cloud_m = indi_merged_cloud;
  indi_source_cloud_m = indi_source_cloud;

}

void CentroidMatching::final_transform(void)
{
  /*ここはicpで推定した回転等を全体に適用する関数です*/
  //回転 icp_rot_matrix
  //並行 icp_tra_vector

  //match_source_cloudを変換
  Eigen::Vector3f point;
  Eigen::Vector3f a_point;
  Eigen::Vector3f a2_point;

  //for_merge_cloud


  for(int i=0;i<for_merge_cloud->points.size();i++)
  {
    point << for_merge_cloud->points[i].x,for_merge_cloud->points[i].y,for_merge_cloud->points[i].z;

    a_point = (rot * (point - offset - trans));
    a2_point = (icp_rot_matrix * a_point) + icp_tra_vector + offset;
    for_merge_cloud->points[i].x = a2_point(0);
    for_merge_cloud->points[i].y = a2_point(1);
    for_merge_cloud->points[i].z = a2_point(2);
  }

  // for(int i=0;i<match_source_cloud->points.size();i++)
  // {
  //   point << match_source_cloud->points[i].x,match_source_cloud->points[i].y,match_source_cloud->points[i].z;
  //   //a_point = rot * point + trans;
  //   // a_point = (rot * (point - offset - trans)) + offset;
  //   a_point =
  //   aa_point = (icp_rot_matrix * apoint) + icp_tra_vector + offset;
  //   match_source_cloud->points[i].x = a_point(0);
  //   match_source_cloud->points[i].y = a_point(1);
  //   match_source_cloud->points[i].z = a_point(2);
  // }

}

void CentroidMatching::merging_cloud(void)
{
  //*centroid_merged_cloud += *match_source_cloud;
  *centroid_merged_cloud += *for_merge_cloud;


  std::cout << "fin << merging_cloud" << '\n';
}

bool CentroidMatching::is_merged_empty(void)
{

  /*マッチングリストではなくマージドが空だという条件が必要*/
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

  centroid_merged_cloud_r.header = info.source_cloud.header;

  // std::cout << "before time << " << info.header.stamp << '\n';
  // std::cout << "after time << " << centroid_merged_cloud_r.header.stamp << '\n';

  //centroid_merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
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
        /*マッチングリストが空でもマージドが空だとは限らないのでは？？*/
        //cm.nan_check();
        cm.merging_cloud();
        //cm.nan_check();
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
          //cm.nan_check();
          cm.merging_cloud();//点群を合成する
          //cm.nan_check();
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
