#include <new_exploration_programs/feature_matching.h>

FeatureMatching::FeatureMatching()
:master_cloud_for_shift(new pcl::PointCloud<pcl::PointXYZRGB>),
orig_master_cloud_for_shift(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  ssc.setCallbackQueue(&sc_queue);
  smc.setCallbackQueue(&mc_queue);
 	sc_sub = ssc.subscribe("/pointcloud_segmentation/source_cloud",1,&FeatureMatching::input_sourcecloud,this);
  mc_sub = smc.subscribe("/pointcloud_segmentation/merged_cloud",1,&FeatureMatching::input_mastercloud,this);
  input_master = false;
  input_source = false;
  changed_master = false;
  matching_threshold = 0.17;
  shift_position = 5.0;
  matching = true;

  shift_master_cloud.header.frame_id = "camera_rgb_optical_frame";

  // sc_pub1 = psc.advertise<sensor_msgs::PointCloud2>("source_cloud/orig_cloud", 1);
 	// sc_pub2 = psc.advertise<sensor_msgs::PointCloud2>("source_cloud/vox_cloud", 1);
 	// sc_pub3 = psc.advertise<sensor_msgs::PointCloud2>("source_cloud/del_cloud", 1);
 	// sc_pub4 = psc.advertise<sensor_msgs::PointCloud2>("source_cloud/clu_cloud", 1);
  //
  // mc_pub1 = pmc.advertise<sensor_msgs::PointCloud2>("master_cloud/orig_cloud", 1);
  // mc_pub2 = pmc.advertise<sensor_msgs::PointCloud2>("master_cloud/vox_cloud", 1);
  // mc_pub3 = pmc.advertise<sensor_msgs::PointCloud2>("master_cloud/del_cloud", 1);
  // mc_pub4 = pmc.advertise<sensor_msgs::PointCloud2>("master_cloud/clu_cloud", 1);

  rsc_pub = prc.advertise<sensor_msgs::PointCloud2>("pointcloud_matching/source_cloud", 1);
  rmc_pub = prc.advertise<sensor_msgs::PointCloud2>("pointcloud_matching/merged_cloud", 1);
  rsm_pub = prc.advertise<visualization_msgs::MarkerArray>("pointcloud_matching/show_matching", 1);


  origs_pub = prc.advertise<sensor_msgs::PointCloud2>("pointcloud_matching/orig_source_cloud", 1);
  origm_pub = prc.advertise<sensor_msgs::PointCloud2>("pointcloud_matching/orig_merged_cloud", 1);

  mi_pub = prc.advertise<new_exploration_programs::matching_info>("pointcloud_matching/matching_info", 1);
};


void FeatureMatching::input_sourcecloud(const new_exploration_programs::segmented_cloud::ConstPtr& sc_msg)
{
  source_cloud = *sc_msg;
  input_source = true;
  std::cout << "input_sourcecloud" << '\n';

  // std::cout << "info in time << " << sc_msg -> header.stamp << '\n';
  // std::cout << "info in time2 << " << source_cloud.header.stamp << '\n';
}

void FeatureMatching::input_mastercloud(const new_exploration_programs::segmented_cloud::ConstPtr& mc_msg)
{
  master_cloud = *mc_msg;
  input_master = true;
  std::cout << "input_mergedcloud" << '\n';
}

bool FeatureMatching::master_is_empty(void)
{
  if(master_cloud.clu_indices.size() > 0)
  {
    std::cout << "merged is not empty" << '\n';
    return false;
  }
  else
  {
    std::cout << "merged is empty" << '\n';
    return true;
  }
}

void FeatureMatching::mastercloud_test(void)
{
  mc_pub1.publish(master_cloud.orig_cloud);
  mc_pub2.publish(master_cloud.vox_cloud);
  mc_pub3.publish(master_cloud.del_cloud);
  mc_pub4.publish(master_cloud.clu_cloud);
}

void FeatureMatching::sourcecloud_test(void)
{
  sc_pub1.publish(source_cloud.orig_cloud);
  sc_pub2.publish(source_cloud.vox_cloud);
  sc_pub3.publish(source_cloud.del_cloud);
  sc_pub4.publish(source_cloud.clu_cloud);
}

void FeatureMatching::matching_calc(void)
{
  std::cout << "matching-process_is_started" << '\n';
  //ソースクラウド
  float diff_linearity;
	float diff_planarity;
	float diff_scattering;
	float diff_omnivariance;
	float diff_anisotropy;
	float diff_eigenentropy;
	float diff_change_of_curvature;

  float diff_vector;

  /*重心確認用*/
  float scx,scy,scz,mcx,mcy,mcz;

  std::vector<Eigen::Vector3f> matching_list;
  Eigen::Vector3f matching_pair;//[0]master,[1]source

  //std::cout << "master_cloud.clu_indices.size() : " << master_cloud.clu_indices.size() << '\n';
  //std::cout << "source_cloud.clu_indices.size() : " << source_cloud.clu_indices.size() << '\n';

  //std::cout << "matching_success_list" << '\n';

  float min_diff;
  int min_num;

  float y_thre = 0.09;
  //float y_thre = 100.0;

  for(int i=0;i<master_cloud.clu_indices.size();i++)
  {
    min_diff = 100.0;
    min_num = -1;

    for(int j=0;j<source_cloud.clu_indices.size();j++)
    {
      diff_linearity = master_cloud.clu_features[i].linearity - source_cloud.clu_features[j].linearity;
      diff_planarity = master_cloud.clu_features[i].planarity - source_cloud.clu_features[j].planarity;
      diff_scattering = master_cloud.clu_features[i].scattering - source_cloud.clu_features[j].scattering;
      diff_omnivariance = master_cloud.clu_features[i].omnivariance - source_cloud.clu_features[j].omnivariance;
      diff_anisotropy = master_cloud.clu_features[i].anisotropy - source_cloud.clu_features[j].anisotropy;
      diff_eigenentropy = master_cloud.clu_features[i].eigenentropy - source_cloud.clu_features[j].eigenentropy;
      diff_change_of_curvature = master_cloud.clu_features[i].change_of_curvature - source_cloud.clu_features[j].change_of_curvature;

      diff_vector = sqrt(pow(diff_linearity,2)+pow(diff_planarity,2)+pow(diff_scattering,2)+pow(diff_omnivariance,2)+pow(diff_anisotropy,2)+pow(diff_eigenentropy,2)+pow(diff_change_of_curvature,2));



      if(diff_vector < matching_threshold)
      {

        if(std::abs(master_cloud.clu_centroids[i].y - source_cloud.clu_centroids[j].y) < y_thre)
        {
          if(diff_vector < min_diff)
          {


            mcx = master_cloud.clu_centroids[i].x;
            mcy = master_cloud.clu_centroids[i].y;
            mcz = master_cloud.clu_centroids[i].z;

            scx = source_cloud.clu_centroids[j].x;
            scy = source_cloud.clu_centroids[j].y;
            scz = source_cloud.clu_centroids[j].z;

            min_num = j;
            min_diff = diff_vector;
          }
        }
        // std::cout << "[matching!!] master_cloud[" << i << "] ("<< mcx << "," << mcy << "," << mcz << ") and source_cloud[" << j << "] (" << scx << "," << scy << "," << scz << ")" << '\n';
        //
        // matching_pair[0] = i;
        // matching_pair[1] = j;
        // matching_pair[2] = diff_vector;
        // matching_list.push_back(matching_pair);
      }
    }
    if(min_num > -1)
    {
      std::cout << "[matching!!] master_cloud[" << i << "] ("<< mcx << "," << mcy << "," << mcz << ") and source_cloud[" << min_num << "] (" << scx << "," << scy << "," << scz << ")" << '\n';


      matching_pair[0] = i;
      matching_pair[1] = min_num;
      matching_pair[2] = min_diff;
      matching_list.push_back(matching_pair);
    }

  }

  /*誤マッチングを削除*/

  //const int match_num = matching_list.size();

  //float min_diff;
  //int min_num;
  std::vector<Eigen::Vector2i> matching_list_true;
  Eigen::Vector2i matching_pair_true;

  //std::cout << "master size:" << master_cloud.clu_indices.size() << "source size:" << source_cloud.clu_indices.size() << '\n';

  for(int i=0;i<source_cloud.clu_indices.size();i++)
  {
    min_diff = 100.0;
    min_num = -1;

    for(int j=0;j<matching_list.size();j++)
    {
      if((int)matching_list[j](1) == i)
      {
        if(matching_list[j](2) < min_diff)
        {
          min_diff = matching_list[j](2);
          min_num = (int)matching_list[j](0);
        }
      }
    }
    if(min_num > -1)
    {
      matching_pair_true[0] = min_num;
      matching_pair_true[1] = i;
      matching_list_true.push_back(matching_pair_true);
    }

  }

  for(int i=0;i<matching_list_true.size();i++)
  {
    std::cout << "[true-matching!!] master_cloud[" << matching_list_true[i](0) << "]" <<  " and source_cloud[" << matching_list_true[i](1) << "]" << '\n';
    std::cout << "[centroid_y_diff] " << std::abs(master_cloud.clu_centroids[matching_list_true[i](0)].y - source_cloud.clu_centroids[matching_list_true[i](1)].y) << '\n';
  }


  if(matching_list_true.size()==0)
  {
    matching = false;
    std::cout << "no matching cloud" << '\n';
  }

  matching_list_m = matching_list_true;
  std::cout << '\n';

  //master_cloud = source_cloud;
}

void FeatureMatching::show_matching(void)
{

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr shift_master_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::fromROSMsg (master_cloud.clu_cloud, *master_cloud_for_shift);
  // for(int i=0;i<master_cloud_for_shift->points.size();i++)
  // {
  //   master_cloud_for_shift->points[i].y += shift_position;
  // }
  // pcl::toROSMsg (*master_cloud_for_shift, shift_master_cloud);


  visualization_msgs::Marker matching_line;
  visualization_msgs::MarkerArray matching_line_list;

  matching_line.header.frame_id = "camera_rgb_optical_frame";
  matching_line.header.stamp = ros::Time::now();
  matching_line.ns = "matching_line";
  matching_line.type = visualization_msgs::Marker::LINE_LIST;
  matching_line.action = visualization_msgs::Marker::ADD;
  matching_line.pose.orientation.w = 1.0;
  matching_line.scale.x = 0.1;
	matching_line.color.r = 1.0f;
	matching_line.color.g = 0.0f;
	matching_line.color.b = 0.0f;
	matching_line.color.a = 1.0;
	matching_line.lifetime = ros::Duration(0.5);

  geometry_msgs::Point s_centroid;
	geometry_msgs::Point m_centroid;

  //std::cout << "matching_list_m.size() : " << matching_list_m.size() << '\n';

  for(int i=0;i<matching_list_m.size();i++)
  {
    m_centroid.x = master_cloud.clu_centroids[matching_list_m[i](0)].x- shift_position;
    m_centroid.y = master_cloud.clu_centroids[matching_list_m[i](0)].y ;
    m_centroid.z = master_cloud.clu_centroids[matching_list_m[i](0)].z;

    s_centroid.x = source_cloud.clu_centroids[matching_list_m[i](1)].x;
    s_centroid.y = source_cloud.clu_centroids[matching_list_m[i](1)].y;
    s_centroid.z = source_cloud.clu_centroids[matching_list_m[i](1)].z;

    matching_line.id = i;
    matching_line.points.push_back(s_centroid);
    matching_line.points.push_back(m_centroid);
    matching_line_list.markers.push_back(matching_line);
  }

  matching_line_list_m.markers = matching_line_list.markers;

}

void FeatureMatching::publish_registeredcloud(void)
{
  rsc_pub.publish(source_cloud.clu_cloud);
  rmc_pub.publish(shift_master_cloud);
  //rsm_pub.publish(matching_line_list_m);
}

void FeatureMatching::publish_matchingline(void)
{
  rsm_pub.publish(matching_line_list_m);
}

void FeatureMatching::shift_mcloud(void)
{
  pcl::fromROSMsg (master_cloud.clu_cloud, *master_cloud_for_shift);
  std::cout << "100" << '\n';
  pcl::fromROSMsg (master_cloud.orig_cloud, *orig_master_cloud_for_shift);
  std::cout << "101" << '\n';

  for(int i=0;i<master_cloud_for_shift->points.size();i++)
  {
    master_cloud_for_shift->points[i].x -= shift_position;
  }

  for(int i=0;i<orig_master_cloud_for_shift->points.size();i++)
  {
    orig_master_cloud_for_shift->points[i].x -= shift_position;
  }

  pcl::toROSMsg (*master_cloud_for_shift, shift_master_cloud);
  std::cout << "102" << '\n';
  pcl::toROSMsg (*orig_master_cloud_for_shift, shift_orig_master_cloud);
  std::cout << "103" << '\n';
}

void FeatureMatching::publish_orig(void)
{
  //std::cout << "orig_cloud size << " << source_cloud.orig_cloud.height*source_cloud.orig_cloud.width << '\n';

  std::cout << "header info << " << '\n' << source_cloud.orig_cloud.header << '\n';

  origs_pub.publish(source_cloud.orig_cloud);
  origm_pub.publish(shift_orig_master_cloud);
}

void FeatureMatching::store(void)
{
  master_cloud = source_cloud;
}

void FeatureMatching::publish_matchinginfo(void)
{
  /*ソース、マージクラウドにマッチング情報を加えてパブリッシュ*/
  new_exploration_programs::matching_info info;
  new_exploration_programs::matching_pair pair;

  info.merged_cloud = master_cloud;
  info.source_cloud = source_cloud;


  //std::cout << "info out time << " << info.source_cloud.header.stamp << '\n';



  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr nantest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //
  // pcl::fromROSMsg (source_cloud.vox_cloud, *nantest_cloud);
  //
  // int nancount = 0;
  //
  // for(int i=0;i<nantest_cloud->points.size();i++)
  // {
  //   if(isnan(nantest_cloud->points[i].x) || isnan(nantest_cloud->points[i].y) || isnan(nantest_cloud->points[i].z))
  //   {
  //     nancount++;
  //   }
  // }
  // if(nancount > 0)
  // {
  //   std::cout << "find_nan << " << nancount << '\n';
  // }
  // else
  // {
  //   std::cout << "no_nan" << '\n';
  // }
  //
  //
  // for(int i=0;i<matching_list_m.size();i++)
  // {
  //   std::cout << "/* matching_pushback */" << '\n';
  //   pair.merged_num = matching_list_m[i](0);
  //   pair.source_num = matching_list_m[i](1);
  //   info.matching_list.push_back(pair);
  // }
  //
  //
  // std::cout << "second" << '\n';
  //
  // pcl::fromROSMsg (info.source_cloud.vox_cloud, *nantest_cloud);
  //
  // nancount = 0;
  //
  // for(int i=0;i<nantest_cloud->points.size();i++)
  // {
  //   if(isnan(nantest_cloud->points[i].x) || isnan(nantest_cloud->points[i].y) || isnan(nantest_cloud->points[i].z))
  //   {
  //     nancount++;
  //   }
  // }
  // if(nancount > 0)
  // {
  //   std::cout << "find_nan << " << nancount << '\n';
  // }
  // else
  // {
  //   std::cout << "no_nan" << '\n';
  // }


  for(int i=0;i<matching_list_m.size();i++)
  {
    std::cout << "/* matching_pushback */" << '\n';
    pair.merged_num = matching_list_m[i](0);
    pair.source_num = matching_list_m[i](1);
    info.matching_list.push_back(pair);
  }

  mi_pub.publish(info);



  std::cout << "publish_matchinginfo" << '\n';

}

void FeatureMatching::publish_matchinginfo_empty(void)
{
  /*ソース、マージクラウドにマッチング情報を加えてパブリッシュ*/
  new_exploration_programs::matching_info info;
  new_exploration_programs::matching_pair pair;

  info.merged_cloud = master_cloud;
  info.source_cloud = source_cloud;


  mi_pub.publish(info);

  std::cout << "publish_emptyinfo" << '\n';

}
