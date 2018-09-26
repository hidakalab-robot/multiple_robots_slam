#include <new_exploration_programs/feature_matching.h>


class CentroidMatching : public FeatureMatching
{
private:
  //new_exploration_programs::segmented_cloud pre_source_cloud;

public:
  CentroidMatching(){};
  virtual ~CentroidMatching(){};

  //void matching_calc(void);
};


// void CentroidMatching::matching_calc(void)
// {
//   std::cout << "matching-process_is_started" << '\n';
//   //ソースクラウド
//   float diff_linearity;
// 	float diff_planarity;
// 	float diff_scattering;
// 	float diff_omnivariance;
// 	float diff_anisotropy;
// 	float diff_eigenentropy;
// 	float diff_change_of_curvature;
//
//   float diff_vector;
//
//   /*重心確認用*/
//   float scx,scy,scz,mcx,mcy,mcz;
//
//   std::vector<Eigen::Vector2i> matching_list;
//   Eigen::Vector2i matching_pair;//[0]master,[1]source
//
//   //std::cout << "master_cloud.clu_indices.size() : " << master_cloud.clu_indices.size() << '\n';
//   //std::cout << "source_cloud.clu_indices.size() : " << source_cloud.clu_indices.size() << '\n';
//
//   //std::cout << "matching_success_list" << '\n';
//
//   for(int i=0;i<master_cloud.clu_indices.size();i++)
//   {
//     for(int j=0;j<source_cloud.clu_indices.size();j++)
//     {
//       diff_linearity = master_cloud.clu_features[i].linearity - source_cloud.clu_features[j].linearity;
//       diff_planarity = master_cloud.clu_features[i].planarity - source_cloud.clu_features[j].planarity;
//       diff_scattering = master_cloud.clu_features[i].scattering - source_cloud.clu_features[j].scattering;
//       diff_omnivariance = master_cloud.clu_features[i].omnivariance - source_cloud.clu_features[j].omnivariance;
//       diff_anisotropy = master_cloud.clu_features[i].anisotropy - source_cloud.clu_features[j].anisotropy;
//       diff_eigenentropy = master_cloud.clu_features[i].eigenentropy - source_cloud.clu_features[j].eigenentropy;
//       diff_change_of_curvature = master_cloud.clu_features[i].change_of_curvature - source_cloud.clu_features[j].change_of_curvature;
//
//       diff_vector = sqrt(pow(diff_linearity,2)+pow(diff_planarity,2)+pow(diff_scattering,2)+pow(diff_omnivariance,2)+pow(diff_anisotropy,2)+pow(diff_eigenentropy,2)+pow(diff_change_of_curvature,2));
//
//
//
//       if(diff_vector < matching_threshold)
//       {
//         mcx = master_cloud.clu_centroids[i].x;
//         mcy = master_cloud.clu_centroids[i].y;
//         mcz = master_cloud.clu_centroids[i].z;
//
//         scx = source_cloud.clu_centroids[j].x;
//         scy = source_cloud.clu_centroids[j].y;
//         scz = source_cloud.clu_centroids[j].z;
//
//         std::cout << "[matching!!] master_cloud[" << i << "] ("<< mcx << "," << mcy << "," << mcz << ") and source_cloud[" << j << "] (" << scx << "," << scy << "," << scz << ")" << '\n';
//
//         matching_pair[0] = i;
//         matching_pair[1] = j;
//         matching_list.push_back(matching_pair);
//       }
//     }
//   }
//
//   if(matching_list_m.size()==0)
//   {
//     matching = false;
//     std::cout << "no matching cloud" << '\n';
//   }
//
//   matching_list_m = matching_list;
//   std::cout << '\n';
//
//   //master_cloud = source_cloud;
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "centroid_matching");
	CentroidMatching cm;

  while(ros::ok())
  {
    cm.mc_queue.callOne(ros::WallDuration(1));
    if(cm.input_master)
    {
      while(ros::ok())
      {
          cm.sc_queue.callOne(ros::WallDuration(1));
          if(cm.input_source)
          {
            //fm.mastercloud_test();
            //fm.sourcecloud_test();
            cm.matching_calc();
            cm.shift_mcloud();
            if(cm.matching)
            {
              cm.show_matching();
              cm.publish_matchingline();
              cm.publish_registeredcloud();
            }
            else
            {
              cm.publish_registeredcloud();
            }
            cm.input_source = false;
            cm.matching = true;
            if(cm.changed_master)
            {
                break;
            }
          }
          else
          {
            std::cout << "source_is_nothing" << '\n';
          }
      }
      cm.input_master = false;
    }
    else
    {
      std::cout << "master_is_nothing" << '\n';
    }
	}


  return 0;
}
