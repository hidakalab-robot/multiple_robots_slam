//shot特徴量のマッチングをテストする用のプログラムです
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>

#include <map_merging/Match.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class ShotMatching
{
private:
  ros::NodeHandle sE;
  ros::NodeHandle pS;
  ros::Subscriber subE;
  ros::Publisher pubShotMatch;

  map_merging::Match sMatch;

  bool input;
  bool matching;


  //shot parameters
  bool use_hough_;
  float model_ss_;
  float scene_ss_;
  float rf_rad_;
  float descr_rad_;
  float cg_size_;
  float cg_thresh_;

  pcl::PointCloud<PointType>::Ptr model;
  pcl::PointCloud<PointType>::Ptr model_keypoints;
  pcl::PointCloud<PointType>::Ptr scene;
  pcl::PointCloud<PointType>::Ptr scene_keypoints;
  pcl::PointCloud<NormalType>::Ptr model_normals;
  pcl::PointCloud<NormalType>::Ptr scene_normals;
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;

  pcl::PointCloud<PointType>::Ptr sCloud;
  pcl::PointCloud<PointType>::Ptr mCloud;

public:
  ros::CallbackQueue queueE;

  ShotMatching();
	~ShotMatching(){};

  void inputEigenMatch(const map_merging::Match::ConstPtr& sEMsg);
  bool isInput(void);
  void resetFlag(void);
  bool isMatch(void);
  void shotPublisher(void);
  void cluster2Scene(void);
  void cluster2LinkCluster(void);
  void cluster2AllCluster(void);
  void loadCluster(int clusterNum, bool isSource);
  void calcMatch(void);

};

ShotMatching::ShotMatching()
:model (new pcl::PointCloud<PointType>()),
model_keypoints (new pcl::PointCloud<PointType>()),
scene (new pcl::PointCloud<PointType>()),
scene_keypoints (new pcl::PointCloud<PointType>()),
model_normals (new pcl::PointCloud<NormalType>()),
scene_normals (new pcl::PointCloud<NormalType>()),
model_descriptors (new pcl::PointCloud<DescriptorType>()),
scene_descriptors (new pcl::PointCloud<DescriptorType>()),
sCloud (new pcl::PointCloud<PointType>()),
mCloud (new pcl::PointCloud<PointType>())
{
  sE.setCallbackQueue(&queueE);

  subE = sE.subscribe("/map_merging/eigenValueMatching",1,&ShotMatching::inputEigenMatch,this);

  pubShotMatch = pS.advertise<map_merging::Match>("map_merging/shotMatching", 1);

  input = false;
  matching = false;

  use_hough_ = true;
  model_ss_ = 0.01f;
  scene_ss_ = 0.03f;
  rf_rad_ = 0.015f;
  descr_rad_ = 0.02f;
  cg_size_ = 0.01f;
  cg_thresh_ = 5.0f;
}

void ShotMatching::inputEigenMatch(const map_merging::Match::ConstPtr& sEMsg)
{
  sMatch = *sEMsg;

  input = true;


  std::cout << "input EigenMatch" << '\n';
}

bool ShotMatching::isInput(void)
{
  return input;
}

void ShotMatching::resetFlag(void)
{
  input = false;
}

bool ShotMatching::isMatch(void)
{
  return matching;
}

void ShotMatching::shotPublisher(void)
{
  sMatch.header.stamp = ros::Time::now();

  /*マッチリストもここで入れる*/

  pubShotMatch.publish(sMatch);
  std::cout << "published" << '\n';
}

void ShotMatching::cluster2Scene(void)
{
  /*リストにあるクラスタごとにシーン全体とSHOTマッチング*/
  pcl::fromROSMsg (sMatch.mergedMap.cloudObstacles, *scene);
  pcl::fromROSMsg (sMatch.sourceMap.cloudObstacles, *sCloud);

  for(int i=0;i<sMatch.matchList.size();i++)
  {
    loadCluster(sMatch.matchList[i].sourceNumber,true);
    calcMatch();
    if(isMatch())
    {
      /*マッチリストの件*/
    }
  }

  if(/*編集したマッチリストのサイズが1以上*/true)
  {
    matching = true;
  }

}

void ShotMatching::cluster2LinkCluster(void)
{
  /*リストにあるクラスタごとに関連付けされたクラスタとSHOTマッチング*/

}

void ShotMatching::cluster2AllCluster(void)
{
  /*リストにあるクラスタごとにシーンにあるクラスタ全てとSHOTマッチング*/

}

void ShotMatching::loadCluster(int clusterNum, bool isSource)
{
  //クラスタの番号とどっちのクラウドkaを受け取ってその番号のクラスタをpcl形式で作成する

  pcl::PointCloud<PointType>::Ptr cluster (new pcl::PointCloud<PointType>());

  if(isSource)
  {
    pcl::fromROSMsg (sMatch.sourceMap.cloudObstacles, *sCloud);

    for(int i=0;i<sMatch.sourceMap.clusterList[clusterNum].index.size();i++)
    {
      //sourceCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]].x;
      //sourceCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]].y;
      //sourceCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]].z;

      cluster -> points.push_back(sCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]]);
    }
    *model = *cluster;
  }
  else
  {
    pcl::fromROSMsg (sMatch.mergedMap.cloudObstacles, *mCloud);

    for(int i=0;i<sMatch.mergedMap.clusterList[clusterNum].index.size();i++)
    {
      //sourceCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]].x;
      //sourceCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]].y;
      //sourceCloud->points[sMatch.sourceMap.clusterList[clusterNum].index[i]].z;

      cluster -> points.push_back(mCloud->points[sMatch.mergedMap.clusterList[clusterNum].index[i]]);
    }
    *scene = *cluster;
  }

}

void ShotMatching::calcMatch(void)
{
  //
  //  Compute Normals
  //

  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);


  //
  // Compute keypoints
  //

  pcl::VoxelGrid<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setLeafSize (model_ss_,model_ss_,model_ss_);
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setLeafSize (scene_ss_,scene_ss_,scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


  //
  //  Compute Descriptor for keypoints
  //

  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  //pcl::SHOTColorEstimationOMP//色情報も使ってそう
  descr_est.setRadiusSearch (descr_rad_);
  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);//要素数1で初期化
    std::vector<float> neigh_sqr_dists (1);//要素数1で初期化
    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    //平方和記述子距離が0.25未満の場合にのみ一致を追加する（SHOT記述子距離は設計上0と1の間である）
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  if(rototranslations.size() > 0)
  {
    matching = true;
  }
  else
  {
    matching = false;
  }



  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

}
