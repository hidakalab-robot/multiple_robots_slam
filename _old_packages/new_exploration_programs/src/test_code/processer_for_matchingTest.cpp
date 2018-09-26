#include <new_exploration_programs/processing_pointcloud.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processer_for_matchingTest");

	ProcessingPointCloud pp;

  std::string s_filename = "pcd_out/full_cloud/fullCloud1.pcd";
  std::string m_filename = "pcd_out/full_cloud/fullCloud0.pcd";

  while(ros::ok())
  {
    pp.include_master(m_filename);
  	pp.euclidean_clustering();
  	pp.feature_extraction();
    pp.publish_mastermap();

    pp.include_source(s_filename);
  	pp.euclidean_clustering();
  	pp.feature_extraction();
  	pp.publish_localmap();
  }


  //std::cout << "end" << '\n';

	return 0;
}
