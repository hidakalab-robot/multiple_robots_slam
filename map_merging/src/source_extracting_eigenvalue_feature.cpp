#include <map_merging/feature_extracting.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "source_extracting_eigenvalue_feature");

  FeatureExtracting fea;

  while(ros::ok())
  {
    fea.queueC.callOne(ros::WallDuration(1));

    if(fea.isInput())
    {
      fea.featureExtraction();
      fea.featurePublisher();
    }
    else
    {
      std::cout << "not input" << '\n';
    }
    fea.resetFlag();
  }

  return 0;
}
