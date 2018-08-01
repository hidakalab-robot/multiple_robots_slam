#include <map_merging/feature_extracting.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "merged_extracting_eigenvalue_feature");

  FeatureExtracting fea(1);//nodeType 0:source, 1:merged

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
