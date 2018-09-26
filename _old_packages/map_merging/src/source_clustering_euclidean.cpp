#include <map_merging/euclidean_clustering.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "source_clustering_euclidean");

  EuclideanClustering ec(0);//nodeType 0:source, 1:merged

  while(ros::ok())
  {
    ec.queueC.callOne(ros::WallDuration(1));

    if(ec.isInput())
    {
      ec.deleteCeiling();
      ec.euclideanClustering();
      ec.coloring();
      ec.ListAndCentroid();
      ec.clusterPublisher();
    }
    else
    {
      std::cout << "not input" << '\n';
    }
    ec.resetFlag();
  }

  return 0;
}
