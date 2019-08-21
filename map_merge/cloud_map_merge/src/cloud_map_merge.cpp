#include <cloud_map_merge/cloud_map_merge.hpp>

int main (int argc, char** argv){
  ros::init(argc, argv, "cloud_map_merge");
  CloudMapMerge cmm;
  cmm.multiThreadMainLoop();
  return 0;
}
