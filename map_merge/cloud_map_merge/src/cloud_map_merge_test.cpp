#include <cloud_map_merge/cloud_map_merge_test.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "cloud_map_merge_test");

  CloudMapMerge cmm;

  while(ros::ok()){
    cmm.robotRegistration();
    ros::spinOnce();
    cmm.mapMerging();
  }

  return 0;
}