#include <ros/ros.h>
#include <map_merge/cloud_map_set.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "cloud_map_set");

  CloudMapSet mm;

  while(ros::ok())
  {
    mm.queue1.callOne(ros::WallDuration(1));
    mm.queue2.callOne(ros::WallDuration(1));

    if(mm.isInput1() && mm.isInput2())
    {
      mm.dataPublish();
    }
    else
    {
      if(!mm.isInput1())
      {
        std::cout << "not input robot1" << '\n';
      }
      if(!mm.isInput2())
      {
        std::cout << "not input robot2" << '\n';
      }
    }
    mm.resetFlag();
  }
  return 0;
}
