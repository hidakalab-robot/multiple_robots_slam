#include <map_merging/eigen_value_matching.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "matching_eigenvalue_feature");

  EigenValueMatching evm;

  while(ros::ok())
  {
    evm.queueS.callOne(ros::WallDuration(1));
    evm.queueM.callOne(ros::WallDuration(1));

    if(evm.isInputS() && evm.isInputM())
    {
      evm.calcMatch();
      evm.missMatchDetection();
      //if(evm.isMatch())
      //{
        evm.emPublisher();
      //}
    }
    else
    {
      if(!evm.isInputS())
      {
        std::cout << "not inputS" << '\n';
      }
      if(!evm.isInputM())
      {
        std::cout << "not inputM" << '\n';
      }
    }
    evm.resetFlag();
  }

  return 0;
}
