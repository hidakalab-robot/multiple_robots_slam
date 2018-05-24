#include <new_exploration_programs/feature_matching.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "feature_matching");
	FeatureMatching fm;


	while(ros::ok())
  {
    fm.mc_queue.callOne(ros::WallDuration(1));
		fm.sc_queue.callOne(ros::WallDuration(1));

    if(fm.input_master && fm.input_source)
    {
      fm.matching_calc();
      fm.shift_mcloud();
      if(fm.matching)
      {
        fm.show_matching();
        fm.publish_matchingline();
        fm.publish_registeredcloud();
        fm.publish_orig();
      }
      else
      {
        fm.publish_registeredcloud();
        fm.publish_orig();
      }

			fm.publish_matchinginfo();

      fm.input_source = false;
		 	fm.input_master = false;
      fm.matching = true;
    }
    else
    {
      std::cout << "source_or_source_is_nothing" << '\n';
    }
  }

	return 0;
}
