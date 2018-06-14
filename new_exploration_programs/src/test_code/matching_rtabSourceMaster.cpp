#include <new_exploration_programs/feature_matching.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "matching_rtabSourceMaster");
	FeatureMatching fm;

	/*一回目はソースだけで行けるようにする*/

	while(ros::ok())
  {
		fm.sc_queue.callOne(ros::WallDuration(1));

    if(fm.input_source)
    {
			fm.publish_matchinginfo_empty();

      fm.input_source = false;

			break;
    }
    else
    {
      std::cout << "source_is_nothing" << '\n';
    }
  }

	/*マスターを受け取ったあと、ソースの方ができるまで待つ*/

	while(ros::ok())
  {
    fm.mc_queue.callOne(ros::WallDuration(1));
		fm.sc_queue.callOne(ros::WallDuration(1));

    if(fm.input_master)
    {
			std::cout << "700" << '\n';
			while(!fm.input_source && ros::ok())
			{
				fm.sc_queue.callOne(ros::WallDuration(1));
				std::cout << "701" << '\n';
			}

			std::cout << "702" << '\n';
			if(fm.master_is_empty())
			{
				fm.shift_mcloud();
				fm.publish_registeredcloud();
				std::cout << "5" << '\n';
				fm.publish_orig();
				fm.publish_matchinginfo_empty();
			}
			else
			{
				fm.matching_calc();
				std::cout << "1" << '\n';
	      fm.shift_mcloud();
				std::cout << "2" << '\n';
	      if(fm.matching)
	      {
	        fm.show_matching();
					std::cout << "3" << '\n';
	        fm.publish_matchingline();
					std::cout << "4" << '\n';
	        fm.publish_registeredcloud();
					std::cout << "5" << '\n';
	        fm.publish_orig();
					std::cout << "6" << '\n';
	      }
	      else
	      {
	        fm.publish_registeredcloud();
	        fm.publish_orig();
	      }

				fm.publish_matchinginfo();
				std::cout << "7" << '\n';
			}
			std::cout << "703" << '\n';
      fm.input_source = false;
		 	fm.input_master = false;
      fm.matching = true;
    }
    else
    {
      std::cout << "merged_or_source_is_nothing" << '\n';
    }
  }

	return 0;
}
