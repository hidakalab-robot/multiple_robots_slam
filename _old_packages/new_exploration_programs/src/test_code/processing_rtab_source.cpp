#include <new_exploration_programs/processing_pointcloud.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processing_rtab_source");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/rtabmap/reset");
	std_srvs::Empty srv;

	ProcessingPointCloud pp;

	while(ros::ok()){
		//std::cout << "0" << std::endl;
		pp.rtab_queue.callOne(ros::WallDuration(1));
    pp.rtab_queue2.callOne(ros::WallDuration(1));
		if(pp.input_m && pp.input_o)
		{
			if(pp.publishJudge())
			{
				pp.euclidean_clustering();
				pp.feature_extraction();
				pp.publish_localmap();

	      /*マップをリセットするコマンドを入れる*/
				if (client.call(srv))
	  		{
	    		std::cout << "map reset is success" << '\n';
	  		}
	  		else
	  		{
	    		std::cout << "map reset is failed" << '\n';
	  		}
			}

		}
		else
		{
			std::cout << '\n' << "not input" << '\n';
		}
    pp.input_m = false;
    pp.input_o = false;
	}
	return 0;
}
