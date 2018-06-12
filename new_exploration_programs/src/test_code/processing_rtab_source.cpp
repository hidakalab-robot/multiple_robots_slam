#include <new_exploration_programs/processing_pointcloud.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processing_rtab_source");
	ProcessingPointCloud pp;

	while(ros::ok()){
		//std::cout << "0" << std::endl;
		pp.rtab_queue.callOne(ros::WallDuration(1));
    pp.rtab_queue2.callOne(ros::WallDuration(1));
		if(pp.input_m && pp.input_o)
		{
			pp.euclidean_clustering();
			pp.feature_extraction();
			pp.publish_localmap();

      /*マップとオドメトリをリセットするコマンドを入れる*/
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
