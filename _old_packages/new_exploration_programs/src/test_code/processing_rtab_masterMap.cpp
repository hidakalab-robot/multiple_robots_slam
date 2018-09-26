#include <new_exploration_programs/processing_pointcloud.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processing_rtab_masterMap");
	ProcessingPointCloud pp;

	//bool okPub = false;

	while(ros::ok()){
		//std::cout << "0" << std::endl;
		pp.rtab_queuem.callOne(ros::WallDuration(1));

		if(pp.input)
		{
			pp.euclidean_clustering();
			pp.feature_extraction();
			pp.publish_mastermap();
		}
		else
		{
			std::cout << '\n' << "not input" << '\n';
		}
		// if(okPub)
		// {
		// 	pp.publish_localmap();
		// }

    pp.input = false;
	}
	return 0;
}
