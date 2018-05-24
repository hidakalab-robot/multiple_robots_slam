#include <new_exploration_programs/processing_pointcloud.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processing_merged_pointcloud");
	ProcessingPointCloud pp;
	while(ros::ok()){
		//std::cout << "0" << std::endl;
		pp.pc_queue2.callOne(ros::WallDuration(1));
		if(pp.input)
		{
			pp.apply_voxelgrid();
			pp.delete_ground();
			pp.euclidean_clustering();
			pp.publish_pointcloud();
			pp.feature_extraction();
			pp.publish_merged_segmented();
			pp.input = false;
		}
		else
		{
			std::cout << '\n' << "not input" << '\n';
		}
	}
	return 0;
}
