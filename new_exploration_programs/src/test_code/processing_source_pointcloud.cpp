#include <new_exploration_programs/processing_pointcloud.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processing_source_pointcloud");
	ProcessingPointCloud pp;
	while(ros::ok()){
		//std::cout << "0" << std::endl;
		pp.pc_queue.callOne(ros::WallDuration(1));
		if(pp.input)
		{
			pp.apply_voxelgrid();
			pp.delete_ground();
			pp.naninf();
			pp.euclidean_clustering();
			//pp.publish_pointcloud();
			pp.feature_extraction();
			pp.publish_source_segmented();
			pp.naninf();
			pp.input = false;
		}
		else
		{
			std::cout << '\n' << "not input" << '\n';
		}
	}
	return 0;
}
