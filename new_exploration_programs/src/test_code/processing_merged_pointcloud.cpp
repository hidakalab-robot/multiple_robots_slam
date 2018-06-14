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
			/*マージドが空のときの処理を加える*/
			if(pp.is_empty())
			{
				pp.publish_empty_merged();
				std::cout << "********50**********" << '\n';
			}
			else
			{
				//pp.apply_voxelgrid();
				std::cout << "51" << '\n';
				//pp.naninf();
				//pp.test_cloud();
				pp.delete_ground();
				std::cout << "52" << '\n';
				//pp.test_cloud();
				pp.euclidean_clustering();
				std::cout << "53" << '\n';
				pp.test_cloud();
				//pp.publish_pointcloud();
				std::cout << "54" << '\n';
				pp.feature_extraction();
				std::cout << "55" << '\n';
				pp.publish_merged_segmented();
				std::cout << "56" << '\n';
			}
			pp.input = false;
		}
		else
		{
			std::cout << '\n' << "not input" << '\n';
		}
	}
	return 0;
}
