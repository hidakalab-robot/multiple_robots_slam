#include <ros/ros.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "new_hybrid_exploration");
	ros::NodeHandle h;

	std::cout << "hybrid_exploration program is begun" << std::endl;
	std::cout << "start: sensor-based exploration" << std::endl;
	system("rosrun hybrid_exploration hybrid_new_sensor_based_exploration"); 
	std::cout << "end  :sensor-based exploration" << std::endl;
	std::cout << "switching program" << std::endl;
	std::cout << "start: frontier-based exploration" << std::endl;
	system("rosrun hybrid_exploration hybrid_new_vector_explore_program");
	std::cout << "end : frontier-based exploration" << std::endl;
	std::cout << "hybrid_exploration program is finished" << std::endl;

	return 0;
}
