#include <exploration/visualization.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualization_sbe");

    Visualization v;

    ros::spin();
    
    return 0;
}