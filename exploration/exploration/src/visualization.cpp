#include <exploration/visualization.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "visualization_fbe");
    Visualization v;
    ros::spin();
    return 0;
}