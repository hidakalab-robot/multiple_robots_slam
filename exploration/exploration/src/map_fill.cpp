#include <exploration/map_fill.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "map_fill");

    MapFill mf;

    ros::spin();
    
    return 0;
}