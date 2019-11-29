#include <exploration_support/map_fill.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "map_fill");
    MapFill mf;
    ros::spin();
    return 0;
}