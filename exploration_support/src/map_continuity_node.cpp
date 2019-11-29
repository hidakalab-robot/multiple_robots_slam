#include <exploration_support/continuity.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "continuity");
    Continuity<nav_msgs::OccupancyGrid> cog("map", "map_continuity");
    while(ros::ok()) cog.publishNow();    
    return 0;
}