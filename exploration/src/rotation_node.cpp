#include <ros/ros.h>
#include <exploration/movement.h>

//mainのところにexploration_endを評価する部分
int main(int argc, char *argv[]){
    ros::init(argc, argv, "rotation");

    Movement mv;
    mv.oneRotation();
    ros::shutdown();
    
    return 0;
}