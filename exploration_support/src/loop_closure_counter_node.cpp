#include <exploration_support/loop_closure_counter.h>
#include <ros/ros.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "loop_closure_counter");
    LoopClosureCounter lcc;
    lcc.loopDetectionLoop();
    return 0;
}