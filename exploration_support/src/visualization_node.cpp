#include <exploration_support/visualization.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "visualization_fbe");
    Visualization v;
    v.multiThreadMain();
    return 0;
}