#include <exploration/movement.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "function_test");

    Movement mv;

    ros::Rate rate(1.0);
    while(ros::ok()){
       mv.functionCallTester();
       rate.sleep();
    }
    
    return 0;
}