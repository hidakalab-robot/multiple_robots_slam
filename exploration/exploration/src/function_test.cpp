#include <exploration/movement.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "function_test");

    Movement mv;

    while(ros::ok()){
       mv.functionCallTester();
    }
    
    return 0;
}