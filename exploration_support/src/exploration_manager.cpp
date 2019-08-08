#include <exploration_support/exploration_manager.hpp>

int main(int argc, char* argv[]){
    ros::init(argc,argv,"exploration_manager");

    int END_CONDITION;
    ros::NodeHandle p("~");
    p.param<int>("end_condition", END_CONDITION, 0);
    

    switch (END_CONDITION){
        case 0:
            ExplorationManager<Management::Area>* em;
            break;
        case 1:
            ExplorationManager<Management::Frontier>* em;
            break;
        case 2:
            ExplorationManager<Management::Timer>*  em;
            break;
        default:
            ROS_WARN_STREAM("end_condition is invalid !!");
            return 0;
    }


    // ExplorationManager<Management::Area> em;
    ros::spin();
    return 0;
}