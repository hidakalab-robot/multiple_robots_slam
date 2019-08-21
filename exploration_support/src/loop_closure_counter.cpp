#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/construct.hpp>

// legacy loop counter

int main(int argc, char* argv[]){
    ros::init(argc, argv, "loop_closure_counter");

    ros::NodeHandle p("~");
    std::string ODOM_FRAME_ID, MAP_FRAME_ID;
    double LOOP_CLOSURE_THRESHOLD;

    ExpLib::Struct::pubStruct<std_msgs::Int8> count("loop_closure_counter/count",1);
    ExpLib::Struct::pubStruct<std_msgs::Float64> accumTemp("loop_closure_counter/temp_accumlate",1);
    ExpLib::Struct::pubStruct<std_msgs::Float64> accumPerm("loop_closure_counter/perm_accumlate",1);

    
    p.param<std::string>("odom_frame_id",ODOM_FRAME_ID,"odom");
    p.param<std::string>("map_frame_id",MAP_FRAME_ID,"map");
    p.param<double>("loop_closure_threshold",LOOP_CLOSURE_THRESHOLD,1.0);

    tf::TransformListener listener;
    listener.waitForTransform(MAP_FRAME_ID, ODOM_FRAME_ID, ros::Time(), ros::Duration(1.0));

    Eigen::Vector2d lastTrans(0,0);
    double accumTrans = 0;
    double accumTransPerm = 0;
    int loopCount = 0;

    while(ros::ok()){
        tf::StampedTransform transform;
        listener.lookupTransform(MAP_FRAME_ID, ODOM_FRAME_ID, ros::Time(0), transform);
        double transX = transform.getOrigin().getX();
        double transY = transform.getOrigin().getY();

        if(transX != lastTrans.x() || transY != lastTrans.y()){
            double trans = Eigen::Vector2d(Eigen::Vector2d(transX, transY) - lastTrans).norm();
            accumTrans += trans;
            accumTransPerm += trans;
            if(accumTrans > LOOP_CLOSURE_THRESHOLD){
                ++loopCount;
                accumTrans = 0;
            }
            lastTrans << transX, transY; 
        }
        count.pub.publish(ExpLib::Construct::msgInt8(loopCount));
        accumTemp.pub.publish(ExpLib::Construct::msgDouble(accumTrans));
        accumPerm.pub.publish(ExpLib::Construct::msgDouble(accumTransPerm));
    }

    return 0;
}
