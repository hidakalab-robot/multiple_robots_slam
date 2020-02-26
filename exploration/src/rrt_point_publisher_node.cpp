#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_libraly/construct.h>


int main(int argc, char *argv[]){
    ros::init(argc, argv, "rrt_point_publisher");

    int MAP_TYPE;
    ros::Publisher pub = ros::NodeHandle().advertise<geometry_msgs::PointStamped>("/clicked_point",1,true);
    ros::NodeHandle("~").param<int>("map_type",MAP_TYPE,0);
    geometry_msgs::PointStamped lt,lb,rt,rb,origin;

    origin.point = ExpLib::Construct::msgPoint(0,0,0);

    switch (MAP_TYPE){
        case 0:
            lt.point = ExpLib::Construct::msgPoint(-5,14,0);
            lb.point = ExpLib::Construct::msgPoint(-5,-2,0);
            rt.point = ExpLib::Construct::msgPoint(44,14,0);
            rb.point = ExpLib::Construct::msgPoint(44,-2,0);
            break;
        case 1:
            lt.point = ExpLib::Construct::msgPoint(-6,21,0);
            lb.point = ExpLib::Construct::msgPoint(-6,-11,0);
            rt.point = ExpLib::Construct::msgPoint(36,21,0);
            rb.point = ExpLib::Construct::msgPoint(36,-11,0);
            break;
        default:
            ROS_INFO_STREAM("unknown MAP_TYPE !!");
            return 0;
    }

    ROS_INFO_STREAM("press any key, publish lt-coordinate");
    std::cin.get();
    pub.publish(lt);

    ROS_INFO_STREAM("press any key, publish lb-coordinate");
    std::cin.get();
    pub.publish(lb);

    ROS_INFO_STREAM("press any key, publish rb-coordinate");
    std::cin.get();
    pub.publish(rb);

    ROS_INFO_STREAM("press any key, publish rt-coordinate");
    std::cin.get();
    pub.publish(rt);

    ROS_INFO_STREAM("press any key, publish origin-coordinate");
    std::cin.get();
    pub.publish(origin);

    ROS_INFO_STREAM("finished rrt_point_publisher");

    ros::shutdown();
    
    return 0;
}