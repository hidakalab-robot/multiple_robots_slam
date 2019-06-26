#include <laser_merge/laser_merge.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_merge");

    ros::NodeHandle nh("~");

    std::string SCAN_TOPIC_ONE;
    std::string SCAN_TOPIC_TWO;

    nh.param<std::string>("scan_topic_one",SCAN_TOPIC_ONE,"scan1");
    nh.param<std::string>("scan_topic_two",SCAN_TOPIC_TWO,"scan2");
    
    message_filters::Subscriber<sensor_msgs::LaserScan> scan1Sub(nh,SCAN_TOPIC_ONE,1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan2Sub(nh,SCAN_TOPIC_TWO,1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan1Sub, scan2Sub);

    // message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync(scan1Sub,scan2Sub,10);

    ROS_INFO_STREAM("main");

    LaserMerge lm;

    sync.registerCallback(boost::bind(&LaserMerge::callback,&lm,_1,_2));

    
    ros::spin();
    return 0;
}