#include <exploration_support/loop_closure_counter.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/construct.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/loop_closure_counter_parameter_reconfigureConfig.h>
#include <fstream>
#include <thread>

// legacy loop counter
namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;

LoopClosureCounter::LoopClosureCounter()
    :count("loop_closure_counter/count",1)
    ,accumTemp("loop_closure_counter/temp_accumlate",1)
    ,accumPerm("loop_closure_counter/perm_accumlate",1)
    ,drs_(ros::NodeHandle("~/loop")){
    loadParams(); 
    drs_.setCallback(boost::bind(&LoopClosureCounter::dynamicParamsCB,this, _1, _2));
}

LoopClosureCounter::~LoopClosureCounter(){
    if(OUTPUT_LOOP_PARAMETERS) outputParams();
}

void LoopClosureCounter::loopDetectionLoop(void){
    ros::spinOnce();
    std::thread loopThread([this]{loopDetection();});
    ros::spin();
    loopThread.join();
}

void LoopClosureCounter::loopDetection(void){
    tf::TransformListener listener;
    listener.waitForTransform(MAP_FRAME_ID, ODOM_FRAME_ID, ros::Time(), ros::Duration(1.0));
    Eigen::Vector2d lastTrans(0,0);
    int loopCount = 0;
    double accumTrans = 0;
    double accumTransPerm = 0;

    ros::Rate rate(PUBLISH_RATE);
    while(ros::ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform(MAP_FRAME_ID, ODOM_FRAME_ID, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
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
        count.pub.publish(ExCos::msgInt8(loopCount));
        accumTemp.pub.publish(ExCos::msgDouble(accumTrans));
        accumPerm.pub.publish(ExCos::msgDouble(accumTransPerm));
        rate.sleep();
    }
}

void LoopClosureCounter::loadParams(void){
    ros::NodeHandle nh("~/loop");
    // dynamic parameters
    nh.param<double>("loop_closure_threshold",LOOP_CLOSURE_THRESHOLD,0.0);
    // static parameters
    nh.param<std::string>("odom_frame_id",ODOM_FRAME_ID,"odom");
    nh.param<std::string>("map_frame_id",MAP_FRAME_ID,"map");
    nh.param<double>("publish_rate",PUBLISH_RATE,10.0);
    nh.param<std::string>("loop_parameter_file_path",LOOP_PARAMETER_FILE_PATH,"loop_last_parameters.yaml");
    nh.param<bool>("output_loop_parameters",OUTPUT_LOOP_PARAMETERS,true);
}

void LoopClosureCounter::dynamicParamsCB(exploration_support::loop_closure_counter_parameter_reconfigureConfig &cfg, uint32_t level){
    LOOP_CLOSURE_THRESHOLD = cfg.loop_closure_threshold;
}

void LoopClosureCounter::outputParams(void){
    std::cout << "writing loop last parameters ... ..." << std::endl;
    std::ofstream ofs(LOOP_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "loop param file open succeeded" << std::endl;
    else {
        std::cout << "loop param file open failed" << std::endl;
        return;
    }

    ofs << "loop_closure_threshold: " << LOOP_CLOSURE_THRESHOLD << std::endl;
}