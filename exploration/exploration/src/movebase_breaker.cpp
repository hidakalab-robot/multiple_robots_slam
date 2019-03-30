#include<ros/ros.h>
#include<actionlib_msgs/GoalStatusArray.h>

void callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg,const ros::Publisher& pub){
    static actionlib_msgs::GoalID lastId;
    static bool adhesion = false;
    if(msg->status_list.empty()){
        return;
    }
    
    actionlib_msgs::GoalID newId = msg->status_list.back().goal_id;
    if(lastId.id != newId.id){
        adhesion = false;
        //その目標を許すかどうかの判断をする
        std::cout << "id : " << newId.id << std::endl;
        std::cout << "ここ行っても良い？ y/n" << std::endl;
        char judge;
        std::cin >> judge;
        if(judge == 'n'){
            pub.publish(newId);
            lastId = newId;
            adhesion = true;
            std::cout << "ダメです" << std::endl;
            return;
        }
        lastId = newId;
        std::cout << "いいね" << std::endl;
    }
    else if(adhesion && msg->status_list.back().status != actionlib_msgs::GoalStatus::PREEMPTED){
        pub.publish(newId);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "movebase_breaker");
    ros::NodeHandle nh;
    ros::NodeHandle p("~");
    std::string ns;
    p.getParam("ns",ns);
    ros::Publisher pub = nh.advertise<actionlib_msgs::GoalID>(ros::names::append(ns,"move_base/cancel"),1,true);
    ros::Subscriber sub = nh.subscribe<actionlib_msgs::GoalStatusArray>(ros::names::append(ns,"move_base/status"),1,boost::bind(callback,_1,pub));

    ros::spin();
    
    return 0;
}