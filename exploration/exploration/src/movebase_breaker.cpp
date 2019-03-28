#include<ros/ros.h>
#include<actionlib_msgs/GoalStatusArray.h>

ros::Subscriber sub;
ros::Publisher pub;

void callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    static actionlib_msgs::GoalID lastId;
    if(msg->status_list.empty()){
        return;
    }
    actionlib_msgs::GoalID newId = msg->status_list.back().goal_id;
    if(lastId.id != newId.id){
        //その目標を許すかどうかの判断をする
        std::cout << "id : " << newId.id << std::endl;
        std::cout << "ここ行っても良い？ y/n" << std::endl;
        char judge;
        std::cin >> judge;
        if((judge == 'n') || (judge== 'no')){
            pub.publish(newId);
            lastId = newId;
            std::cout << "ダメです" << std::endl;
            return;
        }
        lastId = newId;
        std::cout << "ok" << std::endl;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "movebase_breaker");
    ros::NodeHandle nh;
    sub = nh.subscribe("move_base/status",1,callback);
    pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1,true);

    ros::spin();
    
    return 0;
}