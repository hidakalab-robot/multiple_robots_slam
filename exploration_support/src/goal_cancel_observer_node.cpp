#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <exploration_msgs/PointArray.h>
#include <thread>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "goal_cancel_observer");
    // /robot1/move_base/goal が出力されたらmove_base_msgs/MoveBaseActionGoal
    // /robot1/move_base/result を観測 move_base_msgs/MoveBaseActionResult
    // cansel すると status が 2かつtextが空になってる
    // また 4以上は多分自動でキャンセルされたやつ
    // goalは来るたびに保存しておく
    // resultが来た時にgoalの記録を確認して同一idのゴールをキャンセルに登録

    ros::Publisher pub = ros::NodeHandle().advertise<exploration_msgs::PointArray>("canceled_goals",1,true);
    std::vector<move_base_msgs::MoveBaseActionGoal> goals;
    exploeration_msgs::PointArray canceledGoals;
    double CANCELED_GOALS_PUBLISH_RATE;
    ros::NodeHandle("~").param<double>("canceled_goals_publish_rate", CANCELED_GOALS_PUBLISH_RATE, 10.0);
    ros::Subscriber subGoal = ros::NodeHandle().subscribe<move_base_msgs::MoveBaseActionGoal>("move_base/goal",1,[&goals](const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){goals.emplace_back(*msg);});
    ros::Subscriber subResult = ros::NodeHandle().subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result",1,[&](const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
        if((msg->status.status==2&&msg->status.text=="")|| msg->status.status > 3){
            for(int i=goals.size()-1;i!=0;--i){
                if(msg->status.goal_id.id== goals[i].goal_id.id){
                    canceledGoals.points.emplace_back(goals[i].goal.target_pose.pose.position);
                    canceledGoals.header.frame_id = goals[i].goal.target_pose.header.frame_id;
                    break;
                }
            }
        }
    });
    std::thread pubThread([&]{
        ros::Rate rate(CANCELED_GOALS_PUBLISH_RATE);
        while(ros::ok()){
            canceledGoals.header.stamp = ros::Time::now();
            pub.publish(canceledGoals);
            rate.sleep();
        }
    });
    ros::spin();
    pubThread.join();
    return 0;
}