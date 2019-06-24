#include <exploration/multi_exploration_simulator.hpp>
#include <exploration/seamless_hybrid.hpp>
#include <exploration/path_planning.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "multi_exploration_simulator");
    MultiExplorationSimulator mes;

    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig> server;
    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig>::CallbackType cbt;

    // auto fn = [](std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f)->void{std::cout << "print" << std::endl;};

    /**
     * 使い方
     * boost::bind(&MultiExplorationSimulator::callback, &mes, _1, _2, "シミュレータ内で使いたい関数のオブジェクト")
     * 
     * "シミュレータ内で使いたい関数のオブジェクト"の記述方法
     * std::bind("使いたい関数のポインタ","第一引数の関数が使えるインスタンスのポインタ", std::placeholders::_1,std::placeholders::_2, std::placeholders::_3)
     * 
     * 渡せる関数のルール
     * 戻り値:void
     * 引数 3つ : std::vector<geometry_msgs::Pose>& p, std::vector<geometry_msgs::Pose>& b, std::vector<geometry_msgs::Pose>& f
     * p : 全ロボットのポーズ, b : 全分岐領域の座標, f : 全フロンティア領域の座標 // 受け取れるようになっていればok
     */

    // 使用例
    PathPlanning<navfn::NavfnROS> pp("global_costmap","NavfnROS");
    SeamlessHybrid sh(pp);
    
    // cbt = boost::bind(&MultiExplorationSimulator::callback,&mes, _1, _2, std::bind(&SeamlessHybrid::simulatorFunction,&sh,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
    // cbt = boost::bind(&MultiExplorationSimulator::callback,&mes, _1, _2,fn);
    cbt = boost::bind(&MultiExplorationSimulator::callback,&mes, _1, _2);
    server.setCallback(cbt);

    std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn = std::bind(&SeamlessHybrid::simulatorFunction,&sh,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);


    ros::Rate rate(1);

    while(ros::ok()){
        ros::spinOnce();
        mes.readParameter(fn);
        rate.sleep();
    }

    ros::spin();
    return 0;
}