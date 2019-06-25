#include <exploration/multi_exploration_simulator.hpp> // 必須
#include <exploration/seamless_hybrid.hpp> // SeamlessHybrid sh(pp);　用
#include <exploration/path_planning.hpp> // PathPlanning<navfn::NavfnROS> pp("global_costmap","NavfnROS"); 用

int main(int argc, char* argv[]){
    // 見づらいのでusingしておく
    using fnType = std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)>;

    ros::init(argc, argv, "multi_exploration_simulator");
    MultiExplorationSimulator mes;

    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig> server;
    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig>::CallbackType cbt;
    cbt = boost::bind(&MultiExplorationSimulator::callback,&mes, _1, _2);
    server.setCallback(cbt);

    // ここまで共通ルート

    /**
     * 使い方
     * "シミュレータ内で使いたい関数のオブジェクト"の記述方法
     * 
     * std::bind("使いたい関数のポインタ","第一引数の関数が使えるインスタンスのポインタ", std::placeholders::_1,std::placeholders::_2, std::placeholders::_3)
     * 
     * 渡せる関数のルール
     * 戻り値:void
     * 引数 3つ : std::vector<geometry_msgs::Pose>& p, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f
     * p : 全ロボットのポーズ, b : 全分岐領域の座標, f : 全フロンティア領域の座標 <- 受け取れるようになっていれば使わなくてもok
     */

    // 使用例

    PathPlanning<navfn::NavfnROS> pp("simulator_calc_costmap","simulator_calc_path");
    SeamlessHybrid sh(pp);

    // 渡す関数のオブジェクトを作成
    fnType fn = std::bind(&SeamlessHybrid::simulatorFunction,&sh,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

    // ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce(); // 必須
        mes.updateParameters(fn); // 必須 <- 引数に上で作った渡したい関数のオブジェクトを入れる
        // rate.sleep();
    }
    mes.writeParameters();
    return 0;
}