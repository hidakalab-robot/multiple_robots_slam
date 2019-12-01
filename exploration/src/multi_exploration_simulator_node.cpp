#include <ros/ros.h>
#include <exploration/multi_exploration_simulator.h> // 必須
#include <exploration/seamless_hybrid_exploration.h> // SeamlessHybrid sh(pp);　用

int main(int argc, char* argv[]){
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

    // 見づらいのでusingしておく
    using fnType = std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)>;

    ros::init(argc, argv, "multi_exploration_simulator");
    MultiExplorationSimulator mes;

    SeamlessHybridExploration sbe;

    // 渡す関数のオブジェクトを作成
    fnType fn = std::bind(&SeamlessHybridExploration::simBridge,&sbe,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

    while(ros::ok()){
        ros::spinOnce(); // 必須
        mes.updateParams(fn); // 必須 <- 引数に上で作った渡したい関数のオブジェクトを入れる
    }
    return 0;
}