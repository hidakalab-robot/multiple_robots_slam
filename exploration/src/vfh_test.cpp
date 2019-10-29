#include <exploration/movement.hpp>

// 障害物の距離や角度がちゃんと取れてるかを確認する ※特に前進のとき

int main(int argc, char** argv){
    ros::init(argc, argv, "vfh_test");
    Movement mv;

    while(true){
        mv.moveToForward();
    }

    return 0;
}