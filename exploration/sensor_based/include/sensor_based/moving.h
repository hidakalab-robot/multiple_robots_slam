#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


//センサーデータを受け取った後にロボットの動作を決定する
//障害物回避を含む
class Moving 
{
private:

public:
    Moving();
    ~Moving(){};
    void initialize();
};

Moving::Moving(){

}

void Moving::initialize(){

}