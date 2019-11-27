#ifndef STRUCT_HPP
#define STRUCT_HPP

#include <exploration_libraly/enum.hpp>
#include <exploration_libraly/utility.hpp>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace ExpLib
{
namespace Struct
{
template <typename T>
struct subStruct{
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::CallbackQueue q;
    T data;
    subStruct(const std::string& topic,uint32_t queue_size){
        n.setCallbackQueue(&q);
        sub = n.subscribe<T>(topic, queue_size, [this](const boost::shared_ptr<const T>& msg) {data = *msg;});//データをコピーするコールバック関数を自動生成
    };

    template <class U,typename V>
    subStruct(const std::string& topic,uint32_t queue_size, void(U::*fp)(V), U* obj){
        n.setCallbackQueue(&q);
        sub = n.subscribe(topic,queue_size,fp,obj);
    };
};

struct subStructSimple{
    ros::NodeHandle n;
    ros::Subscriber sub;
    template <class U,typename V>
    subStructSimple(const std::string& topic,uint32_t queue_size, void(U::*fp)(V), U *obj){ sub = n.subscribe(topic,queue_size,fp,obj);};

    template<typename V>
    subStructSimple(const std::string& topic,uint32_t queue_size, void(*fp)(V)){ sub = n.subscribe(topic,queue_size,fp);};
};

struct subStructStd{
    ros::NodeHandle n;
    ros::Subscriber sub;
};

template<typename T>
struct pubStruct{
    ros::NodeHandle n;
    ros::Publisher pub;
    pubStruct(const std::string& topic,uint32_t queue_size,bool latch=false){ pub = n.advertise<T>(topic, queue_size, latch);};
};

struct pubStructStd{
    ros::NodeHandle n;
    ros::Publisher pub;
};

struct scanStruct{
    std::vector<float> ranges;
    std::vector<float> angles;
    std::vector<double> x;
    std::vector<double> y;
    // float angleMax;
    // scanStruct(int size,float angle):angleMax(angle){
    //     ranges.reserve(size);
    //     angles.reserve(size);
    // };
    scanStruct(int size){
        ranges.reserve(size);
        angles.reserve(size);
        x.reserve(size);
        y.reserve(size);
    };
};

struct listStruct{
    geometry_msgs::Point point;
    Enum::DuplicationStatus duplication;
    listStruct():duplication(Enum::DuplicationStatus::NOT_DUPLECATION){};
    listStruct(const geometry_msgs::Point& p):point(p),duplication(Enum::DuplicationStatus::NOT_DUPLECATION){};
};

struct mapSearchWindow{// 中心の座標, マップの大きさ, 窓の大きさを引数に取って　窓の上下左右の要素番号を返す
    int top;
    int bottom;
    int left;
    int right;
    mapSearchWindow(const geometry_msgs::Point& cc, const nav_msgs::MapMetaData& info, double lx, double ly=0.0){ // cc : 検索窓の中心座標, info : 地図のメタデータ, lx,ly : 検索窓の辺の長さ(m)
        if(lx < info.resolution) lx = info.resolution;
        if(ly ==  0.0) ly = lx;
        else if(ly < info.resolution) ly = info.resolution;
        Eigen::Vector2i index(ExpLib::Utility::coordinateToMapIndex(cc,info));
        calcWindowSize(index.x(),index.y(),info.width, info.height, lx/info.resolution, ly/info.resolution);
    }
    mapSearchWindow(const int cx, const int cy, const int mx, const int my, int lx, int ly=0){ // cx,cy : 検索窓の中心の二次元配列インデックス, mx,my : 地図の辺の長さ(cell), lx,ly : 検索窓の辺の長さ(cell)
        if(lx < 1) lx = 1;
        if(ly == 0) ly = lx;
        else if(ly<  1) ly = 1;
        calcWindowSize(cx,cy,mx,my,lx,ly);
    }

    void calcWindowSize(const int cx, const int cy, const int mx, const int my, const int lx, const int ly){
        int hx1 = lx/2;
        int hx2 = lx%2 == 1 ? lx/2 : lx/2-1; 
        int hy1 = ly/2;
        int hy2 = ly%2 == 1 ? ly/2 : ly/2-1;
        top = cy < hy1 ? 0 : cy-hy1;
        bottom = cy+hy2 > my-1 ? my-1 : cy+hy2;
        left = cx < hx1 ? 0 : cx-hx1; 
        right = cx+hx2 > mx-1 ? mx-1 : cx+hx2;
    }
};

}
}
#endif // STRUCT_HPP