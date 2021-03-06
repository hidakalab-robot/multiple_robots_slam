#ifndef STRUCT_H
#define STRUCT_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <exploration_libraly/enum.h>
#include <geometry_msgs/Point.h>
#include <vector>

// 前方宣言

/// rosmsgs
namespace nav_msgs{
    template <class ContainerAllocator>
    struct MapMetaData_;
    typedef ::nav_msgs::MapMetaData_<std::allocator<void>> MapMetaData;
}
// 前方宣言 ここまで

namespace ExpLib{
    namespace Struct{
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
            subStructSimple(const std::string& topic,uint32_t queue_size, void(U::*fp)(V), U *obj){ 
                sub = n.subscribe(topic,queue_size,fp,obj);
            };
            template<typename V>
            subStructSimple(const std::string& topic,uint32_t queue_size, void(*fp)(V)){
                sub = n.subscribe(topic,queue_size,fp);
            };
        };
        struct subStructStd{
            ros::NodeHandle n;
            ros::Subscriber sub;
        };
        template<typename T>
        struct pubStruct{
            ros::NodeHandle n;
            ros::Publisher pub;
            pubStruct(const std::string& topic,uint32_t queue_size,bool latch=false){
                pub = n.advertise<T>(topic, queue_size, latch);
            };
        };
        struct pubStructStd{
            ros::NodeHandle n;
            ros::Publisher pub;
        };
        struct scanStruct{
            std::vector<float> ranges;
            std::vector<float> angles;
            std::vector<float> x;
            std::vector<float> y;

            scanStruct(int size);
        };
        struct listStruct{
            geometry_msgs::Point point;
            Enum::DuplicationStatus duplication;

            listStruct();
            listStruct(const geometry_msgs::Point& p);
        };
        struct mapSearchWindow{// 中心の座標, マップの大きさ, 窓の大きさを引数に取って　窓の上下左右の要素番号を返す
            int width;
            int height;
            int top;
            int bottom;
            int left;
            int right;
            
            mapSearchWindow(const geometry_msgs::Point& cc, const nav_msgs::MapMetaData& info, double lx, double ly=0.0); // cc : 検索窓の中心座標, info : 地図のメタデータ, lx,ly : 検索窓の辺の長さ(m)
            mapSearchWindow(const int cx, const int cy, const int mx, const int my, int lx, int ly=0); // cx,cy : 検索窓の中心の二次元配列インデックス, mx,my : 地図の辺の長さ(cell), lx,ly : 検索窓の辺の長さ(cell)
            void calcWindowSize(const int cx, const int cy, const int mx, const int my, const int lx, const int ly);
        };
    }
}
#endif // STRUCT_H