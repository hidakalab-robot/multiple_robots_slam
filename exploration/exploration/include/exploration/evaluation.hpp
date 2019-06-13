#ifndef EVALUATION_HPP
#define EVALUATION_HPP

#include <ros/ros.h>
#include <exploration_msgs/Frontier.h>
#include <geometry_msgs/Point.h>
#include <exploration/common_lib.hpp>



//初期化と比較した結果
//frontierのリストと分岐領域のリストと前方の位置を受け取る
//スルーもこっちに実装すればいい
//liststructなどは顧問ぃ部にいれる
//するー情報もすとらくと　に入れれば良いかも
class Evaluation
{
private:
    struct sumValue{
        double sumDistnance;
        double sumAngle;
        geometry_msgs::Point coordinate;
    };

    struct maxValue{
        double distance;
        double angle;
        maxValue(double d, double a):distance(d),angle(a){};
    };

    double ANGLE_WEIGHT;
    double NORM_WEIGHT;


    std::vector<exploration_msgs::Frontier> frontiers;
    std::vector<geometry_msgs::Point> branches;
    geometry_msgs::Pose pose;

    std::vector<sumValue> sVal;
    maxValue mVal;

public:
    Evaluation(const std::vector<exploration_msgs::Frontier>& f, const std::vector<geometry_msgs::Point>& b, const geometry_msgs::Pose& p);
    
    void initialize(void);
    bool result(geometry_msgs::Point& goal);// 分岐に行く場合はtrueと座標を返して、直進する場合はfalseを返す　
};

Evaluation::Evaluation(const std::vector<exploration_msgs::Frontier>& f, const std::vector<geometry_msgs::Point>& b, const geometry_msgs::Pose& p)
    :frontiers(f)
    ,branches(b)
    ,pose(p)
    ,mVal(-DBL_MAX,-DBL_MAX){

    ros::NodeHandle ph("~");
    ph.param<double>("angle_weight", ANGLE_WEIGHT, 1.5);
    ph.param<double>("norm_weight", NORM_WEIGHT, 2.5); 

    sVal.reserve(branches.size()+1);
};
// Eigen::Vector2d(goal.x-pose.position.x,goal.y-pose.position.y).norm()
void Evaluation::initialize(void){
    //frontierに対する距離と角度の最大値と合計値
    // std::vector
    //分岐領域に関する計算
    // ROS_DEBUG_STREAM("adopted frontier : " << values.size() << " / " << frontiers.size());
    // for();

    //mVal, sValを計算すれば良い

    //直進方向の情報作成

    // Eigen::Vector2d toFrontier(f.coordinate.x - origin.x,f.coordinate.y - origin.y);
//         Eigen::Vector4d temp(std::abs(acos(vec.dot(toFrontier.normalized()))),toFrontier.lpNorm<1>(),f.variance.x>f.variance.y ? f.variance.x : f.variance.y,std::abs(f.covariance));
    // Eigen::Vector2d(g.point.x-pose.position.x,g.point.y-pose.position.y).normalized()

    auto calc = [this](const geometry_msgs::Point& p, const Eigen::Vector2d& v1){
        ROS_DEBUG_STREAM("calc p : (" << p.x << "," << p.y << ")");
        sumValue s{0,0,p};
        for(const auto& f : frontiers){
            Eigen::Vector2d v2 = Eigen::Vector2d(f.coordinate.x - p.x, f.coordinate.y - p.y);

            double angle = std::abs(acos(v1.normalized().dot(v2.normalized())));
            double distance = v2.lpNorm<1>();

            s.sumAngle += angle;
            s.sumDistnance += distance;

            if(angle > mVal.angle) mVal.angle = std::move(angle);
            if(distance > mVal.distance) mVal.distance = std::move(distance);
        }
        return s;
    };

    //現在位置から分岐領域への距離の平均値
    // double forward = 0;
    // for(const auto& b : branches){
    //     sumValue s{0,0,b};
    //     Eigen::Vector2d v1 = Eigen::Vector2d(b.x - pose.position.x, b.y - pose.position.y);
    //     for(const auto& f : frontiers){
    //         Eigen::Vector2d v2 = Eigen::Vector2d(f.coordinate.x - b.x, f.coordinate.y - b.y);

    //         double angle = std::abs(acos(v1.normalized().dot(v2.normalized())));
    //         double distance = v2.lpNorm<1>();

    //         s.sumAngle += angle;
    //         s.sumDistnance += distance;

    //         if(angle > mVal.angle) mVal.angle = std::move(angle);
    //         if(distance > mVal.distance) mVal.distance = std::move(distance);
    //     }
    //     // s.coordinate = b;
    //     sVal.emplace_back(std::move(s));
    //     forward += v1.norm();
    // }

    double forward = 0;
    for(const auto& b : branches){
        Eigen::Vector2d v1 = Eigen::Vector2d(b.x - pose.position.x, b.y - pose.position.y);
        sVal.emplace_back(calc(b,v1));
        forward += v1.norm();
    }
    forward /= branches.size();

    //直進時の計算
    double yaw = CommonLib::qToYaw(pose.orientation);
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);

    sVal.emplace_back(calc(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw)));
}

bool Evaluation::result(geometry_msgs::Point& goal){
    //ここで計算する

    //frontierの数は0にならない
    //分岐領域の数は0ではない
    
    ROS_DEBUG_STREAM("sVal size : " << sVal.size());

    double minE = DBL_MAX;

    for(int i=0,ie=sVal.size();i!=ie;++i){
        double e = NORM_WEIGHT * sVal[i].sumDistnance / mVal.distance + ANGLE_WEIGHT * sVal[i].sumAngle / mVal.angle;
        ROS_DEBUG_STREAM("position : (" << sVal[i].coordinate.x << "," << sVal[i].coordinate.y << "), sum : " << e);
        if(e < minE){
            if(i == ie) return false;
            minE = std::move(e);
            goal = sVal[i].coordinate;
        }
    }
    return true;
}
// double FrontierSearch::evoluatePointToFrontier(const geometry_msgs::Pose& pose, double forward,const std::vector<exploration_msgs::Frontier>& frontiers){
//     //前向きのベクトルを自動生成
//     double yaw = CommonLib::qToYaw(pose.orientation);
//     double cosYaw = cos(yaw);
//     double sinYaw = sin(yaw);
//     ROS_DEBUG_STREAM("forward sum");
//     return evoluatePointToFrontier(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw),frontiers);
// }

// double FrontierSearch::evoluatePointToFrontier(const geometry_msgs::Point& origin,const Eigen::Vector2d& vec,const std::vector<exploration_msgs::Frontier>& frontiers){
//     //frontierの大きさで重みをつけたい
//     //距離にも重みをつける

//     //各要素を正規化したいので初めに全部計算しながら最大値を求める
//     //正規化がこの方法で良いかは謎//全部のoriginについてまとめて計算したほうが良いかもしれない//どっかでinitialize関数を作ってそっちで正規化用の最大値を計算する?
//     //距離の計算、パス作って計算したほうが良いかも
//     std::vector<Eigen::Vector4d> values;
//     values.reserve(frontiers.size());
//     Eigen::Vector4d max(-DBL_MAX,-DBL_MAX,-DBL_MAX,-DBL_MAX);//angle:norm:variance:covariance
//     for(const auto& f : frontiers){
//         Eigen::Vector2d toFrontier(f.coordinate.x - origin.x,f.coordinate.y - origin.y);
//         Eigen::Vector4d temp(std::abs(acos(vec.dot(toFrontier.normalized()))),toFrontier.lpNorm<1>(),f.variance.x>f.variance.y ? f.variance.x : f.variance.y,std::abs(f.covariance));
//         for(int i=0;i<4;++i) {
//             if(temp[i] > max[i]) max[i] = temp[i];
//         }
//         if(temp[3] < COVARIANCE_THRESHOLD && temp[2] < VARIANCE_THRESHOLD) continue;//共分散が小さいフロンティアは考慮しない//ただし、分散が大きければ考慮しても良いかも
//         values.emplace_back(temp);
//     }

//     //valuesを正規化しつつ評価値を計算
//     double sum = 0;

//     ROS_DEBUG_STREAM("adopted frontier : " << values.size() << " / " << frontiers.size());
//     switch (values.size()){
//     case 0:
//         sum = DBL_MAX;
//         break;
//     case 1:
//         sum = ANGLE_WEIGHT*values[0][0] + NORM_WEIGHT*values[0][1];
//         break;
//     default:
//         for(const auto& v : values) sum += (ANGLE_WEIGHT*v[0]/max[0] + NORM_WEIGHT*v[1]/max[1]);
//         break;
//     }

//     ROS_DEBUG_STREAM("position : (" << origin.x << "," << origin.y << "), sum : " << sum);

//     return sum;
// }

#endif //EVALUATION_HPP