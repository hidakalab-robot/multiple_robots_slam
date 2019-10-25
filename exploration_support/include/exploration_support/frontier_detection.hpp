#ifndef FRONTIER_DETECTION_HPP
#define FRONTIER_DETECTION_HPP

#include <exploration_libraly/construct.hpp>
#include <exploration_libraly/struct.hpp>
#include <exploration_msgs/FrontierArray.h>
#include <Eigen/Geometry>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


// frontier を検出して座標をトピックに出す機能だけつける
// 出力型 frontier pointarray posearray ?
// frontier 型 だけだして convert で変換を任せる ?
// map を読んでコールバックするだけ
class FrontierDetection
{
private:
    struct mapStruct{
        nav_msgs::MapMetaData info;
        std::vector<std::vector<int8_t>> source;
        std::vector<std::vector<int8_t>> horizon;
        std::vector<std::vector<int8_t>> frontierMap;
        mapStruct(const nav_msgs::OccupancyGrid& m)
            :source(m.info.width,std::vector<int8_t>(m.info.height))
            ,horizon(m.info.width,std::vector<int8_t>(m.info.height,0))
            ,frontierMap(m.info.width,std::vector<int8_t>(m.info.height,0)){
                
            info = m.info;
            for(int y=0,k=0,ey=info.height;y!=ey;++y){
                for(int x=0,ex=info.width;x!=ex;++x,++k){
                    source[x][y] = m.data[k];
                }
            }
        };
    };

    struct clusterStruct{
        std::vector<Eigen::Vector3i> index;
        std::vector<double> areas;
        std::vector<Eigen::Vector2d> variances;
        std::vector<double> covariance;
        std::vector<pcl::PointIndices> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

        clusterStruct():pc(new pcl::PointCloud<pcl::PointXYZ>){};

        clusterStruct(const clusterStruct& cs)
            :index(cs.index)
            ,areas(cs.areas)
            ,variances(cs.variances)
            ,covariance(cs.covariance)
            ,indices(cs.indices)
            ,pc(cs.pc){};
        
        void reserve(int size){
            index.reserve(size);
            areas.reserve(size);
            variances.reserve(size);
            covariance.reserve(size);
        }
    };

    float FILTER_SQUARE_DIAMETER;
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;

    ExpLib::Struct::subStructSimple map_;
    ExpLib::Struct::pubStruct<exploration_msgs::FrontierArray> frontier_;
    ExpLib::Struct::pubStruct<sensor_msgs::PointCloud2> horizon_;
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void horizonDetection(mapStruct& map);
    clusterStruct clusterDetection(const mapStruct& map);
    void obstacleFilter(mapStruct& map,std::vector<Eigen::Vector3i>& index);
    Eigen::Vector3i coordinateToArray(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info);
    geometry_msgs::Point arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info);

    void publishHorizon(const clusterStruct& cs, const std::string& frameId);
    void publishFrontier(const std::vector<exploration_msgs::Frontier>& frontiers, const std::string& frameId);
    

public:
    FrontierDetection();
};

FrontierDetection::FrontierDetection()
    :map_("map", 1, &FrontierDetection::mapCB, this)
    ,frontier_("frontier",1,true)
    ,horizon_("horizon",1,true){
    
    ros::NodeHandle p("~");
    p.param<float>("filter_square_diameter", FILTER_SQUARE_DIAMETER, 0.75);
    p.param<double>("cluster_tolerance", CLUSTER_TOLERANCE, 0.15);
    p.param<int>("min_cluster_size", MIN_CLUSTER_SIZE, 30);
    p.param<int>("max_cluster_size", MAX_CLUSTER_SIZE, 15000); 
}

void FrontierDetection::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    // map の取り込み
    mapStruct map(*msg);
    horizonDetection(map);

    clusterStruct cluster(clusterDetection(map));
    obstacleFilter(map,cluster.index);

    if(cluster.index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found");
        publishFrontier(std::vector<exploration_msgs::Frontier>(), msg->header.frame_id);
        return;
    }
    //ここでクラスタ表示したい
    publishHorizon(cluster, msg->header.frame_id);
    std::vector<exploration_msgs::Frontier> frontiers;
    frontiers.reserve(cluster.index.size());

    for(int i=0,e=cluster.index.size();i!=e;++i){
        if(cluster.index[i].z() == 0) continue;
        frontiers.emplace_back(ExpLib::Construct::msgFrontier(arrayToCoordinate(cluster.index[i].x(),cluster.index[i].y(),map.info),cluster.areas[i],ExpLib::Construct::msgVector(cluster.variances[i].x(),cluster.variances[i].y()),cluster.covariance[i]));
    }

    ROS_INFO_STREAM("Frontier Found : " << frontiers.size());

    publishFrontier(frontiers, msg->header.frame_id);
}

void FrontierDetection::horizonDetection(mapStruct& map){
    ROS_INFO_STREAM("Horizon Detection");
    //x axis horizon
    for(int y=0,ey=map.info.height;y!=ey;++y){
        for(int x=0,ex=map.info.width-1;x!=ex;++x){
            if(map.source[x][y] == 0 && map.source[x+1][y] == -1) map.horizon[x][y] = 1;
            else if(map.source[x][y] == -1 && map.source[x+1][y] == 0) map.horizon[x+1][y] = 1;
        }
    }

    //y axis horizon
    for(int x=0,ex=map.info.width;x!=ex;++x){
        for(int y=0,ey=map.info.height-1;y!=ey;++y){
            if(map.source[x][y] == 0 && map.source[x][y+1] == -1) map.horizon[x][y] = 1;
            else if(map.source[x][y] == -1 && map.source[x][y+1] == 0) map.horizon[x][y+1] = 1;
        }
    }
    ROS_INFO_STREAM("Horizon Detection complete\n");
}

FrontierDetection::clusterStruct FrontierDetection::clusterDetection(const mapStruct& map){
    
    ROS_INFO_STREAM("Frontier Detection by Clustering");

    std::vector<geometry_msgs::Point> points;
    points.reserve(map.info.height*map.info.width);
    for(int y=0,ey=map.info.height;y!=ey;++y){
        for(int x=0,ex=map.info.width;x!=ex;++x){
            if(map.horizon[x][y] == 1) points.emplace_back(arrayToCoordinate(x,y,map.info));
        }
    }

    clusterStruct cs;
    cs.pc -> points.reserve(points.size());

    for(const auto& p : points) cs.pc -> points.emplace_back(pcl::PointXYZ((float)p.x,(float)p.y,0.0f));

    cs.pc -> width = cs.pc -> points.size();
    cs.pc -> height = 1;
    cs.pc -> is_dense = true;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cs.pc);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  	ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);
	ec.setInputCloud (cs.pc);

	ec.extract (cs.indices);
    cs.reserve(cs.indices.size());

    for (std::vector<pcl::PointIndices>::const_iterator it = cs.indices.begin (); it != cs.indices.end (); ++it){
        Eigen::Vector2d sum(0,0);
        Eigen::Vector2d max(-DBL_MAX,-DBL_MAX);
        Eigen::Vector2d min(DBL_MAX,DBL_MAX);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            sum.x() += cs.pc -> points[*pit].x;
            sum.y() += cs.pc -> points[*pit].y;

            if(cs.pc -> points[*pit].x > max.x()) max.x() = cs.pc -> points[*pit].x;
            else if(cs.pc -> points[*pit].x < min.x()) min.x() = cs.pc -> points[*pit].x;

            if(cs.pc -> points[*pit].y > max.y()) max.y() = cs.pc -> points[*pit].y;
            else if(cs.pc -> points[*pit].y < min.y()) min.y() = cs.pc -> points[*pit].y;
        }
        Eigen::Vector2d centroid(sum.x()/it->indices.size(),sum.y()/it->indices.size());
        Eigen::Vector2d variance(0,0);
        double covariance = 0;
        //分散を求めたい
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            variance.x() += (cs.pc -> points[*pit].x - centroid.x()) * (cs.pc -> points[*pit].x - centroid.x());
            variance.y() += (cs.pc -> points[*pit].y - centroid.y()) * (cs.pc -> points[*pit].y - centroid.y());
            covariance += (cs.pc -> points[*pit].x - centroid.x()) * (cs.pc -> points[*pit].y - centroid.y());
        }

        variance /= it->indices.size();
        covariance /= it->indices.size();

        cs.variances.emplace_back(variance);
        cs.covariance.emplace_back(covariance/sqrt(variance.x())/sqrt(variance.y()));
        cs.index.emplace_back(coordinateToArray(std::move(centroid),map.info));
        Eigen::Vector2d diff(max-min);
        cs.areas.emplace_back(std::abs(diff.x()*diff.y()));
    }

    return cs;
}

void FrontierDetection::obstacleFilter(FrontierDetection::mapStruct& map,std::vector<Eigen::Vector3i>& index){
    ROS_INFO_STREAM("Obstacle Filter");
    
    //add obstacle cell
    for(int x=0,ex=map.info.width;x!=ex;++x){
        for(int y=0,ey=map.info.height;y!=ey;++y){
            if(map.source[x][y] == 100) map.frontierMap[x][y] = 100;
        }
    }

    int FILTER_HALF_CELL = (FILTER_SQUARE_DIAMETER / map.info.resolution) / 2.0;

    if(FILTER_HALF_CELL < 1){
        ROS_ERROR_STREAM("FILTER_SQUARE_DIAMETER is Bad");
        return;
    }

    for(auto&& i : index){
        if(map.frontierMap[i.x()][i.y()] == 100){
            i.z() = 0;
            continue;
        }

        int LEFT = i.x()-FILTER_HALF_CELL < 0 ? i.x() : FILTER_HALF_CELL;
        int RIGHT = i.x()+FILTER_HALF_CELL > map.info.width-1 ? (map.info.width-1)-i.x() : FILTER_HALF_CELL;
        int TOP = i.y()-FILTER_HALF_CELL < 0 ? i.y() : FILTER_HALF_CELL;
        int BOTTOM = i.y()+FILTER_HALF_CELL > map.info.height-1 ? (map.info.height-1)-i.y() : FILTER_HALF_CELL;

        for(int y=i.y()-TOP,ey=i.y()+BOTTOM+1;y!=ey;++y){
            for(int x=i.x()-LEFT,ex=i.x()+RIGHT+1;x!=ex;++x){
                if(map.frontierMap[x][y] == 100){//障害部があったら終了
                    map.frontierMap[i.x()][i.y()] = 0;
                    i.z() = 0;
                    x = ex -1;//ラムダで関数作ってreturnで終わっても良いかも
                    y = ey -1;
                }
            }
        }
    }
    ROS_INFO_STREAM("Obstacle Filter complete");
}

geometry_msgs::Point FrontierDetection::arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info){
    return ExpLib::Construct::msgPoint(info.resolution * indexX + info.origin.position.x,info.resolution * indexY + info.origin.position.y);
}

Eigen::Vector3i FrontierDetection::coordinateToArray(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info){
    return Eigen::Vector3i((coordinate.x()-info.origin.position.x)/info.resolution,(coordinate.y()-info.origin.position.y)/info.resolution,1);
}

void FrontierDetection::publishHorizon(const clusterStruct& cs, const std::string& frameId){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorCloud->points.reserve(cs.pc->points.size());
    float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};
    int i=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cs.indices.begin (); it != cs.indices.end (); ++it,++i){
        if(cs.index[i].z()==0) continue;
        int c = i%12;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            colorCloud -> points.emplace_back(ExpLib::Construct::pclXYZRGB(cs.pc->points[*pit].x,cs.pc->points[*pit].y,0.0f,colors[c][0],colors[c][1],colors[c][2]));
        }
    }
    colorCloud -> width = colorCloud -> points.size();
    colorCloud -> height = 1;
    colorCloud -> is_dense = true;
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*colorCloud,msg);
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    horizon_.pub.publish(msg);
	ROS_INFO_STREAM("Publish horizon");
}

void FrontierDetection::publishFrontier(const std::vector<exploration_msgs::Frontier>& frontiers, const std::string& frameId){
    exploration_msgs::FrontierArray msg;
    msg.frontiers = frontiers;
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    frontier_.pub.publish(msg);
    ROS_INFO_STREAM("Publish frontier");
}
#endif //FRONTIER_DETECTION_HPP