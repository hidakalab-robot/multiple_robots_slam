#include <exploration_support/frontier_detection.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/struct.h>
#include <exploration_msgs/FrontierArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <exploration_libraly/utility.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/frontier_detection_parameter_reconfigureConfig.h>
#include <fstream>
#include <Eigen/Core>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

namespace ExStc = ExpLib::Struct;
namespace ExUtl = ExpLib::Utility;
namespace ExCos = ExpLib::Construct;

struct FrontierDetection::mapStruct{
    nav_msgs::MapMetaData info;
    std::vector<std::vector<int8_t>> source;
    std::vector<std::vector<int8_t>> horizon;
    std::vector<std::vector<int8_t>> frontierMap;
    mapStruct(const nav_msgs::OccupancyGrid& m);
};

FrontierDetection::mapStruct::mapStruct(const nav_msgs::OccupancyGrid& m)
    :source(ExUtl::mapArray1dTo2d(m.data,m.info))
    ,horizon(m.info.width,std::vector<int8_t>(m.info.height,0))
    ,frontierMap(m.info.width,std::vector<int8_t>(m.info.height,0)){
    info = m.info;
}

struct FrontierDetection::clusterStruct{
    std::vector<Eigen::Vector2i> index;
    std::vector<int> isObstacle;
    std::vector<double> areas;
    std::vector<Eigen::Vector2d> variances;
    std::vector<double> covariance;
    std::vector<pcl::PointIndices> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

    clusterStruct();
    clusterStruct(const clusterStruct& cs);
    void reserve(int size);
};
FrontierDetection::clusterStruct::clusterStruct():pc(new pcl::PointCloud<pcl::PointXYZ>){}
FrontierDetection::clusterStruct::clusterStruct(const clusterStruct& cs)
    :index(cs.index)
    ,isObstacle(cs.isObstacle)
    ,areas(cs.areas)
    ,variances(cs.variances)
    ,covariance(cs.covariance)
    ,indices(cs.indices)
    ,pc(cs.pc){}
        
void FrontierDetection::clusterStruct::reserve(int size){
    index.reserve(size);
    isObstacle.reserve(size);
    areas.reserve(size);
    variances.reserve(size);
    covariance.reserve(size);
}

FrontierDetection::FrontierDetection()
    :map_(new ExStc::subStructSimple("map", 1, &FrontierDetection::mapCB, this))
    ,frontier_(new ExStc::pubStruct<exploration_msgs::FrontierArray>("frontier",1,true))
    ,horizon_(new ExStc::pubStruct<sensor_msgs::PointCloud2>("horizon",1,true))
    ,drs_(new dynamic_reconfigure::Server<exploration_support::frontier_detection_parameter_reconfigureConfig>(ros::NodeHandle("~/frontier"))){
    loadParams();
    drs_->setCallback(boost::bind(&FrontierDetection::dynamicParamsCB,this, _1, _2));
}

FrontierDetection::~FrontierDetection(){
    if(OUTPUT_FRONTIER_PARAMETERS) outputParams();
}

void FrontierDetection::mapCB(const nav_msgs::OccupancyGridConstPtr& msg){
    // map の取り込み
    mapStruct map(*msg);
    horizonDetection(map);

    clusterStruct cluster(clusterDetection(map));
    obstacleFilter(map,cluster);

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
        // if(cluster.index[i].z() == 0) continue;
        if(cluster.isObstacle[i] == 0) continue;
        frontiers.emplace_back(ExCos::msgFrontier(ExUtl::mapIndexToCoordinate(cluster.index[i].x(),cluster.index[i].y(),map.info),cluster.areas[i],ExCos::msgVector(cluster.variances[i].x(),cluster.variances[i].y()),cluster.covariance[i]));
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
            if(map.horizon[x][y] == 1) points.emplace_back(ExUtl::mapIndexToCoordinate(x,y,map.info));
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
        cs.index.emplace_back(ExUtl::coordinateToMapIndex(std::move(centroid),map.info));
        cs.isObstacle.emplace_back(1);
        Eigen::Vector2d diff(max-min);
        cs.areas.emplace_back(std::abs(diff.x()*diff.y()));
    }

    return cs;
}

void FrontierDetection::obstacleFilter(FrontierDetection::mapStruct& map,clusterStruct& cs){
    ROS_INFO_STREAM("Obstacle Filter");
    
    //add obstacle cell
    for(int x=0,ex=map.info.width;x!=ex;++x){
        for(int y=0,ey=map.info.height;y!=ey;++y){
            if(map.source[x][y] == 100) map.frontierMap[x][y] = 100;
        }
    }

    for(int i=0,ie=cs.index.size();i!=ie;++i){
        if(map.frontierMap[cs.index[i].x()][cs.index[i].y()] == 100){
            cs.isObstacle[i] = 0;
            continue;
        }
        ExStc::mapSearchWindow msw(cs.index[i].x(),cs.index[i].y(),map.info.width,map.info.height,FILTER_SQUARE_DIAMETER);
        for(int y=msw.top,ey=msw.bottom+1;y!=ey;++y){
            for(int x=msw.left,ex=msw.right+1;x!=ex;++x){
                if(map.frontierMap[x][y] == 100){//障害部があったら終了
                    map.frontierMap[cs.index[i].x()][cs.index[i].y()] = 0;
                    cs.isObstacle[i] = 0;
                    x = ex -1;//ラムダで関数作ってreturnで終わっても良いかも
                    y = ey -1;
                }
            }
        }
    }
    ROS_INFO_STREAM("Obstacle Filter complete");
}

void FrontierDetection::publishHorizon(const clusterStruct& cs, const std::string& frameId){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorCloud->points.reserve(cs.pc->points.size());
    float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};
    int i=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cs.indices.begin (); it != cs.indices.end (); ++it,++i){
        // if(cs.index[i].z()==0) continue;
        if(cs.isObstacle[i]==0) continue;
        int c = i%12;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            colorCloud -> points.emplace_back(ExCos::pclXYZRGB(cs.pc->points[*pit].x,cs.pc->points[*pit].y,0.0f,colors[c][0],colors[c][1],colors[c][2]));
        }
    }
    colorCloud -> width = colorCloud -> points.size();
    colorCloud -> height = 1;
    colorCloud -> is_dense = true;
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*colorCloud,msg);
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    horizon_->pub.publish(msg);
	ROS_INFO_STREAM("Publish horizon");
}

void FrontierDetection::publishFrontier(const std::vector<exploration_msgs::Frontier>& frontiers, const std::string& frameId){
    exploration_msgs::FrontierArray msg;
    msg.frontiers = frontiers;
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    frontier_->pub.publish(msg);
    ROS_INFO_STREAM("Publish frontier");
}

void FrontierDetection::loadParams(void){
    ros::NodeHandle nh("~/frontier");
    // dynamic parameters
    nh.param<double>("cluster_tolerance", CLUSTER_TOLERANCE, 0.15);
    nh.param<int>("min_cluster_size", MIN_CLUSTER_SIZE, 30);
    nh.param<int>("max_cluster_size", MAX_CLUSTER_SIZE, 15000);
    nh.param<float>("filter_square_diameter", FILTER_SQUARE_DIAMETER, 0.75);
    // static parameters
    nh.param<std::string>("frontier_parameter_file_path",FRONTIER_PARAMETER_FILE_PATH,"frontier_last_parameters.yaml");
    nh.param<bool>("output_frontier_parameters",OUTPUT_FRONTIER_PARAMETERS,true);
}

void FrontierDetection::dynamicParamsCB(exploration_support::frontier_detection_parameter_reconfigureConfig &cfg, uint32_t level){
    CLUSTER_TOLERANCE = cfg.cluster_tolerance;
    MIN_CLUSTER_SIZE = cfg.min_cluster_size;
    MAX_CLUSTER_SIZE = cfg.max_cluster_size;
    FILTER_SQUARE_DIAMETER = cfg.filter_square_diameter;
}

void FrontierDetection::outputParams(void){
    std::cout << "writing frontier last parameters ... ..." << std::endl;
    std::ofstream ofs(FRONTIER_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "frontier param file open succeeded" << std::endl;
    else {
        std::cout << "frontier param file open failed" << std::endl;
        return;
    }

    ofs << "cluster_tolerance: " << CLUSTER_TOLERANCE << std::endl;
    ofs << "min_cluster_size: " << MIN_CLUSTER_SIZE << std::endl;
    ofs << "max_cluster_size: " << MAX_CLUSTER_SIZE << std::endl;
    ofs << "filter_square_diameter: " << FILTER_SQUARE_DIAMETER << std::endl;
 }