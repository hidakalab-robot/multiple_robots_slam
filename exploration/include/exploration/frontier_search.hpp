#ifndef FRONTIER_SEARCH_HPP
#define FRONTIER_SEARCH_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <exploration_libraly/common_lib.hpp>
#include <std_msgs/Empty.h>
#include <exploration_msgs/Frontier.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>

#include <exploration_libraly/path_planning.hpp>
#include <navfn/navfn_ros.h>

/*
frontier_search tutorial

In source file

    #include <exploration/frontier_search.hpp>

        FrontierSearch fs;

    if you want to get frontier cluster coordinates

        fs.frontierDetection<"RETURN_TYPE">("BOOL_VALUE");

        RETURN_TYPE is function return value
            void
            std::vector<geometry_msgs::Point>
            std::vector<exploration_msgs::Frontier>

        BOOL_VALUE
            if  true(default value)
                publish goal array (exploration_msgs::PointArray and geometry_msgs::PoseArray)

            if false
                do not publish goal array

*/

class FrontierSearch
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
    bool OBSTACLE_FILTER;
    double PREVIOUS_GOAL_THRESHOLD;
    double DISTANCE_WEIGHT;
    double DIRECTION_WEIGHT;
    std::string MAP_FRAME_ID;
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;
    bool PREVIOUS_GOAL_EFFECT;
    bool USE_MERGE_MAP;
    bool COLOR_CLUSTER;

    CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;
    CommonLib::subStruct<nav_msgs::OccupancyGrid> map_;

    CommonLib::pubStruct<geometry_msgs::PointStamped> goal_;
    CommonLib::pubStruct<exploration_msgs::PointArray> goalArray_;
    CommonLib::pubStruct<geometry_msgs::PoseArray> goalPoseArray_;
    CommonLib::pubStruct<sensor_msgs::PointCloud2> colorCloud_;
    
    void horizonDetection(mapStruct& map);
    clusterStruct clusterDetection(const mapStruct& map);
    void obstacleFilter(mapStruct& map,std::vector<Eigen::Vector3i>& index);
    geometry_msgs::Point arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info);
    Eigen::Vector3i coordinateToArray(double x,double y,const nav_msgs::MapMetaData& info);
    Eigen::Vector3i coordinateToArray(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info);
    bool selectGoal(const std::vector<geometry_msgs::Point>& goals, const geometry_msgs::Pose& pose, geometry_msgs::Point& goal);
    void publishGoal(const geometry_msgs::Point& goal);
	void publishGoalArray(const std::vector<geometry_msgs::Point>& goals);
    void publishGoalArrayAsPose(const std::vector<geometry_msgs::Point>& goals);
    void publishColorCluster(const clusterStruct& cs);
    std::vector<geometry_msgs::Point> frontiersToPoints(const std::vector<exploration_msgs::Frontier>& fa);
    void frontierDetection(const nav_msgs::OccupancyGrid& mapMsg, std::vector<exploration_msgs::Frontier>& frontiers);
    template<typename T> T frontierDetection(const std::vector<exploration_msgs::Frontier>& frontiers, const std::vector<geometry_msgs::Point>& goals);

public:
    FrontierSearch();

    bool getGoal(geometry_msgs::PointStamped& goal);//publish goalList and select goal
    template<typename T> T frontierDetection(bool visualizeGoalArray=true);//return void or std::vector<geometry_msgs::Point> or std::vector<exploration_msgs::Frontier>
    template<typename T> T frontierDetection(const nav_msgs::OccupancyGrid& mapMsg, bool visualizeGoalArray=true);//return void or std::vector<geometry_msgs::Point> or std::vector<exploration_msgs::Frontier>
};

FrontierSearch::FrontierSearch()
    :map_("map",1)
    ,pose_("pose",1)
    ,goal_("goal",1,true)
    ,goalArray_("goal_array",1,true)
    ,goalPoseArray_("goal_pose_array",1,true)
    ,colorCloud_("horizon_cluster/color",1){

    ros::NodeHandle p("~");
    p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    p.param<float>("filter_square_diameter", FILTER_SQUARE_DIAMETER, 0.4);
    p.param<bool>("obstacle_filter", OBSTACLE_FILTER, true);
    p.param<double>("distance_weight", DISTANCE_WEIGHT, 1.0);
    p.param<double>("direction_weight", DIRECTION_WEIGHT, 2.0);
    p.param<bool>("previous_goal_effect", PREVIOUS_GOAL_EFFECT, true);
    p.param<double>("previous_goal_threshold", PREVIOUS_GOAL_THRESHOLD, 1.0);
    p.param<double>("cluster_tolerance", CLUSTER_TOLERANCE, 0.15);
    p.param<int>("min_cluster_size", MIN_CLUSTER_SIZE, 50);
    p.param<int>("max_cluster_size", MAX_CLUSTER_SIZE, 15000);
    p.param<bool>("color_cluster", COLOR_CLUSTER, true);
}

void FrontierSearch::frontierDetection(const nav_msgs::OccupancyGrid& mapMsg, std::vector<exploration_msgs::Frontier>& frontiers){    
    mapStruct map(mapMsg);
    horizonDetection(map);
    clusterStruct cluster(clusterDetection(map));

    if(OBSTACLE_FILTER) obstacleFilter(map,cluster.index);

    if(cluster.index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found");
        return;
    }
    //ここでクラスタ表示したい
    if(COLOR_CLUSTER) publishColorCluster(cluster);
    frontiers.reserve(cluster.index.size());

    for(int i=0,e=cluster.index.size();i!=e;++i){
        if(cluster.index[i].z() == 0) continue;
        frontiers.emplace_back(CommonLib::msgFrontier(arrayToCoordinate(cluster.index[i].x(),cluster.index[i].y(),map.info),cluster.areas[i],CommonLib::msgVector(cluster.variances[i].x(),cluster.variances[i].y()),cluster.covariance[i]));
    }

    ROS_INFO_STREAM("Frontier Found : " << frontiers.size());
}

template<> int FrontierSearch::frontierDetection(const std::vector<exploration_msgs::Frontier>& frontiers, const std::vector<geometry_msgs::Point>& goals){
    return frontiers.size();
}

template<> std::vector<geometry_msgs::Point> FrontierSearch::frontierDetection(const std::vector<exploration_msgs::Frontier>& frontiers, const std::vector<geometry_msgs::Point>& goals){
    return goals;
}

template<> std::vector<exploration_msgs::Frontier> FrontierSearch::frontierDetection(const std::vector<exploration_msgs::Frontier>& frontiers, const std::vector<geometry_msgs::Point>& goals){
    return frontiers;
}

template<> void FrontierSearch::frontierDetection(const std::vector<exploration_msgs::Frontier>& frontiers, const std::vector<geometry_msgs::Point>& goals){
    return;
}

template<typename T>
T FrontierSearch::frontierDetection(const nav_msgs::OccupancyGrid& mapMsg, bool visualizeGoalArray){
    std::vector<exploration_msgs::Frontier> frontiers;
    frontierDetection(mapMsg,frontiers);
    std::vector<geometry_msgs::Point> goals(frontiersToPoints(frontiers));
    if(visualizeGoalArray){
        publishGoalArray(goals);
        publishGoalArrayAsPose(goals);
    }
    return frontierDetection<T>(frontiers, goals);
}

template<typename T>
T FrontierSearch::frontierDetection(bool visualizeGoalArray){
    if(map_.q.callOne(ros::WallDuration(1))) return T();
    return frontierDetection<T>(map_.data,visualizeGoalArray);
}

template<> int FrontierSearch::frontierDetection(bool visualizeGoalArray){
    if(map_.q.callOne(ros::WallDuration(1))) return -1;
    return frontierDetection<int>(map_.data,visualizeGoalArray);
}

bool FrontierSearch::getGoal(geometry_msgs::PointStamped& goal){
    std::vector<geometry_msgs::Point> goals(frontierDetection<std::vector<geometry_msgs::Point>>());

    if(goals.size()==0){
        ROS_INFO_STREAM("Frontier is Not Found !!");
        return false;
    };

    if(pose_.q.callOne(ros::WallDuration(1))) return false;

    if(selectGoal(goals,pose_.data.pose,goal.point)){
        ROS_INFO_STREAM("Selected Frontier : (" << goal.point.x << "," << goal.point.y << ")");
        publishGoal(goal.point);
        goal.header.frame_id = MAP_FRAME_ID;
        return true;
    }
    else{
		ROS_INFO_STREAM("Found Frontier is Too Close");
        return false;
    }
}

std::vector<geometry_msgs::Point> FrontierSearch::frontiersToPoints(const std::vector<exploration_msgs::Frontier>& frontiers){
    std::vector<geometry_msgs::Point> p;
    p.reserve(frontiers.size());
    for(const auto& f : frontiers) p.emplace_back(f.coordinate);
    return p;
}

void FrontierSearch::horizonDetection(mapStruct& map){
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


FrontierSearch::clusterStruct FrontierSearch::clusterDetection(const mapStruct& map){
    
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

Eigen::Vector3i FrontierSearch::coordinateToArray(double x,double y,const nav_msgs::MapMetaData& info){
    return Eigen::Vector3i((x-info.origin.position.x)/info.resolution,(y-info.origin.position.y)/info.resolution,1);
}

Eigen::Vector3i FrontierSearch::coordinateToArray(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info){
    return Eigen::Vector3i((coordinate.x()-info.origin.position.x)/info.resolution,(coordinate.y()-info.origin.position.y)/info.resolution,1);
}

void FrontierSearch::obstacleFilter(FrontierSearch::mapStruct& map,std::vector<Eigen::Vector3i>& index){
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

geometry_msgs::Point FrontierSearch::arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info){
    return CommonLib::msgPoint(info.resolution * indexX + info.origin.position.x,info.resolution * indexY + info.origin.position.y);
}

bool FrontierSearch::selectGoal(const std::vector<geometry_msgs::Point>& goals, const geometry_msgs::Pose& pose,geometry_msgs::Point& goal){
    //現在位置からそれぞれのフロンティア座標に対して距離とベクトルを計算し、評価関数によって目標を決定
    //前回の目標から近いところは目標に取らないようにする
    static geometry_msgs::Point previousGoal;

    //ロボットの向きのベクトル(大きさ1)を計算
    double yaw = CommonLib::qToYaw(pose.orientation);
    Eigen::Vector2d directionVec(cos(yaw),sin(yaw));

    //変更点：前回の移動方向では無く現在のロボットの向きで評価する

    double max = -DBL_MAX;
    for(auto& g : goals){
        if(PREVIOUS_GOAL_EFFECT && goals.size() > 1 && sqrt(pow(g.x - previousGoal.x,2)+pow(g.y - previousGoal.y,2)) <= PREVIOUS_GOAL_THRESHOLD) continue;
        Eigen::Vector2d vec(g.x - pose.position.x,g.y - pose.position.y);
        //評価値が最大となる目標値を選択
        double value = DIRECTION_WEIGHT * vec.normalized().dot(directionVec) - DISTANCE_WEIGHT * vec.norm();
        if(value > max){
            max = std::move(value);
            goal = g;
        }
    }

    if(max > -DBL_MAX){
        previousGoal = goal;
        return true;
    }
    else{
        return false;
    }
}

void FrontierSearch::publishGoal(const geometry_msgs::Point& goal){
    geometry_msgs::PointStamped msg;

	msg.point = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;

    goal_.pub.publish(msg);
	ROS_INFO_STREAM("Publish Goal");
}

void FrontierSearch::publishGoalArray(const std::vector<geometry_msgs::Point>& goals){
    exploration_msgs::PointArray msg;
    
	msg.points = goals;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;

    goalArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList");
}

void FrontierSearch::publishGoalArrayAsPose(const std::vector<geometry_msgs::Point>& goals){
    geometry_msgs::PoseArray msg;
    msg.poses.reserve(goals.size());

    for(const auto& g : goals) msg.poses.emplace_back(CommonLib::pointToPose(g));

	msg.header.frame_id = MAP_FRAME_ID;
    msg.header.stamp = ros::Time::now();

    goalPoseArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList PoseArray");
}

void FrontierSearch::publishColorCluster(const clusterStruct& cs){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorCloud->points.reserve(cs.pc->points.size());
    float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};
    int i=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cs.indices.begin (); it != cs.indices.end (); ++it,++i){
        if(cs.index[i].z()==0) continue;
        int c = i%12;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            colorCloud -> points.emplace_back(CommonLib::pclXYZRGB(cs.pc->points[*pit].x,cs.pc->points[*pit].y,0.0f,colors[c][0],colors[c][1],colors[c][2]));
        }
    }
    colorCloud -> width = colorCloud -> points.size();
    colorCloud -> height = 1;
    colorCloud -> is_dense = true;
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*colorCloud,msg);
    msg.header.frame_id = MAP_FRAME_ID;
    msg.header.stamp = ros::Time::now();
    colorCloud_.pub.publish(msg);
	ROS_INFO_STREAM("Publish ColorCluster");
}

#endif //FRONTIER_SEARCH_HPP