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
#include <exploration/common_lib.hpp>
#include <std_msgs/Empty.h>
#include <exploration_msgs/Frontier.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>



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

        clusterStruct(int size){
            index.reserve(size);
            areas.reserve(size);
            variances.reserve(size);
        };
        clusterStruct(const std::vector<Eigen::Vector3i>& i,const std::vector<double>& a, const std::vector<Eigen::Vector2d>& v)
            :index(i)
            ,areas(a)
            ,variances(v){};

        clusterStruct(const clusterStruct& cs)
            :index(cs.index)
            ,areas(cs.areas)
            ,variances(cs.variances){};
    };

    ros::NodeHandle p;

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
    std::string MERGE_MAP_FRAME_ID;
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
    void mergeMapCoordinateToLocal(std::vector<exploration_msgs::Frontier>& goal);
    std::vector<geometry_msgs::Point> frontiersToPoints(const std::vector<exploration_msgs::Frontier>& fa);

public:
    FrontierSearch();

    bool getGoal(geometry_msgs::Point& goal);//publish goalList and select goal
    template<typename T> T frontierDetection(bool visualize=true);

    double sumFrontierAngle(const geometry_msgs::Point& origin,const Eigen::Vector2d& vec,const std::vector<exploration_msgs::Frontier>& frontiers);
    double sumFrontierAngle(const geometry_msgs::Pose& pose,double forward,const std::vector<exploration_msgs::Frontier>& frontiers);

};

FrontierSearch::FrontierSearch()
    :p("~")
    ,map_("map",1)
    ,pose_("pose",1)
    ,goal_("goal",1,true)
    ,goalArray_("goal_array",1,true)
    ,goalPoseArray_("goal_pose_array",1,true)
    ,colorCloud_("horizon_cluster/color",1){

    p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    p.param<float>("filter_square_diameter", FILTER_SQUARE_DIAMETER, 0.4);
    p.param<bool>("obstacle_filter", OBSTACLE_FILTER, true);
    p.param<double>("distance_weight", DISTANCE_WEIGHT, 1.0);
    p.param<double>("direction_weight", DIRECTION_WEIGHT, 2.0);
    p.param<bool>("previous_goal_effect", PREVIOUS_GOAL_EFFECT, true);
    p.param<double>("previous_goal_threshold", PREVIOUS_GOAL_THRESHOLD, 1.0);
    p.param<double>("cluster_tolerance", CLUSTER_TOLERANCE, 0.3);
    p.param<int>("min_cluster_size", MIN_CLUSTER_SIZE, 50);
    p.param<int>("max_cluster_size", MAX_CLUSTER_SIZE, 15000);
    p.param<bool>("use_merge_map", USE_MERGE_MAP, false);
    p.param<std::string>("merge_map_frame_id", MERGE_MAP_FRAME_ID, "merge_map");
    p.param<bool>("color_cluster", COLOR_CLUSTER, true);
}

double FrontierSearch::sumFrontierAngle(const geometry_msgs::Pose& pose, double forward,const std::vector<exploration_msgs::Frontier>& frontiers){
    //前向きのベクトルを自動生成
    double yaw = CommonLib::qToYaw(pose.orientation);
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);
    return sumFrontierAngle(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw),frontiers);
}

double FrontierSearch::sumFrontierAngle(const geometry_msgs::Point& origin,const Eigen::Vector2d& vec,const std::vector<exploration_msgs::Frontier>& frontiers){
    double sum = 0;
    //frontierの大きさで重みをつけたい
    for(const auto& frontier : frontiers){
        sum += (frontier.variance.x>frontier.variance.y)? std::abs(acos(vec.dot(Eigen::Vector2d(frontier.coordinate.x - origin.x,frontier.coordinate.y - origin.y).normalized())))*frontier.variance.x*frontier.variance.x
                                                        : std::abs(acos(vec.dot(Eigen::Vector2d(frontier.coordinate.x - origin.x,frontier.coordinate.y - origin.y).normalized())))*frontier.variance.y*frontier.variance.y;
    }

    ROS_DEBUG_STREAM("position : (" << origin.x << "," << origin.y << "), sum : " << sum << "\n");

    return sum;
}

template<> std::vector<geometry_msgs::Point> FrontierSearch::frontierDetection(bool publish){
    map_.q.callOne(ros::WallDuration(1));
    
    mapStruct map(map_.data);

    ROS_DEBUG_STREAM("map size : " << map.info.height << " X " << map.info.width << "\n");

    horizonDetection(map);

    clusterStruct cluster(clusterDetection(map));

    if(OBSTACLE_FILTER){
        obstacleFilter(map,cluster.index);
    }
    
    if(cluster.index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found\n");
        return std::vector<geometry_msgs::Point>();
    }
    
    std::vector<exploration_msgs::Frontier> frontiers;
    frontiers.reserve(cluster.index.size());

    for(int i=0,e=cluster.index.size();i!=e;++i){
        if(cluster.index[i].z() == 0){
            continue;
        }
        frontiers.emplace_back(CommonLib::msgFrontier(arrayToCoordinate(cluster.index[i].x(),cluster.index[i].y(),map.info),cluster.areas[i],CommonLib::msgPoint(cluster.variances[i].x(),cluster.variances[i].y())));
    }

    ROS_INFO_STREAM("Frontier Found : " << frontiers.size() << "\n");

    if(USE_MERGE_MAP){
        mergeMapCoordinateToLocal(frontiers);
    }

    std::vector<geometry_msgs::Point> goals(frontiersToPoints(frontiers));

    if(publish){
        publishGoalArray(goals);
        publishGoalArrayAsPose(goals);
    }
    
    return goals;
}

template<> std::vector<exploration_msgs::Frontier> FrontierSearch::frontierDetection(bool publish){
    map_.q.callOne(ros::WallDuration(1));
    mapStruct map(map_.data);

    ROS_DEBUG_STREAM("map size : " << map.info.height << " X " << map.info.width << "\n");

    horizonDetection(map);

    clusterStruct cluster(clusterDetection(map));

    if(OBSTACLE_FILTER){
        obstacleFilter(map,cluster.index);
    }
    
    if(cluster.index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found\n");
        return std::vector<exploration_msgs::Frontier>();
    }
    
    std::vector<exploration_msgs::Frontier> frontiers;
    frontiers.reserve(cluster.index.size());

    for(int i=0,e=cluster.index.size();i!=e;++i){
        if(cluster.index[i].z() == 0){
            continue;
        }
        frontiers.emplace_back(CommonLib::msgFrontier(arrayToCoordinate(cluster.index[i].x(),cluster.index[i].y(),map.info),cluster.areas[i],CommonLib::msgPoint(cluster.variances[i].x(),cluster.variances[i].y())));
    }

    ROS_INFO_STREAM("Frontier Found : " << frontiers.size() << "\n");

    if(USE_MERGE_MAP){
        mergeMapCoordinateToLocal(frontiers);
    }

    if(publish){
        std::vector<geometry_msgs::Point> goals(frontiersToPoints(frontiers));
        publishGoalArray(goals);
        publishGoalArrayAsPose(goals);
    }
    
    return frontiers;
}

bool FrontierSearch::getGoal(geometry_msgs::Point& goal){
    std::vector<geometry_msgs::Point> goals(frontierDetection<std::vector<geometry_msgs::Point>>());

    if(goals.size()==0){
        return false;
    }

    pose_.q.callOne(ros::WallDuration(1));

    if(selectGoal(goals,pose_.data.pose,goal)){
        ROS_INFO_STREAM("Selected Frontier : (" << goal.x << "," << goal.y << ")\n");
        publishGoal(goal);
        return true;
    }
    else{
		ROS_INFO_STREAM("Found Frontier is Too Close\n");
        return false;
    }
}

std::vector<geometry_msgs::Point> FrontierSearch::frontiersToPoints(const std::vector<exploration_msgs::Frontier>& frontiers){
    std::vector<geometry_msgs::Point> p;
    p.reserve(frontiers.size());
    for(const auto& f : frontiers){
        p.emplace_back(f.coordinate);
    }
    return p;
}

void FrontierSearch::horizonDetection(mapStruct& map){
    ROS_INFO_STREAM("Horizon Detection\n");
    //x axis horizon
    for(int y=0,ey=map.info.height;y!=ey;++y){
        for(int x=0,ex=map.info.width-1;x!=ex;++x){
            if(map.source[x][y] == 0 && map.source[x+1][y] == -1){
                map.horizon[x][y] = 1;
            }
            else if(map.source[x][y] == -1 && map.source[x+1][y] == 0){
                map.horizon[x+1][y] = 1;	
            }
        }
    }

    //y axis horizon
    for(int x=0,ex=map.info.width;x!=ex;++x){
        for(int y=0,ey=map.info.height-1;y!=ey;++y){
            if(map.source[x][y] == 0 && map.source[x][y+1] == -1){
                map.horizon[x][y] = 1;
            }
            else if(map.source[x][y] == -1 && map.source[x][y+1] == 0){
                map.horizon[x][y+1] = 1;	
            }
        }
    }
    ROS_INFO_STREAM("Horizon Detection complete\n");
}


FrontierSearch::clusterStruct FrontierSearch::clusterDetection(const mapStruct& map){
    
    ROS_INFO_STREAM("Frontier Detection by Clustering\n");

    std::vector<geometry_msgs::Point> points;
    points.reserve(map.info.height*map.info.width);
    for(int y=0,ey=map.info.height;y!=ey;++y){
        for(int x=0,ex=map.info.width;x!=ex;++x){
            if(map.horizon[x][y] == 1){
                points.emplace_back(arrayToCoordinate(x,y,map.info));
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr horizonMap(new pcl::PointCloud<pcl::PointXYZ>);
    horizonMap -> points.reserve(points.size());

    for(const auto& point : points){
        horizonMap -> points.emplace_back(pcl::PointXYZ((float)point.x,(float)point.y,0.0f));
    }

    horizonMap -> width = horizonMap -> points.size();
    horizonMap -> height = 1;
    horizonMap -> is_dense = true;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (horizonMap);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  	ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);
	ec.setInputCloud (horizonMap);

    std::vector<pcl::PointIndices> indices;//クラスタリングした結果が格納される
	ec.extract (indices);

    //debug 用　カラーリング出力 スコープ
    if(COLOR_CLUSTER){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorMap(new pcl::PointCloud<pcl::PointXYZRGB>);

        float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};
        int i=0;
        for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it,++i){
            int c = i%12;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                colorMap -> points.emplace_back(CommonLib::pclXYZRGB(horizonMap->points[*pit].x,horizonMap->points[*pit].y,0.0f,colors[c][0],colors[c][1],colors[c][2]));
      	    }
  	    }
        colorMap -> width = colorMap -> points.size();
        colorMap -> height = 1;
        colorMap -> is_dense = true;
        
        sensor_msgs::PointCloud2 colorMsg;
        pcl::toROSMsg(*colorMap,colorMsg);
        colorMsg.header.frame_id = MAP_FRAME_ID;
        colorMsg.header.stamp = ros::Time::now();
        colorCloud_.pub.publish(colorMsg);
    }

    clusterStruct cs(indices.size());

    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){
        Eigen::Vector2d sum(0,0);
        Eigen::Vector2d max(-DBL_MAX,-DBL_MAX);
        Eigen::Vector2d min(DBL_MAX,DBL_MAX);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            sum.x() += horizonMap -> points[*pit].x;
            sum.y() += horizonMap -> points[*pit].y;

            if(horizonMap -> points[*pit].x > max.x()){
                max.x() = horizonMap -> points[*pit].x;
            }
            else if(horizonMap -> points[*pit].x < min.x()){
                min.x() = horizonMap -> points[*pit].x;
            }

            if(horizonMap -> points[*pit].y > max.y()){
                max.y() = horizonMap -> points[*pit].y;
            }
            else if(horizonMap -> points[*pit].y < min.y()){
                min.y() = horizonMap -> points[*pit].y;
            }
        }
        Eigen::Vector2d centroid(sum.x()/it->indices.size(),sum.y()/it->indices.size());
        Eigen::Vector2d deviation(0,0);
        //分散を求めたい
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            deviation.x() += (horizonMap -> points[*pit].x - centroid.x()) * (horizonMap -> points[*pit].x - centroid.x());
            deviation.y() += (horizonMap -> points[*pit].y - centroid.y()) * (horizonMap -> points[*pit].y - centroid.y());
        }

        cs.variances.emplace_back(deviation/std::distance(it->indices.begin(),it->indices.end()));
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
    ROS_INFO_STREAM("Obstacle Filter\n");
    
    //add obstacle cell
    for(int x=0,ex=map.info.width;x!=ex;++x){
        for(int y=0,ey=map.info.height;y!=ey;++y){
            if(map.source[x][y] == 100){
                map.frontierMap[x][y] = 100;
            }
        }
    }

    int FILTER_HALF_CELL = (FILTER_SQUARE_DIAMETER / map.info.resolution) / 2.0;
    //FILTER_HALF_CELL += FILTER_HALF_CELL % 2;//奇数ではダメな理由が不明

    if(FILTER_HALF_CELL < 1){
        ROS_ERROR_STREAM("FILTER_SQUARE_DIAMETER is Bad\n");
        return;
    }

    for(auto&& i : index){

        if(map.frontierMap[i.x()][i.y()] == 100){
            i.z() = 0;
            continue;
        }

        int LEFT = (i.x()-FILTER_HALF_CELL < 0) ? i.x() : FILTER_HALF_CELL;
        int RIGHT = (i.x()+FILTER_HALF_CELL > map.info.width-1) ? (map.info.width-1)-i.x() : FILTER_HALF_CELL;
        int TOP = (i.y()-FILTER_HALF_CELL < 0) ? i.y() : FILTER_HALF_CELL;
        int BOTTOM = (i.y()+FILTER_HALF_CELL > map.info.height-1) ? (map.info.height-1)-i.y() : FILTER_HALF_CELL;

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
    ROS_INFO_STREAM("Obstacle Filter complete\n");
}

geometry_msgs::Point FrontierSearch::arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info){
    return CommonLib::msgPoint(info.resolution * indexX + info.origin.position.x,info.resolution * indexY + info.origin.position.y);
}

void FrontierSearch::mergeMapCoordinateToLocal(std::vector<exploration_msgs::Frontier>& frontiers){
    //merge_map - map のtfを取得して座標修正
    static bool initialized = false;
    static tf::TransformListener listener;
    if(!initialized){
        listener.waitForTransform(MERGE_MAP_FRAME_ID, MAP_FRAME_ID, ros::Time(), ros::Duration(1.0));
        initialized = true;
    }
    
    tf::StampedTransform transform;
    listener.lookupTransform(MERGE_MAP_FRAME_ID, MAP_FRAME_ID, ros::Time(0), transform);

    double transYaw = CommonLib::qToYaw(transform.getRotation());
    double transX = transform.getOrigin().getX();
    double transY = transform.getOrigin().getY();
    
    ROS_DEBUG_STREAM(MAP_FRAME_ID << " -> " <<  MERGE_MAP_FRAME_ID << ": ( " << transX << "," << transY << "," << transYaw << " )\n");

    Eigen::Matrix2d rotation;
    rotation << cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw);

    for(auto& f : frontiers){
        Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(f.coordinate.x - transX,f.coordinate.y - transY));
        f.coordinate.x = tempPoint.x();
        f.coordinate.y = tempPoint.y();
    }
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
        if(PREVIOUS_GOAL_EFFECT && sqrt(pow(g.x - previousGoal.x,2)+pow(g.y - previousGoal.y,2)) <= PREVIOUS_GOAL_THRESHOLD){
            continue;
        }
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
	ROS_INFO_STREAM("Publish Goal\n");
}

void FrontierSearch::publishGoalArray(const std::vector<geometry_msgs::Point>& goals){
    exploration_msgs::PointArray msg;
    
	msg.points = goals;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;

    goalArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

void FrontierSearch::publishGoalArrayAsPose(const std::vector<geometry_msgs::Point>& goals){
    geometry_msgs::PoseArray msg;
    msg.poses.reserve(goals.size());

    for(const auto& goal : goals){
        msg.poses.emplace_back(CommonLib::pointToPose(goal));
    }

	msg.header.frame_id = MAP_FRAME_ID;
    msg.header.stamp = ros::Time::now();

    goalPoseArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList PoseArray\n");
}

#endif //FRONTIER_SEARCH_HPP