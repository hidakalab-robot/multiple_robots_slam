#ifndef FRONTIER_SEARCH_HPP
#define FRONTIER_SEARCH_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration_msgs/Goal.h>
#include <exploration_msgs/GoalList.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <exploration/common_lib.hpp>
#include <std_msgs/Empty.h>

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
            ,horizon(m.info.width,std::vector<int8_t>(m.info.height))
            ,frontierMap(m.info.width,std::vector<int8_t>(m.info.height)){
                
            info = m.info;
            for(int y=0,k=0,ey=info.height;y!=ey;++y){
                for(int x=0,ex=info.width;x!=ex;++x,++k){
                    source[x][y] = m.data[k];
                    horizon[x][y] = 0;
                    frontierMap[x][y] = 0;
                }
            }
        }
    };

    ros::NodeHandle p;

    float FRONTIER_DIAMETER_MIN;
    int FRONTIER_THICKNESS;
    int FRONTIER_DETECTION_METHOD;
    float FILTER_SQUARE_DIAMETER;
    bool OBSTACLE_FILTER;
    double DOUBLE_MINUS_INFINITY;
    double PREVIOUS_GOAL_THRESHOLD;
    double DISTANCE_WEIGHT;
    double DIRECTION_WEIGHT;
    std::string MAP_FRAME_ID;
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;
    bool PUBLISH_POSE_ARRAY;
    bool PREVIOUS_GOAL_EFFECT;
    bool USE_MERGE_MAP;
    std::string MERGE_MAP_FRAME_ID;
    bool COLOR_CLUSTER;

    CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;
    CommonLib::subStruct<nav_msgs::OccupancyGrid> map_;

    CommonLib::pubStruct<exploration_msgs::Goal> goal_;
    CommonLib::pubStruct<std_msgs::Empty> goalDel_;
    CommonLib::pubStruct<exploration_msgs::GoalList> goalList_;
    CommonLib::pubStruct<geometry_msgs::PoseArray> goalPoseArray_;
    CommonLib::pubStruct<std_msgs::Empty> goalListDel_;
    CommonLib::pubStruct<sensor_msgs::PointCloud2> colorCloud_;
    
    void horizonDetection(FrontierSearch::mapStruct& map);
    std::vector<Eigen::Vector2i> frontierDetectionByContinuity(FrontierSearch::mapStruct& map);
    std::vector<Eigen::Vector2i> frontierDetectionByClustering(const FrontierSearch::mapStruct& map);
    void obstacleFilter(FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i>& index);
    geometry_msgs::Point arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info);
    Eigen::Vector2i coordinateToArray(double x,double y,const nav_msgs::MapMetaData& info);

    bool selectGoal(const std::vector<geometry_msgs::Point>& goals, const geometry_msgs::Pose& pose, geometry_msgs::Point& goal);
    void publishGoal(const geometry_msgs::Point& goal);
	void publishGoalList(const std::vector<geometry_msgs::Point>& goals);
    void publishGoalListPoseArray(const std::vector<geometry_msgs::Point>& goals);

    void globalCoordinateToLocal(std::vector<geometry_msgs::Point>& goal);

public:
    FrontierSearch();

    bool getGoal(geometry_msgs::Point& goal);//publish goalList and select goal
    bool getGoal(void);//only publish goal list

};

FrontierSearch::FrontierSearch()
    :p("~")
    ,map_("map",1)
    ,pose_("pose",1)
    ,goal_("goal",1,true)
    ,goalList_("goal_list",1,true)
    ,goalDel_("goal/delete",1)
    ,goalListDel_("goal_list/delete",1)
    ,goalPoseArray_("goal_list_pose_array",1,true)
    ,colorCloud_("horizon_cluster/color",1)
    ,DOUBLE_MINUS_INFINITY(-10000000.0){

    p.param<bool>("publish_pose_array", PUBLISH_POSE_ARRAY, false);
    p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    p.param<float>("frontier_diameter_min", FRONTIER_DIAMETER_MIN, 0.4);
	p.param<int>("frontier_thickness", FRONTIER_THICKNESS, 3);
    FRONTIER_THICKNESS += (FRONTIER_THICKNESS+1)%2;//偶数封じ
    p.param<int>("frontier_detection_method", FRONTIER_DETECTION_METHOD, 0);
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

bool FrontierSearch::getGoal(void){
    map_.q.callOne(ros::WallDuration(1));
    goalDel_.pub.publish(CommonLib::msgEmpty());
    
    FrontierSearch::mapStruct map(map_.data);

    horizonDetection(map);

    std::vector<Eigen::Vector2i> index;
    switch (FRONTIER_DETECTION_METHOD){
        case 0:
            index = frontierDetectionByContinuity(map);
            break;
        case 1:
            index = frontierDetectionByClustering(map);
            break;
        default:
            ROS_ERROR_STREAM("Frontier Detection Method is Unknown\n");
            return false;
    }

    //ROS_DEBUG_STREAM("Before Filter index size : " << index.size() << "\n");
    if(OBSTACLE_FILTER){
        obstacleFilter(map,index);
    }
    
    //ROS_DEBUG_STREAM("After Filter Index Size : " << index.size() << "\n");
    if(index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found\n");
        goalListDel_.pub.publish(CommonLib::msgEmpty());
        return false;
    }
    
    std::vector<geometry_msgs::Point> goals;
    goals.reserve(index.size());

    for(const auto& i : index) {
        goals.push_back(arrayToCoordinate(i.x(),i.y(),map.info));
    }

    ROS_INFO_STREAM("Frontier Found : " << goals.size() << "\n");

    if(USE_MERGE_MAP){
        globalCoordinateToLocal(goals);
    }

    publishGoalList(goals);
    
    if(PUBLISH_POSE_ARRAY){
        //ROS_DEBUG_STREAM("call publish_pose_array\n");
        publishGoalListPoseArray(goals);
    }
    return true;
}

bool FrontierSearch::getGoal(geometry_msgs::Point& goal){
    map_.q.callOne(ros::WallDuration(1));
    goalDel_.pub.publish(CommonLib::msgEmpty());
    FrontierSearch::mapStruct map(map_.data);

    horizonDetection(map);

    std::vector<Eigen::Vector2i> index;

    switch (FRONTIER_DETECTION_METHOD){
        case 0:
            index = frontierDetectionByContinuity(map);
            break;
        case 1:
            index = frontierDetectionByClustering(map);
            break;
        default:
            ROS_ERROR_STREAM("Frontier Detection Method is Unknown\n");
            return false;
    }

    //ROS_DEBUG_STREAM("Before Filter index size : " << index.size() << "\n");

    if(OBSTACLE_FILTER){
        obstacleFilter(map,index);
    }
    
    //ROS_DEBUG_STREAM("After Filter Index Size : " << index.size() << "\n");

    if(index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found\n");
        goalListDel_.pub.publish(CommonLib::msgEmpty());
        return false;
    }
    
    std::vector<geometry_msgs::Point> goals;
    goals.reserve(index.size());

    for(const auto& i : index) {
        goals.push_back(arrayToCoordinate(i.x(),i.y(),map.info));
    }

    ROS_INFO_STREAM("Frontier Found : " << goals.size() << "\n");

    if(USE_MERGE_MAP){
        globalCoordinateToLocal(goals);
    }

    publishGoalList(goals);
    
    if(PUBLISH_POSE_ARRAY){
        //ROS_DEBUG_STREAM("call publish_pose_array\n");
        publishGoalListPoseArray(goals);
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

void FrontierSearch::horizonDetection(FrontierSearch::mapStruct& map){
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

std::vector<Eigen::Vector2i> FrontierSearch::frontierDetectionByContinuity(FrontierSearch::mapStruct& map){
    ROS_INFO_STREAM("Frontier Detection by Continuity\n");
    
    const int FRONTIER_EDGE_RENGE = FRONTIER_THICKNESS / 2;

    //ROS_DEBUG_STREAM("FRONTIER_EDGE_RENGE : " << FRONTIER_EDGE_RENGE << "\n");
    //ROS_DEBUG_STREAM("ROBOT_CELLSIZE : " << (int)(FRONTIER_DIAMETER_MIN / resolution) << "\n");

    std::vector<Eigen::Vector2i> index;
    index.reserve(2*map.info.width*map.info.height/FRONTIER_THICKNESS);
    
    
    int continuity = 0;
    int sum = 0;

    // x axis continuity
    int startX = 0;
    int endX = 0;
    for(int y=FRONTIER_EDGE_RENGE,ey=map.info.height-FRONTIER_EDGE_RENGE;y<ey;y+=FRONTIER_THICKNESS){
        for(int ex=map.info.width,countX=0;countX<ex && ros::ok();){
			for(int j=-FRONTIER_EDGE_RENGE,ej=FRONTIER_EDGE_RENGE+1;j!=ej;++j){
				sum += map.horizon[countX][y+j];
			}
			if(sum > 0){
				startX = countX;
                continuity = 0;
				while(sum > 0 && ros::ok()){
					sum = 0;
					continuity++;
                    if(countX++ < map.info.width){// インクリメントする前のxで比較したい
                        for(int j=-FRONTIER_EDGE_RENGE,ej=FRONTIER_EDGE_RENGE+1;j!=ej;++j){
							sum += map.horizon[countX][y+j];
						}
					}
					else{
						break;
					}
				}
				endX = countX-1;
                if(continuity >= (int)(FRONTIER_DIAMETER_MIN / map.info.resolution)){
                    Eigen::Vector2i temp((int)((startX + endX)/2),y);
					map.frontierMap[temp.x()][temp.y()] = 1;
                    index.push_back(std::move(temp));
				}
			}
			else{			
				countX++;
			}
		}
    }

    // y axis continuity
    continuity = 0;
    sum = 0;
    int startY = 0;
    int endY = 0;
    for(int x=FRONTIER_EDGE_RENGE,ex=map.info.width-FRONTIER_EDGE_RENGE;x<ex;x+=FRONTIER_THICKNESS){
        for(int ey=map.info.height,countY=0;countY<ey && ros::ok();){
            for(int i=-FRONTIER_EDGE_RENGE,ei=FRONTIER_EDGE_RENGE+1;i!=ei;++i){
				sum += map.horizon[x+i][countY];
			}
			if(sum > 0){
				startY = countY;
                continuity = 0;
				while(sum > 0 && ros::ok()){
					sum = 0;
					continuity++;
                    if(countY++ < map.info.height){//インクリメントする前のyで比較したい
						for(int i=-FRONTIER_EDGE_RENGE,ei=FRONTIER_EDGE_RENGE+1;i!=ei;++i){
							sum += map.horizon[x+i][countY];
						}
					}
					else{
						break;
					}
				}
				endY = countY-1;
                if(continuity >= (int)(FRONTIER_DIAMETER_MIN / map.info.resolution)){
                    Eigen::Vector2i temp(x,(int)((startY + endY)/2));
					map.frontierMap[temp.x()][temp.y()] = 1;
                    index.push_back(std::move(temp));
				}
			}
			else{			
				countY++;
			}
		}
    }

    ROS_INFO_STREAM("Frontier Detection Complete\n");

    return index;
}

std::vector<Eigen::Vector2i> FrontierSearch::frontierDetectionByClustering(const FrontierSearch::mapStruct& map){
    
    ROS_INFO_STREAM("Frontier Detection by Clustering\n");

    //ROS_DEBUG_STREAM("mapStruct to tempVector\n");

    std::vector<geometry_msgs::Point> points;
    points.reserve(map.info.height*map.info.width);
    for(int y=0,ey=map.info.height;y!=ey;++y){
        for(int x=0,ex=map.info.width;x!=ex;++x){
            if(map.horizon[x][y] == 1){
                points.emplace_back(arrayToCoordinate(x,y,map.info));
            }
        }
    }

    //ROS_DEBUG_STREAM("geometry_msgs::Point to pointcloud\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr horizonMap(new pcl::PointCloud<pcl::PointXYZ>);
    horizonMap -> points.reserve(points.size());

    for(const auto& point : points){
        horizonMap -> points.emplace_back(pcl::PointXYZ((float)point.x,(float)point.y,0.0f));
    }

    horizonMap -> width = horizonMap -> points.size();
    horizonMap -> height = 1;
    horizonMap -> is_dense = true;

    //ROS_DEBUG_STREAM("clustering\n");
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
        //ROS_DEBUG_STREAM("coloring\n");
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

    //ROS_DEBUG_STREAM("Centroids Calculation\n");
    std::vector<Eigen::Vector2d> centroids;
    centroids.reserve(indices.size());
    {
        for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){
            Eigen::Vector2d sum(0,0);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                sum.x() += horizonMap -> points[*pit].x;
                sum.y() += horizonMap -> points[*pit].y;
            }
            centroids.emplace_back(Eigen::Vector2d(sum.x()/it->indices.size(),sum.y()/it->indices.size()));
        }
    }

    //xの分散とyの分散が両方小さかったらそのクラスタは削除
    
    //ROS_DEBUG_STREAM("centroids to index array\n");
    std::vector<Eigen::Vector2i> index;
    index.reserve(centroids.size());
    for(const auto& centroid : centroids){
        index.emplace_back(coordinateToArray(centroid.x(),centroid.y(),map.info));
    }
    return index;
}

Eigen::Vector2i FrontierSearch::coordinateToArray(double x,double y,const nav_msgs::MapMetaData& info){
    return Eigen::Vector2i((x-info.origin.position.x)/info.resolution,(y-info.origin.position.y)/info.resolution);
}

void FrontierSearch::obstacleFilter(FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i>& index){
    ROS_INFO_STREAM("Obstacle Filter\n");
    
    //add obstacle cell
    for(int x=0,ex=map.info.width;x!=ex;++x){
        for(int y=0,ey=map.info.height;y!=ey;++y){
            if((int)map.source[x][y] == 100){
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

    int RIGHT,LEFT;
    int TOP, BOTTOM;
    std::vector<Eigen::Vector2i> filteredIndex;
    filteredIndex.reserve(index.size());

    for(const auto& i : index){
        int sum = 0;
        //left shape
        if(i.x()-FILTER_HALF_CELL < 0){
			LEFT = i.x();
		}
		else{
			LEFT = FILTER_HALF_CELL;
		}
        //right shape
		if(i.x()+FILTER_HALF_CELL > map.info.width-1){
			RIGHT = (map.info.width-1)-i.x();
		}
		else{
			RIGHT  = FILTER_HALF_CELL;
		}
        //top shape
		if(i.y()-FILTER_HALF_CELL < 0){
			TOP = i.y();
		}
		else{
			TOP = FILTER_HALF_CELL;
		}
        //bottom shape
		if(i.y()+FILTER_HALF_CELL > map.info.height-1){
			BOTTOM = (map.info.height-1)-i.y();
		}
		else{
			BOTTOM = FILTER_HALF_CELL;
		}
        for(int y=i.y()-TOP,ey=i.y()+BOTTOM+1;y!=ey;++y){
            for(int x=i.x()-LEFT,ex=i.x()+RIGHT+1;x!=ex;++x){
                sum += (int)map.frontierMap[x][y];
            }
        }
        if(sum>100){
            map.frontierMap[i.x()][i.y()] = 0;
        }
        else{
            filteredIndex.push_back(i);
        }
    }
    index = filteredIndex;
    ROS_INFO_STREAM("Obstacle Filter complete\n");
}

geometry_msgs::Point FrontierSearch::arrayToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info){
    return CommonLib::msgPoint(info.resolution * indexX + info.origin.position.x,info.resolution * indexY + info.origin.position.y);
}

void FrontierSearch::globalCoordinateToLocal(std::vector<geometry_msgs::Point>& goals){
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

    for(auto& goal : goals){
        Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(goal.x - transX,goal.y - transY));
        goal.x = tempPoint.x();
        goal.y = tempPoint.y();
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

    double max = DOUBLE_MINUS_INFINITY;
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

    if(max > DOUBLE_MINUS_INFINITY){
        previousGoal = goal;
        return true;
    }
    else{
        return false;
    }
}

void FrontierSearch::publishGoal(const geometry_msgs::Point& goal){
    exploration_msgs::Goal msg;
	msg.global = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;

    goal_.pub.publish(msg);
	ROS_INFO_STREAM("Publish Goal\n");
}

void FrontierSearch::publishGoalList(const std::vector<geometry_msgs::Point>& goals){
    exploration_msgs::GoalList msg;
	msg.global = goals;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;

    goalList_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

void FrontierSearch::publishGoalListPoseArray(const std::vector<geometry_msgs::Point>& goals){
    geometry_msgs::PoseArray msg;
    msg.poses.reserve(goals.size());

    for(const auto& goal : goals){
        msg.poses.emplace_back(CommonLib::pointToPose(goal));
    }

	msg.header.frame_id = MAP_FRAME_ID;
    msg.header.stamp = ros::Time::now();

    //ROS_DEBUG_STREAM("before publish_pose_array\n");

    goalPoseArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList PoseArray\n");
}

#endif //FRONTIER_SEARCH_HPP