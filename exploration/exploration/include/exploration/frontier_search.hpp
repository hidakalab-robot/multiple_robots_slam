#ifndef FRONTIER_SEARCH_H
#define FRONTIER_SEARCH_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration_msgs/Goal.h>
#include <exploration_msgs/GoalList.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseArray.h>

class FrontierSearch
{
private:
    struct mapStruct{
        int8_t **source;
        int8_t **horizon;
        int8_t **frontierMap;
    };

    struct goalStruct{
        geometry_msgs::Point goal;
        double dot;
        double distance;
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

    std::string mapFrameId;

    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;

    bool PUBLISH_POSE_ARRAY;

    bool PREVIOUS_GOAL_EFFECT;

    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    geometry_msgs::PoseStamped poseData;

    ros::NodeHandle sm;
    ros::Subscriber subMap;
    ros::CallbackQueue qMap;
    nav_msgs::OccupancyGrid mapData;

    ros::NodeHandle pg;
	ros::Publisher pubGoal;

	ros::NodeHandle pgd;
	ros::Publisher pubGoalDel;

	ros::NodeHandle pgl;
	ros::Publisher pubGoalList;

    ros::NodeHandle pglpa;
	ros::Publisher pubGoalListPoseArray;

	ros::NodeHandle pgld;
	ros::Publisher pubGoalListDel;

    ros::NodeHandle pcc;
	ros::Publisher pubColorCloud;

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void initialize(struct FrontierSearch::mapStruct& map, nav_msgs::OccupancyGrid source);
    void release(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY);
    void horizonDetection(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY);
    std::vector<Eigen::Vector2i> frontierDetectionByContinuity(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, float resolution);
    std::vector<Eigen::Vector2i> frontierDetectionByClustering(struct FrontierSearch::mapStruct& map, nav_msgs::MapMetaData& mapInfo);
    void obstacleFilter(struct FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i>& index, int sizeX, int sizeY, float resolution);
    geometry_msgs::Point arrayToCoordinate(int indexX,int indexY,double originX,double originY,float resolution);
    Eigen::Vector2i coordinateToArray(double x,double y,double originX,double originY,float resolution);

    geometry_msgs::Point selectGoal(std::vector<geometry_msgs::Point> goals, geometry_msgs::PoseStamped pose);

    double qToYaw(geometry_msgs::Quaternion q);

    void publishGoal(geometry_msgs::Point goal);
	void publishGoalList(std::vector<geometry_msgs::Point> goals);
    void publishGoalListPoseArray(std::vector<geometry_msgs::Point> goals);

	void publishGoalDelete(void);
	void publishGoalListDelete(void);

public:
    FrontierSearch();
    ~FrontierSearch(){};

    bool getGoal(geometry_msgs::Point& goal);//publish goalList and select goal
    bool getGoal(void);//only publish goal list

};

FrontierSearch::FrontierSearch():p("~"){
    sm.setCallbackQueue(&qMap);
    subMap = sm.subscribe("map",1,&FrontierSearch::mapCB, this);

    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&FrontierSearch::poseCB,this);
    pubGoal = pg.advertise<exploration_msgs::Goal>("goal", 1, true);

    pubGoalList = pgl.advertise<exploration_msgs::GoalList>("goal_list", 1, true);

    p.param<bool>("publish_pose_array", PUBLISH_POSE_ARRAY, false);

    if(PUBLISH_POSE_ARRAY){
        pubGoalListPoseArray = pglpa.advertise<geometry_msgs::PoseArray>("goal_list_pose_array", 1, true);
    }
	
	pubGoalDel = pgd.advertise<std_msgs::Empty>("goal/delete", 1);
	pubGoalListDel = pgld.advertise<std_msgs::Empty>("goal_list/delete", 1);

    pubColorCloud = pcc.advertise<sensor_msgs::PointCloud2>("horizon_cluster/color", 1);

    p.param<std::string>("map_frame_id", mapFrameId, "map");

    p.param<float>("frontier_diameter_min", FRONTIER_DIAMETER_MIN, 0.4);
	p.param<int>("frontier_thickness", FRONTIER_THICKNESS, 3);
    FRONTIER_THICKNESS += (FRONTIER_THICKNESS+1)%2;//偶数封じ
    // if((FRONTIER_THICKNESS%2) == 0){
    //     FRONTIER_THICKNESS++;
    // }
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
    
    DOUBLE_MINUS_INFINITY = -10000000.0;
}

void FrontierSearch::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    mapData = *msg;
    ROS_INFO_STREAM("Input Map\n");
}

void FrontierSearch::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

bool FrontierSearch::getGoal(void){
    qMap.callOne(ros::WallDuration(1));
    publishGoalDelete();
    struct FrontierSearch::mapStruct map;
    initialize(map,mapData);

    horizonDetection(map,mapData.info.width,mapData.info.height);

    std::vector<Eigen::Vector2i> index;

    switch (FRONTIER_DETECTION_METHOD){
        case 0:
            index = frontierDetectionByContinuity(map,mapData.info.width,mapData.info.height,mapData.info.resolution);
            break;
        case 1:
            index = frontierDetectionByClustering(map,mapData.info);
            break;
        default:
            ROS_ERROR_STREAM("Frontier Detection Method is Unknown\n");
            release(map, mapData.info.width,mapData.info.height);
            return false;
    }

    //ROS_DEBUG_STREAM("Before Filter index size : " << index.size() << "\n");


    if(OBSTACLE_FILTER){
        obstacleFilter(map,index,mapData.info.width,mapData.info.height,mapData.info.resolution);
    }
    
    //ROS_DEBUG_STREAM("After Filter Index Size : " << index.size() << "\n");

    if(index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found\n");
        publishGoalListDelete();
        release(map, mapData.info.width,mapData.info.height);
        return false;
    }
    
    std::vector<geometry_msgs::Point> goals;

    for(int i=0;i<index.size();i++){
        goals.push_back(arrayToCoordinate(index[i].x(),index[i].y(),mapData.info.origin.position.x,mapData.info.origin.position.y,mapData.info.resolution));
    }

    ROS_INFO_STREAM("Frontier Found : " << goals.size() << "\n");

    publishGoalList(goals);
    
    if(PUBLISH_POSE_ARRAY){
        //ROS_DEBUG_STREAM("call publish_pose_array\n");
        publishGoalListPoseArray(goals);
    }

    release(map, mapData.info.width,mapData.info.height);

    return true;
}

bool FrontierSearch::getGoal(geometry_msgs::Point& goal){
    qMap.callOne(ros::WallDuration(1));
    publishGoalDelete();
    struct FrontierSearch::mapStruct map;
    initialize(map,mapData);

    horizonDetection(map,mapData.info.width,mapData.info.height);

    std::vector<Eigen::Vector2i> index;

    switch (FRONTIER_DETECTION_METHOD){
        case 0:
            index = frontierDetectionByContinuity(map,mapData.info.width,mapData.info.height,mapData.info.resolution);
            break;
        case 1:
            index = frontierDetectionByClustering(map,mapData.info);
            break;
        default:
            ROS_ERROR_STREAM("Frontier Detection Method is Unknown\n");
            release(map, mapData.info.width,mapData.info.height);
            return false;
    }

    //ROS_DEBUG_STREAM("Before Filter index size : " << index.size() << "\n");


    if(OBSTACLE_FILTER){
        obstacleFilter(map,index,mapData.info.width,mapData.info.height,mapData.info.resolution);
    }
    
    //ROS_DEBUG_STREAM("After Filter Index Size : " << index.size() << "\n");

    if(index.size() == 0){
        ROS_INFO_STREAM("Frontier Do Not Found\n");
        publishGoalListDelete();
        release(map, mapData.info.width,mapData.info.height);
        return false;
    }
    
    std::vector<geometry_msgs::Point> goals;

    for(int i=0;i<index.size();i++){
        goals.push_back(arrayToCoordinate(index[i].x(),index[i].y(),mapData.info.origin.position.x,mapData.info.origin.position.y,mapData.info.resolution));
    }

    ROS_INFO_STREAM("Frontier Found : " << goals.size() << "\n");

    publishGoalList(goals);
    
    if(PUBLISH_POSE_ARRAY){
        //ROS_DEBUG_STREAM("call publish_pose_array\n");
        publishGoalListPoseArray(goals);
    }

    release(map, mapData.info.width,mapData.info.height);

    qPose.callOne(ros::WallDuration(1));

    goal = selectGoal(goals,poseData);


    if((int)goal.x == 0 && (int)goal.y == 0 && (int)goal.z == 0){
        ROS_INFO_STREAM("Found Frontier is Too Close\n");
        return false;
    }
    else{
		ROS_INFO_STREAM("Selected Frontier : (" << goal.x << "," << goal.y << ")\n");
        publishGoal(goal);
        return true;
    }
}

void FrontierSearch::initialize(struct FrontierSearch::mapStruct& map, nav_msgs::OccupancyGrid source){
    
    const int sizeX = source.info.width;
    const int sizeY = source.info.height;

    ROS_INFO_STREAM("Memory initialize\n");

    //mapのメモリを確保
    map.source = new int8_t*[sizeX];
    map.horizon = new int8_t*[sizeX];
    map.frontierMap = new int8_t*[sizeX];

    for(int i=0;i<sizeX;i++){
        map.source[i] = new int8_t[sizeY];
        map.horizon[i] = new int8_t[sizeY];
        map.frontierMap[i] = new int8_t[sizeY];
    }

    //初期化
    int k=0;
    for(int y=0;y<sizeY;y++){
        for(int x=0;x<sizeX;x++){
            map.source[x][y] = source.data[k];
            map.horizon[x][y] = 0;
            map.frontierMap[x][y] = 0;
            k++;
        }
    }

    ROS_INFO_STREAM("Memory initialize complete\n");
}

void FrontierSearch::horizonDetection(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY){
    ROS_INFO_STREAM("Horizon Detection\n");
    //x axis horizon
    for(int y=0;y<sizeY;y++){
        for(int x=0;x<sizeX-1;x++){
            if(map.source[x][y] == 0 && map.source[x+1][y] == -1){
                map.horizon[x][y] = 1;
            }
            else if(map.source[x][y] == -1 && map.source[x+1][y] == 0){
                map.horizon[x+1][y] = 1;	
            }
        }
    }

    //y axis horizon
    for(int x=0;x<sizeX;x++){
        for(int y=0;y<sizeY-1;y++){
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

std::vector<Eigen::Vector2i> FrontierSearch::frontierDetectionByContinuity(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, float resolution){
    ROS_INFO_STREAM("Frontier Detection by Continuity\n");
    
    const int FRONTIER_EDGE_RENGE = FRONTIER_THICKNESS / 2;

    //ROS_DEBUG_STREAM("FRONTIER_EDGE_RENGE : " << FRONTIER_EDGE_RENGE << "\n");
    //ROS_DEBUG_STREAM("ROBOT_CELLSIZE : " << (int)(FRONTIER_DIAMETER_MIN / resolution) << "\n");

    std::vector<Eigen::Vector2i> index;
    Eigen::Vector2i temp;
    
    int continuity = 0;
    int sum = 0;

    // x axis continuity
    int countX = 0;
    int startX = 0;
    int endX = 0;
    for(int y=FRONTIER_EDGE_RENGE;y<sizeY-FRONTIER_EDGE_RENGE;y+=FRONTIER_THICKNESS){
		countX = 0;
		while(countX < sizeX && ros::ok()){
			for(int j=-FRONTIER_EDGE_RENGE;j<=FRONTIER_EDGE_RENGE;j++){
				sum += map.horizon[countX][y+j];
			}
			if(sum > 0){
				startX = countX;
                continuity = 0;
				while(sum > 0 && ros::ok()){
					sum = 0;
					continuity++;
					if(countX++ < sizeX){// インクリメントする前のxで比較したい
						for(int j=-FRONTIER_EDGE_RENGE;j<=FRONTIER_EDGE_RENGE;j++){
							sum += map.horizon[countX][y+j];
						}
					}
					else{
						break;
					}
				}
				endX = countX-1;
				if(continuity >= (int)(FRONTIER_DIAMETER_MIN / resolution)){
                    temp.x() = (int)((startX + endX)/2);
                    temp.y() = y;
					map.frontierMap[temp.x()][temp.y()] = 1;
                    index.push_back(temp);
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
    int countY = 0;
    int startY = 0;
    int endY = 0;
    for(int x=FRONTIER_EDGE_RENGE;x<sizeX-FRONTIER_EDGE_RENGE;x+=FRONTIER_THICKNESS){
		countY = 0;
		while(countY < sizeY && ros::ok()){
			for(int i=-FRONTIER_EDGE_RENGE;i<=FRONTIER_EDGE_RENGE;i++){
				sum += map.horizon[x+i][countY];
			}
			if(sum > 0){
				startY = countY;
                continuity = 0;
				while(sum > 0 && ros::ok()){
					sum = 0;
					continuity++;
                    if(countY++ < sizeY){//インクリメントする前のyで比較したい
						for(int i=-FRONTIER_EDGE_RENGE;i<=FRONTIER_EDGE_RENGE;i++){
							sum += map.horizon[x+i][countY];
						}
					}
					else{
						break;
					}
				}
				endY = countY-1;
				if(continuity >= (int)(FRONTIER_DIAMETER_MIN / resolution)){
                    temp.x() = x;
                    temp.y() = (int)((startY + endY)/2);
					map.frontierMap[temp.x()][temp.y()] = 1;
                    index.push_back(temp);
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

std::vector<Eigen::Vector2i> FrontierSearch::frontierDetectionByClustering(struct FrontierSearch::mapStruct& map, nav_msgs::MapMetaData& mapInfo){
    
    ROS_INFO_STREAM("Frontier Detection by Clustering\n");

    //ROS_DEBUG_STREAM("mapStruct to tempVector\n");

    std::vector<geometry_msgs::Point> points;
    for(int y=0;y<mapInfo.height;++y){
        for(int x=0;x<mapInfo.width;++x){
            if(map.horizon[x][y] == 1){
                points.emplace_back(arrayToCoordinate(x,y,mapInfo.origin.position.x,mapInfo.origin.position.y,mapInfo.resolution));
            }
        }
    }

    //ROS_DEBUG_STREAM("geometry_msgs::Point to pointcloud\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr horizonMap(new pcl::PointCloud<pcl::PointXYZ>);
    horizonMap -> points.reserve(points.size());

    for(int i=0;i<points.size();++i){
        horizonMap -> points.emplace_back(pcl::PointXYZ((float)points[i].x,(float)points[i].y,0.0f));
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
    {
        //ROS_DEBUG_STREAM("coloring\n");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorMap(new pcl::PointCloud<pcl::PointXYZRGB>);

        float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};
        int i=0;
        int j=0;
        for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                colorMap -> points.emplace_back(pcl::PointXYZRGB());
                colorMap -> points[j].x = horizonMap->points[*pit].x;
                colorMap -> points[j].y = horizonMap->points[*pit].y;
                colorMap -> points[j].z = 0.0f;
                colorMap -> points[j].r = colors[i%12][0];
			    colorMap -> points[j].g = colors[i%12][1];
			    colorMap -> points[j].b = colors[i%12][2];
                ++j;
      	    }
    	    ++i;
  	    }
        colorMap -> width = colorMap -> points.size();
        colorMap -> height = 1;
        colorMap -> is_dense = true;
        
        sensor_msgs::PointCloud2 colorMsg;
        pcl::toROSMsg(*colorMap,colorMsg);
        colorMsg.header.frame_id = mapFrameId;
        colorMsg.header.stamp = ros::Time::now();
        pubColorCloud.publish(colorMsg);
    }

    //ROS_DEBUG_STREAM("Centroids Calculation\n");
    std::vector<Eigen::Vector2d> centroids(indices.size());
    {
        Eigen::Vector2d sum;
        int i = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){
            sum = Eigen::Vector2d::Zero();
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                sum.x() += horizonMap -> points[*pit].x;
                sum.y() += horizonMap -> points[*pit].y;
            }
            centroids[i] = Eigen::Vector2d(sum.x()/indices[i].indices.size(),sum.y()/indices[i].indices.size());
            ++i;
        }
    }
    
    //ROS_DEBUG_STREAM("centroids to index array\n");
    std::vector<Eigen::Vector2i> index(centroids.size());
    for(int i=0;i<centroids.size();++i){
        index[i] = coordinateToArray(centroids[i].x(),centroids[i].y(),mapInfo.origin.position.x,mapInfo.origin.position.y,mapInfo.resolution);
    }
    return index;
}

Eigen::Vector2i FrontierSearch::coordinateToArray(double x,double y,double originX,double originY,float resolution){
    return Eigen::Vector2i((x-originX)/resolution,(y-originY)/resolution);
}


void FrontierSearch::obstacleFilter(struct FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i>& index, int sizeX, int sizeY, float resolution){
    ROS_INFO_STREAM("Obstacle Filter\n");
    
    //add obstacle cell
    for(int x=0;x<sizeX;x++){
        for(int y=0;y<sizeY;y++){
            if((int)map.source[x][y] == 100){
                map.frontierMap[x][y] = 100;
            }
        }
    }

    int FILTER_HALF_CELL = (FILTER_SQUARE_DIAMETER / resolution) / 2.0;
    //FILTER_HALF_CELL += FILTER_HALF_CELL % 2;//奇数ではダメな理由が不明

    if(FILTER_HALF_CELL < 1){
        ROS_ERROR_STREAM("FILTER_SQUARE_DIAMETER is Bad\n");
        return;
    }

    int RIGHT,LEFT;
    int TOP, BOTTOM;
    int sum;
    std::vector<Eigen::Vector2i> filteredIndex;

    for(int i=0;i<index.size();i++){
        sum = 0;
        //left shape
        if(index[i].x()-FILTER_HALF_CELL < 0){
			LEFT = index[i].x();
		}
		else{
			LEFT = FILTER_HALF_CELL;
		}
        //right shape
		if(index[i].x()+FILTER_HALF_CELL > sizeX-1){
			RIGHT = (sizeX-1)-index[i].x();
		}
		else{
			RIGHT  = FILTER_HALF_CELL;
		}
        //top shape
		if(index[i].y()-FILTER_HALF_CELL < 0){
			TOP = index[i].y();
		}
		else{
			TOP = FILTER_HALF_CELL;
		}
        //bottom shape
		if(index[i].y()+FILTER_HALF_CELL > sizeY-1){
			BOTTOM = (sizeY-1)-index[i].y();
		}
		else{
			BOTTOM = FILTER_HALF_CELL;
		}
        for(int y=index[i].y()-TOP;y<=index[i].y()+BOTTOM;y++){
            for(int x=index[i].x()-LEFT;x<=index[i].x()+RIGHT;x++){
                sum += (int)map.frontierMap[x][y];
            }
        }
        if(sum>100){
            map.frontierMap[index[i].x()][index[i].y()] = 0;
        }
        else{
            filteredIndex.push_back(index[i]);
        }
    }
    index = filteredIndex;
    ROS_INFO_STREAM("Obstacle Filter complete\n");
}

geometry_msgs::Point FrontierSearch::arrayToCoordinate(int indexX,int indexY,double originX,double originY,float resolution){
    geometry_msgs::Point coordinate;

    coordinate.x = resolution * indexX + originX;
    coordinate.y = resolution * indexY + originY;

    return coordinate;
}

geometry_msgs::Point FrontierSearch::selectGoal(std::vector<geometry_msgs::Point> goals, geometry_msgs::PoseStamped pose){
    //現在位置からそれぞれのフロンティア座標に対して距離とベクトルを計算し、評価関数によって目標を決定
    //前回の目標から近いところは目標に取らないようにする
    static geometry_msgs::Point previousGoal;

    //ロボットの向きのベクトル(大きさ1)を計算
    Eigen::Vector2d directionVec;
    double yaw = qToYaw(pose.pose.orientation);
    directionVec.x() = cos(yaw);
    directionVec.y() = sin(yaw);


    //変更点：前回の移動方向では無く現在のロボットの向きで評価する

    Eigen::Vector2d tempVec;
    std::vector<struct FrontierSearch::goalStruct> calced;
    struct FrontierSearch::goalStruct tempStruct;

    for(int i=0;i<goals.size();i++){
        if(PREVIOUS_GOAL_EFFECT){
            //処理が有効になっている場合前回の目標との差を計算
            double temp = sqrt(pow(goals[i].x - previousGoal.x,2)+pow(goals[i].y - previousGoal.y,2));
            if(temp > PREVIOUS_GOAL_THRESHOLD){
                tempVec.x() = goals[i].x - pose.pose.position.x;
                tempVec.y() = goals[i].y - pose.pose.position.y;
                tempVec.normalize();

                tempStruct.goal = goals[i];
                tempStruct.distance = tempVec.norm();
                tempStruct.dot = tempVec.dot(directionVec);

                calced.push_back(tempStruct);
            }
        }
        else{
            tempVec.x() = goals[i].x - pose.pose.position.x;
            tempVec.y() = goals[i].y - pose.pose.position.y;
            tempVec.normalize();

            tempStruct.goal = goals[i];
            tempStruct.distance = tempVec.norm();
            tempStruct.dot = tempVec.dot(directionVec);

            calced.push_back(tempStruct);
        }
        
    }

    geometry_msgs::Point goal;

    if(calced.size() == 0){
        return  goal;
    }

    double value;
    double max = DOUBLE_MINUS_INFINITY;
    //評価値が最大となる目標値を選択
    for(int i=0;i<calced.size();i++){
        value = DIRECTION_WEIGHT * calced[i].dot - DISTANCE_WEIGHT * calced[i].distance;
        if(value > max){
            max = value;
            goal = calced[i].goal;
        }
    }

    previousGoal = goal;

    return goal;
}

double FrontierSearch::qToYaw(geometry_msgs::Quaternion q){
    tf::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(tq).getRPY(roll,pitch,yaw);
    return yaw;
}

void FrontierSearch::release(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY){
    ROS_INFO_STREAM("Memory release\n");

    //mapのメモリを解放
    for(int i=0;i<sizeX;i++){
        delete[] map.source[i];
        delete[] map.horizon[i];
        delete[] map.frontierMap[i];
    }
    delete[] map.source;
    delete[] map.horizon;
    delete[] map.frontierMap;
    ROS_INFO_STREAM("Memory release complete\n");
}

void FrontierSearch::publishGoal(geometry_msgs::Point goal){
    exploration_msgs::Goal msg;
	msg.global = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = mapFrameId;

	pubGoal.publish(msg);
	ROS_INFO_STREAM("Publish Goal\n");
}

void FrontierSearch::publishGoalList(std::vector<geometry_msgs::Point> goals){
    exploration_msgs::GoalList msg;
	msg.global = goals;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = mapFrameId;

	pubGoalList.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

void FrontierSearch::publishGoalListPoseArray(std::vector<geometry_msgs::Point> goals){
    geometry_msgs::PoseArray msg;
    msg.poses.reserve(goals.size());

	geometry_msgs::Pose temp;

    for(int i=0;i<goals.size();++i){
        temp.position = goals[i];
        msg.poses.emplace_back(temp);
    }

	msg.header.frame_id = mapFrameId;
    msg.header.stamp = ros::Time::now();

    //ROS_DEBUG_STREAM("before publish_pose_array\n");

	pubGoalListPoseArray.publish(msg);
	ROS_INFO_STREAM("Publish GoalList PoseArray\n");
}

void FrontierSearch::publishGoalDelete(void){
	std_msgs::Empty msg;
	pubGoalDel.publish(msg);
	
	//ROS_INFO_STREAM("Publish Goal Delete\n");
}

void FrontierSearch::publishGoalListDelete(void){
	std_msgs::Empty msg;
	pubGoalListDel.publish(msg);
	
	//ROS_INFO_STREAM("Publish GoalList Delete\n");
}


#endif //FRONTIER_SEARCH_H