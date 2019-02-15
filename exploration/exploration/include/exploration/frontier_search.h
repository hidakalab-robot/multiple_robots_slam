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

class FrontierSearch
{
private:
    struct mapStruct{
        int8_t **source;
        int8_t **horizon;
        int8_t **frontierMap;
    };

    ros::NodeHandle p;

    double FRONTIER_DIAMETER_MIN;
    int FRONTIER_THICKNESS;//奇数
    int FRONTIER_DETECTION_METHOD;
    double FILTER_SQUARE_DIAMETER;

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

	ros::NodeHandle pgld;
	ros::Publisher pubGoalListDel;

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void initialize(struct FrontierSearch::mapStruct& map, nav_msgs::OccupancyGrid source);
    void release(struct FrontierSearch::mapStruct& map, double sizeX, double sizeY);
    void horizonDetection(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY);
    std::vector<Eigen::Vector2i> frontierDetectionByContinuity(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, double resolution);
    std::vector<Eigen::Vector2i> frontierDetectionByClustering(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY);
    bool obstacleFilter(struct FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i> index, int sizeX, int sizeY, double resolution);

public:
    FrontierSearch();
    ~FrontierSearch(){};

    bool getGoal(geometry_msgs::Point& goal);

};

FrontierSearch::FrontierSearch():p("~"){
    sm.setCallbackQueue(&qMap);
    subMap = sm.subscribe("map",1,&FrontierSearch::mapCB, this);

    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&FrontierSearch::poseCB,this);

    pubGoal = pg.advertise<exploration_msgs::Goal>("goal", 1);
	pubGoalList = pgl.advertise<exploration_msgs::GoalList>("goal_list", 1);
	pubGoalDel = pgd.advertise<std_msgs::Empty>("goal/delete", 1);
	pubGoalListDel = pgld.advertise<std_msgs::Empty>("goal_list/delete", 1);

    p.param<double>("frontier_diameter_min", FRONTIER_DIAMETER_MIN, 0.4);
	p.param<int>("frontier_thickness", FRONTIER_THICKNESS, 3);
    p.param<int>("frontier_detection_method", FRONTIER_DETECTION_METHOD, 0);
    p.param<double>("filter_square_diameter", FILTER_SQUARE_DIAMETER, 0.4);
    
}

void FrontierSearch::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    mapData = *msg;
}

void FrontierSearch::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

bool FrontierSearch::getGoal(geometry_msgs::Point& goal){
    qMap.callOne(ros::WallDuration(1));
    struct FrontierSearch::mapStruct map;
    initialize(map,mapData);

    horizonDetection(map,mapData.info.width,mapData.info.height);

    std::vector<Eigen::Vector2i> index;

    switch (FRONTIER_DETECTION_METHOD){
        case 0:
            index = frontierDetectionByContinuity(map,mapData.info.width,mapData.info.height,mapData.info.resolution);
            break;
        case 1:
            //frontiers = frontierDetectionByClustering(map,mapData.info.width,mapData.info.height,mapData.info.resolution);
            break;
        default:
            return;
    }

    if(!obstacleFilter(map,index,mapData.info.width,mapData.info.height)){
        ROS_ERROR_STREAM("FILTER_SQUARE_DIAMETER is Bad");
        return;
    }



    release(map, mapData.info.width,mapData.info.height);

    return true;
}

void FrontierSearch::initialize(struct FrontierSearch::mapStruct& map, nav_msgs::OccupancyGrid source){
    const int sizeX = source.info.width;
    const int sizeY = source.info.height;

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
}

void FrontierSearch::horizonDetection(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY){
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
}

std::vector<Eigen::Vector2i> FrontierSearch::frontierDetectionByContinuity(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, double resolution){
    const int FRONTIER_EDGE_RENGE = FRONTIER_THICKNESS / 2;

    std::vector<Eigen::Vector2i> index;
    Eigen::Vector2i temp;
    
    int continuity = 0;
    int sum = 0;

    // x axis continuity
    int countX = 0;
    int startX = 0;
    int endX = 0;
    for(int y=FRONTIER_EDGR_RENGE;y<sizeY-FRONTIER_EDGR_RENGE;y+=FRONTIER_THICKNESS){
		countX = 0;
		while(countX < sizeX && ros::ok()){
			for(int j=-FRONTIER_EDGR_RENGE;j<=FRONTIER_EDGR_RENGE;j++){
				sum += map.horizon[countX][y+j];
			}
			if(sum > 0){
				startX = countX;
                continuity = 0;
				while(sum > 0 && ros::ok()){
					sum = 0;
					continuity++;
                    //countX++;// インクリメントする前のxで比較したい
					if(countX++ < sizeX){
						for(int j=-FRONTIER_EDGR_RENGE;j<=FRONTIER_EDGR_RENGE;j++){
							sum += map.horizon[countX][y+j];
						}
					}
					else{	
						break;
					}
				}
				endX = countX-1;
				if(continuity >= (int)(FRONTIER_DIAMETER_MIN / resolution)){
                    temp.x = (int)((startX + endX)/2);
                    temp.y = y;
					map.frontierMap[temp.x][temp.y] = 1;
                    index.push_back(temp);
				}
			}
			else{			
				countX++;
			}
		}
    }

    // y axis continuity
    int countY = 0;
    int startY = 0;
    int endY = 0;
    for(int x=FRONTIER_EDGR_RENGE;x<sizeX-FRONTIER_EDGR_RENGE;x+=FRONTIER_EDGR_RENGE){
		countY = 0;
		while(countY < sizeY && ros::ok()){
			for(int i=-FRONTIER_EDGR_RENGE;i<=FRONTIER_EDGR_RENGE;i++){
				sum += map.horizon[x+i][countY];
			}
			if(sum > 0){
				startY = countY;
				while(sum > 0 && ros::ok()){
					sum = 0;
					continuity++;
                    //countY++;//インクリメントする前のyで比較したい
					if(countY++ < sizeY){
						for(int i=-FRONTIER_EDGR_RENGE;i<=FRONTIER_EDGR_RENGE;i++){
							sum += map.horizon[x+i][countY];
						}
					}
					else{	
						break;
					}
				}
				endY = countY-1;
				if(continuity >= (int)(FRONTIER_DIAMETER_MIN / resolution)){
                    temp.x = x;
                    temp.y = (int)((start + end)/2);
					map.frontierMap[temp.x][temp.y] = 1;
                    index.push_back(temp);
				}
			}
			else{			
				countY++;
			}
		}
    }

    return index;
}

bool FrontierSearch::obstacleFilter(struct FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i> index, int sizeX, int sizeY, double resolution){
    //add obstacle cell
    for(int x=0;x<sizeX;x++){
        for(int y=0;y<sizeY;y++){
            if(map.source[x][y] == 100){
                map.frontierMap[x][y] = 100;
            }
        }
    }

    FILTER_SQUARE_DIAMETER

    int FILTER_HALF_CELL = FILTER_SQUARE_DIAMETER / resolution / 2;
    //FILTER_HALF_CELL += FILTER_HALF_CELL % 2;//奇数ではダメな理由が不明

    if(FILTER_HALF_CELL < 1){
        return false;
    }

    int RIGHT_X,LEFT_X;
    int TOP_Y, BOTTOM_Y;

    for(int i=0;index.size();i++){

    }

}

void FrontierSearch::release(struct FrontierSearch::mapStruct& map, double sizeX, double sizeY){
    //mapのメモリを解放
    for(int i=0;i<sizeX;i++){
        delete[] map.source[i];
        delete[] map.horizon[i];
        delete[] map.frontierMap[i];
    }
    delete[] map.source;
    delete[] map.horizon;
    delete[] map.frontierMap;
}

#endif //FRONTIER_SEARCH_H