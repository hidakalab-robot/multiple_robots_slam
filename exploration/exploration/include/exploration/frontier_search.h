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
    void release(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY);
    void horizonDetection(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY);
    std::vector<Eigen::Vector2i> frontierDetectionByContinuity(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, float resolution);
    std::vector<Eigen::Vector2i> frontierDetectionByClustering(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, float resolution);
    void obstacleFilter(struct FrontierSearch::mapStruct& map,std::vector<Eigen::Vector2i>& index, int sizeX, int sizeY, float resolution);
    geometry_msgs::Point arrayToCoordinate(int indexX,int indexY,double originX,double originY,float resolution);
    geometry_msgs::Point selectGoal(std::vector<geometry_msgs::Point> goals, geometry_msgs::PoseStamped pose);

    double qToYaw(geometry_msgs::Quaternion q);

    void publishGoal(geometry_msgs::Point goal);
	void publishGoalList(std::vector<geometry_msgs::Point> goals);

	void publishGoalDelete(void);
	void publishGoalListDelete(void);

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

    p.param<std::string>("map_frame_id", mapFrameId, "map");

    p.param<float>("frontier_diameter_min", FRONTIER_DIAMETER_MIN, 0.4);
	p.param<int>("frontier_thickness", FRONTIER_THICKNESS, 3);
    if((FRONTIER_THICKNESS%2) == 0){
        FRONTIER_THICKNESS++;
    }
    p.param<int>("frontier_detection_method", FRONTIER_DETECTION_METHOD, 0);
    p.param<float>("filter_square_diameter", FILTER_SQUARE_DIAMETER, 0.4);
    p.param<bool>("obstacle_filter", OBSTACLE_FILTER, true);
    p.param<double>("distance_weight", DISTANCE_WEIGHT, 1.0);
    p.param<double>("direction_weight", DIRECTION_WEIGHT, 2.0);

    p.param<double>("previous_goal_threshold", PREVIOUS_GOAL_THRESHOLD, 1.0);
    
    DOUBLE_MINUS_INFINITY = -10000000.0;
}

void FrontierSearch::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    mapData = *msg;
    ROS_INFO_STREAM("Input Map\n");
}

void FrontierSearch::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
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
            index = frontierDetectionByClustering(map,mapData.info.width,mapData.info.height,mapData.info.resolution);
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

    qPose.callOne(ros::WallDuration(1));

    goal = selectGoal(goals,poseData);

    release(map, mapData.info.width,mapData.info.height);

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
    ROS_INFO_STREAM("Frontier Detection\n");
    
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

std::vector<Eigen::Vector2i> FrontierSearch::frontierDetectionByClustering(struct FrontierSearch::mapStruct& map, int sizeX, int sizeY, float resolution){
    //future work
    std::vector<Eigen::Vector2i> index;
    return index;
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
        //前回の目標との差を計算
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