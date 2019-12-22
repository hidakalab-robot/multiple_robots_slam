#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <costmap_2d/costmap_2d_ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace ExpLib{
    template<typename T>
    class PathPlanning{
        private:
            double PATH_TO_VECTOR_RATIO;
            tf::TransformListener tfl;
            costmap_2d::Costmap2DROS gcr;
            T planner;

        public:
            PathPlanning():tfl(ros::Duration(10)),gcr("costmap", tfl){
                ros::NodeHandle("~").param<double>("path_to_vector_ratio", PATH_TO_VECTOR_RATIO, 0.8);
                planner.initialize("path_planner",&gcr);
                ros::spinOnce();
            };
            
            PathPlanning(const std::string& costmapName, const std::string& plannerName):tfl(ros::Duration(10)),gcr(costmapName, tfl){
                ros::NodeHandle("~").param<double>("path_to_vector_ratio", PATH_TO_VECTOR_RATIO, 0.5);
                planner.initialize(plannerName,&gcr);
                ros::spinOnce();
            };

            bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal){
                ros::spinOnce();
                std::vector<geometry_msgs::PoseStamped> plan;
                return planner.makePlan(start,goal,plan);
            };

            bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
                ros::spinOnce();
                return planner.makePlan(start,goal,plan);
            };

            bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGrid& map){//only voronoi
                ros::spinOnce();
                return planner.makePlan(start,goal,plan,map);
            };

            void getDistance(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance, std::vector<geometry_msgs::PoseStamped>& plan){
                distance = 0;
                for(int i=1,ie=plan.size();i!=ie;++i) distance += Eigen::Vector2d(plan[i].pose.position.x - plan[i-1].pose.position.x, plan[i].pose.position.y - plan[i-1].pose.position.y).norm();
            }

            bool getDistance(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance){
                ros::spinOnce();
                std::vector<geometry_msgs::PoseStamped> plan;
                if(planner.makePlan(start,goal,plan)){
                    getDistance(start, goal, distance, plan);
                    return true;
                }
                return false;
            };

            void getVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, Eigen::Vector2d& vec, std::vector<geometry_msgs::PoseStamped>& plan){
                int b = plan.size() - 1;
                int a = b * PATH_TO_VECTOR_RATIO;
                //a-b間のベクトル
                vec = Eigen::Vector2d(plan[b].pose.position.x - plan[a].pose.position.x, plan[b].pose.position.y - plan[a].pose.position.y).normalized();
            };

            bool getVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, Eigen::Vector2d& vec){
                ros::spinOnce();
                std::vector<geometry_msgs::PoseStamped> plan;
                if(planner.makePlan(start,goal,plan)){
                    getVec(start,goal,vec,plan);
                    return true;
                }
                return false;
            };

            bool getDistanceAndVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance, Eigen::Vector2d& vec){
                ros::spinOnce();
                std::vector<geometry_msgs::PoseStamped> plan;
                if(planner.makePlan(start,goal,plan)){
                    getDistance(start,goal,distance,plan);
                    getVec(start,goal,vec,plan);
                    return true;
                }
                return false;
            };
    };
}

#endif //PATH_PLANNING_H