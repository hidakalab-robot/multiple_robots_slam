#include <exploration_libraly/utility.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/convert.h>
#include <tf/transform_listener.h>
#include <nav_msgs/MapMetaData.h>
#include <pcl_ros/point_cloud.h>

namespace ExpLib{
    namespace Utility{
        template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p){
            tf::StampedTransform transform;
            try {
                l.lookupTransform(destFrame, origFrame, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex){
                ROS_ERROR("%s",ex.what());
                ROS_ERROR_STREAM("transform is failed");
                return;
            }
            tf::Quaternion transQ = transform.getRotation();
            double transYaw = Convert::qToYaw(transQ);
            double transX = transform.getOrigin().getX();
            double transY = transform.getOrigin().getY();

            // Eigen::Matrix2d rotation;
            // rotation << cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw);
            Eigen::Matrix2d rotation = Construct::eigenMat2d(cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw));
            Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.position.x, p.position.y) + Eigen::Vector2d(transX, transY));
            p.position = Construct::msgPoint(tempPoint.x(),tempPoint.y());
            p.orientation = Convert::tfQuaToGeoQua(tf::Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)*=transQ);
        }

        template<> geometry_msgs::Pose coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p){
            coordinateConverter2d<void>(l,destFrame,origFrame,p);
            return p;
        }

        template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p){
            geometry_msgs::Pose ps = Convert::pointToPose(p);
            coordinateConverter2d<void>(l,destFrame,origFrame,ps);
            p = ps.position;
        }

        template<> geometry_msgs::Point coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p){
            geometry_msgs::Point pl = p;
            coordinateConverter2d<void>(l,destFrame,origFrame,pl);
            return pl;
        }

        template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, pcl::PointXYZ& p){
            geometry_msgs::Pose ps = Convert::pointToPose(Convert::pclPointXYZToPoint(p));
            coordinateConverter2d<void>(l,destFrame,origFrame,ps);
            p = Convert::pointToPclPointXYZ(ps.position);
        }

        geometry_msgs::Point mapIndexToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info){
            return ExpLib::Construct::msgPoint(info.resolution * indexX + info.origin.position.x,info.resolution * indexY + info.origin.position.y);
        }

        Eigen::Vector2i coordinateToMapIndex(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info){
            return Eigen::Vector2i((coordinate.x()-info.origin.position.x)/info.resolution,(coordinate.y()-info.origin.position.y)/info.resolution);
        }

        Eigen::Vector2i coordinateToMapIndex(const geometry_msgs::Point& coordinate,const nav_msgs::MapMetaData& info){
            return Eigen::Vector2i((coordinate.x-info.origin.position.x)/info.resolution,(coordinate.y-info.origin.position.y)/info.resolution);
        }

        std::vector<std::vector<int8_t>> mapArray1dTo2d(const std::vector<int8_t>& data, const nav_msgs::MapMetaData& info){
            std::vector<std::vector<int8_t>> map2d(info.width,std::vector<int8_t>(info.height));
            for(int y=0,k=0,ey=info.height;y!=ey;++y){
                for(int x=0,ex=info.width;x!=ex;++x,++k){
                    map2d[x][y] = data[k];
                }
            }
            return map2d;
        }

        double shorterRotationAngle(const double orig, const double dest){
            double diff = dest - orig;
            return diff > M_PI ? diff-2*M_PI : diff < -M_PI ? diff+2*M_PI : diff;
        }

        double shorterRotationAngle(const geometry_msgs::Quaternion& orig, const geometry_msgs::Quaternion& dest){
            return shorterRotationAngle(Convert::qToYaw(orig), Convert::qToYaw(dest));
        }
    }
}