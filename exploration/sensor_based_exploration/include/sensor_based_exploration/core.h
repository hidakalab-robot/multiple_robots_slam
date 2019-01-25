#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_based_exploration/moving.h>
#include <sensor_based_exploration/branch_search.h>

#include <boost/bind.hpp>

//sensor-based-explorationの根幹クラス

namespace sensor_based_exploration
{
    class Core
    {
    private:
        ros::NodeHandle param;

        ros::NodeHandle ss;
        ros::Subscriber subScan;
        ros::CallbackQueue qScan;
        std::string scanTopic;
        sensor_msgs::LaserScan scanData;

        ros::NodeHandle sp;
        ros::Subscriber subPose;
        ros::CallbackQueue qPose;
        std::string poseTopic;
        geometry_msgs::PoseStamped poseData;

        ros::NodeHandle sb;
        ros::Subscriber subBumper;
        ros::CallbackQueue qBumper;
        std::string bumperTopic;
        kobuki_msgs::BumperEvent bumperData;

        bool flag;

        void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
        void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);

        void approx(std::vector<float>& scan);
        void forBranch(std::vector<float>& scan);

    public:

        Moving MV;
        BranchSearch BS;

        bool scanEdit;
    
        Core();
        ~Core(){};

        

    };

    Core::Core():param("~"){
        MV.initialize();
        //BS.initialize();
        param.getParam("scan_topic", scanTopic);
        ss.setCallbackQueue(&qScan);
        subScan = ss.subscribe(scanTopic,1,&Core::scanCB, this);

        param.getParam("pose_topic", poseTopic);
        sp.setCallbackQueue(&qPose);
        subPose = sp.subscribe(poseTopic,1,&Core::poseCB,this);

        param.getParam("bumper_topic", bumperTopic);
        sp.setCallbackQueue(&qBumper);
        subPose = sp.subscribe(bumperTopic,1,&Core::bumperCB,this);

        scanEdit = false;

    }

    void Core::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        scanData = *msg;
        if(scanEdit){
            approx(scanData.ranges);
        }
    }

    void Core::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
        poseData = *msg;
    }

    void Core::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
        bumperData = *msg;
    }

    void Core::approx(std::vector<float>& scan){
        float depth,depth1,depth2;
	    depth = depth1 = depth2 =0;

        for(int j=0,count=0;j<scan.size()-1;j++){
            depth=scan[j];
            //|val|nan|のとき

            if(!std::isnan(depth) && std::isnan(scan[j+1])){
                depth1=depth;
                count++;
            }

            if(std::isnan(depth)){
                //|nan|nan|の区間
                if(std::isnan(scan[j+1])){
                    count++;
                }
                //|nan|val|のとき
                else{
                    depth2=scan[j+1];
                    //左端がnanのとき
                    if(std::isnan(depth1)){
                        for(int k=0;k<count+1;k++)
                            scan[j-k]=0.01;//depth2;
                    }
                    else{
                        for(int k=0;k<count;k++)
                            scan[j-k]=depth2-(depth2-depth1)/(count+1)*(k+1);
                    }
                    count=0;
                }
            }
            //右端がnanのとき
            if(j==(scan.size()-1)-1 && std::isnan(scan[j+1])){
                for(int k=0;k<count;k++)
                    scan[j+1-k]=0.01;//depth1;
                count=0;
            }
        }		
        if(std::isnan(scan[0])){
            scan[0] = scan[1] - (scan[2] - scan[1]);
            if(scan[0] < 0){
                scan[0] = 0;
            }
        }
    }
}
