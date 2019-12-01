#ifndef LOOP_CLOSURE_COUNTER_H
#define LOOP_CLOSURE_COUNTER_H

#include <memory>

// 前方宣言

/// my packages
namespace ExpLib{
    namespace Struct{
        template<typename T>
        struct pubStruct;
    }
}
namespace exploration_support{
    class loop_closure_counter_parameter_reconfigureConfig;
}
/// ros
namespace dynamic_reconfigure{
    template <class ConfigType>
    class Server;
}
/// rosmsgs
namespace std_msgs{
    template <class ContainerAllocator>
    struct Float64_;
    typedef ::std_msgs::Float64_<std::allocator<void>> Float64;
    template <class ContainerAllocator>
    struct Int8_;
    typedef ::std_msgs::Int8_<std::allocator<void>> Int8;
}
// 前方宣言ここまで

// legacy loop counter
namespace ExStc = ExpLib::Struct;

class LoopClosureCounter{
    private:
        // dynamic parameters
        double LOOP_CLOSURE_THRESHOLD;

        // static parameters
        std::string ODOM_FRAME_ID;
        std::string MAP_FRAME_ID;
        double PUBLISH_RATE;
        std::string LOOP_PARAMETER_FILE_PATH;
        bool OUTPUT_LOOP_PARAMETERS;

        // variables
        std::unique_ptr<ExStc::pubStruct<std_msgs::Int8>> count;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Float64>> accumTemp;
        std::unique_ptr<ExStc::pubStruct<std_msgs::Float64>> accumPerm;
        std::unique_ptr<dynamic_reconfigure::Server<exploration_support ::loop_closure_counter_parameter_reconfigureConfig>> drs_;

        // functions
        void loopDetection(void);
        void loadParams(void);
        void dynamicParamsCB(exploration_support::loop_closure_counter_parameter_reconfigureConfig &cfg, uint32_t level);
        void outputParams(void);

    public:
        LoopClosureCounter();
        ~LoopClosureCounter();
        void loopDetectionLoop(void);
};

#endif // LOOP_CLOSURE_COUNTER_HPP