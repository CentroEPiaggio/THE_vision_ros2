#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "geometry_msgs/msg/pose.hpp"

using namespace BT;

class PoseDeserializeUtility : public SyncActionNode
{
    public:
        PoseDeserializeUtility(const std::string& name, const NodeConfig& conf);
        ~PoseDeserializeUtility()
        {};
        static BT::PortsList providedPorts()
        {
            return { 
                InputPort<double>("x"),
                InputPort<double>("y"),
                InputPort<double>("z"),
                InputPort<double>("qx"),
                InputPort<double>("qy"),
                InputPort<double>("qz"),
                InputPort<double>("qw"),
                OutputPort<std::string>("out")
                };
        };

        NodeStatus tick() override;
    private:
        std::vector<uint8_t> buffer_;
        RosMsgParser::ROS2_Deserializer deserialize_;
        std::unique_ptr<RosMsgParser::Parser> parser_;
        
        const std::string msg_type_ = "geometry_msgs/Pose";


};