#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

#include "behaviortree_ros2/bt_service_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "the_interfaces/srv/move_to.hpp"

using namespace BT;

class SendPoseGoal : public RosServiceNode<the_interfaces::srv::MoveTo>
{
    public:
        SendPoseGoal(
                const std::string& instance_name, 
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params
            );
        ~SendPoseGoal()
        {
            RCLCPP_WARN(rclcpp::get_logger(this->name()),"Destroying SendPoseGoal Node");
        }
        static PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    InputPort<std::string>("target_pose")
                }
                );
        }
        bool setRequest(Request::SharedPtr& request) override;
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
    private:
        const std::string pose_type_ = "geometry_msgs/Pose";



};
