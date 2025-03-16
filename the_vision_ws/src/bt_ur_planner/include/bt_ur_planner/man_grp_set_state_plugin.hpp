#include "behaviortree_ros2/bt_service_node.hpp"

#include "the_interfaces/srv/go_to_group_state.hpp"

using namespace BT;
class ManipulatorGroupMove : public RosServiceNode<the_interfaces::srv::GoToGroupState>
{
    public:
    ManipulatorGroupMove(
                const std::string& instance_name, 
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params
    );
    ~ManipulatorGroupMove()
    {
        RCLCPP_WARN(rclcpp::get_logger(this->name()),"Destroying ManipulatorGroupNode Node");
    };
    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {
                InputPort<std::string>("group_state_target")
            }
            );
    }
    bool setRequest(Request::SharedPtr& request) override;
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
};
