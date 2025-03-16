#include "behaviortree_ros2/bt_service_node.hpp"

#include "std_srvs/srv/trigger.hpp"

using namespace BT;

class ExecuteTrajectory : public RosServiceNode<std_srvs::srv::Trigger>
{
    public:
        ExecuteTrajectory(
                const std::string& instance_name, 
                    const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params
        );
        ~ExecuteTrajectory()
        {
            RCLCPP_WARN(rclcpp::get_logger(this->name()),"Destroying ExecuteTrajectory Node");
        };
        static PortsList providedPorts()
        {
            return providedBasicPorts({});
        }
        bool setRequest(Request::SharedPtr& request) override;
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
        

};