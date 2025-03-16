#include "bt_ur_planner/execute_trajectory_plugin.hpp"
#include "behaviortree_ros2/plugins.hpp"

ExecuteTrajectory::ExecuteTrajectory(
                    const std::string& instance_name, 
                    const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params
                ):
    RosServiceNode<std_srvs::srv::Trigger>(instance_name,conf,params)
{
    RCLCPP_INFO(rclcpp::get_logger(this->name()),"Creating ExecuteTrajectory Node");
};

bool ExecuteTrajectory::setRequest(Request::SharedPtr& request)
{
    return true;
};

NodeStatus ExecuteTrajectory::onResponseReceived(const Response::SharedPtr& response)
{
    if(response->success)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
};

CreateRosNodePlugin(ExecuteTrajectory,"ExecutePlan");