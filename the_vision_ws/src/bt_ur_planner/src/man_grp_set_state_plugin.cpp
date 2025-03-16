#include "bt_ur_planner/man_grp_set_state_plugin.hpp"
#include "behaviortree_ros2/plugins.hpp"

ManipulatorGroupMove::ManipulatorGroupMove(
            const std::string& instance_name, 
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params     
    ):
    RosServiceNode<the_interfaces::srv::GoToGroupState>(instance_name,conf,params)
{

};

bool ManipulatorGroupMove::setRequest(Request::SharedPtr& request)
{
    std::string group_state_name;
    getInput("group_state_target",group_state_name);
    request->set__group_state_name(group_state_name);
    return true;
}
NodeStatus ManipulatorGroupMove::onResponseReceived(const Response::SharedPtr& response)
{
    if(response->result)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
};
        

CreateRosNodePlugin(ManipulatorGroupMove,"MoveGroupPlanning");