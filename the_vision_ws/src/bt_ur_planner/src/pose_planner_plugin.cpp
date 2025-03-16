#include "bt_ur_planner/pose_planner_plugin.hpp"
#include "behaviortree_ros2/plugins.hpp"

SendPoseGoal::SendPoseGoal(
                    const std::string& instance_name, 
                    const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params
                    ):
    RosServiceNode<the_interfaces::srv::MoveTo>(instance_name,conf,params)
    {
        RCLCPP_INFO(rclcpp::get_logger(this->name()),"Creating SendPoseGoal Node");
    };
bool SendPoseGoal::setRequest(Request::SharedPtr& request)
{
    geometry_msgs::msg::Pose target_pose;
    RosMsgParser::Parser goal_parser(
        "pose",
        RosMsgParser::ROSType(pose_type_),
        RosMsgParser::GetMessageDefinition(pose_type_)
    );
    RosMsgParser::ROS2_Serializer serialize;
    std::vector<uint8_t> buffer;
    std::string goal_input;
    // get input pose target 
    getInput("target_pose",goal_input);
    goal_parser.serializeFromJson(
        goal_input,
        &serialize
    );
    target_pose = RosMsgParser::BufferToMessage<geometry_msgs::msg::Pose>(
        serialize.getBufferData(),
        serialize.getBufferSize()
    );
    // set target pose request
    request->set__target_pose(target_pose); 
    return true;
};

NodeStatus SendPoseGoal::onResponseReceived(const Response::SharedPtr& respose)
{
    if(respose->result)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
};


CreateRosNodePlugin(SendPoseGoal,"PosePlanning");

