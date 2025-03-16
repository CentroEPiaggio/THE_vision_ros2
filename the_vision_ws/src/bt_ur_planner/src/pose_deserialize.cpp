#include "bt_ur_planner/pose_deserialize.hpp"
#include "behaviortree_cpp/bt_factory.h"

PoseDeserializeUtility::PoseDeserializeUtility(const std::string& name, const NodeConfig& conf):
SyncActionNode(name,conf),
buffer_({}),
deserialize_(RosMsgParser::ROS2_Deserializer())
{
    parser_ = std::make_unique<RosMsgParser::Parser>(
         "pose",
        RosMsgParser::ROSType(msg_type_),
        RosMsgParser::GetMessageDefinition(msg_type_)
    );
};

NodeStatus PoseDeserializeUtility::tick()
{
    geometry_msgs::msg::Pose pose;
    std::string output;
    getInput("x",pose.position.x);
    getInput("y",pose.position.y);
    getInput("z",pose.position.z);
    getInput("qx",pose.orientation.x);
    getInput("qy",pose.orientation.y);
    getInput("qz",pose.orientation.z);
    getInput("qw",pose.orientation.w);

    buffer_ = RosMsgParser::BuildMessageBuffer(
        pose,
        msg_type_
    );
    parser_->deserializeIntoJson(
        buffer_,
        &output,
        &deserialize_);
    
    setOutput("out",output);

    return NodeStatus::SUCCESS;

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<PoseDeserializeUtility>("DeserializePose");
}