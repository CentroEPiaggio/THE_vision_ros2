#include "bt_ur_planner/listen_tf_client_plugin.hpp"
#include "behaviortree_ros2/plugins.hpp"



ListenTransformService::ListenTransformService(const std::string& instance_name, const BT::NodeConfig& conf,
                            const BT::RosNodeParams& params):
    RosServiceNode<the_interfaces::srv::ListenTransform>(instance_name, conf, params)
{
    RCLCPP_INFO(rclcpp::get_logger(this->name()),"Creating ListenTransformService");
};

bool ListenTransformService::setRequest(Request::SharedPtr& request)
{
    std::string frame_id, child_frame_id;
    int sample;
    getInput("frame_id", frame_id);
    getInput("child_frame_id", child_frame_id);
    getInput("sample",sample);
    request->frame_id = frame_id;
    request->child_frame_id = child_frame_id;
    request->sample = sample;
    return true;
}

BT::NodeStatus ListenTransformService::onResponseReceived(const Response::SharedPtr& response)
{
    geometry_msgs::msg::Pose pose_result;
    std_msgs::msg::Float64MultiArray covariance_result;
    RosMsgParser::ROS2_Deserializer deserializer;
    std::vector<uint8_t> buffer;
    std::string pose_des, covariance_des;
    
    if(response->success == false)
    {
        return BT::NodeStatus::FAILURE;
    }

    
    //deserialize pose
    pose_result = response->pose;
    buffer = RosMsgParser::BuildMessageBuffer(pose_result,pose_type);
    RosMsgParser::Parser parser("pose",RosMsgParser::ROSType(pose_type),RosMsgParser::GetMessageDefinition(pose_type));
    parser.deserializeIntoJson(buffer,&pose_des,&deserializer);
    setOutput("pose_result", pose_des);
    RCLCPP_INFO(rclcpp::get_logger(this->name()),"pose: %s",pose_des.c_str());
    //deserialize covariance
    covariance_result.data.resize(response->covariance.size());
    for(uint64_t i = 0; i < response->covariance.size(); i++)
        covariance_result.data[i] = response->covariance[i];
    
    buffer = RosMsgParser::BuildMessageBuffer(covariance_result,covariance_type);
    parser = RosMsgParser::Parser("covariance",RosMsgParser::ROSType(covariance_type),RosMsgParser::GetMessageDefinition(covariance_type));
    parser.deserializeIntoJson(buffer,&covariance_des,&deserializer);
    setOutput("covariance_result", covariance_des);
    return BT::NodeStatus::SUCCESS;
}





CreateRosNodePlugin(ListenTransformService, "ListenTF2");

