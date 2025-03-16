#include "bt_ur_planner/update_tf_plugin.hpp"
#include "behaviortree_ros2/plugins.hpp"

UpdateTransform::UpdateTransform(const std::string& instance_name, const BT::NodeConfig& conf,
                            const BT::RosNodeParams& params):
    RosServiceNode<the_interfaces::srv::SendPose>(instance_name, conf, params)
{
    pose_average_ = std::make_unique<fixed_eye_calibration::WeightedPoseAverage>(
        fixed_eye_calibration::MeasureType::POSE_W_COVARIANCE,
        fixed_eye_calibration::WeightType::MAHALANOBIS
    );
    RCLCPP_INFO(rclcpp::get_logger(this->name()),"Creating ListenTransformService");
};


bool UpdateTransform::setRequest(Request::SharedPtr& request)
{
    //Ros msgs
    geometry_msgs::msg::Pose pose1,pose2;
    std_msgs::msg::Float64MultiArray cov1,cov2;


    // rosx inspector stuff
    RosMsgParser::Parser pose_parser("pose",
        RosMsgParser::ROSType(pose_type),
        RosMsgParser::GetMessageDefinition(pose_type));
    RosMsgParser::Parser cov_parser("covariance",
        RosMsgParser::ROSType(covariance_type),
        RosMsgParser::GetMessageDefinition(covariance_type));
    RosMsgParser::ROS2_Serializer serialize;
    RosMsgParser::ROS2_Deserializer deserialize;
    std::vector<u_int8_t> buffer;
    std::string data_values;
    

    //get first input
    getInput("pose1",data_values);
    pose_parser.serializeFromJson(data_values,&serialize);
    pose1 = RosMsgParser::BufferToMessage<geometry_msgs::msg::Pose>(
        serialize.getBufferData(),
        serialize.getBufferSize()
    );
    getInput("covariance1",data_values);
    cov_parser.serializeFromJson(data_values,&serialize);
    cov1 = RosMsgParser::BufferToMessage<std_msgs::msg::Float64MultiArray>
    (
        serialize.getBufferData(),
        serialize.getBufferSize()
    );

    //get second input
    getInput("pose2",data_values);
    pose_parser.serializeFromJson(data_values,&serialize);
    pose2 = RosMsgParser::BufferToMessage<geometry_msgs::msg::Pose>(
        serialize.getBufferData(),
        serialize.getBufferSize()
    );
    getInput("covariance2",data_values);
    cov_parser.serializeFromJson(data_values,&serialize);
    cov2 = RosMsgParser::BufferToMessage<std_msgs::msg::Float64MultiArray>
    (
        serialize.getBufferData(),
        serialize.getBufferSize()
    );

    // request->set__target_pose(tranform(
    //     pose1,
    //     pose2,
    //     cov1,
    //     cov2
    // ));
    request->set__target_pose(pose1);
    RCLCPP_INFO(rclcpp::get_logger("DIO"),"pass");
    return true;
};

geometry_msgs::msg::Pose UpdateTransform::tranform(
            geometry_msgs::msg::Pose p1,
            geometry_msgs::msg::Pose p2,
            std_msgs::msg::Float64MultiArray c1,
            std_msgs::msg::Float64MultiArray c2
)
{
    geometry_msgs::msg::Pose result;
    Eigen::Affine3d pose1, pose2, poser;
    Eigen::Quaterniond q1,q2,qr;
    convert_pose_to_eigen(p1,q1,pose1);
    convert_pose_to_eigen(p2,q2,pose2);

    poser = pose1*pose2.inverse();

    qr = Eigen::Quaterniond(poser.rotation());

    result.position.set__x(poser.translation()(0));
    result.position.set__y(poser.translation()(1));
    result.position.set__z(poser.translation()(2));

    result.orientation.set__w(qr.w());
    result.orientation.set__x(qr.x());
    result.orientation.set__y(qr.y());
    result.orientation.set__z(qr.z());

    return result;
}

BT::NodeStatus UpdateTransform::onResponseReceived(const Response::SharedPtr& response)
{
    if(response->result)
        return NodeStatus::SUCCESS;
    else
        return  NodeStatus::FAILURE;
};

CreateRosNodePlugin(UpdateTransform, "UpdateTF2");