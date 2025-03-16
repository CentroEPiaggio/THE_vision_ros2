#include "bt_ur_planner/transform_ave_tf.hpp"
#include "behaviortree_cpp/bt_factory.h"

TransformAverageAction::TransformAverageAction(const std::string& name, const BT::NodeConfig& config):
    SyncActionNode(name, config),
    // parser_pose_(RosMsgParser::Parser("pose",RosMsgParser::ROSType(pose_type_),RosMsgParser::GetMessageDefinition(pose_type_))),
    // parser_covariance_(RosMsgParser::Parser("covariance",RosMsgParser::ROSType(covariance_type_),RosMsgParser::GetMessageDefinition(covariance_type_))),
    deserializer_(RosMsgParser::ROS2_Deserializer()),
    serializer_(RosMsgParser::ROS2_Serializer()),
    buffer_(std::vector<uint8_t>())
{
    RCLCPP_INFO(rclcpp::get_logger(this->name()),"Creating TransformAverageAction");
    pose_weighted_average_ = std::make_unique<fixed_eye_calibration::WeightedPoseAverage>(
        fixed_eye_calibration::MeasureType::POSE_W_COVARIANCE,
        fixed_eye_calibration::WeightType::MAHALANOBIS
    );
    parser_pose_ = std::make_unique<RosMsgParser::Parser>(
        "pose",
        RosMsgParser::ROSType(pose_type_),
        RosMsgParser::GetMessageDefinition(pose_type_)
    );
    parser_covariance_ = std::make_unique<RosMsgParser::Parser>(
        "covariance",
        RosMsgParser::ROSType(covariance_type_),
        RosMsgParser::GetMessageDefinition(covariance_type_)
    );
};

TransformAverageAction::~TransformAverageAction()
{
    RCLCPP_INFO(rclcpp::get_logger(this->name()),"Destroying TransformAverageAction");
};

NodeStatus TransformAverageAction::tick()
{
    geometry_msgs::msg::Pose pose;
    std_msgs::msg::Float64MultiArray covariance;

    std::string data_str;
    // get first pose and covariance
    getInput("pose1", data_str);
    parser_pose_->serializeFromJson(data_str,&serializer_);
    pose = RosMsgParser::BufferToMessage<geometry_msgs::msg::Pose>(serializer_.getBufferData(),serializer_.getBufferSize());
    convert_pose_to_eigen(pose,quat1_,pose1_);
    getInput("covariance1",data_str);
    parser_covariance_->serializeFromJson(data_str,&serializer_);
    covariance = RosMsgParser::BufferToMessage<std_msgs::msg::Float64MultiArray>(serializer_.getBufferData(),serializer_.getBufferSize());
    convert_vect_to_eigen(covariance, cov1_);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->name()),"the pose is "<< pose1_.translation().transpose());


    
    // pose_weighted_average_->add_measure();
    return BT::NodeStatus::SUCCESS;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TransformAverageAction>("AverageTF");
}
