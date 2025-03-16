#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

#include "behaviortree_ros2/bt_service_node.hpp"

#include "the_interfaces/srv/listen_transform.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "fixedeye_calibration/pose_weighted_average.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace BT;

class TransformAverageAction : public SyncActionNode
{
    public:
        TransformAverageAction(const std::string& name, const NodeConfig& config);
        ~TransformAverageAction();

        NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return { 
                InputPort<std::string>("pose1"),
                InputPort<std::string>("covariance1"),
                InputPort<std::string>("pose2"),
                InputPort<std::string>("covariance2"),
                OutputPort<std::string>("average_pose"),
                OutputPort<std::string>("average_covariance")
             };
        }


    private:

        inline void convert_pose_to_eigen(geometry_msgs::msg::Pose pose, Eigen::Quaterniond& q, Eigen::Affine3d& t)
        {
            q = Eigen::Quaterniond(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
                );
            t = Eigen::Affine3d();
            t.translation() << pose.position.x,pose.position.y,pose.position.z;
            t.linear() = q.toRotationMatrix();
        }
        inline void convert_vect_to_eigen(std_msgs::msg::Float64MultiArray msg, fixed_eye_calibration::PoseCovariance& cov)
        {
            cov = fixed_eye_calibration::PoseCovariance().Zero();
            cov(0,0) = 1;
            RCLCPP_INFO(rclcpp::get_logger("debug"), "convert %d",msg.data.size());
            for(int i = 0; i < 7; i++)
            {
                for(int j = 0; j < 7; j++)
                {
                    RCLCPP_INFO(rclcpp::get_logger("debug"), "convert %d, %d,%d",j + 7*i, i, j);
                    cov(i,j) = msg.data[j + 7*i];
                    
                }
            }
        }

        std::unique_ptr<fixed_eye_calibration::WeightedPoseAverage> pose_weighted_average_;
        RosMsgParser::ROS2_Deserializer deserializer_;
        RosMsgParser::ROS2_Serializer serializer_;
        std::unique_ptr<RosMsgParser::Parser> parser_pose_,  parser_covariance_;
        std::vector<uint8_t> buffer_;
        Eigen::Affine3d pose1_, pose2_;
        Eigen::Quaterniond quat1_, quat2_;
        fixed_eye_calibration::PoseCovariance cov1_, cov2_;
        const std::string pose_type_ = "geometry_msgs/Pose";
        const std::string covariance_type_ = "std_msgs/Float64MultiArray";

};