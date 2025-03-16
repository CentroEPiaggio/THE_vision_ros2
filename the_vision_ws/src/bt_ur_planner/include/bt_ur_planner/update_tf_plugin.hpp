#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

#include "behaviortree_ros2/bt_service_node.hpp"

#include "the_interfaces/srv/listen_transform.hpp"
#include "the_interfaces/srv/send_pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "fixedeye_calibration/pose_weighted_average.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"


using namespace BT;

class UpdateTransform : public RosServiceNode<the_interfaces::srv::SendPose>
{
    public:
        UpdateTransform(
                const std::string& instance_name, 
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params
            );
        ~UpdateTransform()
        {
            RCLCPP_WARN(rclcpp::get_logger(this->name()),"Destroying UpdateTranformService");

        }

        static PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    InputPort<std::string>("pose1"),
                    InputPort<std::string>("covariance1"),
                    InputPort<std::string>("pose2"),
                    InputPort<std::string>("covariance2")
                }
                );
        }
        bool setRequest(Request::SharedPtr& request) override;
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    private:
        geometry_msgs::msg::Pose tranform(
            geometry_msgs::msg::Pose p1,
            geometry_msgs::msg::Pose p2,
            std_msgs::msg::Float64MultiArray c1,
            std_msgs::msg::Float64MultiArray c2
        );

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
            // RCLCPP_INFO(rclcpp::get_logger("debug"), "convert %d",msg.data.size());
            for(int i = 0; i < 7; i++)
            {
                for(int j = 0; j < 7; j++)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("debug"), "convert %d, %d,%d",j + 7*i, i, j);
                    cov(i,j) = msg.data[j + 7*i];
                    
                }
            }
        }

        std::unique_ptr<fixed_eye_calibration::WeightedPoseAverage> pose_average_;
        const std::string pose_type = "geometry_msgs/Pose";
        const std::string covariance_type = "std_msgs/Float32MultiArray";
};