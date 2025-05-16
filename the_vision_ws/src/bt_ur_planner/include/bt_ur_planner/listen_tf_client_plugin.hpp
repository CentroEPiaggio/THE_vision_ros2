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


// class plugin to call a tf listen service
class ListenTransformService : public RosServiceNode<the_interfaces::srv::ListenTransform>
{   
    public:
        ListenTransformService(const std::string& instance_name, const BT::NodeConfig& conf,
                            const BT::RosNodeParams& params);
        ~ListenTransformService() override
        {
            RCLCPP_WARN(rclcpp::get_logger(this->name()),"Destroying ListenTransformService");
        };
        static PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    InputPort<std::string>("frame_id"),
                    InputPort<std::string>("child_frame_id"),
                    InputPort<int>("sample"),
                    OutputPort<std::string>("pose_result"),
                    OutputPort<std::string>("covariance_result")
                }
                );
        }
        bool setRequest(Request::SharedPtr& request) override;
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    private:
        const std::string pose_type = "geometry_msgs/Pose";
        const std::string covariance_type = "std_msgs/Float32MultiArray";
};





// debug class plugin to print the response of the service
