#include "rclcpp/rclcpp.hpp"
#include "the_interfaces/srv/listen_transform.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "fixedeye_calibration/pose_weighted_average.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include <chrono>
#include <functional>
#include <memory>

namespace fixed_eye_calibration
{
    using namespace std::chrono_literals;
    using  std::placeholders::_1, std::placeholders::_2;
    using Transform = geometry_msgs::msg::TransformStamped;
    using Pose = geometry_msgs::msg::Pose;
    using ListenService = the_interfaces::srv::ListenTransform;
    using ListenServiceReq = ListenService::Request;
    using ListenServiceRes = ListenService::Response;

    class TransformListener : public rclcpp::Node
    {
    private:
        std::chrono::nanoseconds listen_wait_;
        int listen_num_ = 0;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<WeightedPoseAverage> pose_average_;
        rclcpp::Service<ListenService>::SharedPtr listen_service_;

        void ListenTF(std::shared_ptr<ListenServiceReq> req, std::shared_ptr<ListenServiceRes> res);
    public:
        TransformListener(std::string name="transform_listener");

        ~TransformListener();

       
    };
    
    
    

};



