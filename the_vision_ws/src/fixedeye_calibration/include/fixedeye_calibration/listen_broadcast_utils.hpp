#include "fixedeye_calibration/pose_cov_listener.hpp"

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "the_interfaces/srv/send_pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"


namespace fixed_eye_calibration
{
    using UpdateSrv = the_interfaces::srv::SendPose;
    using UpdateSrvRes = UpdateSrv::Response;
    using UpdateSrvReq = UpdateSrv::Request;
    using GetParamSrv = rcl_interfaces::srv::GetParameters;
    using GetParamSrvReq = GetParamSrv::Request;
    using GetParamSrvRes = GetParamSrv::Response;

    class ListenBroadCastTF : public TransformListener
    {
        public:
            ListenBroadCastTF();

            bool get_pose_from_service();
            
            
        private:
            void broad_funct();

            void update_pose(const UpdateSrvReq::SharedPtr req, const UpdateSrvRes::SharedPtr res);


            std::string frame_id_, child_frame_id_;
            int srv_wait_thick_;
            geometry_msgs::msg::Transform actual_tansform_;

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Service<UpdateSrv>::SharedPtr update_srv_;
            rclcpp::Client<GetParamSrv>::SharedPtr get_p_srv_;

            rclcpp::CallbackGroup::SharedPtr cb_;

            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;

            

    };
}