#include "fixedeye_calibration/listen_broadcast_utils.hpp"
#include "rclcpp/callback_group.hpp"

namespace fixed_eye_calibration
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    ListenBroadCastTF::ListenBroadCastTF():
    TransformListener("listen_broadcast_tf")
    {
        std::string srv_name;
        // declare parameter 
        
        declare_parameter<std::string>("frame_id","world");
        declare_parameter<std::string>("child_frame_id","camera_link");
        declare_parameter<std::string>("node_name","the_calibration_planner");

        //get parameter
        frame_id_ = get_parameter("frame_id").as_string();
        child_frame_id_ = get_parameter("child_frame_id").as_string();

        // create client 
        srv_name = "/" + get_parameter("node_name").as_string() + "/get_parameters";
        get_p_srv_ = create_client<GetParamSrv>(srv_name);
        
    };

    bool ListenBroadCastTF::get_pose_from_service()
    {
        GetParamSrvReq::SharedPtr req = std::make_shared<GetParamSrvReq>();
        GetParamSrvRes::SharedPtr res = std::make_shared<GetParamSrvRes>();
        req->set__names({"camera_frame_position","camera_frame_orientation"});
        auto future = get_p_srv_->async_send_request(req);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            res = future.get();
            if(res->values.size() != 2)
            {
                RCLCPP_ERROR(this->get_logger(),"Not all parameter value are returned");
                return false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Get parameters");
                
                std::vector<double> pos = res->values[0].double_array_value, ori = res->values[1].double_array_value;

                actual_tansform_.translation.set__x(pos[0]);
                actual_tansform_.translation.set__y(pos[1]);
                actual_tansform_.translation.set__z(pos[2]);
                RCLCPP_INFO(get_logger(),"the position is [%f,%f,%f]",actual_tansform_.translation.x,actual_tansform_.translation.y,actual_tansform_.translation.z);
                
                actual_tansform_.rotation.set__w(ori[0]);
                actual_tansform_.rotation.set__x(ori[1]);
                actual_tansform_.rotation.set__y(ori[2]);
                actual_tansform_.rotation.set__z(ori[3]);
                RCLCPP_INFO(get_logger(),"the orientation is [%f,%f,%f%f]",actual_tansform_.rotation.w,actual_tansform_.rotation.x,actual_tansform_.rotation.y,actual_tansform_.rotation.z);
                //set up service, timer and boradcaster 
                tf_broad_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                cb_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                update_srv_ = create_service<UpdateSrv>(
                    "~/update_pose",
                    std::bind(&ListenBroadCastTF::update_pose,this,_1,_2),
                    rmw_qos_profile_services_default,
                    cb_
                    );

                timer_ = create_wall_timer(
                    std::chrono::milliseconds(20),
                    std::bind(&ListenBroadCastTF::broad_funct,this),
                    cb_);

                return true;
            }
        }
        else
            return false;
    }

    void ListenBroadCastTF::update_pose(const UpdateSrvReq::SharedPtr req, const UpdateSrvRes::SharedPtr res)
    {
        actual_tansform_.translation.set__x(req->target_pose.position.x);
        actual_tansform_.translation.set__y(req->target_pose.position.y);
        actual_tansform_.translation.set__z(req->target_pose.position.z);
        actual_tansform_.rotation.set__w(req->target_pose.orientation.w);
        actual_tansform_.rotation.set__x(req->target_pose.orientation.x);
        actual_tansform_.rotation.set__y(req->target_pose.orientation.y);
        actual_tansform_.rotation.set__z(req->target_pose.orientation.z);
        RCLCPP_INFO(this->get_logger(),"ASDASD");
        res->set__result(true);
    }

    void ListenBroadCastTF::broad_funct()
    {
        geometry_msgs::msg::TransformStamped tranf;
        tranf.set__child_frame_id(child_frame_id_);
        tranf.header.set__frame_id(frame_id_);
        tranf.header.set__stamp(this->get_clock()->now());
        tranf.set__transform(actual_tansform_);
        tf_broad_->sendTransform(tranf);
    }
}