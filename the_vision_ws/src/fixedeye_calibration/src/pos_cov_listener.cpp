#include "fixedeye_calibration/pose_cov_listener.hpp"

namespace fixed_eye_calibration
{
    TransformListener::TransformListener():
    Node("transform_listener"),
    listen_wait_(40ms)
    {
        int sleep;
        this->declare_parameter<int>("listen_sleep",40);
        this->declare_parameter<int>("sample_number",5);

        listen_num_ = this->get_parameter("sample_number").as_int();
        sleep = this->get_parameter("listen_sleep").as_int();
        sleep *= std::pow(10,6);
        listen_wait_ = std::chrono::nanoseconds(sleep);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,true);

        pose_average_ = std::make_unique<WeightedPoseAverage>(MeasureType::POSE,WeightType::UNIFORM);

        listen_service_ = this->create_service<ListenService>(
            "~/get_pose_cov",
            std::bind(&TransformListener::ListenTF,this,_1,_2)
            );
        

    };

    TransformListener::~TransformListener()
    {};

    void TransformListener::ListenTF(std::shared_ptr<ListenServiceReq> req, std::shared_ptr<ListenServiceRes> res)
    {
        Transform t;
        Pose pose;
        rclcpp::Time old_time, act_time;
        PoseCovariance cov;
        RCLCPP_INFO(this->get_logger(),"Call listen Service");
        // try to see if exist transform 

        if(!tf_buffer_->canTransform(req->child_frame_id,req->frame_id,tf2::TimePointZero,&res->message))
            res->set__success(false);
        else
        {
            for(int i = 0; i < listen_num_; i++)
            {
                RCLCPP_INFO(this->get_logger(),"Listen");
                try
                {
                    t = tf_buffer_->lookupTransform(
                    req->frame_id,
                    req->child_frame_id,
                    tf2::TimePoint()
                );
                }
                catch(const std::exception& e)
                {
                   RCLCPP_ERROR(this->get_logger(),"%s",e.what());
                   res->set__success(false);
                   res->set__message("Error in tf lookupTransform " + std::string(e.what()));
                }
                if(i == 0)
                {
                    old_time = rclcpp::Time(t.header.stamp.sec,t.header.stamp.nanosec);
                    pose.set__orientation(t.transform.rotation);
                    
                    pose.position.set__x(t.transform.translation.x);
                    pose.position.set__y(t.transform.translation.y);
                    pose.position.set__z(t.transform.translation.z);
                    pose_average_->add_measure(pose);
                }
                else
                {
                    act_time = rclcpp::Time(t.header.stamp.sec,t.header.stamp.nanosec);
                    if((act_time>old_time))
                    {
                        pose.set__orientation(t.transform.rotation);
                        pose.position.set__x(t.transform.translation.x);
                        pose.position.set__y(t.transform.translation.y);
                        pose.position.set__z(t.transform.translation.z);

                        pose_average_->add_measure(pose);

                    }
                    else
                    {   
                        i--;
                        RCLCPP_WARN(this->get_logger(),"transform has not been updated");
                    }
                    
                }
                rclcpp::sleep_for(listen_wait_);
            }
            pose_average_->ave_and_cov_compute(cov,res->pose,false);
            RCLCPP_INFO_STREAM(this->get_logger(),cov);
            res->covariance.resize(49);
            auto res_cov = cov.reshaped<Eigen::RowMajor>().transpose();
            for(int i =0 ; i  < 49 ; i++)
                res->covariance[i] = res_cov[i];

            res->set__success(true);

            
        }
        
    }
}