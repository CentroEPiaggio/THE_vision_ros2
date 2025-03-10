#include "fixedeye_calibration/fixedeye_calibrator.hpp"

namespace fixed_eye_calibration
{

   
    FixedEyeCalibrator::FixedEyeCalibrator():
    Node("fixedeye_calibrator")
    {
        std::vector<double> init_pos,init_ori;
        //declare parameters
        this->declare_parameter<std::vector<double>>("init_position",{0.0,0.0,0.0});
        this->declare_parameter<std::vector<double>>("init_orientation",{0.0,0.0,0.0,0.0});
        this->declare_parameter<bool>("automatic_calibration",false);
        this->declare_parameter<int>("calibration_step_cond",20);
        this->declare_parameter<double>("position_norm2_cond",0.0);
        this->declare_parameter<double>("orientation_norm2_cond",0.0);
        this->declare_parameter<std::string>("camera_frame","camera_link");
        this->declare_parameter<std::string>("marker_frame","marker_1");
        
        
        
        // get initial position and set actual_est
        init_pos = this->get_parameter("init_position").as_double_array();
        init_ori = this->get_parameter("init_orientation").as_double_array();

        if(init_pos.size() != 3 || init_ori.size() != 4)
        {
            RCLCPP_ERROR(this->get_logger(),"Position or Orientation not correcly initialize");
            throw std::runtime_error("init position or oriantation has wrong dimension");        
        }
        
        actual_estimation_ << init_pos[0],init_pos[1],init_pos[2],init_ori[0],init_ori[1],init_ori[2],init_ori[3];

        // get the stop condition 
        cond_num_ = this->get_parameter("calibration_step_cond").as_int();
        
        if(cond_num_ == 0)
        {
            RCLCPP_INFO(this->get_logger(),"Iteration COndition parameter is Zero, Getting Position and Orientation N2 Condition");
            iteration_cond_ = false;
            ori_n2_ = this->get_parameter("orientation_norm2_cond").as_double();
            pos_n2_ = this->get_parameter("position_norm2_cond").as_double();
            if(ori_n2_ <= 0.0 || pos_n2_ <= 0.0)
            {
                RCLCPP_ERROR(this->get_logger(),"Error condition must be striclty greater then zero");
                throw std::runtime_error("Not Valid Error Condition");
            }
        }

        // create client
        tf_service_ = this->create_client<ListenSrv>("transform_listener/get_pose_cov");
        //create service
        if(!automatic_calibration_)
            calib_srv_ = this->create_service<TriggerSrv>(
            "/start_calib",
            std::bind(&FixedEyeCalibrator::calib_service,this,_1,_2)
        );
        
        //create average class
        weight_average_ = std::make_unique<WeightedPoseAverage>(MeasureType::POSE_W_COVARIANCE,WeightType::MAHALANOBIS);
        

    }

    void FixedEyeCalibrator::calib_service(std::shared_ptr<TriggerSrvReq> , std::shared_ptr<TriggerSrvRes> res)
    {
        TraslationWithCovariance t1,t2;
        RotationWithCovariance r1,r2;
        if(t1_resolved_.load() || t2_resolved_.load())
        {
            RCLCPP_ERROR(this->get_logger(),"Service is already running");
            res->success = false;
            return;
        }
        std::shared_ptr<ListenSrvReq> tf_req = std::make_shared<ListenSrvReq>();
        if(automatic_calibration_)
        {
            //TODO
        }
        else
        {
            
            // call service 1 and extract data 
            tf_req->frame_id = WORLDFRAME;
            tf_req->child_frame_id = ARUCOFRAME;

            tf_service_->async_send_request(tf_req,
            [this](rclcpp::Client<ListenSrv>::SharedFuture future) {
                auto res = future.get();
                set_rt_from_srv(res,this->t1,this->r1);
                this->notify_srv(true);   
            });   
            // RCLCPP_INFO(this->get_logger(),"value is %d",future1.valid());
            // future1.wait();
            // call service 2 and extract data
            tf_req->frame_id = this->get_parameter("camera_frame").as_string();
            tf_req->child_frame_id = this->get_parameter("marker_frame").as_string();
            

            auto fut = tf_service_->async_send_request(tf_req,
            [this,&t2,&r2](rclcpp::Client<ListenSrv>::SharedFuture future) {
                auto res = future.get();
                set_rt_from_srv(res,this->t2,this->r2);
                this->notify_srv(false);
            });  
            //wait both the service call resolution 
            // RCLCPP_INFO(this->get_logger(),"value is %d",future1.valid());
            
            
            

            // propagate measure
                      
            res->success = true;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&FixedEyeCalibrator::timer_callback,this));
            
        }
    }
    void FixedEyeCalibrator::timer_callback()
    {
        if(t1_resolved_.load() && t2_resolved_.load())
        {
            RCLCPP_INFO_STREAM(this->get_logger(),"Translation1 "<<t1.first.transpose()<<" and T2 "<<t2.first.transpose());
            // propagate measure
            TraslationWithCovariance tr;
            RotationWithCovariance rr;
            Eigen::Affine3d t1_e,t2_e;
            t1_e.translate(Eigen::Vector3d({1,2,3}));
            t1_e.rotate(r1.first.toRotationMatrix());
            t2_e.translate(t2.first);
            t2_e.rotate(r2.first.toRotationMatrix());
            RCLCPP_INFO_STREAM(this->get_logger(),"Affine 1 "<< t1_e.translation().transpose());
            RCLCPP_INFO_STREAM(this->get_logger(),"Affine 2 "<< t2_e.translation().transpose());

            auto aff_r = t2_e * t1_e.inverse();
            RCLCPP_INFO_STREAM(this->get_logger(),"Affine "<< aff_r.translation().transpose());
            tr = propagate_trasnlation(t1,t2,r1.first,r2.first);
            rr = propagate_rotation(r1,r2);
            geometry_msgs::msg::Pose pose,err;
            pose.position.x = tr.first(0);
            pose.position.y = tr.first(1);
            pose.position.z = tr.first(2);
            pose.orientation.w = rr.first.w();  
            pose.orientation.x = rr.first.x();
            pose.orientation.y = rr.first.y();
            pose.orientation.z = rr.first.z();
            PoseCovariance cov;
            cov.block<3,3>(0,0) = tr.second;
            cov.block<4,4>(3,3) = rr.second;
            // add measure to the average
            weight_average_->add_measure(pose,cov);
            // update translaction 
            err = weight_average_->w_ave_compute(pose,WeightType::MAHALANOBIS,false);
            RCLCPP_INFO(this->get_logger(),"PASS");
            RCLCPP_INFO_STREAM(this->get_logger(),"Translation "<< pose.position.x << " " << pose.position.y << " " << pose.position.z);
            RCLCPP_INFO_STREAM(this->get_logger(),"Orientation "<< pose.orientation.w << " " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z);
            t1_resolved_.store(false);  
            t2_resolved_.store(false);  
            timer_->cancel();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for service to resolve");
        }
    }
    // propagate Traslation and traslation covariance using first order approximation
    TraslationWithCovariance FixedEyeCalibrator::propagate_trasnlation(TraslationWithCovariance t1, TraslationWithCovariance t2, Eigen::Quaterniond r1, Eigen::Quaterniond r2)
    {
        Eigen ::Matrix3d compose_R = r1.toRotationMatrix() * (r2.inverse()).toRotationMatrix();
        Eigen::Vector3d compose_t = t1.first - compose_R*t2.first;
        Eigen::Matrix3d compose_cov = t1.second + compose_R*t2.second*compose_R.transpose();
        return std::make_pair(compose_t,compose_cov);
    }

    // propagate Rotation and Rotation covariance using first order approximation
    RotationWithCovariance FixedEyeCalibrator::propagate_rotation(RotationWithCovariance r1, RotationWithCovariance r2)
    {
        Eigen::Quaterniond r2_inv = r2.first.inverse();
        Eigen::Matrix3d compose_R = r1.first.toRotationMatrix() * r2_inv.toRotationMatrix();
        Eigen::Matrix4d Jr1,Jr2;
        Jr1 << r2_inv.w(),-r2_inv.x(),-r2_inv.y(),-r2_inv.z(),
               r2_inv.x(),r2_inv.w(),-r2_inv.x(),-r2_inv.y(),
               r2_inv.y(),-r2_inv.z(),r2_inv.w(),r2_inv.x(),
               r2_inv.z(),r2_inv.y(),-r2_inv.z(),r2_inv.w();
        Jr2 << r1.first.w(),-r1.first.x(),-r1.first.y(),-r1.first.z(),
               r1.first.x(),r1.first.w(),-r1.first.z(),r1.first.y(),
               r1.first.y(),r1.first.z(),r1.first.w(),-r1.first.x(),
               r1.first.z(),-r1.first.y(),r1.first.x(),r1.first.w();

        Eigen::Matrix4d compose_cov = Jr1*r1.second*Jr1.transpose() + Jr2*r2.second*Jr2.transpose();
        Eigen::Quaterniond compose_q(compose_R);
        return std::make_pair(compose_q,compose_cov);
    }
    

}