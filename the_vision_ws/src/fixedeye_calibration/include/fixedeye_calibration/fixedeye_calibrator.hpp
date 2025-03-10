#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "the_interfaces/srv/listen_transform.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "fixedeye_calibration/pose_weighted_average.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <future>
#include <chrono>
#include <atomic>

#define WORLDFRAME "world"
#define ARUCOFRAME "aruco_frame"
namespace fixed_eye_calibration
{   
    using namespace std::chrono_literals;
    using  std::placeholders::_1, std::placeholders::_2;
    using ListenSrv = the_interfaces::srv::ListenTransform;
    using ListenSrvReq = ListenSrv::Request;
    using ListenSrvRes = ListenSrv::Response;
    using TriggerSrv = std_srvs::srv::Trigger;
    using TriggerSrvReq = TriggerSrv::Request;
    using TriggerSrvRes = TriggerSrv::Response;
    typedef std::pair<Eigen::Vector3d,Eigen::Matrix3d> TraslationWithCovariance;
    typedef std::pair<Eigen::Quaterniond,Eigen::Matrix4d> RotationWithCovariance;
    class FixedEyeCalibrator : public rclcpp::Node
    {
        public :
            
            FixedEyeCalibrator();

            


        private:
            void timer_callback();  

            void calib_service(std::shared_ptr<TriggerSrvReq> req, std::shared_ptr<TriggerSrvRes> res);

            TraslationWithCovariance propagate_trasnlation(TraslationWithCovariance t1, TraslationWithCovariance t2,Eigen::Quaterniond r1,  Eigen::Quaterniond r2);

            RotationWithCovariance propagate_rotation(RotationWithCovariance r1, RotationWithCovariance r2);

            void set_rt_from_srv(ListenSrvRes::SharedPtr srv_res,TraslationWithCovariance& t1, RotationWithCovariance& r1)
            {
                t1.first = Eigen::Vector3d(srv_res->pose.position.x,srv_res->pose.position.y,srv_res->pose.position.z);
                t1.second << srv_res->covariance[0],srv_res->covariance[1],srv_res->covariance[2],
                             srv_res->covariance[7],srv_res->covariance[8],srv_res->covariance[9],
                             srv_res->covariance[14],srv_res->covariance[15],srv_res->covariance[16];
                RCLCPP_INFO_STREAM(this->get_logger(),"First pos:\n " << t1.first);
                r1.first = Eigen::Quaternion(srv_res->pose.orientation.w,srv_res->pose.orientation.x,srv_res->pose.orientation.y,srv_res->pose.orientation.z);
                r1.second << srv_res->covariance[24],srv_res->covariance[25],srv_res->covariance[26],srv_res->covariance[27],
                            srv_res->covariance[31],srv_res->covariance[32],srv_res->covariance[33],srv_res->covariance[34],
                            srv_res->covariance[38],srv_res->covariance[39],srv_res->covariance[40],srv_res->covariance[41],
                            srv_res->covariance[45],srv_res->covariance[46],srv_res->covariance[47],srv_res->covariance[48];
                RCLCPP_INFO_STREAM(this->get_logger(),"First ori:\n " << r1.first);
            }

            void notify_srv( bool t1)
            {
                if(t1)
                {
                    RCLCPP_INFO(this->get_logger(),"Notify 1");
                    t1_resolved_.store(true);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),"Notify 2");
                    t2_resolved_.store(true);
                }
            };
            

            rclcpp::Client<ListenSrv>::SharedPtr tf_service_;   

            rclcpp::Service<TriggerSrv>::SharedPtr calib_srv_;
            
            std::unique_ptr<WeightedPoseAverage> weight_average_;



            bool automatic_calibration_, iteration_cond_;
            double ori_n2_=0.0,pos_n2_=0.0;
            int cond_num_;
            PoseMeasure actual_estimation_;
            std::promise<void> promise_T1_,promise_T2_;
            std::future<void> future_T1_,future_T2_;
            std::atomic<bool> t1_resolved_,t2_resolved_;
            bool t1_resolved_flag_,t2_resolved_flag_;   
            std::mutex mtx_;
            rclcpp::TimerBase::SharedPtr timer_;
            TraslationWithCovariance t1,t2;
            RotationWithCovariance r1,r2;
            

    };
}