#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "fixedeye_calibration/listen_broadcast_utils.hpp"

using namespace fixed_eye_calibration;

int main( int argc,char** argv)
{
    rclcpp::init(argc,argv);
    
    auto node = std::make_shared<ListenBroadCastTF>();
    node->get_pose_from_service();
    
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}