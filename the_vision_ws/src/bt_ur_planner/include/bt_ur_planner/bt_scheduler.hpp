#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include "fixedeye_calibration/pose_weighted_average.hpp"
namespace the_bt_scheduler
{
    class TheBTActionServer : public BT::TreeExecutionServer
    {
    private:
        std::shared_ptr<fixed_eye_calibration::WeightedPoseAverage> pose_ave_;
        std::shared_ptr<BT::StdCoutLogger> logger_cout_;
        
    public:

        TheBTActionServer(const rclcpp::NodeOptions options);

        ~TheBTActionServer();

        void onTreeCreated(BT::Tree& tree) override;

        std::optional<std::string> onTreeExecutionCompleted(
            BT::NodeStatus status,
            bool was_cancelled) override;

        void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override;
    };
      
};
