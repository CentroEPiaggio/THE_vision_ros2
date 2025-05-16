#include "bt_ur_planner/bt_scheduler.hpp"

namespace the_bt_scheduler
{
    TheBTActionServer::TheBTActionServer(const rclcpp::NodeOptions options):
    TreeExecutionServer(options)
    {

    }

    TheBTActionServer::~TheBTActionServer()
    {   
                this->pose_ave_.reset();
    }

    void TheBTActionServer::onTreeCreated(BT::Tree& tree)
    {
        logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    }

    std::optional<std::string> TheBTActionServer::onTreeExecutionCompleted(
        BT::NodeStatus status,
            bool was_cancelled
    )
    {
        logger_cout_.reset();
        return std::nullopt;
    }

    void TheBTActionServer::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
    {

    }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<the_bt_scheduler::TheBTActionServer>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}
