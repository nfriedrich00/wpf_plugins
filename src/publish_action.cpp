#include "wpf_plugins/behavior_tree/publish_action.hpp"

namespace wpf_plugins
{

PublishAction::PublishAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

  getInput("topic_name", topic_name_);

  publisher_ = node_->create_publisher<std_msgs::msg::Empty>(topic_name_, 10);
  /*
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_frequency_));

  auto period = std::chrono::duration<double>(1.0 / publish_frequency_);

  publish_timer_ = node_->create_wall_timer(
    period_ns,
    [this]()
    {
      if (!rclcpp::ok()) {
        return;
      }
      std_msgs::msg::Empty msg;
      RCLCPP_DEBUG(node_->get_logger(), "Publishing empty message on: %s", topic_name_.c_str());
      publisher_->publish(msg);
    },
    callback_group_
  );
    */
}
/*
void PublishAction::initialize()
{
  // Retrieve ROS2 Node from the blackboard
  // This assumes that your Behavior Tree has stored a shared_ptr<rclcpp::Node> under "node"
  node_ = config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("node");

  // Read the input ports
  getInput("topic_name", topic_name_);
  getInput("publish_frequency", publish_frequency_);

  // Create publisher if not already created
  if (!publisher_) {
    publisher_ = node_->create_publisher<std_msgs::msg::Empty>(topic_name_, 10);
  }

  // Create or reset the timer
  // This timer will continuously publish at the given frequency
  auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
  publish_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]()
    {
      if (!rclcpp::ok()) {
        return;
      }
      std_msgs::msg::Empty msg;
      RCLCPP_DEBUG(node_->get_logger(), "Publishing empty message on: %s", topic_name_.c_str());
      publisher_->publish(msg);
    }
  );
}
  */
/*
PublishAction::~PublishAction()
{
  // Optionally cancel the timer (if you want to stop publishing)
  if (publish_timer_) {
    publish_timer_->cancel();
  }
}
  */

BT::NodeStatus PublishAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  publisher_->publish(std_msgs::msg::Empty());
  return BT::NodeStatus::RUNNING;
  /*
  // If not already initialized, do it once
  if (!node_ || !publisher_ || !publish_timer_) {
    try {
      initialize();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("PublishAction"), "Initialization failed: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  
  // Keep running indefinitely (never return SUCCESS)
  return BT::NodeStatus::RUNNING;
  */
}
/*

void PublishAction::halt()
{
  // Called when this node transitions out of RUNNING
  // Cancel any ongoing timers if you desire, or let them continue
  if (publish_timer_) {
    publish_timer_->cancel();
  }
  setStatus(BT::NodeStatus::IDLE);
}
  */

}  // namespace wpf_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wpf_plugins::PublishAction>("Publish");
}

/* 
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<wpf_plugins::AnalyzeLogsAction>(name, "analyze_logs", config);
    };

  factory.registerBuilder<wpf_plugins::PublishAction>("Publish", builder);
}
  */
