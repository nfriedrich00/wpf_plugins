#ifndef WPF_PLUGINS__BEHAVIOR__ACTION__PUBLISH_ACTION_HPP_
#define WPF_PLUGINS__BEHAVIOR__ACTION__PUBLISH_ACTION_HPP_

#include <string>
#include <memory>

#include "std_msgs/msg/empty.hpp"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"


namespace wpf_plugins
{

/**
 * @brief A Behavior Tree node that publishes std_msgs::msg::Empty at a specified frequency
 */
class PublishAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for PublishAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PublishAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "empty_topic", "Topic on which to publish"),
    };
  }
private:
  /**
   * @brief Called when leaving the RUNNING state
   */
  void halt() override {}

  /**
   * @brief Called by the BT engine once per tick
   */
  BT::NodeStatus tick() override;

  // Called once to set up publisher, timer, etc.
////  void initialize();

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string topic_name_;
};

}  // namespace wpf_plugins

#endif  // WPF_PLUGINS__BEHAVIOR__ACTION__PUBLISH_ACTION_HPP_
