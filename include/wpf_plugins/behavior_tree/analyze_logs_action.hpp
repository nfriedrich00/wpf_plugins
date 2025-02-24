#ifndef WPF_PLUGINS__BEHAVIOR__ACTION__ANALYZE_LOGS_ACTION_HPP_
#define WPF_PLUGINS__BEHAVIOR__ACTION__ANALYZE_LOGS_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "wpf_msgs/action/analyze_logs.hpp"

namespace wpf_plugins
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps wpf_msgs::action::AnalyzeLogs
 */
class AnalyzeLogsAction : public nav2_behavior_tree::BtActionNode<wpf_msgs::action::AnalyzeLogs>
{
public:
  /**
   * @brief A constructor for wpf_plugins::AnalyzeLogsAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  AnalyzeLogsAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("logs_directory", ""),
        BT::InputPort<bool>("overwrite_results", false, "Overwrite logs or not"),
        BT::InputPort<float>("start_time", 0.0f, "Start timestamp for the analysis interval"),
        BT::InputPort<float>("end_time", 0.0f, "End timestamp for the analysis interval"),
        BT::InputPort<float>("start_position", 0.0f, "Start position for the analysis interval"),
        BT::InputPort<float>("end_position", 0.0f, "End position for the analysis interval")
      });
  }
};

}  // namespace wpf_plugins

#endif  // WPF_PLUGINS__BEHAVIOR__ACTION__ANALYZE_LOGS_ACTION_HPP_
