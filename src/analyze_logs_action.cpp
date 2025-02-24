#include <string>
#include <memory>
#include <cmath>

#include "wpf_plugins/behavior_tree/analyze_logs_action.hpp"

namespace wpf_plugins
{

AnalyzeLogsAction::AnalyzeLogsAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<wpf_msgs::action::AnalyzeLogs>(xml_tag_name, action_name, conf)
{
}

void AnalyzeLogsAction::initialize()
{
  std::string logs_directory;
  getInput("logs_directory", logs_directory);
  goal_.logs_directory = logs_directory;
  bool overwrite_results;
  getInput("overwrite_results", overwrite_results);
  goal_.overwrite_results = overwrite_results;
  double start_time;
  getInput("start_time", start_time);
  goal_.start_time = static_cast<float>(start_time);
  double end_time;
  getInput("end_time", end_time);
  goal_.end_time = static_cast<float>(end_time);
  getInput("start_position", start_position);
  goal_.start_position = static_cast<float>(start_position);
  double end_position;
  getInput("end_position", end_position);
  goal_.end_position = static_cast<float>(end_position);
}

void AnalyzeLogsAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  increment_recovery_count();
}

}  // namespace wpf_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<wpf_plugins::AnalyzeLogsAction>(name, "analyze_logs", config);
    };

  factory.registerBuilder<wpf_plugins::AnalyzeLogsAction>("AnalyzeLogs", builder);
}
