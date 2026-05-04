#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace custom_behavior_tree
{

inline double euclidean_distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2) {
  return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

class MovePathStartPose : public BT::SyncActionNode
{
public:
  MovePathStartPose(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original path"),
      BT::InputPort<double>("new_start_pose_fraction", "Fraction of path length to move the start to (0.0 to 1.0)"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path with the moved start pose")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path input_path;
    double new_start_pose_fraction = 0.0;

    if (!getInput("input_path", input_path)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput("new_start_pose_fraction", new_start_pose_fraction)) {
      return BT::NodeStatus::FAILURE;
    }

    if (input_path.poses.size() < 2) {
      setOutput("output_path", input_path);
      return BT::NodeStatus::SUCCESS;
    }

    new_start_pose_fraction = std::clamp(new_start_pose_fraction, 0.0, 1.0);
    std::vector<double> accumulated_distances;
    accumulated_distances.push_back(0.0);
    double total_length = 0.0;

    for (size_t i = 1; i < input_path.poses.size(); ++i) {
      double dist = euclidean_distance(input_path.poses[i-1], input_path.poses[i]);
      total_length += dist;
      accumulated_distances.push_back(total_length);
    }

    double target_distance = total_length * new_start_pose_fraction;
    size_t split_index = 0;
    for (size_t i = 1; i < accumulated_distances.size(); ++i) {
      if (accumulated_distances[i] >= target_distance) {
        split_index = i;
        break;
      }
    }

    geometry_msgs::msg::PoseStamped split_pose = input_path.poses[split_index];
    if (split_index > 0) {
      const auto& p1 = input_path.poses[split_index - 1];
      const auto& p2 = input_path.poses[split_index];

      double segment_length = accumulated_distances[split_index] - accumulated_distances[split_index - 1];
      double remaining_dist = target_distance - accumulated_distances[split_index - 1];
      double ratio = (segment_length > 1e-6) ? (remaining_dist / segment_length) : 0.0;

      split_pose.header = p1.header;
      split_pose.pose.position.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
      split_pose.pose.position.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
      split_pose.pose.position.z = p1.pose.position.z + ratio * (p2.pose.position.z - p1.pose.position.z);

      tf2::Quaternion q1, q2, q_interp;
      tf2::fromMsg(p1.pose.orientation, q1);
      tf2::fromMsg(p2.pose.orientation, q2);
      q_interp = q1.slerp(q2, ratio);
      split_pose.pose.orientation = tf2::toMsg(q_interp);
    }

    nav_msgs::msg::Path output_path;
    output_path.header = input_path.header;

    output_path.poses.push_back(split_pose);

    for (size_t i = split_index; i < input_path.poses.size(); ++i) {
      output_path.poses.push_back(input_path.poses[i]);
    }

    for (size_t i = 0; i < split_index; ++i) {
      output_path.poses.push_back(input_path.poses[i]);
    }

    setOutput("output_path", output_path);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace custom_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_behavior_tree::MovePathStartPose>("MovePathStartPose");
}
