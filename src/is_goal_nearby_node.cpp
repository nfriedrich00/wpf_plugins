// AI generated version of https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/condition/is_goal_nearby_condition.cpp
// without dependencies, so that we can use it with ros2 jazzy from binaries before this node is available in the binaries.
// without having to build the entire nav2 stack from source.

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace custom_behavior_tree
{

// Helper functions to replace nav2_util/geometry_utils.hpp
inline double euclidean_distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2) {
  return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

inline double calculate_path_length(const nav_msgs::msg::Path& path, size_t start_index) {
  double len = 0.0;
  for (size_t i = start_index + 1; i < path.poses.size(); ++i) {
    len += euclidean_distance(path.poses[i - 1], path.poses[i]);
  }
  return len;
}

// Inline class definition replacing the .hpp
class IsGoalNearbyCondition : public BT::ConditionNode
{
public:
  IsGoalNearbyCondition(const std::string & condition_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf), transform_tolerance_(0.1)
  {
    // Try to get the node from the blackboard, using standard rclcpp_lifecycle node
    node_ = config().blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    node_->get_parameter_or("transform_tolerance", transform_tolerance_, 0.1);

    // Replicating deconflictPortAndParamFrame logic manually
    if (!getInput("global_frame", global_frame_)) {
      node_->get_parameter_or("global_frame", global_frame_, std::string("map"));
    }
    if (!getInput("robot_base_frame", robot_base_frame_)) {
      node_->get_parameter_or("robot_base_frame", robot_base_frame_, std::string("base_link"));
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check"),
      BT::InputPort<double>("proximity_threshold", "Distance threshold to goal"),
      BT::InputPort<double>("max_robot_pose_search_dist", -1.0, "Max path distance to search for closest point"),
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path new_path;
    double prox_thr = 0.0;
    double max_robot_pose_search_dist = -1.0;
    getInput("path", new_path);
    getInput("proximity_threshold", prox_thr);
    getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

    if (new_path.poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Path is empty");
      return BT::NodeStatus::FAILURE;
    }

    bool path_pruning = (max_robot_pose_search_dist >= 0.0);

    // Turn global search back on if path_pruning is disabled (-1.0)
    if (!path_pruning || new_path != path_) {
      path_ = new_path;
      closest_pose_detection_begin_ = path_.poses.begin();
    }

    // Replace nav2_util::getCurrentPose
    geometry_msgs::msg::PoseStamped pose;
    try {
      auto transform = tf_buffer_->lookupTransform(
        global_frame_, robot_base_frame_, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(transform_tolerance_));
        
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot pose: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    // Replace nav2_util::transformPoseInTargetFrame
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
      robot_pose = tf_buffer_->transform(pose, path_.header.frame_id, 
                                         tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to transform robot pose to path frame '%s': %s",
                   path_.header.frame_id.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    auto closest_pose_upper_bound = path_.poses.end();
    if (path_pruning) {
      double accumulated_dist = 0.0;
      closest_pose_upper_bound = closest_pose_detection_begin_;
      
      // Inline replacement for first_after_integrated_distance
      if (closest_pose_upper_bound != path_.poses.end()) {
        auto prev_it = closest_pose_upper_bound;
        ++closest_pose_upper_bound;
        while (closest_pose_upper_bound != path_.poses.end()) {
          accumulated_dist += euclidean_distance(*prev_it, *closest_pose_upper_bound);
          if (accumulated_dist > max_robot_pose_search_dist) {
            break;
          }
          prev_it = closest_pose_upper_bound;
          ++closest_pose_upper_bound;
        }
      }
    }

    // Use std::min_element to replace nav2_util::geometry_utils::min_by
    auto closest_pose_it = std::min_element(
      closest_pose_detection_begin_, closest_pose_upper_bound,
      [&robot_pose](const geometry_msgs::msg::PoseStamped & ps1, const geometry_msgs::msg::PoseStamped & ps2) {
        return euclidean_distance(robot_pose, ps1) < euclidean_distance(robot_pose, ps2);
      });

    closest_pose_detection_begin_ = closest_pose_it;

    const std::size_t closest_index = static_cast<std::size_t>(closest_pose_it - path_.poses.begin());
    const double remaining_length = calculate_path_length(path_, closest_index);

    return (remaining_length < prox_thr) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
  nav_msgs::msg::Path path_;
  std::vector<geometry_msgs::msg::PoseStamped>::iterator closest_pose_detection_begin_;
};

}  // namespace custom_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_behavior_tree::IsGoalNearbyCondition>("IsGoalNearby");
}
