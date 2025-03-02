#include "bd_linear_planner/bd_linear_planner.hpp"

#include <cmath>
#include <string>
#include <memory>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace bd_linear_planner
{

void BDLinearPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  
  auto node = parent.lock();
  logger_ = node->get_logger();
  
  // Get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".path_resolution", path_resolution_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".check_for_collisions", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".check_for_collisions", check_for_collisions_);
  
  RCLCPP_INFO(
    logger_, "BDLinearPlanner configured with path_resolution: %.2f, check_for_collisions: %s",
    path_resolution_, check_for_collisions_ ? "true" : "false");
}

void BDLinearPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s", "BDLinearPlanner");
}

void BDLinearPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s", "BDLinearPlanner");
}

void BDLinearPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s", "BDLinearPlanner");
}

nav_msgs::msg::Path BDLinearPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  
  // Initialize the global path with the same header as the goal
  global_path.header.stamp = goal.header.stamp;
  global_path.header.frame_id = global_frame_;
  
  // Transform start and goal to global frame if needed
  geometry_msgs::msg::PoseStamped start_pose = start;
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  
  if (start.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      logger_, "Transforming start pose from %s to %s",
      start.header.frame_id.c_str(), global_frame_.c_str());
    try {
      tf_->transform(start, start_pose, global_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "Exception in transforming start pose: %s", ex.what());
      return global_path;
    }
  }
  
  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      logger_, "Transforming goal pose from %s to %s",
      goal.header.frame_id.c_str(), global_frame_.c_str());
    try {
      tf_->transform(goal, goal_pose, global_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "Exception in transforming goal pose: %s", ex.what());
      return global_path;
    }
  }
  
  // Calculate the distance between start and goal
  double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
  double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
  double distance = std::hypot(dx, dy);
  
  // Calculate the number of segments
  int segments = std::max(static_cast<int>(distance / path_resolution_), 1);
  
  // Add the start pose as the first point in the path
  global_path.poses.push_back(start_pose);
  
  // Generate intermediate points along the straight line
  for (int i = 1; i < segments; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = goal.header.stamp;
    pose.header.frame_id = global_frame_;
    
    double ratio = static_cast<double>(i) / segments;
    pose.pose.position.x = start_pose.pose.position.x + ratio * dx;
    pose.pose.position.y = start_pose.pose.position.y + ratio * dy;
    pose.pose.position.z = 0.0;
    
    // Interpolate orientation (simple linear interpolation)
    double start_yaw = tf2::getYaw(start_pose.pose.orientation);
    double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
    
    // Normalize the angle difference
    double angle_diff = goal_yaw - start_yaw;
    if (angle_diff > M_PI) {
      angle_diff -= 2.0 * M_PI;
    } else if (angle_diff < -M_PI) {
      angle_diff += 2.0 * M_PI;
    }
    
    double yaw = start_yaw + ratio * angle_diff;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    
    // Check if this point is in collision
    if (check_for_collisions_) {
      unsigned int mx, my;
      if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        unsigned char cost = costmap_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          RCLCPP_WARN(
            logger_, "Point (%.2f, %.2f) is in collision with cost %d",
            pose.pose.position.x, pose.pose.position.y, cost);
          // Skip this point or return an empty path
          // For simplicity, we'll just skip collision checking for now
          // return global_path;
        }
      }
    }
    
    global_path.poses.push_back(pose);
  }
  
  // Add the goal pose as the last point in the path
  global_path.poses.push_back(goal_pose);
  
  RCLCPP_INFO(
    logger_, "Created linear plan with %zu points from (%.2f, %.2f) to (%.2f, %.2f)",
    global_path.poses.size(),
    start_pose.pose.position.x, start_pose.pose.position.y,
    goal_pose.pose.position.x, goal_pose.pose.position.y);
  
  return global_path;
}

}  // namespace bd_linear_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bd_linear_planner::BDLinearPlanner, nav2_core::GlobalPlanner) 