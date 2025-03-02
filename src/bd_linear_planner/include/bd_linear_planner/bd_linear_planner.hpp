#ifndef BD_LINEAR_PLANNER__BD_LINEAR_PLANNER_HPP_
#define BD_LINEAR_PLANNER__BD_LINEAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace bd_linear_planner
{

class BDLinearPlanner : public nav2_core::GlobalPlanner
{
public:
  BDLinearPlanner() = default;
  ~BDLinearPlanner() = default;

  // Plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Plugin cleanup
  void cleanup() override;

  // Plugin activate
  void activate() override;

  // Plugin deactivate
  void deactivate() override;

  // Create a plan from start and goal poses
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // The global frame of the costmap
  std::string global_frame_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("BDLinearPlanner")};

  // Parameter for path resolution (distance between points)
  double path_resolution_;

  // Parameter for checking if the path is valid
  bool check_for_collisions_;
};

}  // namespace bd_linear_planner

#endif  // BD_LINEAR_PLANNER__BD_LINEAR_PLANNER_HPP_ 