# Adapted NavFn Planner with Bidirectional A*

An enhanced version of the NavFn planner that introduces bidirectional path planning while maintaining Ros2 and Nav2 compatibility. This implementation is adapted from:
- Nav2's  [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner)
- Ivanbgd's [bidirectional_a_star_implementation](https://github.com/ivanbgd/A-Star_Algorithm/blob/master/Bidirectional_A-Star.py)


## Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_bd_navfn_planner/NavfnPlanner"
      tolerance: 0.5                 # How close to goal we consider success (meters)
      use_bidirectional_astar: true  # Defaults to true if not specified
      allow_unknown: true            # Whether to allow planning through unknown space
```

## Implementation Notes

The planner optimizes path finding through several key strategies:

1. Adaptive Distance-Based Planning:
   ```cpp
   // Quick path for nearby goals - avoid bidirectional search overhead
   float direct_distance = hypot(goal_x - start_x, goal_y - start_y);
   if (direct_distance < 5.0) {
       // Initialize potential field with goal at zero
       potarr[goalCell] = 0;
       // Simple gradient calculation follows...
   }
   ```

2. Efficient Search Strategy:
   ```cpp
   // Balance search effort by adjusting expansion rates when queues become uneven
   if (startQueue.size() > 2 * goalQueue.size()) {
       // Slow down forward search, speed up backward search
       forward_expand_rate = 1;
       backward_expand_rate = 2;
   } else if (goalQueue.size() > 2 * startQueue.size()) {
       // Slow down backward search, speed up forward search
       forward_expand_rate = 2;
       backward_expand_rate = 1;
   }
   ```

3. Path Quality Improvements:
   ```cpp
   // Define straight movement directions (up, down, left, right)
   const int dx_ortho[4] = {0, 1, 0, -1};
   const int dy_ortho[4] = {-1, 0, 1, 0};
   
   // Apply different costs for straight vs diagonal movements
   float move_cost;
   if (dx == 0 || dy == 0) {
       // Straight movements get base cost
       move_cost = costarr[nbr];
   } else {
       // Diagonal movements cost ~1.414 times more
       move_cost = INVSQRT2 * costarr[nbr];
   }
   ```

## Running the demo on the simulation

To test the bidirectional A* planner with a TurtleBot3 in simulation:

1. Build the package:
```bash
colcon build --packages-select nav2_bd_navfn_planner --symlink-install
```

2. Source the setup file:
```bash
source install/setup.bash
```

3. Launch the Gazebo simulation with TurtleBot3:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

4. In a new terminal, launch the navigation stack with the bidirectional A* planner:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml params_file:=/workspaces/ros2_tutorial/src/nav2_bd_navfn_planner/burger.yaml
```
