# Adapted NavFn Planner with Bidirectional A*

An enhanced version of the NavFn planner that introduces bidirectional path planning while maintaining ROS 2 Nav2 compatibility. This implementation is adapted from the original NavFn planner in the ROS 2 Navigation Stack and draws inspiration from bidirectional search implementations at [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner) and [ivanbgd's bidirectional A* implementation](https://github.com/ivanbgd/A-Star_Algorithm/blob/master/Bidirectional_A-Star.py).

## Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_bd_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_bidirectional_astar: true  # Defaults to true if not specified
      allow_unknown: true
```

## Implementation Notes

The planner makes several key decisions to optimize performance:

1. For goals closer than 5.0 units:
   - Uses direct path planning
   - Creates simple gradient to goal
   - Avoids overhead of bidirectional search

2. For longer paths:
   - Runs simultaneous searches from start and goal
   - Dynamically adjusts expansion rates
   - Increases meeting point checks in later stages

3. Path smoothing:
   - Prioritizes orthogonal movements first
   - Adds diagonal movements for completeness
   - Uses weighted costs for natural paths

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
