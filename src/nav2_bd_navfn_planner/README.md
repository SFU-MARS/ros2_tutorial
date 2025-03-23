# Navfn Planner

## Original Implementation

The NavfnPlanner is a global planner plugin for the Nav2 Planner server. It implements the Navigation Function planner with either A\* or Dijkstra expansions. It is largely equivalent to its counterpart in ROS 1 Navigation. The Navfn planner assumes a circular robot (or a robot that can be approximated as circular for the purposes of global path planning) and operates on a weighted costmap.

The original `global_planner` package from ROS (1) is a refactor on NavFn to make it more easily understandable, but it lacks in run-time performance and introduces suboptimal behaviors. As NavFn has been extremely stable for about 10 years at the time of porting, the maintainers felt no compelling reason to port over another, largely equivalent (but poorer functioning) planner. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-navfn.html) for additional parameter descriptions.

---

## Bidirectional A* Enhancement

This enhanced version of NavFn includes a bidirectional A* algorithm implementation while preserving the original Dijkstra and A* algorithms. This means you can choose between three path planning algorithms based on your needs:

1. **Dijkstra's algorithm** (original)
2. **A* algorithm** (original) 
3. **Bidirectional A* algorithm** (new)

The bidirectional A* searches from both the start and goal positions simultaneously, which can significantly improve performance for long paths. Key features include:

- **Bidirectional search**: Runs two simultaneous searches (forward from start and backward from goal)
- **Four-directional movement**: Uses the same movement model as the original A* implementation
- **Obstacle avoidance**: Properly respects costmap obstacles and creates paths around them
- **Efficient path reconstruction**: Traces the optimal path through the best meeting point
- **Gradient field**: Creates a smooth potential field for path following

### Usage

To use the bidirectional A* algorithm, set the following parameters in your configuration:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_bd_navfn_planner/NavfnPlanner"
      use_astar: false               # Disable regular A*
      use_bidirectional_astar: true  # Enable bidirectional A*
      allow_unknown: true            # Allow planning through unknown space
```

For the original algorithms:
- Dijkstra: Set both `use_astar: false` and `use_bidirectional_astar: false`
- A*: Set `use_astar: true` and `use_bidirectional_astar: false`

### Running the Demo

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
