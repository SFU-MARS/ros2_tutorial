# ROS2 Examples
- [ROS2 Examples](#ros2-examples)
  - [Quickstart for nav2](#quickstart-for-nav2)
    - [Host computer](#host-computer)
    - [Dev container](#dev-container)
    - [Robot (real world, e.g. TurtleBot3)](#robot-real-world-eg-turtlebot3)
    - [Known issues](#known-issues)
  - [Installing packages that come with the dev container](#installing-packages-that-come-with-the-dev-container)
  - [Building the dev container](#building-the-dev-container)
    - [Prerequisites](#prerequisites)
    - [Open it in vscode](#open-it-in-vscode)
  - [Tips / other resources](#tips--other-resources)
    - [Forcing simulations to run locally (advanced)](#forcing-simulations-to-run-locally-advanced)
      - [On host computer](#on-host-computer)
      - [Inside the devcontainer](#inside-the-devcontainer)
    - [Rosbags](#rosbags)
      - [Tools for working with rosbags (and ROS2 in general)](#tools-for-working-with-rosbags-and-ros2-in-general)
      - [Recording and playing back data using rosbags and plotjuggler](#recording-and-playing-back-data-using-rosbags-and-plotjuggler)

Example workspace for using ROS2 Humble with navigation 2 installed. 

Initial workspace template is built upon [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace)

* An example of a basic publisher and subscriber can be found [here](src/controller/controller/controller.py).
* An example of a basic launch file can be found [here](src/controller/launch/experiment.launch.py)

Note the necessary changes to the default `src/controller/setup.py` file to correctly run the node and launch file.
