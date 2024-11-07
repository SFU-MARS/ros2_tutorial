# ROS2 Examples
[Building the container](#building-the-dev-container) | [Nav2 Quickstart](#quickstart-for-nav2) | [Misc tips](#tips--other-resources)

Example workspace for using ROS2 Humble with navigation 2 installed. 

Initial workspace template is built upon [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace)

* An example of a basic publisher and subscriber can be found [here](src/controller/controller/controller.py).
* An example of a basic launch file can be found [here](src/controller/launch/experiment.launch.py)

Note the necessary changes to the default `src/controller/setup.py` file to correctly run the node and launch file.

# Quickstart for nav2
## Host computer
1. [Build the dev container](#building-the-dev-container)
2. Run `ifconfig` in a terminal on the **host** to obtain the desired network interface 
  - It should be something like `eth0` or `enp39s0` for ethernet connections, or `wlan0` for wireless connections

## Dev container
1. Replace `eth0` in `cyclonedds.xml` with the correct network interface obtained from the above
2. Run turtlebot3 simulation: `ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`

## Robot (real world, e.g. TurtleBot3)
1. Make sure to have the following in `~/.bashrc`:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5 # This domain ID should be the same as the computer's
export CYCLONEDDS_URI=~/ros2_ws/cyclonedds.xml # Make sure the workspace directory is correct
```
2. Install cyclonedds if needed: `sudo apt install ros-humble-rmw-cyclonedds-cpp`
3. Make sure the the network interface in `cyclonedds.xml` is correct


## Known issues
- If Gazebo does not load correctly (e.g. the turtlebot is not found), the following may help
  - Stop the simulation (Ctrl + C in terminal) and restart the simulation
  - Run `pkill -9 gzclient` and/or `pkill -9 gzserver` in the terminal before restarting the simulation

# Installing packages that come with the dev container
```
colcon build --symlink-install
source install/setup.bash
ros2 launch controller experiment.launch.py
```

# Building the dev container

## Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

If VSCode complains that `docker` cannot be run without `sudo`, run the following in terminal (from the [Docker postinstall instructions](https://docs.docker.com/engine/install/linux-postinstall/)).
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## Open it in vscode

Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder). 

When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

# Tips / other resources
## Forcing simulations to run locally (advanced)
This is useful for avoiding broadcasting any messages to the network

### On host computer
Run `ip link set lo multicast on` in terminal

### Inside the devcontainer
1. In `~/.bashrc`, change the line `export CYCLONEDDS_URI=/workspaces/ros2_tutorial/cyclonedds.xml` to `export CYCLONEDDS_URI=/workspaces/ros2_tutorial/cyclonedds_lo.xml`
2. Run `ros2 daemon stop && ros2 daemon start` in terminal

## Rosbags
Rosbags are used to record ROS data.

[Basic tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) | [Advanced tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced.html) | [Repo](https://github.com/ros2/rosbag2)

### Tools for working with rosbags (and ROS2 in general)
- PlotJuggler: [Website](https://plotjuggler.io/) | [Repo](https://github.com/facontidavide/PlotJuggler)
- This [Medium post](https://medium.com/evocargo/9-awesome-open-source-tools-to-manage-your-rosbags-b350fdb651c8) contains some other tools (potentially outdated)

### Recording and playing back data using rosbags and plotjuggler
1. `sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros` 
2. Record the desired topics to some output folder: `ros2 bag record <topics> -o <output_folder>`
  - Example: `ros2 bag record /tf /cmd_vel -o test`
  - A default file name will be used if the `-o` option is not used
  - **Warning**: Using `ros2 bag record --all` will usually result in _gigantic_ file sizes
3. `ros2 run plotjuggler plotjuggler`
4. Load data or stream from current topics
