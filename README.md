# ROS2 Examples
[Installation](#installation)

# Installation
```
colcon build --symlink-install
ros2 launch controller experiment.launch.py
```

## Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

To use docker with sudo and connect to the container with VS code following the [postinstall instructions](https://docs.docker.com/engine/install/linux-postinstall/).

`sudo groupadd docker`

`sudo usermod -aG docker $USER`

`newgrp docker`

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