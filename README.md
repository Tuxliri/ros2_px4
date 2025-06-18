\# ROS 2 + Gazebo Harmonic + Px4 docker

The repository contains a docker setup with ROS 2 Humble, Gazebo Harmonic, and Px4.

## Requirements

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]  
> The Ubuntu operating system is required due to GUI support (a different approach is needed for Windows).

## First launch

Open VS Code in the project directory.  
Go to the bottom left corner and click the blue icon with two arrows pointing toward each other. From the dropdown menu, choose **"Open Folder in Container..."** and wait for the docker to build. This may take up to 10 minutes with a slower internet connection.

> [!TIP]  
> For Windows and WSL 2 users, `Dockerfile.windows` and `compose.windows.yaml` have been prepared.

Run:

```
cd ..
sudo ./setup.sh
./build.sh
source install/setup.bash
```

The above commands will build your workspace. Then go to the PX4 directory to build the SITL (Software in the Loop) version of the firmware.
```bash
cd ~/PX4-Autopilot  
HEADLESS=1 make px4_sitl gz_x500  
```

After running the above commands, a Gazebo simulator window should appear with the added drone.

> [!NOTE]  
> To launch the drone simulation in Gazebo, the command `make px4_sitl gz_x500` must be used each time.

Once the simulation is running, open a new terminal and run:
```bash
# this step can also be added to a ROS launch file  
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888  
```

To check available topics, in a new terminal window run:
```bash
ros2 topic list  
```

Now you can inspect and subscribe to all drone topics.

Once you’re logged into the docker container, it will behave similarly to running ROS on the host computer. All GUI applications will use the host's default window manager, and you'll also have access to all host devices and GPU acceleration.  
The docker has [ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials.html) and most required dependencies preinstalled, as well as the [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) simulator.

## Getting Started

For those who haven't worked with the ROS 2 + Gazebo environment, we recommend going through this tutorial: [Gazebo Tutorial](https://gazebosim.org/docs/harmonic/tutorials/).  
This will help you become familiar with the environment and create more advanced simulations in the future.

Next, to control robots properly in the simulation environment, a good starting point is this repository: [Gazebo ROS 2 Control](https://github.com/ros-controls/gz_ros2_control/).

As a final step and a practical summary, check out the tutorial provided by [Husarion](https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/) for several of their robots.

> [!IMPORTANT]  
After building, remember to run the command or work in a new terminal:  
>
> ```bash
> source ~/.bashrc  
> ```  
>
> This file already includes two important paths:
>
> ```bash
> /opt/ros/$ROS_DISTRO/setup.bash  
> /home/developer/ros2_ws/install/setup.bash  
> ```

## Example

1. Build the workspace along with the `simple_example` package.  
2. Launch `example.launch.py` to demonstrate how to connect Gazebo with ROS 2 to enable mutual communication.

## Additional materials

* [Getting Started](getting_started.md)
* [ROS 2 Command Cheat Sheet](cheatsheet.md)
* [ROS 2 Example packages in Python](example.md)
* [Bridge communication between ROS and Gazebo](ros_gz_bridge.md)