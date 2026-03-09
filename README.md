# RoboticSensingAndManipulation CW1

ROS 2 / MoveIt coursework workspace for COMP0250. The main implementation is in
`courseworks/cw1_team_8`.

## Repository Structure

- `courseworks/cw1_team_8`: solution package containing the node, helper
  functions, launch file, and package configuration
- `courseworks/cw1_world_spawner`: task generation and service trigger package
- `courseworks/rpl_panda_with_rs`: Panda, Gazebo, and camera launch stack

## Dependencies

This workspace expects ROS 2 Humble together with the MoveIt and PCL packages
used by the coursework.

```bash
sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-moveit-core \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-point-cloud-transport \
  ros-humble-gazebo-ros2-control \
  python3-catkin-pkg \
  python3-catkin-pkg-modules
```

Optional camera-processing packages:

```bash
sudo apt-get install -y \
  ros-humble-image-proc \
  ros-humble-depth-image-proc
```

If you use Conda, run `conda deactivate` before building or launching.

## Build

```bash
cd /home/leander/comp0250_s26_labs
source /opt/ros/humble/setup.bash
colcon build --packages-select cw1_team_8 --mixin release
source install/setup.bash
```

## Launch

Open a first terminal and run:

```bash
cd /home/leander/comp0250_s26_labs
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0
ros2 launch cw1_team_8 run_solution.launch.py \
  use_gazebo_gui:=true use_rviz:=true \
  enable_realsense:=true enable_camera_processing:=false \
  control_mode:=effort
```

This launches Gazebo, MoveIt, the controllers, `cw1_world_spawner`, and
`cw1_solution_node`.

## Trigger a Task

Open a second terminal after the system is ready:

```bash
cd /home/leander/comp0250_s26_labs
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then call `/task`:

Task 1:

```bash
ros2 service call /task cw1_world_spawner/srv/TaskSetup "{task_index: 1}"
```

Task 2:

```bash
ros2 service call /task cw1_world_spawner/srv/TaskSetup "{task_index: 2}"
```

Task 3:

```bash
ros2 service call /task cw1_world_spawner/srv/TaskSetup "{task_index: 3}"
```

## Main Files

- `courseworks/cw1_team_8/src/cw1_class.cpp`
- `courseworks/cw1_team_8/include/cw1_class.h`
- `courseworks/cw1_team_8/src/cw1_node.cpp`
- `courseworks/cw1_team_8/launch/run_solution.launch.py`

## Notes

- The solution node provides `/task1_start`, `/task2_start`, and `/task3_start`,
  but these are normally called by `cw1_world_spawner` after you trigger
  `/task`.
- For a clean GitHub repository, generated directories such as `build/`,
  `install/`, and `log/` are usually excluded from version control.
