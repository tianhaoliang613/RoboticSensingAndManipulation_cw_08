# CW1 Team 8 — Pick, Place, Detection, and Multi-Object Planning

ROS 2 (Humble) package for **COMP0250 Coursework 1**: MoveIt pick-and-place (**Task 1**), basket colour identification from RGB-D data (**Task 2**), and autonomous multi-colour pick-and-place (**Task 3**).

## Authors

- **Xinyan Guan**
- **Tianhao Liang**
- **Zehao Yu**

Team number: **8** (`cw1_team_8`).

## License

This package is released under the **MIT License** (see [`package.xml`](package.xml)).  
The coursework template and upstream materials from [`comp0250_s26_labs`](https://github.com/surgical-vision/comp0250_s26_labs) remain under their original terms.

## Workspace context

Submit **only** this package folder (`cw1_team_8`). Markers use their **own clean copy** of `cw1_world_spawner` and the rest of the labs stack. Your code must live in this repository layout:

- Parent workspace root (e.g. `comp0250_s26_labs`) contains `courseworks/` with `cw1_team_8`, `cw1_world_spawner`, `rpl_panda_with_rs`, etc. (as in the official repo).

## Prerequisites

- **ROS 2 Humble**
- Full coursework workspace cloned from the official repository (so dependencies such as `cw1_world_spawner` and `rpl_panda_with_rs` are present).
- Typical packages (see also the coursework PDF / `install_ros2_deps.sh` in the labs repo):

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

If you use Conda, run `conda deactivate` before building or launching.

## Build

From the **workspace root** (directory that contains `courseworks/`), **not** from inside `cw1_team_8` alone:

```bash
cd /path/to/comp0250_s26_labs
source /opt/ros/humble/setup.bash
colcon build --mixin release
source install/setup.bash
```

To build only this package (after dependencies are already built):

```bash
colcon build --packages-select cw1_team_8 --mixin release
source install/setup.bash
```

## Run (marking command)

Use a new shell after each successful build so `install/setup.bash` is applied.

```bash
cd /path/to/comp0250_s26_labs
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0
# On CS lab machines, if required:
# export ROS_LOCALHOST_ONLY=1

ros2 launch cw1_team_8 run_solution.launch.py \
  use_gazebo_gui:=true use_rviz:=true \
  enable_realsense:=true enable_camera_processing:=false \
  control_mode:=effort
```

What this does (see also `launch/run_solution.launch.py`):

1. Includes **`rpl_panda_with_rs`** (`display.launch.py`): Gazebo, MoveIt, robot, RealSense bridge, RViz as selected.
2. After **`launch_delay`** (default **5 s**), includes **`cw1_world_spawner`** so `/task` and scene spawning are available.
3. Starts **`cw1_solution_node`**, which hosts the three task services.

The node advertises:

- `/task1_start` — `cw1_world_spawner/srv/Task1Service`
- `/task2_start` — `cw1_world_spawner/srv/Task2Service`
- `/task3_start` — `cw1_world_spawner/srv/Task3Service`

### Triggering tasks

With the stack running, in a **second** terminal:

```bash
cd /path/to/comp0250_s26_labs
source /opt/ros/humble/setup.bash
source install/setup.bash
```

| Task | Command |
|------|---------|
| 1 | `ros2 service call /task cw1_world_spawner/srv/TaskSetup "{task_index: 1}"` |
| 2 | `ros2 service call /task cw1_world_spawner/srv/TaskSetup "{task_index: 2}"` |
| 3 | `ros2 service call /task cw1_world_spawner/srv/TaskSetup "{task_index: 3}"` |

The spawner prepares Gazebo and then calls the matching `/taskX_start` service on this node.

## Package layout

| Path | Role |
|------|------|
| `src/cw1_node.cpp` | Node entry: creates `cw1`, spins executors |
| `src/cw1_class.cpp` | Main task logic and MoveIt helpers |
| `include/cw1_class.h` | Class declaration and tunable parameters |
| `launch/run_solution.launch.py` | **Only** launch file used for evaluation |
| `srv/Example.srv` | Example service type; package builds generated interfaces |
| `package.xml` / `CMakeLists.txt` | Dependencies and build |

## Team contribution and rough effort

The coursework brief asks for **rough hours per task** and how work was shared, with **broadly equal overall contribution**. We do **not** use fine-grained percentage splits; roles were as follows.

**Who did what**

- **Xinyan Guan:** Basic framework for **Task 1** and **Task 3**.  
- **Tianhao Liang:** **Task 1** implementation; **Task 3** revised and completed until the solution **meets the coursework requirements** and runs successfully.  
- **Zehao Yu:** **Task 2** implementation.

**Approximate total hours per task (team — fill in if you keep records)**

| Task | Rough total hours (entire team) |
|------|----------------------------------|
| Task 1 | *e.g. ___ h* |
| Task 2 | *e.g. ___ h* |
| Task 3 | *e.g. ___ h* |

---

If the automated build fails on the markers’ side, they will follow **this README** first and will not attempt major fixes beyond what you document here.
