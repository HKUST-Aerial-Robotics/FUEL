# FUEL

**ROS 2 Humble port:** This repository provides a ROS 2 Humble adaptation of the original ROS 1
FUEL framework, maintained by Zhang RY <zry1286631670@gmail.com>. The original FUEL algorithm,
publication, and author attribution are preserved below.

__News:__

- Feb 24, 2023: the code for **multi-UAV exploration** is released! check [this link](https://github.com/SYSU-STAR/RACER).
- Aug 24, 2021: The CPU-based simulation is released, CUDA is no longer required. Richer exploration environments are provided.
  
**FUEL** is a powerful framework for **F**ast **U**AV **E**xp**L**oration.
Our method is demonstrated to complete challenging exploration tasks **3-8 times** faster than state-of-the-art approaches at the time of publication.
Central to it is a Frontier Information Structure (FIS), which maintains crucial information for exploration planning incrementally along with the online built map. Based on the FIS, a hierarchical planner plans frontier coverage paths, refine local viewpoints, and generates minimum-time trajectories in sequence to explore unknown environment agilely and safely. Try [Quick Start](#quick-start) to run a demo in a few minutes!  

<p align="center">
  <img src="files/1.gif" width = "400" height = "225"/>
  <img src="files/2.gif" width = "400" height = "225"/>
  <img src="files/3.gif" width = "400" height = "225"/>
  <img src="files/4.gif" width = "400" height = "225"/>
  <!-- <img src="files/icra20_1.gif" width = "320" height = "180"/> -->
</p>


Recently, we further develop a fully decentralized approach for exploration tasks using a fleet of quadrotors. The quadrotor team operates with asynchronous and limited communication, and does not require any central control. The coverage paths and workload allocations of the team are optimized and balanced in order to fully realize the system's potential. The associated paper has been published in IEEE TRO. Check code [here](https://github.com/SYSU-STAR/RACER).

<p align="center">
  <img src="files/racer1.gif" width = "400" height = "225"/>
  <img src="files/racer2.gif" width = "400" height = "225"/>
</p>

__Complete videos__: [video1](https://www.youtube.com/watch?v=_dGgZUrWk-8), [video2](https://www.bilibili.com/video/BV1yf4y1P7Vj).

__Authors__: [Boyu Zhou](http://sysu-star.com) from SYSU and [Shaojie Shen](http://uav.ust.hk/group/) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/).

Please cite our paper if you use this project in your research:
- [__FUEL: Fast UAV Exploration using Incremental Frontier Structure and Hierarchical Planning__](https://arxiv.org/abs/2010.11561), Boyu Zhou, Yichen Zhang, Xinyi Chen, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**) with ICRA 2021 option

```
@article{zhou2021fuel,
  title={FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning},
  author={Zhou, Boyu and Zhang, Yichen and Chen, Xinyi and Shen, Shaojie},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={2},
  pages={779--786},
  year={2021},
  publisher={IEEE}
}
```

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.

## Table of Contents

- [FUEL](#fuel)
  - [Table of Contents](#table-of-contents)
  - [Quick Start](#quick-start)
  - [Exploring Different Environments](#exploring-different-environments)
  - [Creating a _.pcd_ Environment](#creating-a-pcd-environment)
  - [Acknowledgements](#acknowledgements)

## Quick Start

This copy targets ROS 2 Humble and `colcon`. The active launch files are ROS 2 Python launch
files under [fuel_planner/exploration_manager/launch](fuel_planner/exploration_manager/launch).

### Prerequisites

Use Ubuntu 22.04 with ROS 2 Humble. Install ROS 2 first, then install the common build tools and
non-ROS dependencies:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep libarmadillo-dev libnlopt-dev
source /opt/ros/humble/setup.bash
```

If `rosdep` has not been initialized on the machine yet:

```bash
sudo rosdep init
rosdep update
```

### Build

Build from the workspace root. Replace `/path/to/FUEL` with your own workspace path. This
repository should be under `$FUEL_WS/src/FUEL`.

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

After editing C++ or launch/config files, rebuild the affected packages and source the workspace
again:

```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

### Run the demo

Open RViz2 first:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch exploration_manager rviz.launch.py
```

In a second terminal, start the simulator and planner:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch exploration_manager exploration.launch.py
```

By default RViz2 shows the office environment. Trigger exploration with the `2D Nav Goal` tool in
RViz2. The goal is published on `/move_base_simple/goal`. Unexplored structures are shown in grey,
explored voxels are shown in color, and the FoV and trajectories are displayed.

<!-- You will find a cluttered scene to be explored (20m x 12m x 2m) and the drone . You can trigger the exploration to start by  A sample simulation is shown in the figure. The unknown obstacles are shown in grey, while the frontiers are shown as colorful voxels. The planned and executed trajectories are also displayed. -->

 <p id="demo1" align="center">
  <img src="files/office.gif" width = "600" height = "325"/>
 </p>

### ROS 2 launch graph

`rviz.launch.py` starts:

- `rviz2` with [plan_manage/config/traj.rviz](fuel_planner/plan_manage/config/traj.rviz)
- `tf2_ros/static_transform_publisher` for the `world -> navigation` transform

`exploration.launch.py` starts the main simulation and planning graph:

- `exploration_manager/exploration_node`
- `plan_manage/traj_server`
- `waypoint_generator/waypoint_generator`
- `map_generator/map_pub`
- `so3_quadrotor_simulator/quadrotor_simulator_so3`
- `so3_control/so3_control_node`
- `so3_disturbance_generator/so3_disturbance_generator`
- `odom_visualization/odom_visualization`
- `local_sensing_node/pcl_render_node`

Important runtime topics:

- Goal input: `/move_base_simple/goal`
- Global map: `/map_generator/global_cloud`
- Rendered sensor output: `/pcl_render_node/depth`, `/pcl_render_node/cloud`,
  `/pcl_render_node/sensor_pose`
- Odometry: `/visual_slam/odom`, `/state_ukf/odom`
- Planning command: `/planning/pos_cmd`, `so3_cmd`
- Visualization: `/planning/travel_traj`, `/planning_vis/*`

`map_pub` publishes `/map_generator/global_cloud` when a new subscriber appears instead of
continuously streaming the same static map. This keeps RViz2 stable and avoids unnecessary point
cloud traffic.

### ROS 2 migration notes

- The current local sensing path is the CPU `local_sensing_node/pcl_render_node`.
- `uav_simulator/Utils/rviz_plugins` still contains ROS 1 RViz plugin code and is isolated with
  `COLCON_IGNORE`.
- `uav_simulator/Utils/multi_map_server/quadrotor_msgs` is isolated with `COLCON_IGNORE` because
  it duplicates the canonical ROS 2 interface package at `uav_simulator/Utils/quadrotor_msgs`.


## Exploring Different Environments

The exploration environments in our simulator are represented by [.pcd files](https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
We provide several sample environments, which can be selected in [simulator.launch.py](fuel_planner/exploration_manager/launch/simulator.launch.py):


```python
  # Change office.pcd to specify the exploration environment.
  # We provide office.pcd, office2.pcd, office3.pcd and pillar.pcd in this repo.
  map_file = PathJoinSubstitution([FindPackageShare("map_generator"), "resource", "office.pcd"])
```

Other examples are listed below.

_office2.pcd_:
<p id="demo2" align="center">
<img src="files/office2.gif" width = "600" height = "325"/>
</p>

_office3.pcd_:
<p id="demo3" align="center">
<img src="files/office3.gif" width = "600" height = "325"/>
</p>

_pillar.pcd_:
<p id="demo4" align="center">
<img src="files/pillar.gif" width = "320" height = "325"/>
</p>

If you want to use your own environments, simply place the .pcd files in [map_generator/resource](uav_simulator/map_generator/resource), and follow the comments above to specify it.
You may also need to change the map size launch arguments and the explored-space bounding box.
Map size can be overridden from the command line:

```bash
ros2 launch exploration_manager exploration.launch.py map_size_x:=50.0 map_size_y:=50.0 map_size_z:=10.0
```

The explored-space bounding box is passed from
[exploration.launch.py](fuel_planner/exploration_manager/launch/exploration.launch.py) to
`algorithm.launch.py`:

```python
            "box_min_x": "-10.0",
            "box_min_y": "-15.0",
            "box_min_z": "0.0",
            "box_max_x": "10.0",
            "box_max_y": "15.0",
            "box_max_z": "2.0",
```

To create your own .pcd environments easily, check the [next section](#creating-a-pcd-environment).

## Creating a _.pcd_ Environment

We provide a simple ROS2 tool to create `.pcd` environments from RViz2 clicks. The workflow uses
three terminals: one for the click-map node, one for RViz2, and one for recording the generated
point cloud.

In terminal 1, start the click-map node:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run map_generator click_map
```

In terminal 2, start RViz2:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch exploration_manager rviz.launch.py
```

In RViz2, use the `2D Nav Goal` tool (shortcut `G`) to create the map:

1. Click two points to create one wall.
2. Click another two points to create the next wall.
3. Repeat until the environment is complete.

Each pair of clicked points forms one wall. The generated cloud is published on
`/map_generator/click_map`. If the wall is not visible in RViz2, add or enable a `PointCloud2`
display and set its topic to:

```text
/map_generator/click_map
```

An example is illustrated:

<p id="demo5" align="center">
<img src="files/create_map.gif" width = "600" height = "340"/>
</p>

In terminal 3, after you have clicked at least one wall, check that the generated cloud is non-empty:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --once /map_generator/click_map
```

The output should contain a non-zero `width`. Then record the map:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run map_generator map_recorder /tmp/
```

This saves:

```text
/tmp/tmp.pcd
```

You may replace `/tmp/` with another output directory. To use the generated map in the simulator,
copy it into [map_generator/resource](uav_simulator/map_generator/resource):

```bash
cp /tmp/tmp.pcd "$FUEL_WS/src/FUEL/uav_simulator/map_generator/resource/my_map.pcd"
```

Then edit [simulator.launch.py](fuel_planner/exploration_manager/launch/simulator.launch.py) and
point `map_file` to the new file:

```python
map_file = PathJoinSubstitution([FindPackageShare("map_generator"), "resource", "my_map.pcd"])
```

If the workspace was not built with `--symlink-install`, rebuild the affected packages before
relaunching:

```bash
export FUEL_WS=/path/to/FUEL
cd "$FUEL_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select map_generator exploration_manager
source install/setup.bash
```

Finally, launch the exploration demo again:

```bash
ros2 launch exploration_manager exploration.launch.py
```

## Acknowledgements
  We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.
