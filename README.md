# FUEL

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

__Authors__: [Boyu Zhou](http://boyuzhou.net) from SYSU and [Shaojie Shen](http://uav.ust.hk/group/) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/).

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

  - [Quick Start](#quick-start)
  - [Exploring Different Environments](#exploring-different-environments)
  - [Creating a _.pcd_ Environment](#creating-a-pcd-environment)
  - [Known issues](#known-issues)

## Quick Start

This project has been tested on Ubuntu 16.04(ROS Kinetic) and 18.04(ROS Melodic). Take Ubuntu 18.04 as an example, run the following commands to install required tools:

```
  sudo apt-get install libarmadillo-dev ros-melodic-nlopt
```

<!-- To simulate the depth camera, we use a simulator based on CUDA Toolkit. Please install it first following the [instruction of CUDA](https://developer.nvidia.com/zh-cn/cuda-toolkit). 

After successful installation, in the **local_sensing** package in **uav_simulator**, remember to change the 'arch' and 'code' flags in CMakelist.txt according to your graphics card devices. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility). For example:

```
  set(CUDA_NVCC_FLAGS 
    -gencode arch=compute_61,code=sm_61;
  ) 
``` -->

Then simply clone and compile our package (using ssh here):

```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone git@github.com:HKUST-Aerial-Robotics/FUEL.git
  cd ../ 
  catkin_make
```

After compilation you can start a sample exploration demo. Firstly run ```Rviz``` for visualization: 

```
  source devel/setup.bash && roslaunch exploration_manager rviz.launch
```
then run the simulation (run in a new terminals): 
```
  source devel/setup.bash && roslaunch exploration_manager exploration.launch
```

By default you can see an office-like environment. Trigger the quadrotor to start exploration by the ```2D Nav Goal``` tool in ```Rviz```. A sample is shown below, where unexplored structures are shown in grey and explored ones are shown in colorful voxels. The FoV and trajectories of the quadrotor are also displayed.

<!-- You will find a cluttered scene to be explored (20m x 12m x 2m) and the drone . You can trigger the exploration to start by  A sample simulation is shown in the figure. The unknown obstacles are shown in grey, while the frontiers are shown as colorful voxels. The planned and executed trajectories are also displayed. -->

 <p id="demo1" align="center">
  <img src="files/office.gif" width = "600" height = "325"/>
 </p>


## Exploring Different Environments

The exploration environments in our simulator are represented by [.pcd files](https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
We provide several sample environments, which can be selected in [simulator.xml](fuel_planner/exploration_manager/launch/simulator.xml):


```xml
  <!-- Change office.pcd to specify the exploration environment -->
  <!-- We provide office.pcd, office2.pcd, office3.pcd and pillar.pcd in this repo -->
  <node pkg ="map_generator" name ="map_pub" type ="map_pub" output = "screen" args="$(find map_generator)/resource/office.pcd"/>    
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
You may also need to change the bounding box of explored space in [exploration.launch](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/exploration_manager/launch/exploration.launch):

```xml
    <arg name="box_min_x" value="-10.0"/>
    <arg name="box_min_y" value="-15.0"/>
    <arg name="box_min_z" value=" 0.0"/>
    <arg name="box_max_x" value="10.0"/>
    <arg name="box_max_y" value="15.0"/>
    <arg name="box_max_z" value=" 2.0"/>
```

To create your own .pcd environments easily, check the [next section](#creating-a-pcd-environment).

## Creating a _.pcd_ Environment

We provide a simple tool to create .pcd environments.
First, run:

```
  rosrun map_generator click_map
```

Then in ```Rviz```, use the ```2D Nav Goal``` tool (shortcut G) to create your map. Two consecutively clicked points form a wall.
An example is illustrated:

<p id="demo5" align="center">
<img src="files/create_map.gif" width = "600" height = "340"/>
</p>

After you've finished, run the following node to save the map in another terminal:

```
  rosrun map_generator map_recorder ~/
```

Normally, a file named __tmp.pcd__ will be saved at ```~/```. You may replace ```~/``` with any locations you want.
Lastly, you can use this file for exploration, as mentioned [here](#exploring-different-environments).

## Known issues

### Compilation issue

When running this project on Ubuntu 20.04, C++14 is required. Please add the following line in all CMakelists.txt files:

```
set(CMAKE_CXX_STANDARD 14)
```

### Unexpected crash

If the ```exploration_node``` dies after triggering a 2D Nav Goal, it is possibly caused by the ros-nlopt library. In this case, we recommend to uninstall it and [install nlopt following the official document](https://nlopt.readthedocs.io/en/latest/NLopt_Installation/). Then in the [CMakeLists.txt of bspline_opt package](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/bspline_opt/CMakeLists.txt), change the associated lines to link the nlopt library:

```
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

...

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

...

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARIES}
    # /usr/local/lib/libnlopt.so
    )  

```

## Acknowledgements
  We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.
