MAV Simple Planner
========================

A simple planner in order to generate basic highlevel trajectories as point-to-point, circle, square... This tool is intended to provide basic movements for MAVs and ease the test of simulation environments or basic mission planning.

License
------
there is copyright code from other libraries. Mostly commented in the source code.

Installation
------
* Requirements:
  * ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

//TODO
* Initialize catkin workspace (using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)):
```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws  
  $ source /opt/ros/melodic/setup.bash
  $ catkin init  # initialize your catkin workspace
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin config --merge-devel
```
* Get the simulator and dependencies TODO
```sh
  $ cd ~/catkin_ws/src
  $ 
```
* Build the workspace  
```sh
  $ catkin build
```

Usage
------


Roadmap
------
* 



# mav_simple_planner
