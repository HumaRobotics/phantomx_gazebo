## phantomx_gazebo

ROS package providing Gazebo simulation of the Phantom X Hexapod robot.
Also provides a Python interface to the joints and some walk capabilities.

These have been tested in simulation and need some work to be used on the real robot, do not use as-is.

![Phantom X model in Gazebo](/phantomx.png?raw=true "Phantom X model in Gazebo")


## Install

Clone in your catkin workspace and catkin_make it.
Make sure you also have the following packages in your workspace
* phantomx_description: https://github.com/HumaRobotics/phantomx_description
* phantomx_control: https://github.com/HumaRobotics/phantomx_control
    
## Usage

You can launch the simulation with:

    roslaunch phantomx_gazebo phantomx_gazebo.launch
    
PRESS PLAY IN GAZEBO ONLY WHEN EVERYTHING IS LOADED (wait for controllers)

You can run a walk demo with:

    rosrun phantomx_gazebo walker_demo.py

## ROS API

All topics are provided in the /phantomx namespace.

Sensors:

    /phantomx/joint_states

Actuators (radians for position control, arbitrary normalized speed for cmd_vel):

    /phantomx/cmd_vel
    /phantomx/j_c1_lf_position_controller/command
    /phantomx/j_c1_lm_position_controller/command
    /phantomx/j_c1_lr_position_controller/command
    /phantomx/j_c1_rf_position_controller/command
    /phantomx/j_c1_rm_position_controller/command
    /phantomx/j_c1_rr_position_controller/command
    /phantomx/j_thigh_lf_position_controller/command
    /phantomx/j_thigh_lm_position_controller/command
    /phantomx/j_thigh_lr_position_controller/command
    /phantomx/j_thigh_rf_position_controller/command
    /phantomx/j_thigh_rm_position_controller/command
    /phantomx/j_thigh_rr_position_controller/command
    /phantomx/j_tibia_lf_position_controller/command
    /phantomx/j_tibia_lm_position_controller/command
    /phantomx/j_tibia_lr_position_controller/command
    /phantomx/j_tibia_rf_position_controller/command
    /phantomx/j_tibia_rm_position_controller/command
    /phantomx/j_tibia_rr_position_controller/command


## Python API

Basic usage:
```python
import rospy
from phantomx_gazebo.phantomx import PhantomX

rospy.init_node("walker_demo")

phantomx=PhantomX()
rospy.sleep(1)

phantomx.set_walk_velocity(1,0,0) # Set full speed ahead for 5 secs
rospy.sleep(5)
phantomx.set_walk_velocity(0,0,0) # Stop
```
## Dependencies

The following ROS packages have to be installed:
* gazebo_ros_control

## License

This software is provided by Génération Robots http://www.generationrobots.com and HumaRobotics http://www.humarobotics.com under the Simplified BSD license