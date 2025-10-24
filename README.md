# allegro_hand_ros

Allegro Hand ROS2
================================

This is a fork of the official release to control Allegro Hand with ROS2 Jazzy : [https://github.com/simlabrobotics/allegro_hand_ros_v4](https://github.com/simlabrobotics/allegro_hand_ros_v4). This repo has been heavily modified in order to :
* Patch some tremendous errors (e.g the velocity computation)
* Simplify it
* Follow ROS2 standard and making it compatible with ros_control
* Provide a simulator that replicate the real robot behavior, using Gazebo (using previous point).

**Note:** The repo [inria-paris-robotics-lab/allegro_hand_hardware_v4](https://github.com/inria-paris-robotics-lab/allegro_hand_hardware_v4) contains CAD files for mounting the Allegro Hand on a UR5 arm.

Launch file instructions:
------------------------

There is now a single file, [allegro_hand.launch.py](src/allegro_hand/launch/allegro_hand.launch.py) that starts the hand standalone. It takes many arguments, but at a minimum you must specify the handedness:

    ros2 launch allegro_hand allegro_hand.launch.py CHIRALITY:=right

Optional (recommended) arguments:

          CAN_DEVICE:=can0 | 

The second launch file is for simulation,

    ros2 launch allegro_hand allegro_hand_gazebo_standalone.launch.py CHIRALITY:=right

Packages and installation
--------

 * **allegro_hand** Contains launchfiles to start everything easily. (standalone use)
 * **allegro_hand_driver** Driver for talking with the allegro hand.
 * **allegro_hand_interface** Expose the allegro hand to be compatible with ros2_control.
 * **allegro_hand_controllers** Contains config files and launch for controllers using ros2_control.
 * **allegro_hand_description** xacro descriptions for the kinematics of the hand, rviz configuration and meshes.
* **allegro_hand_ros_v4** Empty meta-package to compile all the above package conveniently (i.e. `colcon build --packages-select allegro_hand_ros_v4` build all of them).

>Note on polling (from Wonik Robotics): The preferred sampling method is utilizing the Hand's own real time clock running @ 333Hz by polling the CAN communication. In fact, ROS's interrupt/sleep combination might cause instability in CAN communication resulting unstable hand motions.


Useful Links
------------

 * [Allegro Hand wiki](http://wiki.wonikrobotics/AllegroHand/wiki).


Controlling More Than One Hand
------------------------------

When running more than one hand using ROS, you must specify the number of the hand when launching.

    ros2 launch allegro_hand.launch.py CHIRALITY:=right CAN_DEVICE:=can0

    ros2 launch allegro_hand.launch.py CHIRALITY:=left CAN_DEVICE:=can1


Known Issues:
-------------

In simulation, the hand sometimes drifts away from the initial position. This is due to numerical errors accumulating over time in the physics engine.


Installing the CAN driver (for using the real robot only)
--------------------------
**Note:** It is recommended to not follow this section of the tutorial if you do not intend to control the real Allegro Hand robot with your machine.

Before using the hand, you must install the pcan drivers. This assumes you have a peak-systems pcan to usb (or pcan to PCI) adapter.

1. Install these packages
```bash
    sudo apt install can-utils
```

2. Download this repo and compile it

```bash
    cd ws/src
    git clone https://github.com/peak-system/pcan-basic.git
    cd ..
    colcon build --symlink-install
```



To test if the installation was successful, plug in your pcan device and run

```bash
    ros2 launch allegro_hand allegro_hand.launch.py CHIRALITY:=right CAN_DEVICE:=can0
```
If everything is working, you should see the hand starting to send and receive messages over the CAN bus.
