youbot_object_grasp
===================

ROS Indigo package for controlling the [KUKA youBot](http://www.kuka-labs.com/en/service_robotics/research_education/youbot/) to navigate to a block that has been detected by some sensor, pick it up, and drive back to where it started.

The main node in this package is arm_control.  This node handles the state machine for the entire process.  It is responsible for controlling the arm as well as the base.  Currently this node is not capable of controlling the arm all the way down to pick up the block.  It will drive next to the block, go through some motions as though it is attempting to pick up the block, and drive back to where it started.

It should be noted that this package can operate both the physical youBot and the youBot in Gazebo.  The Gazebo simulation is explained later.

While developing this code I ran the segmentation and control code on my laptop and communicated over WiFi to the youBot as it ran the driver code.  This is the setup I will explain below.  This was mainly done for convenience of development.  Getting everything to run on the youBot would take less setup.

While the arm_control node can accept any transformation information that is published on the topic is subscribes to it was developed to run in conjunction with the [pcl_auto_seg](https://github.com/mattmongeon/pcl_auto_seg) package.

Software Dependencies
=====================

To run the control code on a separate laptop, the following packages are required.
  * [youbot-manipulation](https://github.com/svenschneider/youbot-manipulation/tree/indigo) (indigo branch)
  * [eband_local_planner](http://wiki.ros.org/eband_local_planner)
  * [youbot_ros_tools](https://github.com/micpalmia/youbot_ros_tools)

The eband_local_planner will require some modifications for it to run in ROS Indigo.  First open CMakeLists.txt and add the line 

```
find_package(cmake_modules REQUIRED)
```

Then open package.xml and add the following lines:

```xml
<build_depend>cmake_modules</build_depend>
<run_depend>cmake_modules</run_depend>
```

To run the driver code on the youBot, the package [youbot-manipulation](https://github.com/svenschneider/youbot-manipulation/tree/hydro) (hydro branch) is required.

Running on the youBot
=====================

To run the entire setup on a separate computer and the youBot, first make sure the youBot and computer are connected to a common ethernet or WiFi conection so that they can communicate with each other.  Open two terminals on the computer.  In one of them, run the following commands

```bash
export ROS_MASTER_URI=http://ubuntu.local:11311/
export ROS_HOSTNAME=<mycomputername>.local
```

This terminal will be used for launching the code that will be running on the development computer.  The first ```export``` call sets up this terminal to communicate with the roscore running on the youBot.  Open a second terminal and ssh to the youBot.  This terminal will be used to launch the youBot's ROS sessions.

Next set up the development computer's launch files to expect to run with the youBot.  Open test.launch in youbot_object_grasp and find the line

```xml
<arg name="launch_gazebo" default="false"/>
```

and make sure it is set to false to run with the youBot.  If this is set to true it will run with Gazebo locally on the laptop.

To start everything, first launch the youBot files by running

```bash
roslaunch youbot_object_grasp youbot.launch
```

Once the arm has initialized and move_base has reported that it can start planning, launch the files on the development computer by running

```bash
roslaunch youbot_object_grasp test.launch
```


Running with Gazebo
===================

To run with Gazebo open test.launch and fine the line 

```xml
<arg name="launch_gazebo" default="false"/>
```

and make sure it is set to true.  Run 

```bash
roslaunch youbot_control youbot_control.launch
```

from the youbot_ros_tools metapackage, and then run

```bash
roslaunch youbot_object_grasp test.launch
```
