# Notes on **Simulate a Robot** project

Up to [ROS2 - Installation and Learning](README.md)

## Milestone 1: Creating a Robot Description Package `dribot_description`: [Setting up the urdf](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html) (sambot)

* URDF & XACRO are the same in ROS2 as they were in ROS1
* Wheras I wrote one big xacro file, the solution broke it into pieces:
    * Main file (importing three files)
    * file where xacro macros are defined
    * file containing wheel info
    * file containing caster info
* In the FAQ, there was a useful tip on
### How do I include a macro in a .xacro file?
To include a macro you will need to make sure that the .xacro file you want to import is defined within a `xacro:macro` element as follows:
``` xml
<?xml version="1.0"?>
<robot name="dribot_wheel" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="wheel" params="prefix x_pos y_pos">
    <!-- YOUR MACRO CODE GOES HERE-->
  </xacro:macro>

</robot>
```
To import this file you would use the following:

1. Import your `macro—<xacro:include filename="$(find dribot_description)/urdf/wheel.xacro" />` —making sure to provide the correct path relative to the workspace.
2. Call the macro— `<xacro:wheel prefix="wheel_right" x_pos="0" y_pos="-${base_radius+wheel_ygap}" />` —making sure you provide values for all defined parameters.

    
* **Launching** is a **whole lot more trouble** in ROS2 than it was in ROS1:
    * This [tutorial on urdf / xacro & launching RViz](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#build-and-launch) provided all the detail I needed to complete the task.
    * However, the tutorial placed the urdf/xacro files in src/description/ whereas the instructions for this milestone has them in a folder named urdf/.
    * Consequently, CMakeLists.txt needs to correctly specify `urdf` rather than `src` as install DIRECTORY
    * Also, the launch file needs to specify the correct `default_model_path` to the model (urdf or xacro)
    * To launch: `ros2 launch dribot_description rviz_launch.py`
* Because of the intracacies of launch files, I studied quite a few [Launch file tutorials](write_a_launch_file.md), taking notes on things I learned.
## Milestone 2: Simulate the Robot in Gazebo 11: [Getting Ready for ROS Part 8: Simulating with Gazebo](https://articulatedrobotics.xyz/ready-for-ros-8-gazebo/)
1. Display dribot in Gazebo 11
    * In each terminal:
        * `cd ~/dev_ws`
        * `source install/setup.bash`
    * Launch Gazebo: `ros2 launch gazebo_ros gazebo.launch.py`
    * Launch dribot in Rviz: `ros2 launch dribot_description rviz_launch.py`
    * Spawn robot in Gazebo: `ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity dribot`
2. Check whether your collisions are aligned with the visuals of the body (in Gazebo click on View > Collisions).
    * This is where it is important to have <origin> tags within the <collision> tags.
    
