# Things to learn more about (and things learned)

## General ROS2 Learning Resources
* [ROS2 Galactic Documentation](https://docs.ros.org/en/galactic/index.html)
* [Nav2](https://navigation.ros.org/) (in general) and [Behavior Tree XML Nodes](https://navigation.ros.org/configuration/packages/configuring-bt-xml.html) (specifically)
* [Turtlebot3 Nav demo](https://navigation.ros.org/getting_started/index.html)
* [managed-life cycle nodes](https://design.ros2.org/articles/node_lifecycle.html)
* From Michael Ferguson's blog: [Navigation in ROS2](http://www.robotandchisel.com/2020/09/01/navigation2/)
* [Introduction to Behavior Trees](https://robohub.org/introduction-to-behavior-trees/)
* Article on [Robotic Path Planning](https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378)
* 
## In the solution to sensors & sensor_fusion
* In ekf.yaml, I am wondering why odom0_config and imu0_config are as they are:
```
ekf_filter_node:
    ros__parameters:
        frequency: 30.0
        sensor_timeout: 0.1
        two_d_mode: true
        transform_time_offset: 0.0
        transform_timeout: 0.0
        print_diagnostics: true

        publish_tf: true

        map_frame: map
        odom_frame: odom
        base_link_frame: base_link
        world_frame: odom

        odom0: odom
        odom0_config: [false, false, false, #x, y, z
                       false, false, false, #roll, pitch, yaw
                       true, true, true, #vx, vy, vz
                       false, false, true,  #vroll, vpitch, vyaw
                       false, false, false] #ax, ay, az
        odom0_queue_size: 2
        imu0: imu/data
        imu0_config: [false, false, false, #x, y, z
                      false, false, false, #roll, pitch, yaw
                      false, false, false, #vx, vy, vz
                      true, true, true, #vroll, vpitch, vyaw
                      false, false, false] #ax, ay, az
        imu0_nodelay: false
        imu0_differential: false
        imu0_relative: true
        imu0_queue_size: 5
```
## How to launch gazebo with a small house world
some online resources:
* [Launch gazebo.launch.py from my own launch file (ros2)](https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/)
* [Getting ready for ROS Part 8: Simulating](https://articulatedrobotics.xyz/ready-for-ros-8-gazebo/)
* Just run `gazebo ~/house.world` which I made by sticking a wall model between a pair of "world" tags.
I tried to "save world" from Gazebo, but this function seems to be broken. (Actually, it's not broken. See [Medium article](https://medium.com/creating-a-gazebo-simulation-with-ros2-for-your/introduction-8daf6efa12f4) which explains how to get this to work.)

## Trouble watching 2 odometry topics simultaneously in RViz.
* /odometry/combined (ekf output)
* /odom (wheels)

I would get a yellow translucent "film" over the entire viewport. As soon as I drove the robot, the film would become more opaque. I discovered that the 2nd one (/odom topic) had 'covariance' checked. When I unchecked it, the film went away.
********************************************************************************
# Making source files available at runtime:
## Edit CMakeLists.txt (in cmake package) to install subdirectories under share/
```
install(DIRECTORY
  launch
  params
  res
  DESTINATION share/${PROJECT_NAME}
)
```
## Edit setup.py (in python package)
This directs the build process to make files in src/ available under `<workspace>/install/<package_name>/share/<package_name>/`
```
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
        # launch files...
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        # yaml files...
        (os.path.join('share', package_name), glob('params/*.yaml')),
    ],
```
data_files instructs build where to put links from files in src/ to install/
```
    data_files=[
        (<target_location_in_install/>, [<src_files>]),
    ],
```
***************************************************************************
***************************************************************************
## Saving generated map (after mapping with `slam_toolbox_mapping_launch.py`)
### Finding the correct syntax for saving map in .pgm format
* This was pretty tricky and mysterious.
There seem to be 2 parameters, but you don't need to fill in anything for `name`. Just `data`, which is the absolute path to the save filename (without file extension). For example: `/home/doug/house_map` or `/home/doug/ws/slam/src/dribot_slam/res/house_map` both work, writing 2 files:
    * house_map.pgm
    * house_map.yaml

* Tab completion returns this advice:
```
doug@raspi4:~$ ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap 
name:\^J\ \ data:\ \'\'\  -r                        --rate
```

* The only way I can get this to work is to copy/paste from the "hint" for step 7 (but with my path to workspace filled in):

```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '/home/doug/ws/slam/src/dribot_slam/res/house_map'"
```
* I asked Mat how "tab completion" worked on his machine.
> Mat Sadowski: It actually should be fine. If you now press ‘n’ followed by tab it should autocomplete to:
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: ''"
```
> Filling in the data field should make the service work.

## Running `ros2 launch dribot_slam slam_toolbox_mapping_launch.py` again causes saved map to flash on and off.
* RViz flashes back and forth between the saved map and the new 'embryonic' map on roughly 2 second intervals. I am guessing that this is a 'safety feature' intended to remind the user that there is alreaady a saved map.

## How does Mat generate that cool [Side_by_Side Video](https://www.youtube.com/watch?v=DqVGbUCOyRk)?
## Working on Nav Milestone 3, Step 3
* I can select parameter files by pointing a sym link to either:
    * nav2_params_M2sol.yaml (uses smac 2d controller)
    * nav2_params_DWB.yaml (uses DWB controller)
* I had modified local_costmap/local_costmap/ros__parameters/voxel_layer/observation_sources: `scan` to `broken_scan`
* Verified that the local costmap did not show but the robot could still navigate.
* Launched gzclient
* I tried to see if the robot would navigate around a block added to the scene.
    * It **did** still plan a path around it. The block obstacle showed up in the global costmap.
* Next, I added `camera` to rviz
    * Set topic to `camera/raw_image` and reliability policy to `Best Effort`
    * Camera works.
* I wondered if I could remove the rviz launch from the nav launch by just commenting out the line `#ld.add_action(rviz_cmd)`
    * This caused nav launch to fail.
## 
