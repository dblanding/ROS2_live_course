# ROS2 - Installation and Learning

## Getting Help with ROS
[ROS2 Documentation](https://docs.ros.org/en/humble/index.html)

ROS 2 has quite a community. The main meeting place of the community is [ROS Discourse](https://discourse.ros.org/). In that forum you will find announcements, discussions about projects, working groups, packages, and more.

If you are looking for help with a package, your setup, or ROS 2, the best place to seek help is [answers.ros.org](https://answers.ros.org/). If you find a bug or an issue with a certain package, the best way to seek help will most likely be to reach out to the authors of the package through the issue tracker. If the packages are hosted on GitHub, look for “Issues” embedded within the repo.

## Coming up the ROS2 learning curve (with Galactic Geochelone)

A series of Manning Live Courses: [Build Mobile Robots with ROS2](https://www.manning.com/liveprojectseries/build-mobile-robots-with-ROS2?trk_msg=2FP5H94EJ9V4NDBBTDHGIGLMN8&trk_contact=13BOR5C0GRADA55LJ9PQ6E5C6S&trk_sid=6D42V74A28B5V3RD98OS54SUP8&trk_link=DENFJJHFO0RK70FF2ND44N0DHO&utm_source=Listrak&utm_medium=Email&utm_term=https%3a%2f%2fwww.manning.com%2fliveprojectseries%2fbuild-mobile-robots-with-ROS2&utm_campaign=This+week+at+Manning!+40%25+off+pBooks+ends)

### Project 1: Get Started

1. [Installed Ros2](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
    * df (on /) went from 73% before installation to 74% after.
    * set up the ROS 2 build tool colcon by following this [Colcon Tutorial](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html).
2. Followed [Beginner tutorials](https://liveproject.manning.com/module/855_2_3/get-started/setup/tutorials?) which points to: [Galactic Tutorials](https://docs.ros.org/en/galactic/Tutorials.html)
    * Beginner: CLI tools &
    * Beginner: Client libraries
    > Before [sourcing the overlay](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-the-overlay), it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.
    
3. List of commonly used ones:
    * `ros2 run <pkg_name> <exe_name>`
    * `ros2 node list`
    * `ros2 node info <node_name>`
    * `ros2 topic list`
    * `ros2 topic list -t` --> type
    * `ros2 topic echo <topic_name>`
    * `ros2 topic info <topic_name>` --> msg_type pub_count sub_count
    * `ros2 interface show <msg_type>`
    * `ros2 topic pub [--once | --rate 1] <topic_name> <msg_type> "args"`
    * `ros2 service list`
    * `ros2 service list -t` --> type
    * `ros2 service find <type_name>`
    * `ros2 service call <service_name> <srvc_type> <args>`
        * eg: `ros2 service call /clear std_srvs/srv/Empty`
    * `ros2 param list>`
    * `ros2 action list`
    * `ros2 action list -t`
    * `ros2 interface show <action_type>` --> goal result feedback
    * `ros2 action send_goal <action_name> <action_type> "args"`
    * `ros2 pkg create --build-type ament_python | ament_cmake <pkg_name> --dependencies ...`
    * colcon build --packages-select <pkg_name>

4. Nice summary of [ROS2 command line tools](https://osrf.github.io/ros2multirobotbook/ros2_cli.html)

#### Project Milestones (Deliverables)

1. Creating Launch Files [various ROS2 launch tutorials](write_a_launch_file.md)
    * [Series of launch tutorials](https://docs.ros.org/en/galactic/Tutorials/Launch/Launch-Main.html)
    * [Is teleop_twist_keyboard launchable in ROS 2?](https://answers.ros.org/question/337885/is-teleop_twist_keyboard-launchable-in-ros2/)
    * [Launching/monitoring multiple nodes with Launch](https://docs.ros.org/en/galactic/Tutorials/Launch/Launch-system.html)

2. Creating Workspaces and Nodes
    * [Creating a workspace tutorial](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html)
    * [Creating your first ROS 2 package](https://docs.ros.org/en/galactic/Tutorials/Creating-Your-First-ROS2-Package.html)
    * [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
    * [Using parameters in a class (Python)](https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-Python.html)

3. Custom Services
    * [Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/galactic/Tutorials/Custom-ROS2-Interfaces.html)
    > We create a new package for our services because ament_python packages [don’t support message generation](https://answers.ros.org/question/350084/define-custom-messages-in-python-package-ros2/) —a potential surprise for anyone switching from ROS 1 to ROS 2.
4. Custom Messages

### Project 2: [Simulating a Robot](simulate_robot_in_Gazebo.md)

#### Milestones (Deliverables)
1. Creating a Robot Description
    * The [Navigation URDF Tutorial](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html) might come in handy when you start prototyping your robot.
2. Simulating the Robot in Gazebo 11
    * [Getting Ready for ROS Part 8: Simulating with Gazebo](https://articulatedrobotics.xyz/ready-for-ros-8-gazebo/) is a very good introduction to Gazebo. Even though you don’t strictly need to cover this for this project series, it’s a good entry point to getting more familiar with Gazebo 11.
    * On Including a launch file in your launch file:
    > In this project there will be cases in which you will need to include a launch file inside your launch file, but unfortunately this process is not well described in the documentation yet.
    To import a launch file you can use IncludeLaunchDescription from launch.actions. To make your life easier, here is an example of what the snippet could look like:
    
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    your_pkg_dir = get_package_share_directory('your_pkg_name')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(your_pkg_dir, 'launch', 'your_launchfile_name.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(rviz_cmd)
    return ld
```

Explore [twist_mux](https://index.ros.org/p/twist_mux/), available for install using apt-get.

3. Sending Velocities to the Robot
4. Extra Task: Color It Up
5. Explore [ROS2 Control](https://control.ros.org/master/index.html)
### Project 3: Sensors & Sensor Fusion

### Project 4: Simultaneous Localization and Mapping

### Project 5: Navigation

