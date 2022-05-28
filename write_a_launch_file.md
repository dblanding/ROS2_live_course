# Notes on how to write launch files

Back to [Simulate a Robot](simulate_robot_in_Gazebo.md)

## How-to tutorials on Writing launch files in ROS2:
* [The ROS2 Beginner Tutorials](https://docs.ros.org/en/galactic/Tutorials.html#beginner) includes a link to [Creating a launch file](https://docs.ros.org/en/galactic/Tutorials/Launch/Creating-Launch-Files.html) -> `turtlesim_mimic_launch.py`
* [introducing ROS2 Launch](https://docs.ros.org/en/galactic/Tutorials/Launch/CLI-Intro.html) shows how to write a launch file that launches 2 turtlesim robots, one mimicing the other. -> `multisim.launch.py`
* A [series of launch tutorials](https://docs.ros.org/en/galactic/Tutorials/Launch/Launch-Main.html)
    * Creating a ROS 2 Launch File (outside of a package) -> `turtlesim_mimic_launch.py`
    * Launching and Monitor Multiple Nodes with Launch -> `ros2 launch my_package my_script.launch.py`
    * Using Substitutions -> `ros2 launch launch_tutorial example_main.launch.py`
    * Using Event Handlers -> `ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200`
    * Using ROS 2 Launch For Large Projects                                                                 -> `ros2 launch launch_tutorial launch_turtlesim.launch.py`
* For [Milestone 1 of Simulate a Robot project](https://github.com/manning-lp/dblanding-simulate-a-robot-lp) I wrote a launch file that shows dribot in Rviz.
    * launch with `ros2 launch dribot_description rviz_launch.py`

## Notes on successfully run examples of ROS2 launch files

Tutorial  |  Package  |  build-type  |  Launch File  |  Notes
----------|-----------|--------------|---------------|---------
[Creating a launch file](https://docs.ros.org/en/galactic/Tutorials/Launch/Creating-Launch-Files.html) |  N/A | N/A | turtlesim_mimic_launch.py | Note 1
[Launching Multiple Nodes](https://docs.ros.org/en/galactic/Tutorials/Launch/Launch-system.html) | my_package | ament_python | my_script.launch.py |
[Using Substitutions](https://docs.ros.org/en/galactic/Tutorials/Launch/Using-Substitutions.html) | launch_tutorial | ament_python | example_main.launch.py | Note 2
[First-Time Robot Setup Guide](https://navigation.ros.org/setup_guides/index.html) | sam_bot_description |  ament_cmake | `display.launch.py` | Note 3
Milestone 1 of [Simulate a Robot](https://github.com/manning-lp/dblanding-simulate-a-robot-lp) project | dribot_description | ament_cmake | rviz_launch.py | Note 3

### Notes
1. stand alone / No package... launch with: `ros2 launch launchfilename`
2. This tutorial didn't work 'out of the box'. It launches a subordinate launch file within the main launch file. During compile, each launch_file gets mapped from src/launch/ to install/pkgname/share/pkgname/ (without 'launch/') and this was causing a FileNotFoundError when looking for the subordinate launchfile. I fixed it by commenting out one line in the main launch_file as below:

```
def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    #'launch',
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```
3. sam_bot keeps its urdf/xacro model in src/description/ folder whereas dribot models are in urdf/. Both CMakeLists.txt and the launch file need to correctly specify the location of the model files.

