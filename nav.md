# Notes on Manning LiveCourse: [Navigation](https://liveproject.manning.com/project/859/559/navigation?)

* [Back to TOP](https://github.com/dblanding/ROS2_live_course)

# Milestone 1: Create launch files for first automatic drive
* `ros2 launch dribot_simulation gazebo_house_launch.py`
* `ros2 launch dribot_navigation navigation_launch.py`

# Milestone 2: Planners & Controllers
## Learning Resources:
* [Setting up navigation plugins](https://navigation.ros.org/setup_guides/algorithm/select_algorithm.html)
    * Has a link to [Navigation servers](https://navigation.ros.org/concepts/index.html#navigation-servers)
* [ROS2 Navigation Concepts](https://navigation.ros.org/concepts/index.html)
## Experiment with different planners / controllers / parameters
* Be sure to use `colcon build --symlink-install` to allow parameter file to be installed as a synbolic link.
* Use this [Configuration Guide](https://navigation.ros.org/configuration/index.html) when trying out different planners and controllers.
## Screenshots showing paths of differenet planners
### NavFn:
![NavFn](images/navfn.png)
### Theta Star:
![Theta Star](images/theta_star.png)
### Smac 2D Planner:
![Smac 2D Planner](images/smac2d.png)
### Smac Hybrid-A* Planner:
![Smac Hybrid-A* Planner](images/smac_hybrid_astar.png)
## Experiment with different Controllers
* Two different Controllers available:
    * **DWB**
    * **Regulated Pure Pursuit**
* The yaml format dosn't provide an easy way to comment out blocks
    * It's a pain to comment out blocks for unused controllers
    * Instead, create a symbolic link to access file for desired controller
    * `ln -s <target_file> nav2_params.yaml`
* Use target file: `nav2_params_DWB.yaml` for **DWB**
* Use target file: `nav2_params_M2sol_RPP.yaml` for **Regulated Pure Pursuit**

