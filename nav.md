# Notes on Manning LiveCourse [navigation](https://liveproject.manning.com/project/859/559/navigation?)

* [Back to TOP](https://github.com/dblanding/ROS2_live_course)

## Milestone 1: Create lauanch files for first automatic drive
* `ros2 launch dribot_simulation gazebo_house_launch.py`
* `ros2 launch dribot_navigation navigation_launch.py`

## Milestone 2: Planners & Controllers
### Learning Resources:
* [Setting up navigation plugins](https://navigation.ros.org/setup_guides/algorithm/select_algorithm.html)
    * Has a link to [Navigation servers](https://navigation.ros.org/concepts/index.html#navigation-servers)
* 
### Experiment with different parameters
Now that I am going to be twidling the parmeters, I will begin to use the sym-link flag when building. 
`colcon build --symlink-install`

