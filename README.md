# Sentry Navigation
## environment

- ros-noetic

- ceres-solver
- Livox-SDK2
- opencv
- RealSense-SDK2
- PCL
- other ros packages installed using apt 

## build

### order

- `livox_ros_driver2` 

- `sentry_srvs`

- other packages

``` shell
source /opt/ros/noetic/setup.bash
catkin_make
```
## run

``` shell
source /devel/setup.bash
roslaunch nav_bringup sentry_nav.launch
```
