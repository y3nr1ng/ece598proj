# ECE 598 Project

URDF provided by TA [cite]

Run rosdep from the workspace root and tell it to ignore source packages
```
cd ~/ece598proj
rosdep install -i --from-paths . --ignore-src -r -y
```

TA's URDF has Joint8 wrongly named, Joint9 marked as fixed, mixing ros2_control block

To run Gazebo
```
cd robotis_mini_sim
source install/setup.bash
// on client
LIBGL_DRI3_DISABLE=1 gz sim -g
gz sim -g --render-engine ogre
```
https://gazebosim.org/docs/latest/troubleshooting/#unable-to-create-the-rendering-window

Running ROS2 on macOS is still not functional.
hit this issue https://github.com/ros-controls/gz_ros2_control/issues/604

TODO use radians internally, fix ticks_to_rad using dxl_to_rad