# ECE 598 Project

URDF provided by TA [cite]

Run rosdep from the workspace root and tell it to ignore source packages
```
cd ~/ece598proj
rosdep install -i --from-paths . --ignore-src -r -y
```

TA's URDF has Joint8 wrongly named, Joint9 marked as fixed, mixing ros2_control block