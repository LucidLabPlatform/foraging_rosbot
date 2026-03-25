```bash
sudo apt install ros-$ROS_DISTRO-explore-lite

sudo apt-get install ros-$ROS_DISTRO-navigation

sudo apt-get install ros-$ROS_DISTRO-slam-gmapping
```


Rules to add fopr move_base:

1. Move slower -> less odom drift and better puck detection
2. When give target pose less strict with exact ending pose
3. When calculating trajectory can pass closer to pucks, not that far