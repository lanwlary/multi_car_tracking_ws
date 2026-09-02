# Multi-Car Path Tracking (ROS Noetic)

Clean Catkin workspace for three Pioneer 3-DX robots following CSV paths in
Gazebo, with planned/actual paths and moving robot models displayed in RViz.

## Build

```bash
cd ~/桌面/multi_car_tracking_ws
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch multi_robot_scenario multi_pursuit_all.launch
```

Disable graphical windows when needed:

```bash
roslaunch multi_robot_scenario multi_pursuit_all.launch gazebo_gui:=false rviz:=false
```

## Layout

- `launch/multi_pursuit_all.launch`: one-command entry point
- `launch/kongbai.world`: empty Gazebo world
- `launch/multi_pursuit.rviz`: RViz vehicle/path view
- `scripts/chungenzong4.py`: pure-pursuit controller and Path publishers
- `paths/zzrobot*.csv`: three reference paths
- `xacro/`: minimal P3DX model and Gazebo differential-drive plugin
