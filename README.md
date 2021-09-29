# ur5-moveit-example1

Code for Simulate UR5 robot in Gazebo. 
## First Time using ROS on your UBUNTU
http://wiki.ros.org/noetic/Installation/Ubuntu

## requirements
```
sudo apt install ros-noetic-desktop-full ros-noetic-moveit ros-noetic-moveit-resources-prbt-moveit-config ros-noetic-joint-trajectory-controller`
```

## installation
1. Download zip of this repo and https://github.com/cu-asl/universal-robot-archive
2. Go to your ROS workspace or create new one by follow below instruction
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace
cd src
```
3. Extract files from github in workspace/src
4. Use catkin_make or catkin build to create your package or
```
cd ~/catkin_ws/src
catkin build
```
5. Source your workspace by
```
source ~/catkin_ws/devel/bash
```

## test
Run simulation in gazebo, you should see UR5 robot opened in RVIZ and Gazebo

`roslaunch ur5_data_collect_fw start_gazebo.launch`

To control the robot there are 2 .py files to do so, which have to chmod it before running, using:

`chmod +x ~/catkin_ws/src/ur5_data_collect_fw/src/*`

or specify the packages location in your workspace. Then try to control by using:

`rosrun ur5_data_collect_fw joint_control.py` or 
`rosrun ur5_data_collect_fw position_control.py`
