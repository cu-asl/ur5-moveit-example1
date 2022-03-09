# ur5-moveit-example1

Code for Simulate UR5 robot in Gazebo. 

## Recommended System
- Ubuntu 20.04
- Python 3
- ROS Noetic

## First Time using ROS on your UBUNTU
http://wiki.ros.org/noetic/Installation/Ubuntu

Update Current Packages
```
sudo apt update && sudo apt upgrade
```
Install ROS (make sure to execute every commands)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo rosdep init
rosdep update
```

## Other Requirement
### MoveIt
```
sudo apt install ros-noetic-moveit
```
### Catkin Build
```
sudo apt-get install python3-catkin-tools
```
### Controller
```
sudo apt-get install ros-noetic-joint-trajectory-controller
```

## installation
1. Download zip of this repo
2. Go to your ROS workspace or create new one by follow instruction below
```
mkdir -p ~/catkin_ws/src
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
3. Extract files from github in workspace/src
4. Use `catkin build` again to create downloaded packages 
## Test 1 basic simulation
Run simulation in gazebo, you should see UR5 robot opened in Rviz and Gazebo

`roslaunch ur5_data_collect_fw start_gazebo.launch`

In Rviz:

<img src="https://user-images.githubusercontent.com/91130166/135245288-abf2614d-aa9a-4343-abe9-b2d6cecd1aa0.png" width="50%" height="50%">

In Gazebo:

<img src="https://user-images.githubusercontent.com/91130166/135245296-7c18c3d0-17c5-4d06-9d53-0a3cec3dcaa1.png" width="50%" height="50%">

To control the robot there are 2 .py files to do so, which have to chmod it before running, using:

```
roscd ur5_data_collect_fw
chmod +x src/*
```

or specify the packages location in your workspace. Then try to control by using:

`rosrun ur5_data_collect_fw joint_control.py` or 
`rosrun ur5_data_collect_fw position_control.py`

## visualize data from sensors
There are a couple of sensors in this simulation. To visualize them, run the commands below while Gazebo and Rviz from above roslaunch is running.
1. RGB-D Camera: 
`rosrun rqt_image_view rqt_image_view`
<img src="https://user-images.githubusercontent.com/91130166/135246844-df744dc6-9481-4071-93b4-a3ed1481ffa9.png" width="50%" height="50%">

Change to view depth image by changing signal in upper-left box to /camera/depth/image_raw

<img src="https://user-images.githubusercontent.com/91130166/135246854-3a31addd-aca4-4f6d-bdc1-368548faafa0.png" width="50%" height="50%">


2. Force Torque Sensor: 
`rostopic echo /ee_link2_contactsensor_state`

You should see something like this in terminal when idle:
```
header: 
  seq: 163597
  stamp: 
    secs: 1457
    nsecs: 377000000
  frame_id: "ee_link2"
states: []
---

```
Or if the end effector touch something:
```
header: 
  seq: 162120
  stamp: 
    secs: 1168
    nsecs:  58000000
  frame_id: "ee_link2"
states: 
  - 
    info: "Debug:  i:(0/1)     my geom:robot::wrist_3_link::wrist_3_link_fixed_joint_lump__ee_link2_collision_2\
  \   other geom:unit_box::link::collision         time:1168.057000000\n"
    collision1_name: "robot::wrist_3_link::wrist_3_link_fixed_joint_lump__ee_link2_collision_2"
    collision2_name: "unit_box::link::collision"
    wrenches: 
      - 
        force: 
          x: 79.26940572202356
          y: -35.58187303449895
          z: -28.735882098653548
        torque: 
          x: -2.835562198811275
          y: 0.7579882957031993
          z: -8.760614092954796
    total_wrench: 
      force: 
        x: 79.26940572202356
        y: -35.58187303449895
        z: -28.735882098653548
      torque: 
        x: -2.835562198811275
        y: 0.7579882957031993
        z: -8.760614092954796
    contact_positions: 
      - 
        x: 0.6784500904465107
        y: 0.5476533902406753
        z: 0.32647787534256906
    contact_normals: 
      - 
        x: -0.08886207905186597
        y: -0.989431794212753
        z: -0.11457859969257896
    depths: [9.661818781364628e-05]
---

```

## Test 2 Full Pick&Place Simulation
```
roslaunch ur5_data_collect_fw gripper3f.launch
```
