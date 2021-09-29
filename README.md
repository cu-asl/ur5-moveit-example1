# ur5-moveit-example1

Code for Simulate UR5 robot in Gazebo. 
## First Time using ROS on your UBUNTU
http://wiki.ros.org/noetic/Installation/Ubuntu

Don't forget to source ROS main package
```
source /opt/ros/noetic/setup.bash
```
or for long-term uses
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## requirements
```
sudo apt install ros-noetic-desktop-full ros-noetic-moveit ros-noetic-moveit-resources-prbt-moveit-config ros-noetic-joint-trajectory-controller`
```

## installation
1. Download zip of this repo and https://github.com/cu-asl/universal-robot-archive
2. Go to your ROS workspace or create new one by follow instruction below
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
cd src
```
3. Extract files from github in workspace/src
4. Use catkin_make or catkin build to create your package or
```
cd ~/catkin_ws
catkin_make
```
5. Source your workspace by
```
source ~/catkin_ws/devel/setup.bash
```
or for long-term uses
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## test
Run simulation in gazebo, you should see UR5 robot opened in Rviz and Gazebo

`roslaunch ur5_data_collect_fw start_gazebo.launch`

To control the robot there are 2 .py files to do so, which have to chmod it before running, using:

`chmod +x ~/catkin_ws/src/ur5_data_collect_fw/src/*`

or specify the packages location in your workspace. Then try to control by using:

`rosrun ur5_data_collect_fw joint_control.py` or 
`rosrun ur5_data_collect_fw position_control.py`

## visualize data from sensors
There are a couple of sensors in this simulation. To visualize them, run the commands below while Gazebo and Rviz from above roslaunch is running.
1. RGB-D Camera: 
`rosrun rqt_image_view rqt_image_view`
2. Force Torque Sensor: 
`rostopic echo /ee_link2_contactsensor_state`
