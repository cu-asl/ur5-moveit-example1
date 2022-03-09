#!/usr/bin/env python3
import moveit_commander
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import sys
import moveit_msgs.msg
from ur5_data_collect_fw.srv import Object
from std_srvs.srv import Empty, EmptyResponse
from rospkg import RosPack
from urdf_parser_py.urdf import URDF
from math import sqrt
from visualization_msgs.msg import Marker
import copy

class ControlRandomPick():
    def __init__(self):
        
        rospy.wait_for_service('/gazebo/get_model_state')
        get_link_prop = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
        set_link_prop = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_wolrd_prop = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        # pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        rospy.set_param("record",1)
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        world_prop = [item for item in get_wolrd_prop().model_names if not item in ["ground_plane","kinect","kinect_pilar","robot"]]
        for item in world_prop:
            xyz = get_model_state(item, "world").pose.position
            self.pickKnown(xyz.x, xyz.y ,-1,0)
        rospy.set_param("record",2)
        rospy.signal_shutdown('')
        
    def pickKnown(self,x,y,bin_x,bin_y):
        print("go to ", x, y)
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        
        self.manipulator.set_pose_target([x,y,1.27,-1.5708,0,0])
        self.manipulator.go(wait=True)
        
        waypoints = []
        wpose = self.manipulator.get_current_pose().pose
        wpose.position.z -= 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
        self.gripper.set_joint_value_target([0,0.2,0,0,0.2,0,0,0.2,0])
        self.gripper.go(wait=True)
        
        # waypoints = []
        # wpose = self.manipulator.get_current_pose().pose
        # wpose.position.z += 0.1
        # waypoints.append(copy.deepcopy(wpose))
        # wpose.position.x += bin_x - wpose.position.x
        # wpose.position.y += bin_y - wpose.position.y
        # waypoints.append(copy.deepcopy(wpose))
        # self.manipulator.execute(self.manipulator.compute_cartesian_path(waypoints,0.01,0.0)[0])
                
        self.manipulator.set_pose_target([bin_x,bin_y,1.25,-1.5708,0,0])
        self.manipulator.go(wait=True)
        
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        
        
def createMarker(x,y,z):
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

    marker = Marker()

    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

if __name__ == "__main__":
    try:
        rospy.init_node("control_random_pick")
        rospy.set_param("spawn_complete",0)
        while not rospy.is_shutdown():
            if rospy.get_param("spawn_complete") == 1:
                ControlRandomPick()
    except:
        pass