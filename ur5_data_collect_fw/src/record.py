#!/usr/bin/env python3
from rospkg.rospack import RosPack
import os
import glob
import subprocess, shlex, psutil
import rospy
rospy.init_node('record')
parent_path = RosPack().get_path('ur5_data_collect_fw') + "/bag/"
# topic_list = ['/camera/depth/image_raw','/camera/rgb/image_raw','/ee_link2_contactsensor_state','/joint_states']
topic_list = ['/camera/depth/image_raw','/camera/rgb/image_raw','/joint_states']
topic_name = " ".join(topic_list)
topic_bag = parent_path+".bag"
rospy.set_param('record',0)

while not rospy.is_shutdown():
    if rospy.get_param('record') == 1:
        rospy.set_param('record',0)
        command = "rosbag record -o " + topic_bag + " " + topic_name
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)
    if rospy.get_param('record') == 2:
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)
        rosbag_proc.send_signal(subprocess.signal.SIGINT)
        rospy.sleep(2)
        files = list(filter(os.path.isfile, glob.glob(parent_path + "*")))
        files.sort(key=lambda x: os.path.getmtime(x))
        command = "rosbag compress --lz4 " + files[-1]
        command = os.system(command)
        rospy.signal_shutdown('')