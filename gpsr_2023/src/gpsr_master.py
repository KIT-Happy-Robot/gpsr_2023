#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import roslib
import actionlib
import smach
import smach_ros
import time

from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import LaserScan
from happymimi_navigation.srv import NaviLocation
from happymimi_msgs.srv import StrTrg
from geometry_msgs.msg import Twist
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from find_bag.srv import FindBagSrv, FindBagSrvResponse, GraspBagSrv, GraspBagSrvResponse
#from actplan_executor.msg import APExecutorAction, APExecutorGoal
#from find_bag.srv import FindBagSrv\
from enter_room.srv import EnterRoom

base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl

tts_srv = rospy.ServiceProxy('/tts', StrTrg)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)


class Enter(smach.State):
    def __init__(self):
        smach.state.__init__(self,outcomes = ["enter_finish"])

        self.enter = rospy.ServiceProxy('/enter_room_server', EnterRoom)

    def execute(self,userdate):
        self.enter(1.0,0.5)