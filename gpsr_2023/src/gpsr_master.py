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
        smach.State.__init__(self,outcomes = ["enter_finish"])

        self.enter = rospy.ServiceProxy('/enter_room_server', EnterRoom)

    def execute(self,userdate):
        self.enter(1.0,0.5)
        return "enter_finish"


class DecideMove(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ["decide_finish",
                                              "cmd_finish"])

        self.navi = rospy.ServiceProxy("/navi_location_server",NaviLocation)

        self.current_loc = "None"
        self.cmd_count =0

    def execute(self,userdate):
        if self.cmd_count >=4:
            self.navi("##############")
            return "cmd_finish"
        
        elif self.current_loc != 'operator':
            self.navi_srv('operator')
            return 'decide_finish'

        else:
            return "decide_finish"
        

class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes="")








if __name__ == "__main__":
    rospy.init_node('gpsr_master')
    sm_top = smach.StateMachine(outcomes = "finish_sm")

    with sm_top:
        smach.StateMachine.add(
                    "ENTER",
                    Enter(),
                    transitions = {"enter_finish":"DECIDEMOVE"})
        
        smach.StateMachine.add(
                    "DECIDEMOVE",
                    DecideMove(),
                    transitions = {"decide_finish":"LISTHENCOMMAND",
                                    "cmd_finish":"finish_sm"})
        

        smach.StateMachine.add

    outcome = sm_top.execute()
        