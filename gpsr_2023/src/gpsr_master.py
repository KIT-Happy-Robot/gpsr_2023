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
        rospy.loginfo("Executing stata: ENTER")
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
        rospy.loginfo("Executing stata: DECIDE_MOVE")
        if self.cmd_count >=4:
            self.navi("entrance")
            tts_srv("finish gpsr")
            return "cmd_finish"
        
        elif self.current_loc != 'operator':
            self.navi_srv('operator')
            return 'decide_finish'

        else:
            return "decide_finish"
        

class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["listen_success",
                             "listen_failure",
                             "next_cmd"])
        
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)

        self.listen_srv = rospy.ServiceProxy(('/planning_srv', ActionPlan))
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)

        self.listen_count = 1

    def execute(self, userdata):
        rospy.loginfo("Executing stata: LISTEN_COMMAND")

        self.head_pub(0)
        if self.listen_count <= 3:

            tts_srv("ListenCount is" + str(self.listen_count))
            tts_srv("Please instruct me")
            actplan_res = self.listen_srv()

            if actplan_res.result:
                tts_srv("Is this correct?")






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
        