import numpy as np
import tf
import rospy
import copy
import time
import miro2 as miro
import soundfile as sf
import os
from std_msgs.msg import Int16MultiArray, UInt32MultiArray, Float32MultiArray, UInt16MultiArray
from pydub import AudioSegment
from io import BytesIO
from action.miro_movebank import *

from . import action_types

class ActionTest(action_types.ActionTemplate):

    def finalize(self):
        
        self.name = "test"      # need to define a name for the action
        self.prio = 0.0             # helps track priority when computing

        self.talking = False
        self.d = 0 
        self.data = []
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        self.pub_stream = rospy.Publisher(topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)
        self.joint_response = MiroMoveset()

    def compute_priority(self):
        self.prio = 5
        return self.prio


    def start(self):

        self.talking = True
        # ~40-50 steps per second
        step = 1
        self.parent.nodes.vision.look_for_apriltag = True
    

        # start pattern
        self.clock.start(step)

    def service(self):
        if self.parent.nodes.vision.look_for_apriltag:
            self.rotate_to_face_user()
        else:
            print("done")
        
        
    def stop(self):

        self.action_start_time = 0

        # Setting the priority to 0 after the action switches can be good for preventing the 
        # action from going on and off repeatedly
        self.prio = 0 

    def rotate_to_face_user(self):
        push_array = [0,0.1,0]
        self.apply_push_body(np.array(push_array), flags=miro.constants.PUSH_FLAG_VELOCITY)
            
    def rotate_to_center(self):
        joints, status = self.joint_response.look_up(self.start_joints,self.start_time,2)

