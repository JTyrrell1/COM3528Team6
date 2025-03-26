#!/usr/bin/env python3
import math
import os
import numpy as np
import time
import random
import rospy  # ROS Python interface
from std_msgs.msg import (
    UInt32
)  # Used in flag publisher

from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control)
import miro2 as miro

# May be useful for better wheel moves
from miro2.lib import wheel_speed2cmd_vel

class WheelPublisher(object):

    TICK = 0.02  # Main loop frequency (in secs, default is 50Hz)

    def __init__(self):
        # self.command indicates what MiRo should be doing by checking it in 
        # the main loop
        self.command = ""
        self.tempo = 0
        self.in_move = False
        self.move = 0 
        self.move_end = 0
        self.move_length = 0 
        self.speed = 0 
        self.push_array = [0,0,0]

        # Indicates how long a move is, should be a multiple of the songs beat length 
        #self.moveLength = 0.0

        # Get robot name
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

    def set_tempo(self,tempo):
        self.tempo = tempo/60

    def simple_move(self,t,push_array,move_length):
        if self.move_end == 0:
            self.move_end = t + move_length

        self.push_array = push_array

        if t > self.move_end:
            self.in_move = False
            self.move_end = 0 

    def wait(self,t):
        if self.move_end == 0:
            self.move_end = t + 2

        self.push_array = [0,0,0]

        if t > self.move_end:
            self.in_move = True
            self.move_end = 0 
            self.move = random.randint(0,3)

    # Main Loop                                                                       
    def loop(self,t):
        if not self.in_move:
            self.wait(t)
        else:
            if self.move == 0:
                # full spin 
                push_array = [0,0.5,0]
                self.simple_move(t,push_array,1.5)
                # print("full")
            elif self.move == 1:
                # small spin
                push_array = [0,0.1,0]
                self.simple_move(t,push_array,2)
                # print("small")

            elif self.move == 2:
                # move forward
                push_array = [0.1,0,0]
                self.simple_move(t,push_array,0.8)
                # print("back")
            elif self.move == 3:
                # move backward
                push_array = [-0.1,0,0]
                self.simple_move(t,push_array,0.8)
                # print("forward")


        return self.push_array        
