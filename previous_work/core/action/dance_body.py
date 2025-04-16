#!/usr/bin/env python3
import os
import time
import rospy            # ROS Python interface
import numpy as np
import random
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt32MultiArray, String

class JointsPublisher(object):
    
    def __init__(self):
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Define Joint parameters for easier publishing together
        # Cosmetic Joints 
        self.ear = 0.0
        self.eye = 0.0 
        self.tail = 0.0

        # Kinematic Joints
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.neck_lift = 0.0

        # Kinematic Variables
        self.pitch_max = 0.14
        self.pitch_min = -0.38
        self.pitch_phase = 0

        self.yaw_max = 0 
        self.yaw_min = 0 
        self.yaw_phase = 0
        self.yaw_modifier = 1
        
        self.lift_max = 0
        self.lift_min = 0 
        self.lift_phase = 0
        self.lift_modifier = 1
        
        self.joint_chooser = 0 
        self.t_2bars = 0.0
        self.tempo = 0
        self.command = ""

        # Publishers
        self.kinematic_pub = rospy.Publisher(
            self.topic_base_name + "/control/kinematic_joints", JointState, queue_size=10
        )
        self.cosmetic_pub = rospy.Publisher(
            self.topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=10
        )

        self.sub = rospy.Subscriber(self.topic_base_name + "/sensors/kinematic_joints",JointState, self.kinematic_callback)
        self.curJoint = 0
        self.curPitch = 0
        self.curYaw = 0
        self.curLift = 0
        # Defining joint commands
        self.kinematic_joint_cmd = JointState()
        self.kinematic_joint_cmd.position = [0, 0, 0, 0]
        self.cosmetic_joint_cmd = Float32MultiArray()   
        self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]

        rospy.loginfo("Head/Neck Moves node is active...")

    def set_tempo(self,tempo):
        self.tempo = 60 / tempo
        
    # Callback for when subscriber to head_topic
    # reads in parameters published from Miro_Dance Node
    def cmd_callback(self,topic_message):  
        self.command = topic_message.move_name 
        if topic_message.tempo != 0:
            # Tempo is used in terms of beats per second so it gets converted when it is passed
            self.tempo = 60 / topic_message.tempo
        else:
            self.tempo = 0
    
    def kinematic_callback(self, message):
        #print(message.position[1])
        self.curLift = message.position[1]
        self.curYaw = message.position[2]
        self.curPitch = message.position[3]
    
    def new_sine_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
        return ((mx-mn)) * (np.sin(freq*(t-t0)*2*math.pi+phase) / 2) + (mx - ((mx-mn)/2)) 
        # NEED TO GET RID OF (freq/2*math.pi)

    # Would be the same as using new_sine_generator with a phase of pi/2
    def cosine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.cos (freq*(t-t0) + phase) / 2.0 + offset)

    # General cosmetic functions --------------------------------------------
    def move_ears(self,t,t0,freq):
        self.ear = self.new_sine_generator(1,0,freq,0,t,t0)
        self.cosmetic_joint_cmd.data[4]=self.ear
        self.cosmetic_joint_cmd.data[5]=self.ear
        
    def move_eyes(self,t,t0,freq):
        self.eye = self.new_sine_generator(1,0,freq,0,t,t0)

    def wag_tail(self,t,t0,freq):
        self.tail = self.new_sine_generator(0.5,0.2,freq,0,t,t0)

    # Sets his eyes openness based on bpm 
    def set_eyes(self,tempo):
        if tempo < 0.45:
            self.eye = 0.15
        elif tempo > 0.75:
            self.eye = 0.5
        else:
            self.eye = tempo - 0.3
        self.cosmetic_joint_cmd.data[2] = self.eye
        self.cosmetic_joint_cmd.data[3] = self.eye
    
    # General kinematic joints ---------------------------------------------
    # YAW MAX = 0.95, MIN = -0.95 (RAD) MAX = 55, MIN = -55 (DEG) 
    def move_head_yaw(self,t,t0,freq, y_max, y_min,y_phase):
        self.head_yaw = self.new_sine_generator(y_max,y_min,freq,0,t,t0)
        #print(self.head_yaw)

    # PITCH MAX = 0.14, MIN = -0.38 (RAD) MAX = 8, MIN = 22 (DEG)
    def move_head_pitch(self,t,t0,freq,p_max,p_min,p_phase):
        self.head_pitch = self.new_sine_generator(p_max,p_min,freq,0,t,t0)

    # LIFT MAX = 1.04, MIN = 0.14 (RAD) MAX = 60, MIN = 8 (DEG)
    def move_neck_lift(self,t,t0,freq,l_max,l_min,l_phase):
        self.neck_lift = self.new_sine_generator(l_max,l_min,freq,l_phase,t,t0)
        #print(self.neck_lift)

    # Publishes joint info, done every iteration --------------------------
    def publish_cosmetics(self):
        self.cosmetic_joint_cmd = Float32MultiArray()
        self.cosmetic_joint_cmd.data= [0,self.tail,self.eye,self.eye,self.ear,self.ear]
        #self.parent.parent.nodes.express.set_ears(self.ear,self.ear)
        #self.parent.parent.nodes.express.set_tail(self.tail)
        #self.parent.parent.nodes.express.set_eyes(self.eye,self.eye)
        #self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def publish_kinematics(self):
        self.kinematic_joint_cmd = JointState()

        self.kinematic_joint_cmd.position = [0,self.neck_lift,self.head_yaw,self.head_pitch]
        #self.kinematic_pub.publish(self.kinematic_joint_cmd)
      
    # Pre-programmed Moveset (One will be picked randomly every other iteration) ------------------------------
    def head_bop(self,t,t0):
        #print("head bop")
        self.kinematic_joint_cmd = JointState()
        freq = (1 / self.tempo)/2

        pitch = abs(self.new_sine_generator(0.26,-0.26,freq,0,t,t0))-0.2
        self.kinematic_joint_cmd.position = [0,0,0,pitch]

    def full_head_spin(self,t,t0):
        #print("head spin")
        self.kinematic_joint_cmd = JointState()
        freq = (1 / self.tempo)/2

        yaw = self.new_sine_generator(0.95,-0.95,freq,0,t,t0)
        pitch = self.new_sine_generator(0.14,-0.38,freq,math.pi/2,t,t0)
        self.kinematic_joint_cmd.position = [0,0,yaw,pitch]

    def soul_Head_Bounce(self,t,t0):
        #print("head bounce")
        self.kinematic_joint_cmd = JointState()
        # range = 0.9 / 2 
        # upper = 0.59
        # 1.04 / 0.14
        bounce_f = (1 / self.tempo)/2 
        lift = abs(self.new_sine_generator(0.3,-0.3,bounce_f/2,0,t,t0)) + 0.45

        yaw_f = (1 / self.tempo)/2 
        yaw = self.new_sine_generator(0.95,-0.95,yaw_f/2,math.pi/2,t,t0)
        self.kinematic_joint_cmd.position = [0,lift,yaw,0]

    def head_Banging(self,t,t0):
        #print ("Head Banging")
        self.kinematic_joint_cmd = JointState()
        #self.cosmetic_joint_cmd = Float32MultiArray()

        #blink_f= (1 / self.tempo)/4
        #tempo * 2 coz otherwise even the blinks will be in sync with the head banging
        # maybe that'll look good though idk
        #self.eye = self.new_sine_generator(0.8,0,blink_f,0,t,t0)
        #self.cosmetic_joint_cmd.data= [0,0,blink,blink,0,0]

        pitch_freq = (1 / self.tempo)/4
        #pitch = self.sine_generator(8, -22, 0, pitch_freq, 0, t, t0)
        pitch = self.new_sine_generator(0.14,-0.38,pitch_freq,0,t,t0)
        lift_freq = (1 / self.tempo)/4
        #lift = self.sine_generator(8, -22, 0, lift_freq, 0, t, t0)
        lift = self.new_sine_generator(1.04,0.14,lift_freq,0,t,t0)
        #print(pitch)
        self.kinematic_joint_cmd.position = [0,lift,0,pitch]

        #self.kinematic_pub.publish(self.kinematic_joint_cmd)
        #self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def change_lift_values(self):
        # MAX = 1.04 and MIN = 0.14 but random can only get a random integer
        joint_min = 14
        joint_max = 104
        joint_range = abs(joint_max)+abs(joint_min)
        #print(joint_range)
        new_joint_min = 0 
        new_joint_max = 0 
        if self.tempo <= 0.45:
            # FAST SONG, so uses a 1/3 of the range of motion in wave
            new_joint_range = joint_range - int(joint_range /3)
            new_joint_min = random.randint(0,new_joint_range) + joint_min 
            new_joint_max = new_joint_min + (joint_range / 3)

            self.lift_min = float(new_joint_min/100)
            self.lift_max = float(new_joint_max/100)
            self.lift_modifier = 2
        elif self.tempo < 0.75:
            # MED SONG, uses 1/2 of the range 
            new_joint_range = joint_range - int(joint_range / 2)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 2)

            self.lift_min = float(new_joint_min/100)
            self.lift_max = float(new_joint_max/100)
            self.lift_modifier = 1
        else:
            # SLOW SONG, uses full range
            # DONT EVEN KNOW IF I NEED THESE LINES
            self.lift_min = float(joint_min/100)
            self.lift_max = float(joint_max/100)
            self.lift_modifier = 0.5
        self.lift_phase = round(random.uniform(0,math.pi))

        if (random.randint(1,4) == 1):
            #print("neck turning off")
            self.lift_min = self.neck_lift
            #self.lift_min = round(random.uniform(0.14,1.04),2)
            self.lift_max = self.lift_min

    def change_pitch_values(self):
        # MAX = 0.14 and MIN = -0.38 but random can only get a random integer
        joint_min = -38
        joint_max = 14
        joint_range = abs(joint_max)+abs(joint_min)
        new_joint_min = 0 
        new_joint_max = 0 
        if self.tempo <= 0.45:
            # FAST SONG, so uses a 1/3 of the range of motion in wave
            new_joint_range = joint_range - int(joint_range / 3)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 3)

            self.pitch_min = float(new_joint_min/100)
            self.pitch_max = float(new_joint_max/100)
            self.pitch_modifier = 2
        elif self.tempo < 0.75:
            # MED SONG, uses 1/2 of the range 
            new_joint_range = joint_range - int(joint_range / 2)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 2)

            self.pitch_min = float(new_joint_min/100)
            self.pitch_max = float(new_joint_max/100)
            self.pitch_modifier = 0.5
            # THESE WILL PROBABLY NEED TO BE SELF. VALUES
        else:
            # SLOW SONG, uses full range
            # DONT EVEN KNOW IF I NEED THESE LINES
            self.pitch_min = float(joint_max/100)
            self.pitch_max = float(joint_max/100)
            self.pitch_modifier = 0.5
        self.pitch_phase = round(random.uniform(0,math.pi/2))

        if (random.randint(1,4) == 1):
            #print("head tilt turning off")
            self.pitch_min = round(random.uniform(-0.38,0.14),2)
            self.pitch_max = self.pitch_min
    
    def change_yaw_values(self):
        # MAX = 0.95 and MIN = -0.95 but random can only get a random integer
        joint_min = -95
        joint_max = 95
        joint_range = abs(joint_max)+abs(joint_min)
        new_joint_min = 0 
        new_joint_max = 0 
        if self.tempo <= 0.45:
            # FAST SONG, so uses a 1/3 of the range of motion in wave
            new_joint_range = joint_range - int(joint_range / 3)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 3)

            self.yaw_min = float(new_joint_min/100)
            self.yaw_max = float(new_joint_max/100)
            self.yaw_modifier = 2
        elif self.tempo < 0.75:
            # MED SONG, uses 1/2 of the range 
            new_joint_range = joint_range - int(joint_range / 2)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 2)

            self.yaw_min = float(new_joint_min/100)
            self.yaw_max = float(new_joint_max/100)
            self.yaw_modifier = 1
            # THESE WILL PROBABLY NEED TO BE SELF. VALUES
        else:
            # SLOW SONG, uses full range
            # DONT EVEN KNOW IF I NEED THESE LINES
            self.yaw_min = float(joint_min/100)
            self.yaw_max = float(joint_max/100)
            self.yaw_modifier = 0.5
        self.yaw_phase = round(random.uniform(0,math.pi))
        if (random.randint(1,4) == 1):
            #print("head side to side turning off")
            self.yaw_min = round(random.uniform(-0.95,0.95),2)
            self.yaw_min = self.head_yaw
            self.yaw_max = self.yaw_min

    # Puts all joints to a "normal" position, after song and when not dancing
    # just for user experience sakes
    def reset_all_joints(self):
        self.eye = self.eye
        self.neck_lift = 0.29
        self.head_pitch = -0.15
        self.head_yaw = 0.0

        self.ear = 0.5

        self.publish_cosmetics()
        self.publish_kinematics()  

    def loop(self,t,t0):
        # Specific Dance Move
        if self.command != "" and self.command != "done":
            #if self.tempo != 0.0:
            if self.command == "reset":
                self.reset_all_joints()
            elif self.command == "head_bounce":
                self.soul_Head_Bounce(t,t0)
            elif self.command == "head_bang":
                self.head_Banging(t,t0)
            elif self.command == "full_head_spin":
                self.full_head_spin(t,t0)
            elif self.command == "head_bop":
                self.head_bop(t,t0)
            else:
                self.head_bop(t,t0)
                    #print("head bop but not the real one")
            #rospy.sleep(0.02)

        # General Dancing 
        # LIFT = NECK, YAW = SIDE TO SIDE, PITCH = UP AND DOWN
        elif self.tempo != 0.0:

            # Publish Eye / Ear Commands, these dont change once set  
            self.set_eyes(self.tempo)
            self.move_ears(t,t0,self.tempo)

            # Publish Yaw / Pitch / Lift Commands
            self.move_head_pitch(t,t0,self.tempo,self.pitch_max,self.pitch_min,self.pitch_phase)
            self.move_neck_lift(t,t0,self.tempo*self.lift_modifier,self.lift_max,self.lift_min,self.lift_phase)
            self.move_head_yaw(t,t0,self.tempo*self.yaw_modifier,self.yaw_max,self.yaw_min,self.yaw_phase)
            #rospy.sleep(0.02)

            # Every 2 bars, switches the max / min of a joint
            # random chance as well that that joint stops moving, adds dynamicity
            if self.t_2bars <= t:
       
                self.t_2bars = t + (8*self.tempo)

                if self.joint_chooser == 0:
                    self.change_yaw_values()
                    self.joint_chooser =+1

                elif self.joint_chooser == 1:
                    self.change_lift_values()   
                    self.joint_chooser = 0

            self.publish_cosmetics()
            self.publish_kinematics()
        #print(self.cosmetic_joint_cmd.data)
        return self.cosmetic_joint_cmd, self.kinematic_joint_cmd

"""
movement = JointPublisher()
t0 = rospy.get_time()
while not rospy.is_shutdown():
    t = rospy.get_time() 
    movement.loop(t,t0)
"""
