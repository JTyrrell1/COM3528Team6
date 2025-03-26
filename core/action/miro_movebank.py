# Here we will have all the predefined moves that can be accessed, right
# now they are only being used by LLM but this can be expanded for other
# modules in the future.

# can also add cosmetic joints to, and make it so multiple can be executed at once
# just needs to make sure not multiple functions using the same joints

# format kinematic joints = [tilt, lift, yaw, pitch]
#   tilt is irrelevant though

# sine gen params  (mn,mx,freq,period,t,t0)


# yaw = [0.95, -0.95]       
# lift = [1.04, 0.14]
# pitch = [0.14, -0.38]
import rospy
import numpy as np
import math
import random
import os
from geometry_msgs.msg import TwistStamped

class MiroMoveset():

    def __init__(self):
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.lean_yaw = 0
        self.start_lift = 0
        self.pub_cmd_vel = rospy.Publisher(
                self.topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=1
            )
        self.velocity = TwistStamped()

        # For ranom movements:
        self.lift_target_reached = False
        self.yaw_target_reached = False
        self.pitch_target_reached = False
        self.target_lift = 100
        self.target_yaw = 100
        self.target_pitch = 100

        self.ear_count = 0

    def look_up(self,cur_joints, t0, move_length):
        
        t = rospy.get_time() - t0
        freq = move_length / 2
        lift = self.s_curve_generator(cur_joints[1], 0.14, freq,0, t, 0)
            # 0.14, -0.38
        pitch = self.s_curve_generator(cur_joints[3], -0.38, freq,0, t, 0)

        kinematic_joints = [0,lift,cur_joints[2],pitch]
        # allows for move to stop / change when
        status_code = True if t > move_length else False

        return kinematic_joints, status_code
    
    def look_down(self,cur_joints, t0, move_length):
        
        t = rospy.get_time() - t0
        freq = move_length / 2
        lift = self.s_curve_generator(1.07 , cur_joints[1], freq,0, t, 0)
            # 0.14, -0.38
        pitch = self.s_curve_generator(0.12, cur_joints[3], freq,0, t, 0)
        yaw = 0.19

        kinematic_joints = [0,lift,yaw,pitch]
        # allows for move to stop / change when

        status_code = True if t > move_length else False


        return kinematic_joints, status_code
    
    def reset(self):
        lift = 0.64
        yaw = 0.19
        pitch = -0.02

        kinematic_joints = [0, lift, yaw, pitch]
        return kinematic_joints

    def shake_head(self,t0, move_length):
        t = rospy.get_time() - t0
        yaw = self.sine_generator(0.95, -0.95, 1,0, t, 0)
        pitch = -0.38
        lift = 0.14
        #illuminations = self.illumPub.red(t, 0) 

        kinematic_joints = [0,lift,yaw,pitch]
        #elf.parent.nodes.express.set_leds(illuminations.data)

        status_code = True if t > move_length else False

        return kinematic_joints, status_code

    def sine_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
        sine = ((mx-mn)) * (np.cos(freq*(t-t0)*math.pi*2+phase) / 2) + (mx - ((mx-mn)/2))
        return sine

    def s_curve_generator(self,mx=1, mn=0, freq=1,phase=0, t=1, t0=0):
        # freq = how gradual you want the incline
        # when freq = 1, takes ~2 seconds to go from mn to mx 
        curve = (1 + np.tanh(((t-t0)/freq)-2))*(abs(mx-mn)/2) + mn 
        return curve

    def party_nod(self,cur_joints,t0,move_length):
        t = rospy.get_time() - t0
        # range = 0.9 / 2 
        # upper = 0.59
        # 1.04 / 0.14
        tempo = 1
        bounce_f = (1 / tempo)
        lift = abs(self.sine_generator(0.3,-0.3,bounce_f,0,t,t0)) + 0.45

        yaw_f = (1 / tempo)
        yaw = self.sine_generator(0.95,-0.95,yaw_f,math.pi/2,t,t0)
        kinematic_joints = [0,lift,yaw,0]

        status_code = True if t > move_length else False

        return kinematic_joints, status_code

    def head_Banging(self,cur_joints, t0, move_length):
        t = rospy.get_time() - t0

        tempo = 0.5

        blink_f= (1 / tempo)
        #tempo * 2 coz otherwise even the blinks will be in sync with the head banging
        # maybe that'll look good though idk

        #self.eye = self.sine_generator(0.8,0,blink_f,0,t,t0)
        #self.cosmetic_joint_cmd.data= [0,0,blink,blink,0,0]

        pitch_freq = (1 / tempo)/2
        pitch = self.sine_generator(0.14,-0.38,pitch_freq,0,t,t0)
        lift_freq = (1 / tempo)/2
        lift = self.sine_generator(1.04,0.14,lift_freq,0,t,t0)

        status_code = True if t > move_length else False

        return [0,lift,0,pitch], status_code

    def nod(self,cur_joints,t0,move_length):
        t = rospy.get_time() - t0
        #print(t)
        cur_pitch = cur_joints[3]
        z = ((cur_pitch + 0.38)/0.52)*2-1
        period = np.arcsin(z)
        pitch = self.sine_generator(0.14,-0.38,2,period,t,0)

        kinematic_joints = [0,cur_joints[1],cur_joints[2],pitch]

        status_code = True if t > move_length else False

        return kinematic_joints, status_code

    def lean_forward(self,cur_joints,t0, move_length):
        if self.lean_yaw == 0: 
            self.lean_yaw = 0.75 if random.randint(0,1) == 1 else -0.75 
        
        if self.start_lift == 0: 
            self.start_lift = cur_joints[1] 

        t = rospy.get_time() - t0 

        #current_lift = cur_joints[1]
        #current_pitch = cur_joints[3]
        freq = move_length / 5

        lift = self.s_curve_generator(self.start_lift,0.75,freq,0,t,0)
        #more_lift = current_lift + 0.3
        pitch = -0.35
        #print(pitch , " ", lift)
        kinematic_joints = [0,lift,self.lean_yaw,pitch]
        #if (more_lift < 0.9):
        #    kinematic_joints = [0,more_lift,cur_joints[2],pitch]
        #else:
        #    kinematic_joints = [0,cur_joints[1],cur_joints[2],pitch]
        if t > move_length:
            status_code = 1
            self.start_lift = 0
            self.lean_yaw = 0
        else:
            status_code = 0

        return kinematic_joints, status_code
    
    def turn_head(self,cur_joints,t0, move_length):
        if self.lean_yaw == 0: 
            self.lean_yaw = cur_joints[2]
        
        if self.start_lift == 0: 
            self.start_lift = cur_joints[1] 

        t = rospy.get_time() - t0 
        freq = move_length / 5

        lift = self.s_curve_generator(self.start_lift,0.75,freq,0,t,0)

        if self.lean_yaw > 0.19:
            self.lean_yaw += 0.002
        else:
            self.lean_yaw -= 0.002

        pitch = -0.35
        
        kinematic_joints = [0,lift,self.lean_yaw,pitch]
        
        if t > move_length:
            status_code = 1
            self.start_lift = 0
            self.lean_yaw = 0
        else:
            status_code = 0

        return kinematic_joints, status_code

    def slight_move(self, cur_joints):
        # can either move lift, yaw or both 
        move_choice = random.randint(1,2)
        new_joints = cur_joints
        # lift
        #if move_choice == 0:
        #    new_lift = cur_joints[1] + round(random.uniform(-0.1,0.1),2)
        #    #print("lift: ", new_lift)
        #    new_joints[1] = new_lift
        # yaw
        if move_choice == 1:
            new_yaw = cur_joints[2] + round(random.uniform(-0.4,0.4),2)
            #print("yaw: ", new_yaw)
            new_joints[2] = new_yaw
        
        # both 
        else:
            new_lift = cur_joints[1] + round(random.uniform(-0.2,0.2),2)
            new_joints[1] = new_lift
            new_yaw = cur_joints[2] + round(random.uniform(-0.2,0.2),2)
            new_joints[2] = new_yaw
            #print("yaw: ", new_yaw)

        return new_joints
    
    def new_slight_move(self, cur_joints):
        # can either move lift, yaw or both 
        move_choice = random.randint(1,2)
        new_joints = cur_joints
        # lift
        #if move_choice == 0:
        #    new_lift = cur_joints[1] + round(random.uniform(-0.1,0.1),2)
        #    #print("lift: ", new_lift)
        #    new_joints[1] = new_lift
        
        # yaw
        #if move_choice == 1:
        new_yaw = cur_joints[2] + round(random.uniform(-0.4,0.4),2)
        #print("yaw: ", new_yaw)
        new_joints[2] = new_yaw
        
        # both 
        #else:
        new_lift = cur_joints[1] + round(random.uniform(-0.2,0.2),2)
        new_joints[1] = new_lift
        new_yaw = cur_joints[2] + round(random.uniform(-0.2,0.2),2)
        new_joints[2] = new_yaw
        #print("yaw: ", new_yaw)

        return new_joints

    def random_move(self, cur_joints, emotion):

        current_lift = cur_joints[1]
        current_yaw = cur_joints[2]
        current_pitch = cur_joints[3]

        if self.target_lift == 100:
            if emotion == "Happy" or emotion == "Angry":
                lower_range = 0.25
                upper_range = 0.6
            elif emotion == "Sad" or emotion == "Worried": 
                lower_range = 0.7
                upper_range = 1.04
            else: # Check other emotions
                lower_range = 0.25
                upper_range = 0.6    

    
            self.target_lift = round(random.uniform(lower_range, upper_range),2)
        
        if self.target_yaw == 100:
            lower_range = max(current_yaw - 0.25, -0.7)
            upper_range = min(current_yaw + 0.25, 0.7)
            if lower_range == -0.7 or lower_range == 0.7:
                self.target_yaw = round(random.uniform(0.1, 0.2), 2)
            self.target_yaw = round(random.uniform(lower_range, upper_range), 2)

        if self.target_pitch == 100:
            #self.target_pitch = round(random.uniform(-0.38, 0.14),2)
            self.target_pitch = round(random.uniform(-0.24, 0.09),2)
        
        # Lift
        if not self.lift_target_reached:
            #print(f"Current Lift: {current_lift}")
            #print(f"Target Lift: {self.target_lift}")
            if current_lift > self.target_lift:
                #print("Reducing Lift")
                current_lift -= 0.07
                if current_lift <= self.target_lift:
                    self.lift_target_reached =  True

            elif current_lift < self.target_lift:
                #print("Increasing lift")
                current_lift += 0.07
                if current_lift >= self.target_lift:
                    self.lift_target_reached =  True
        
        # Yaw
        if not self.yaw_target_reached:
            #print(f"Current Yaw: {current_yaw}")
            #print(f"Target Yaw: {self.target_yaw}")
            if current_yaw > self.target_yaw:
                #print("Reducing yaw")
                current_yaw -= 0.07
                #print(f"Reduced yaw: {current_yaw}")
                if current_yaw <= self.target_yaw:
                    self.yaw_target_reached =  True

            elif current_yaw < self.target_yaw:
                #print("increasing yaw")
                current_yaw += 0.07
                #print(f"Increased yaw: {current_yaw}")
                if current_yaw >= self.target_yaw:
                    self.yaw_target_reached =  True
        
        # Pitch
        if not self.pitch_target_reached:
            #print(f"Current Pitch: {current_pitch}")
            #print(f"Target Pitch: {self.target_pitch}")
            if current_pitch > self.target_pitch:
                current_pitch -= 0.07
                if current_pitch <= self.target_pitch:
                    self.pitch_target_reached =  True

            elif current_pitch < self.target_pitch:
                current_pitch += 0.07
                if current_pitch >= self.target_pitch:
                    self.pitch_target_reached =  True
        
        if self.lift_target_reached and self.pitch_target_reached and self.yaw_target_reached:
            self.lift_target_reached = False
            self.yaw_target_reached = False
            self.pitch_target_reached = False
            self.target_lift = 100
            self.target_yaw = 100
            self.target_pitch = 100
            done = True
        else:
            done = False
        
        new_joints = [0, current_lift, current_yaw, current_pitch]
        return new_joints, done
    
    def head_shake():
        print("hello")

    def light_up(self):
        lift = 0.399
        yaw = 0
        pitch = -0.21
        cur_joints = [0, lift, yaw, pitch]
        return cur_joints

    def ear_victory(self):
        if self.ear_count < 4:
            if rospy.get_time() - self.start_time < 0.5:
                self.parent.nodes.express.set_ears(1, 1)
            elif rospy.get_time() - self.start_time < 1:
                self.parent.nodes.express.set_ears(0, 0)
            else:
                self.start_time = rospy.get_time()
                self.ear_count += 1
        else:
            self.ear_count = 0
            return True

        return False
