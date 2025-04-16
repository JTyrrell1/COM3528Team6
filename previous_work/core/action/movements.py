import numpy as np
import rospy  # ROS Python interface
from std_msgs.msg import Int16MultiArray, UInt32MultiArray, Float32MultiArray, UInt16MultiArray
from geometry_msgs.msg import TwistStamped
from pynput import keyboard
from sensor_msgs.msg import JointState
import os
import miro2 as miro
import time
import threading
import random

# Class definition for handling MiRo robot control
class MiRoControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("keyboard_listener")

        # ros publishers to be used
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Response publisher
        self.pub_response = rospy.Publisher(self.topic_base_name + "/gpt_speech/prompt_response", Int16MultiArray, queue_size=0)
        self.pub_stream = rospy.Publisher(self.topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)

        # movement for either tilt, lift, yaw or pitch
        self.pub_kinematic = rospy.Publisher(
            self.topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )

        # movement for the tail, eye lid, ears
        self.pub_cosmetic = rospy.Publisher(
            self.topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

        # color of lights on MiRo (r,g,b)
        self.pub_illumination = rospy.Publisher(
            self.topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

        self.pub_cmd_vel = rospy.Publisher(
            self.topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # JointState message
        self.joint_cmd = JointState()
        self.joint_cmd.position = [0, 0, 0, 0]

        # Illumination message
        self.color_change = UInt32MultiArray()
        self.color_change.data = [0xFFFFFFFF] * 6

        # Cosmetic movement (Eyes, tails, ears)
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.3333, 0.3333] # droop, wag, left_eye, right_eye, left_ear, right_ear

        # Tone message
        self.tone = UInt16MultiArray()
        self.tone.data = [0, 0, 0]
        
        # Start the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def set_move_kinematic(self, tilt=0, lift=None, yaw=None, pitch=None):
        if lift is None:
            lift = self.joint_cmd.position[1]
        if yaw is None:
            yaw = self.joint_cmd.position[2]
        if pitch is None:
            pitch = self.joint_cmd.position[3]

        self.joint_cmd.position = [tilt, lift, yaw, pitch]

    def set_move_cosmetic(self, droop=None, wag=None, left_eye=None, right_eye=None, left_ear=None, right_ear=None):
        if droop is None:
            droop = self.cos_joints.data[0]
        if wag is None:
            wag = self.cos_joints.data[1]
        if left_eye is None:
            left_eye = self.cos_joints.data[2]
        if right_eye is None:
            right_eye = self.cos_joints.data[3]
        if left_ear is None:
            left_ear = self.cos_joints.data[4]
        if right_ear is None:
            right_ear = self.cos_joints.data[5]

        self.cos_joints.data = [droop, wag, left_eye, right_eye, left_ear, right_ear]

    def get_illumination(self, red = 0, green = 0, blue = 0):
        # changing the rgb format into android format to be published in MiRo message
        color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%color_detail
        color = int(color, 16)
        return color

    def done(self):
        color = self.get_illumination(green=150)
        self.color_change.data = [color] * 6

    def no_process(self):
        color = self.get_illumination()
        self.color_change.data = [color] * 6
    
    def reset(self):
        self.set_move_kinematic(tilt=0, lift=0.59, yaw=0, pitch=0)
        self.set_move_cosmetic(0.0, 0.5, 0.0, 0.0, 0.3333, 0.3333)
        self.no_process()
        self.pub_kinematic.publish(self.joint_cmd)
        self.pub_cosmetic.publish(self.cos_joints)
        self.pub_illumination.publish(self.color_change)

    def on_press(self, key):
        try:
            k = key.char  # single-char keys
        except AttributeError:
            k = key.name  # other keys
        print(f"Key pressed: {k}")
        if key == keyboard.Key.esc:
            return False  # stop listener if escape key is pressed

        elif k == "a":
            print("Raise head and light up green")
            self.set_move_kinematic(tilt=0, lift=0.399, yaw=0, pitch=-0.21)
            self.done()
            self.pub_kinematic.publish(self.joint_cmd)
            self.pub_illumination.publish(self.color_change)

        elif k == "r":
            print("Reset position")
            self.reset()

        elif k == "y":
            print("Nod")
            self.nod()

        elif k == "u":
            print("Direct Nod")
            self.direct_nod()
    
        elif k == "n":
            print("Shake head")
            self.shake_head()
        
        elif k == "m":
            print("Direct shake head")
            self.direct_shake()
        
        elif k == "f":
            self.lean_forward()
        
        elif k == "s":
            # Slow lowering of head, slow closing of eyes, slight and slow lower of pitch
            print("Going to sleep...")
            current_lift = self.joint_cmd.position[1]
            current_pitch = self.joint_cmd.position[3]
            current_eyes = self.cos_joints.data[2] #left eye
            print(current_eyes)

            # Create threads
            pitch_thread = threading.Thread(target=self.slow_pitch_movement, args=(current_pitch,))
            lift_thread = threading.Thread(target=self.slow_lift_movement, args=(current_lift,))
            eye_thread = threading.Thread(target=self.slow_eye_movement, args=(current_eyes,))

            # Start threads
            eye_thread.start()
            pitch_thread.start()
            lift_thread.start()

            # Wait for all threads to complete
            eye_thread.join()
            pitch_thread.join()
            lift_thread.join()
        
        elif k == "left":
            print("Turning head left")
            current_yaw = self.joint_cmd.position[2]
            turn_left = current_yaw - 0.41
            turn_left = max(turn_left, -0.77)
            self.set_move_kinematic(yaw=turn_left)
            self.pub_kinematic.publish(self.joint_cmd)
        
        elif k == "right":
            print("Turning head right")
            current_yaw = self.joint_cmd.position[2]
            turn_right = current_yaw + 0.41
            turn_right = min(turn_right, 1.21)

            self.set_move_kinematic(yaw=turn_right)
            self.pub_kinematic.publish(self.joint_cmd)
        
        elif k == "o":
            self.hey_miro_lean()

 #---------------------- Sleep functions - slow lowering of head and eyes ---------------------
    def slow_pitch_movement(self, current_pitch):
        rate = rospy.Rate(50)  # 10 Hz
        for pitch in np.arange(current_pitch, 0.13, 0.001):  # Increased increment
            self.set_move_kinematic(pitch=pitch)
            self.pub_kinematic.publish(self.joint_cmd)
            rate.sleep()  # Control the rate of publishing

    def slow_lift_movement(self, current_lift):
        rate = rospy.Rate(50)  # 10 Hz
        for lift in np.arange(current_lift, 1.07, 0.001):  # Increased increment
            self.set_move_kinematic(lift=lift)
            self.pub_kinematic.publish(self.joint_cmd)
            rate.sleep()  # Control the rate of publishing

    def slow_eye_movement(self, current_eyes):
        rate = rospy.Rate(130)  # 10 Hz
        for eye in np.arange(current_eyes, 1, 0.001):  # Increased increment
            self.set_move_cosmetic(left_eye=eye, right_eye=eye)
            self.pub_cosmetic.publish(self.cos_joints)
            rate.sleep()  # Control the rate of publishing

#---------------------- Nod that goes through entire range ----------------------
    
    def nod(self):
        current_pitch = self.joint_cmd.position[3]
        lower_range = current_pitch - 0.33
        upper_range = current_pitch + 0.33
        for nods in range(2):
            self.move_pitch_range(current_pitch, upper_range, 0.001)
            self.move_pitch_range(upper_range, lower_range, -0.001)
            self.move_pitch_range(lower_range, current_pitch, 0.001)
    
    def move_pitch_range(self, start, end, step):
        rate = rospy.Rate(3000)
        for angle in np.arange(start, end, step):
            self.set_move_kinematic(pitch=angle)
            self.pub_kinematic.publish(self.joint_cmd)
            rate.sleep()

#---------------------- Direct nod that jumps from three values (Worked better with my code) ----------------------
    
    def direct_nod(self):
        current_pitch = self.joint_cmd.position[3]
        lower_range = current_pitch - 0.33
        upper_range = current_pitch + 0.33
        for nods in range(2):
            self.set_move_kinematic(pitch=upper_range)
            self.pub_kinematic.publish(self.joint_cmd)
            time.sleep(0.3)
            self.set_move_kinematic(pitch=lower_range)
            self.pub_kinematic.publish(self.joint_cmd)
            time.sleep(0.3)
            self.set_move_kinematic(pitch=current_pitch)
            self.pub_kinematic.publish(self.joint_cmd)
    
#--------------------- Head shake that goes through entire range ----------------------
    def shake_head(self):
        current_yaw = self.joint_cmd.position[2]
        lower_range = current_yaw - 0.18
        upper_range = current_yaw + 0.18
        for nods in range(2):
            self.move_yaw_range(current_yaw, upper_range, 0.001)
            self.move_yaw_range(upper_range, lower_range, -0.001)
            self.move_yaw_range(lower_range, current_yaw, 0.001)
    

    def move_yaw_range(self, start, end, step):
        rate = rospy.Rate(3000)
        for angle in np.arange(start, end, step):
            self.set_move_kinematic(yaw=angle)
            self.pub_kinematic.publish(self.joint_cmd)
            rate.sleep()

#---------------------- Head shake that jumps from three values ----------------------

    def direct_shake(self):
            current_yaw = self.joint_cmd.position[2]
            lower_range = current_yaw - 0.18
            upper_range = current_yaw + 0.18
            for nods in range(2):
                self.set_move_kinematic(yaw=upper_range)
                self.pub_kinematic.publish(self.joint_cmd)
                time.sleep(0.2)
                self.set_move_kinematic(yaw=lower_range)
                self.pub_kinematic.publish(self.joint_cmd)
                time.sleep(0.2)
                self.set_move_kinematic(yaw=current_yaw)
                self.pub_kinematic.publish(self.joint_cmd)

#---------------------- "Hey Miro!" Lean in and random head turn ----------------------
    def hey_miro_lean(self):
        self.lean_forward()
        self.random_head_turn()
    
    def lean_forward(self):
        current_lift = self.joint_cmd.position[1]
        current_pitch = self.joint_cmd.position[3]
        more_lift = current_lift + 0.3
        pitch = -0.21
        if (more_lift < 0.9):
            self.set_move_kinematic(lift=more_lift, pitch=pitch)
            self.pub_kinematic.publish(self.joint_cmd)
            print(self.joint_cmd)
        else:
            print("Maximum forward lean!")
    
    def random_head_turn(self):
        current_yaw = self.joint_cmd.position[2]
        angle = random.uniform(-0.57, 1.0)
        angle = round(angle, 2)
        print(f"HEHEHE: {angle}")
        #turn_right = current_yaw + angle
        #turn_right = min(turn_right, 1.21)

        self.set_move_kinematic(yaw=angle)
        self.pub_kinematic.publish(self.joint_cmd)


    def main(self):
        rospy.spin()  # keep the ROS node running


if __name__ == "__main__":
    miro_control = MiRoControl()
    miro_control.main()