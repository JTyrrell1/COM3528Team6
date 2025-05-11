import os
import time
import math
import random
import subprocess
import numpy as np
import rospy

from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import miro2 as miro

# ROS Messages
cos_joints = Float32MultiArray(data=[0.0] * 6)
kin_joints = JointState(name=["tilt", "lift", "yaw", "pitch"], position=[0.0, math.radians(34.0), 0.0, 0.0])
tone_msg = UInt16MultiArray(data=[440, 120, 2])
vel_msg = TwistStamped()

# LED Helpers
def led_color(r, g, b):
    return UInt32MultiArray(data=[(63 << 24) | (r << 16) | (g << 8) | b] * 6)

LED_IDLE = led_color(128, 0, 128)
LED_SAD = led_color(0, 0, 255)
LED_HAPPY = led_color(255, 255, 0)
LED_SPEAKING = led_color(255, 165, 0)
LED_LISTENING = led_color(0, 255, 0)

# Microphone Control
def is_mic_muted():
    result = subprocess.run(["amixer", "get", "Capture"], stdout=subprocess.PIPE, text=True)
    return "[off]" in result.stdout

def mute_mic():
    subprocess.run(["amixer", "set", "Capture", "nocap"], stdout=subprocess.DEVNULL)

def unmute_mic():
    subprocess.run(["amixer", "set", "Capture", "cap"], stdout=subprocess.DEVNULL)

# ROS Publishers
def init_publishers():
    topic_base = "/" + os.getenv("MIRO_ROBOT_NAME")
    return {
        "cos": rospy.Publisher(topic_base + "/control/cosmetic_joints", Float32MultiArray, queue_size=0),
        "kin": rospy.Publisher(topic_base + "/control/kinematic_joints", JointState, queue_size=0),
        "illum": rospy.Publisher(topic_base + "/control/illum", UInt32MultiArray, queue_size=0),
        "vel": rospy.Publisher(topic_base + "/control/cmd_vel", TwistStamped, queue_size=0),
        "tone": rospy.Publisher(topic_base + "/control/tone", UInt16MultiArray, queue_size=0)
    }

# Behaviors
happy_tone_played = False
happy_vel_phase = 0.0
def happy_behavior_step(pubs, wag_phase):
    global happy_tone_played, happy_vel_phase
    wag_phase += math.pi * 5.0 / 50.0
    wag_phase %= 2 * math.pi
    cos_joints.data[0] = 0.0
    cos_joints.data[1] = math.sin(wag_phase) * 0.5 + 0.5
    kin_joints.position[1] = math.radians(22.0)
    pubs["cos"].publish(cos_joints)
    pubs["kin"].publish(kin_joints)

    if not happy_tone_played:
        pubs["tone"].publish(tone_msg)
        happy_tone_played = True

    happy_vel_phase += 0.3
    vel_msg.twist.linear.x = 0.02 * math.sin(happy_vel_phase)
    vel_msg.twist.angular.z = 0.5 * math.sin(happy_vel_phase * 0.5)
    pubs["vel"].publish(vel_msg)
    return wag_phase

sad_phase = 0.0
def sad_behavior_step(pubs):
    global sad_phase
    sad_phase += 2 * math.pi * 0.3 / 50.0
    sad_phase %= 2 * math.pi
    cos_joints.data[2] = miro.constants.EYE_DEFAULT + 1.0
    cos_joints.data[3] = miro.constants.EYE_DEFAULT + 1.0
    cos_joints.data[0] = 1.0
    cos_joints.data[1] = 0.1
    kin_joints.position[2] = math.radians(10.0 * math.sin(sad_phase))
    kin_joints.position[1] = math.radians(150.0)
    kin_joints.position[3] = math.radians(250.0)
    pubs["cos"].publish(cos_joints)
    pubs["kin"].publish(kin_joints)

speaking_phase = 0.0
def speaking_behavior_step(pubs):
    global speaking_phase
    speaking_phase += 2 * math.pi * 1.5 / 20.0
    speaking_phase %= 2 * math.pi
    kin_joints.position[3] = math.radians(10.0 * math.sin(speaking_phase))
    pubs["kin"].publish(kin_joints)

listening_active = False
def listening_enter(pubs):
    global listening_active
    listening_active = True
    if is_mic_muted():
        unmute_mic()
    kin_joints.position[2] = math.radians(35 * random.choice([-1, 1]))
    cos_joints.data[4] = 0.2
    cos_joints.data[5] = 0.8
    pubs["kin"].publish(kin_joints)
    pubs["cos"].publish(cos_joints)

def listening_exit(pubs):
    global listening_active
    listening_active = False
    mute_mic()
    kin_joints.position[2] = 0.0
    kin_joints.position[3] = 0.0
    cos_joints.data[4] = 0.5
    cos_joints.data[5] = 0.5
    pubs["kin"].publish(kin_joints)
    pubs["cos"].publish(cos_joints)

# Exit/reset behavior per mode
def exit_mode(mode, pubs):
    global happy_tone_played
    if mode == "happy":
        happy_tone_played = False
        vel_msg.twist.linear.x = 0.0
        vel_msg.twist.angular.z = 0.0
        kin_joints.position[1] = math.radians(34.0)
        pubs["vel"].publish(vel_msg)
        pubs["kin"].publish(kin_joints)
    elif mode == "sad":
        kin_joints.position[1] = math.radians(34.0)
        kin_joints.position[2] = 0.0
        kin_joints.position[3] = 0.0
        cos_joints.data[0] = 0.0
        cos_joints.data[2] = miro.constants.EYE_DEFAULT
        cos_joints.data[3] = miro.constants.EYE_DEFAULT
        pubs["cos"].publish(cos_joints)
        pubs["kin"].publish(kin_joints)
    elif mode == "speaking":
        kin_joints.position[3] = 0.0
        pubs["kin"].publish(kin_joints)
    elif mode == "listening":
        listening_exit(pubs)

# Idle
def idle_behavior_step(pubs, wag_phase):
    pubs["illum"].publish(LED_IDLE)
    wag_phase += 2 * math.pi * 0.5 / 20.0
    wag_phase %= 2 * math.pi
    cos_joints.data[1] = math.sin(wag_phase) * 0.5 + 0.5
    pubs["cos"].publish(cos_joints)
    pubs["kin"].publish(kin_joints)
    return wag_phase

def blink_if_needed(pubs, last_blink_time, blink_interval=6.0):
    now = time.time()
    if now - last_blink_time >= blink_interval:
        cos_joints.data[2] = 1.0
        cos_joints.data[3] = 1.0
        pubs["cos"].publish(cos_joints)
        rospy.sleep(0.5)
        cos_joints.data[2] = miro.constants.EYE_DEFAULT
        cos_joints.data[3] = miro.constants.EYE_DEFAULT
        pubs["cos"].publish(cos_joints)
        return now
    return last_blink_time

# Entrypoint
def run_miro(shared_state):
    global happy_tone_played
    rospy.init_node("miro_emotion_modes")
    pubs = init_publishers()

    # Initial reset
    if not is_mic_muted():
        mute_mic()

    cos_joints.data = [0.0] * 6
    cos_joints.data[2] = miro.constants.EYE_DEFAULT
    cos_joints.data[3] = miro.constants.EYE_DEFAULT
    cos_joints.data[4] = 0.5
    cos_joints.data[5] = 0.5
    cos_joints.data[1] = 0.5
    pubs["cos"].publish(cos_joints)

    kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
    pubs["kin"].publish(kin_joints)
    pubs["illum"].publish(LED_IDLE)

    current_mode = None
    wag_phase = 0.0
    last_blink_time = time.time()
    rate = rospy.Rate(50)

    try:
        while not rospy.is_shutdown():
            new_mode = shared_state["current_mode"]
            if new_mode != current_mode:
                print(f"[MIRO] Switching mode: {current_mode} → {new_mode}")
                exit_mode(current_mode, pubs)
                current_mode = new_mode
                if current_mode == "happy":
                    pubs["illum"].publish(LED_HAPPY)
                elif current_mode == "sad":
                    pubs["illum"].publish(LED_SAD)
                elif current_mode == "speaking":
                    pubs["illum"].publish(LED_SPEAKING)
                elif current_mode == "listening":
                    pubs["illum"].publish(LED_LISTENING)
                    listening_enter(pubs)
                else:
                    pubs["illum"].publish(LED_IDLE)

            if current_mode == "happy":
                wag_phase = happy_behavior_step(pubs, wag_phase)
            elif current_mode == "sad":
                sad_behavior_step(pubs)
            elif current_mode == "speaking":
                speaking_behavior_step(pubs)
            elif current_mode == "listening" and listening_active:
                pass
            else:
                wag_phase = idle_behavior_step(pubs, wag_phase)
                last_blink_time = blink_if_needed(pubs, last_blink_time)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("[MIRO] Final reset.")
        exit_mode(current_mode, pubs)
        pubs["illum"].publish(LED_IDLE)
        mute_mic()
