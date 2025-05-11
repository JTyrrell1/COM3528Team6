import os
import sys
import time
import math
import random
import select
import termios
import tty
import subprocess
import numpy as np
import rospy

from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import miro2 as miro

# Joint indices
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
tilt, lift, yaw, pitch = range(4)

# Initialize ROS
rospy.init_node("miro_emotion_modes")
topic_base = "/" + os.getenv("MIRO_ROBOT_NAME")
pub_cos = rospy.Publisher(topic_base + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
pub_kin = rospy.Publisher(topic_base + "/control/kinematic_joints", JointState, queue_size=0)
pub_illum = rospy.Publisher(topic_base + "/control/illum", UInt32MultiArray, queue_size=0)
pub_vel = rospy.Publisher(topic_base + "/control/cmd_vel", TwistStamped, queue_size=0)
pub_tone = rospy.Publisher(topic_base + "/control/tone", UInt16MultiArray, queue_size=0)

# ROS Messages
cos_joints = Float32MultiArray(data=[0.0] * 6)
kin_joints = JointState(name=["tilt", "lift", "yaw", "pitch"], position=[0.0, math.radians(34.0), 0.0, 0.0])
tone_msg = UInt16MultiArray(data=[440, 120, 2])
vel_msg = TwistStamped()

# Terminal key reading setup
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

def get_pressed_key():
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None

# LED helpers
def led_color(r, g, b):
    return UInt32MultiArray(data=[(63 << 24) | (r << 16) | (g << 8) | b] * 6)

LED_IDLE = led_color(128, 0, 128)
LED_SAD = led_color(0, 0, 255)
LED_HAPPY = led_color(255, 255, 0)
LED_SPEAKING = led_color(255, 165, 0)
LED_LISTENING = led_color(0, 255, 0)

# Microphone control
def is_mic_muted():
    result = subprocess.run(["amixer", "get", "Capture"], stdout=subprocess.PIPE, text=True)
    return "[off]" in result.stdout

def mute_mic():
    subprocess.run(["amixer", "set", "Capture", "nocap"], stdout=subprocess.DEVNULL)

def unmute_mic():
    subprocess.run(["amixer", "set", "Capture", "cap"], stdout=subprocess.DEVNULL)

# Idle behavior
def idle_behavior_step(wag_phase):
    pub_illum.publish(LED_IDLE)
    wag_rate = 0.5
    wag_phase += 2 * math.pi * wag_rate / 20.0
    wag_phase %= 2 * math.pi
    cos_joints.data[wag] = math.sin(wag_phase) * 0.5 + 0.5
    pub_cos.publish(cos_joints)
    pub_kin.publish(kin_joints)
    return wag_phase

def blink_if_needed(last_blink_time, blink_interval=6.0):
    now = time.time()
    if now - last_blink_time >= blink_interval:
        cos_joints.data[left_eye] = 1.0
        cos_joints.data[right_eye] = 1.0
        pub_cos.publish(cos_joints)
        rospy.sleep(0.5)
        cos_joints.data[left_eye] = miro.constants.EYE_DEFAULT
        cos_joints.data[right_eye] = miro.constants.EYE_DEFAULT
        pub_cos.publish(cos_joints)
        return now
    return last_blink_time

# Happy mode
happy_tone_played = False
happy_vel_phase = 0.0
def happy_behavior_step(wag_phase):
    global happy_tone_played, happy_vel_phase
    wag_rate = 5.0
    wag_phase += math.pi * wag_rate / 50.0
    wag_phase %= 2 * math.pi
    cos_joints.data[droop] = 0.0
    cos_joints.data[wag] = math.sin(wag_phase) * 0.5 + 0.5
    kin_joints.position[lift] = math.radians(22.0)  # lower neck slightly
    pub_cos.publish(cos_joints)
    pub_kin.publish(kin_joints)

    if not happy_tone_played:
        pub_tone.publish(tone_msg)
        happy_tone_played = True

    happy_vel_phase += 0.3
    vel_msg.twist.linear.x = 0.02 * math.sin(happy_vel_phase)
    vel_msg.twist.angular.z = 0.5 * math.sin(happy_vel_phase * 0.5)
    pub_vel.publish(vel_msg)

    return wag_phase

sad_phase = 0.0 
def sad_behavior_step():
    global sad_phase
    sad_phase += 2 * math.pi * 0.3 / 50.0  # 0.3 Hz gentle sway
    sad_phase %= 2 * math.pi

    # Eyes dimmed
    cos_joints.data[left_eye] = miro.constants.EYE_DEFAULT + 1.0
    cos_joints.data[right_eye] = miro.constants.EYE_DEFAULT + 1.0

    # Tail droop
    cos_joints.data[droop] = 1.0
    cos_joints.data[wag] = 0.1

    # Gentle head sway and lowered neck
    kin_joints.position[yaw] = math.radians(10.0 * math.sin(sad_phase))
    kin_joints.position[lift] = math.radians(150.0)
    kin_joints.position[pitch] = math.radians(250.0)

    pub_cos.publish(cos_joints)
    pub_kin.publish(kin_joints)


# Speaking mode
speaking_phase = 0.0
def speaking_behavior_step():
    global speaking_phase
    nod_rate = 1.5
    speaking_phase += 2 * math.pi * nod_rate / 20.0
    speaking_phase %= 2 * math.pi
    kin_joints.position[pitch] = math.radians(10.0 * math.sin(speaking_phase))
    pub_kin.publish(kin_joints)

# Listening mode
listening_active = False
def listening_enter():
    global listening_active
    listening_active = True
    if is_mic_muted():
        unmute_mic()
    kin_joints.position[yaw] = math.radians(35 * random.choice([-1, 1]))
    cos_joints.data[left_ear] = 0.2
    cos_joints.data[right_ear] = 0.8
    pub_kin.publish(kin_joints)
    pub_cos.publish(cos_joints)

def listening_exit():
    global listening_active
    listening_active = False
    mute_mic()
    kin_joints.position[yaw] = 0.0
    kin_joints.position[pitch] = 0.0
    cos_joints.data[left_ear] = 0.5
    cos_joints.data[right_ear] = 0.5
    pub_kin.publish(kin_joints)
    pub_cos.publish(cos_joints)

def switch_mode(key):
    global current_mode, happy_tone_played
    if current_mode == key:
        print(f"Exiting mode {key}")
        if key == 't':
            listening_exit()
        if key == 'h':
            happy_tone_played = False
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.angular.z = 0.0
            kin_joints.position[lift] = math.radians(34.0)  # reset neck height to default
            pub_vel.publish(vel_msg)
            pub_kin.publish(kin_joints)
        if key == 's':
            # Reset pitch and eyes when leaving sad mode
            kin_joints.position[pitch] = 0.0  # reset head pitch
            kin_joints.position[yaw] = 0.0    # stop head shake
            kin_joints.position[lift] = math.radians(34.0)  # reset neck height to default
            cos_joints.data[left_eye] = miro.constants.EYE_DEFAULT
            cos_joints.data[right_eye] = miro.constants.EYE_DEFAULT
            cos_joints.data[droop] = 0.0
            pub_kin.publish(kin_joints)
            pub_cos.publish(cos_joints)
        if key == 'x':
            # Reset pitch when exiting speaking mode
            kin_joints.position[pitch] = 0.0
            pub_kin.publish(kin_joints)

        current_mode = None
        pub_illum.publish(LED_IDLE)
    else:
        print(f"Entering mode {key}")
        if current_mode == 't':
            listening_exit()
        current_mode = key
        if key == 'h':
            pub_illum.publish(LED_HAPPY)
            happy_tone_played = False
        elif key == 's':
            pub_illum.publish(LED_SAD)
        elif key == 'x':
            pub_illum.publish(LED_SPEAKING)
        elif key == 't':
            pub_illum.publish(LED_LISTENING)
            listening_enter()



# Main loop
print("Press keys to toggle modes: h=happy, s=sad, t=listening, x=speaking. Ctrl+C to exit.")
if not is_mic_muted():
    mute_mic()

# --- INITIAL IDLE RESET BLOCK ---
cos_joints.data = [0.0] * 6
cos_joints.data[left_eye] = miro.constants.EYE_DEFAULT
cos_joints.data[right_eye] = miro.constants.EYE_DEFAULT
cos_joints.data[left_ear] = 0.5
cos_joints.data[right_ear] = 0.5
cos_joints.data[wag] = 0.5
cos_joints.data[droop] = 0.0
pub_cos.publish(cos_joints)

kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
pub_kin.publish(kin_joints)

pub_illum.publish(LED_IDLE)
# --------------------------------

current_mode = None
wag_phase = 0.0
last_blink_time = time.time()
rate = rospy.Rate(50)

try:
    while not rospy.is_shutdown():
        key = get_pressed_key()
        if key in ['h', 's', 't', 'x']:
            switch_mode(key)
        if current_mode == 'h':
            wag_phase = happy_behavior_step(wag_phase)
        elif current_mode == 's':
            sad_behavior_step()
        elif current_mode == 'x':
            speaking_behavior_step()
        elif current_mode == 't' and listening_active:
            pass
        else:
            wag_phase = idle_behavior_step(wag_phase)
            last_blink_time = blink_if_needed(last_blink_time)

        rate.sleep()
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
