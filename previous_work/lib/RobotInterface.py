#	@section COPYRIGHT
#	Copyright (C) 2023 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

################################################################

from __future__ import print_function

# python
import sys
import os
import copy
import math
import numpy as np
import time
import miro2 as miro
import threading
import traceback
import subprocess

# ROS
import rospy
import std_msgs
import geometry_msgs
import sensor_msgs

# Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

# constants
KIN_JOINTS = [miro.constants.JOINT_LIFT, miro.constants.JOINT_YAW, miro.constants.JOINT_PITCH]
COS_JOINTS = [miro.constants.JOINT_DROOP, miro.constants.JOINT_WAG, miro.constants.JOINT_EYE_L, miro.constants.JOINT_EYE_R, miro.constants.JOINT_EAR_L, miro.constants.JOINT_EAR_R]

class Sensors:
	def __init__(self):
		self.cliff = None
		self.sonar = None
		self.light = None
		self.touch_head = None
		self.touch_body = None
		self.accel_head = None
		self.accel_body = None

		self.wheel_speed = None
		self.kinematic_joints = None
		self.flags = None

class CameraFrame:
	def __init__(self, camera_index, timestamp, data):
		self.camera_index = camera_index
		self.timestamp = timestamp
		self.data = data

class MicrophoneFrame:
	def __init__(self):
		self.data = None

	def left(self):
		if self.data is not None:
			return self.data[:,0]
		else:
			return None

	def right(self):
		if self.data is not None:
			return self.data[:,1]
		else:
			return None

	def tail(self):
		if self.data is not None:
			return self.data[:,2]
		else:
			return None

	def head(self):
		if self.data is not None:
			return self.data[:,3]
		else:
			return None

class RobotInterfaceCallbacks:

	def __init__(self):

		self.cameras = None
		self.camera_left = None
		self.camera_right = None
		self.sensors = None
		self.microphones = None

class RobotInterfaceOutputValue:

	def __init__(self, default_value):

		self.default_value = default_value
		self.value = copy.deepcopy(self.default_value)
		self.timeout = 0

	def set(self, value, timeout):

		self.value = value
		self.timeout = int(timeout * 50.0)

	def get(self):

		ret = copy.deepcopy(self.value)
		if self.timeout > 0:
			self.timeout -= 1
			if self.timeout == 0:
				self.value = copy.deepcopy(self.default_value)
		return ret

class RobotInterface:

	def __init__(self, node_name="robot_interface", robot_name=None,
			control_period=0.02, flags=0, command=None, mirocode_mode=False,
			use_pose_control=False, wait_for_ready=True):

		# debug
		self.n_debug = -1
		self.t_debug = time.time()
		if os.path.isfile("/tmp/RobotInterface.debug"):
			self.n_debug = 0

		# arguments
		self.node_name = node_name
		self.robot_name = robot_name
		self.control_period = control_period
		self.mirocode_mode = mirocode_mode
		self.use_pose_control = use_pose_control

		# callbacks
		self.callbacks = RobotInterfaceCallbacks()

		# List of generated warnings
		self.warned = []

		# List of accepted flag commands
		self.flag_cmds = [None, "set", "clear", "toggle"]

		# get default robot name from environment
		if self.robot_name is None:
			self.robot_name = os.getenv("MIRO_ROBOT_NAME")

		# Check that robot name has been correctly set
		assert self.robot_name is not None, "MIRO_ROBOT_NAME environment variable has\
			not been set. Please set it or specify robot_name"

		# interface state
		self.active = True

		# Generate Objects to hold sensor sensor data
		self.sensors = Sensors()
		self.cameras = [None, None]
		self.mics = MicrophoneFrame()

		# generate objects for dispatch thread
		self.cameras_dispatch = [None, None]

		# Variables for tracking population of sensors objects
		self.sens_pop = [False, False, False]

		# Variable to hold temp imu data
		self.imu = [None, None, None]

		# Create ROS to OpenCV converter
		self.image_converter = CvBridge()

		# Pose Controller
		self.pose_controller = miro.lib.PoseController()
		self.t_pose = time.time()
		self.n_pose = 0
		self.n_pose_lim = 0 # set to 0 to turn off debugging, 50 to enable

		#### Control State ####

		# velocity control state
		self.msg_cmd_vel = RobotInterfaceOutputValue(geometry_msgs.msg.TwistStamped())

		# kinematic joint control state
		default = sensor_msgs.msg.JointState()
		default.name = ["tilt", "lift", "yaw", "pitch"]
		default.position = [0.0, miro.constants.LIFT_RAD_CALIB, 0.0, 0.0]
		self.msg_kin_joints = RobotInterfaceOutputValue(default)

		# cosmetic joint control state
		default = std_msgs.msg.Float32MultiArray()
		default.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
		self.msg_cos_joints = RobotInterfaceOutputValue(default)

		# illum control state
		default = std_msgs.msg.UInt32MultiArray()
		default.data = [0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF]
		self.msg_illum = RobotInterfaceOutputValue(default)

		# audio tone control state
		default = std_msgs.msg.UInt16MultiArray()
		default.data = [0, 0, 0]
		self.msg_tone = RobotInterfaceOutputValue(default)

		# flags state
		default = std_msgs.msg.UInt32()
		# We disable status LEDs as standard and add any flags from init
		default.data = miro.constants.PLATFORM_D_FLAG_DISABLE_STATUS_LEDS | flags
		self.msg_flags = RobotInterfaceOutputValue(default)

		## STATE ##
		self.time_now = 0.0
		self.time_last = None
		self.time_no_data = 0.0
		self.time_control_boundary = 0.0
		self.release_threads = False
		self.cameras_active = False

		#### ROS Setup ####

		# init ROS node
		if self.node_name is not None:
			rospy.init_node(self.node_name, anonymous=True)

		# init ROS topics
		topic_base_name = "/" + self.robot_name

		## Subscribers ##
		topic = topic_base_name + "/sensors/package"
		self.verbose_print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

		# Subrscribe to individual touch ros topics if required for mirocode
		if self.mirocode_mode:
			topic = topic_base_name + "/sensors/touch_body"
			self.verbose_print ("subscribe", topic)
			self.sub_touch_body = rospy.Subscriber(topic, std_msgs.msg.UInt16, self.callback_touch_body, queue_size=1, tcp_nodelay=True)

			topic = topic_base_name + "/sensors/touch_head"
			self.verbose_print ("subscribe", topic)
			self.sub_touch_head = rospy.Subscriber(topic, std_msgs.msg.UInt16, self.callback_touch_head, queue_size=1, tcp_nodelay=True)

		topic = topic_base_name + "/sensors/caml/compressed"
		self.verbose_print ("subscribe", topic)
		self.cam_left_sub = rospy.Subscriber(topic, sensor_msgs.msg.CompressedImage, self.callback_cam_left, tcp_nodelay=True)

		topic = topic_base_name + "/sensors/camr/compressed"
		self.verbose_print ("subscribe", topic)
		self.cam_right_sub = rospy.Subscriber(topic, sensor_msgs.msg.CompressedImage, self.callback_cam_right, tcp_nodelay=True)

		topic = topic_base_name + "/sensors/mics"
		self.verbose_print ("subscribe", topic)
		self.mics_sub = rospy.Subscriber(topic, std_msgs.msg.Int16MultiArray, self.callback_mics, tcp_nodelay=True)

		## Publishers ##
		topic = topic_base_name + "/control/cmd_vel"
		self.verbose_print ("publish", topic)
		self.pub_cmd_vel = rospy.Publisher(topic, geometry_msgs.msg.TwistStamped, queue_size=0)

		topic = topic_base_name + "/control/kinematic_joints"
		self.verbose_print ("publish", topic)
		self.pub_kin_joints = rospy.Publisher(topic, sensor_msgs.msg.JointState, queue_size=0)

		topic = topic_base_name + "/control/cosmetic_joints"
		self.verbose_print ("publish", topic)
		self.pub_cos_joints = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		topic = topic_base_name + "/control/illum"
		self.verbose_print ("publish", topic)
		self.pub_illum = rospy.Publisher(topic, std_msgs.msg.UInt32MultiArray, queue_size=0)

		topic = topic_base_name + "/control/tone"
		self.verbose_print ("publish", topic)
		self.pub_tone = rospy.Publisher(topic, std_msgs.msg.UInt16MultiArray, queue_size=0)

		topic = topic_base_name + "/control/flags"
		self.verbose_print ("publish", topic)
		self.pub_flags = rospy.Publisher(topic, std_msgs.msg.UInt32, queue_size=0)

		if not command is None:
			topic = topic_base_name + "/control/command"
			self.verbose_print ("publish", topic)
			self.pub_command = rospy.Publisher(topic, std_msgs.msg.String, queue_size=0)

		# create control thread
		self.thread_control_active = True
		self.thread_control = threading.Thread(target = self.thread_control_proc)
		self.thread_control.start()

		# create dispatch thread
		self.thread_dispatch_active = True
		self.thread_dispatch = threading.Thread(target = self.thread_dispatch_proc)
		self.thread_dispatch.start()

		# NB: we could include a newline, even though it means the formatting is imperfect,
		# because it means the line gets sent upstream to the mirocode log immediately
		# and gives the impression of a process that takes a little time, which it does.
		sys.stdout.write("connecting to robot '" + self.robot_name + "'...\n")
		sys.stdout.flush()

		# wait for connect
		time.sleep(1)

		# send flags
		self.pub_flags.publish(self.msg_flags.get())

		# if command was included
		if not command is None:

			# send command
			msg = std_msgs.msg.String()
			msg.data = command
			self.pub_command.publish(msg)

			# wait for command to take effect, unless flagged off
			if wait_for_ready:
				self.ready()

		# connected
		print ("...OK")

		# release threads
		self.release_threads = True

	def ready(self):

		# after creating the interface, call this function which will return
		# when the interface is ready, or raise an exception on timeout.
		# we may have caused a frame size or rate change on init(), but there
		# may be a frame in the pipe, so we have to wait a moment before running
		# our camera test for the pipe to empty. that means that even if the
		# cameras are not changing config, we'll still wait at least one second
		# here. that's not ideal, but we can't easily know any better.
		time.sleep(1)

		# wait for cameras to come back
		watchdog = 0
		self.cameras_active = False
		while not self.cameras_active:
			time.sleep(0.1)
			watchdog += 1
			if watchdog == 50:
				raise Exception("timeout waiting for cameras to be ready")

	def get_pose(self):
		return self.pose_controller.pose_est

	def verbose_print(self, *args):
		if not self.mirocode_mode:
			print(" ".join(args))

	def target_mdk_release(self):
		return os.getenv("MIRO_MDK_RELEASE")

	def target_supports(self, feature):
		# specials
		if feature == "is_running_on_robot":
			return subprocess.check_output('hostname').strip() == "miropi"
		if feature == "is_simulator":
			return (self.sensors.flags & miro.constants.PLATFORM_U_FLAG_SIMULATOR) != 0
		if feature == "is_robot":
			return (self.sensors.flags & miro.constants.PLATFORM_U_FLAG_SIMULATOR) == 0
		# if "feature" is a release string "Rxxxxxx"
		if len(feature) == 7 and feature[0:1] == "R":
			vreq = int(feature[1:])
			# test support for version "vreq"
			m = self.target_mdk_release()
			# special dev code should allow support for everything so that
			# testing is not held back
			if m == "U000000":
				#print ("feature test passed (MDK release dev)", feature, m)
				return True
			vcur = int(m[1:])
			if vcur >= vreq:
				#print ("feature test passed (MDK release)", feature, m)
				return True
			else:
				#print ("feature test failed (MDK release)", feature, m)
				return False
		# at this point, returns None to all other features
		#print ("feature test not hit", feature)
		return None

	def register_callback(self, type, callback):

		if type == "cameras":
			self.callbacks.cameras = callback
		elif type == "camera_left":
			self.callbacks.camera_left = callback
		elif type == "camera_right":
			self.callbacks.camera_right = callback
		elif type == "sensors":
			self.callbacks.sensors = callback
		elif type == "microphones":
			self.callbacks.microphones = callback
		else:
			print ("callback type unrecognised")

	def warn(self, warning, persist=False):
		# Check warning hasn't been made before
		if warning not in self.warned:
			print ("[WARNING]", warning)
			if not persist:
				# Add warning to list of warnings to avoid repeats
				self.warned.append(warning)

	def clip_with_warn(self, var, min, max, func, input):
		# Keep value between bounds
		val = np.clip(var, min, max)
		# Alert user if value has changed
		if val != var:
			warning = func + "() was called with out of range " + input + " (value was " + str(var) + ", coerced into range)"
			self.warn(warning)
		# Return adjusted value
		return val

	def term(self):

		self.active = False

	def disconnect(self):

		# disconnect callbacks
		self.callbacks = RobotInterfaceCallbacks()

		# self-terminate like Arnie can't
		self.term()

		# wait for threads to terminate
		if not self.mirocode_mode:
			sys.stdout.write ("wait for robot thread terminate...\n")
		while self.thread_control_active:
			time.sleep(0.1)
		while self.thread_dispatch_active:
			time.sleep(0.1)
		if not self.mirocode_mode:
			print ("...OK")

		# disconnect from robot (just wait)
		sys.stdout.write ("disconnecting from robot...\n")
		sys.stdout.flush()
		self.sub_package = None
		time.sleep(1)
		print ("...OK")

	def is_active(self):

		return self.active and not rospy.core.is_shutdown()

	def wait_for_control_boundary(self):

		t = self.time_control_boundary
		watchdog = 0

		while t == self.time_control_boundary:
			time.sleep(0.005)
			watchdog += 1
			if watchdog == 200:
				raise Exception("timeout in wait_for_control_boundary()")

		return self.time_now

	def thread_control_proc(self):

		# wait for release
		while not self.release_threads:
			time.sleep(0.1)

		# check often enough to reasonably accurately hit control boundary
		T_check = 0.005

		# protect
		try:

			# until term() called or ROS falls over
			while self.is_active():

				# if reached next publish time
				if self.time_now >= self.time_control_boundary:

					# do pose controller
					cmd_vel = self.msg_cmd_vel.get()
					if self.use_pose_control:
						dpose = miro.lib.DeltaPose(
							cmd_vel.twist.linear.x,
							cmd_vel.twist.angular.z
							)
						dpose = self.pose_controller.command_velocity(dpose, T=self.control_period)
						cmd_vel.twist.linear.x = dpose.dr
						cmd_vel.twist.angular.z = dpose.dtheta

					# send out latest control signal
					self.pub_cmd_vel.publish(cmd_vel)
					self.pub_kin_joints.publish(self.msg_kin_joints.get())
					self.pub_cos_joints.publish(self.msg_cos_joints.get())
					self.pub_illum.publish(self.msg_illum.get())

					# report vel for measuring lag
					#print(self.msg_cmd_vel.get().twist.linear.x, self.sensors.wheel_speed[0], self.sensors.wheel_speed[1])
					#print(self.msg_cmd_vel.get().twist.linear.x, self.sensors.cliff)

					# advance publish time
					self.time_control_boundary += self.control_period

					# ok
					continue

				# check that data is being received
				if self.time_no_data >= 1.0:
					self.warn("No data received from robot for 1 second", persist=True)
					self.time_no_data = 0.0

				# sleep
				time.sleep(T_check)
				self.time_no_data += T_check

		# protect
		except Exception as e:
			print ("An error occurred in the control thread, traceback follows")
			print(traceback.format_exc())

		# mark inactive
		self.thread_control_active = False
		self.active = False

	def thread_dispatch_proc(self):

		# wait for release
		while not self.release_threads:
			time.sleep(0.1)

		# protect
		try:

			# cached frames
			camera_frames = [None, None]

			# current triggering camera (start with right, which may be correct more often)
			camera_index_trigger = 1

			# until term() called or ROS falls over
			while self.is_active():

				# if callback is inactive
				if self.callbacks.cameras is None:

					# sleep and continue
					time.sleep(0.1)
					continue

				# get frame
				camera_index = None
				camera_frame = None
				for proposed_camera_index in [0, 1]:
					if not self.cameras_dispatch[proposed_camera_index] is None:
						camera_index = proposed_camera_index
						camera_frame = self.cameras_dispatch[camera_index]
						self.cameras_dispatch[camera_index] = None

				# if no new frame is available yet
				if camera_frame is None:

					# sleep and continue
					time.sleep(0.01)
					continue

				# if we have two cachedframes to make a comparison
				# of timing across the two streams
				if all(camera_frames):

					# dts
					dt = [0, 0]

					# measure dt in both directions between streams
					dt[camera_index] = (camera_frame.timestamp - camera_frames[1 - camera_index].timestamp).to_sec()
					dt[1 - camera_index] = (camera_frames[1 - camera_index].timestamp - camera_frames[camera_index].timestamp).to_sec()

					# add bias for currently selected trigger stream so we don't
					# constantly flip-flop if the two camera streams are equally spaced
					dt[camera_index_trigger] *= 0.9

					# check both dts are positive so we are not fooled by transients
					if dt[0] > 0 and dt[1] > 0:

						# choose preferred trigger stream as the one that gives
						# the smaller dt versus the previous frame of the other stream
						if dt[0] < dt[1]:
							new_camera_index_trigger = 0
						else:
							new_camera_index_trigger = 1

						# if that's changed, update and report
						if camera_index_trigger != new_camera_index_trigger:
							camera_index_trigger = new_camera_index_trigger
							print("change cameras callback trigger stream to", camera_index_trigger)

				# cache frame
				camera_frames[camera_index] = camera_frame

				# dispatch and clear
				if camera_index == camera_index_trigger and all(camera_frames):
					self.callbacks.cameras(camera_frames)

		# protect
		except Exception as e:
			print ("An error occurred in the dispatch thread, traceback follows")
			print(traceback.format_exc())

		# mark inactive
		self.thread_dispatch_active = False
		self.active = False

	def populate_sensor_data(self, type, msg):

		if type == "package":
			# Update Stored variables
			self.sensors.cliff = msg.cliff.data
			self.sensors.sonar = msg.sonar.range
			self.sensors.light = msg.light.data

			if not self.mirocode_mode:
				# Format touch sensor data as array of 14 bools
				self.sensors.touch_head = [bool(int(x)) for x in '{:014b}'.format(msg.touch_head.data)]
				self.sensors.touch_body = [bool(int(x)) for x in '{:014b}'.format(msg.touch_body.data)]
				self.sens_pop[1] = True
				self.sens_pop[2] = True

			self.imu[0] = msg.imu_head.linear_acceleration.x
			self.imu[1] = msg.imu_head.linear_acceleration.y
			self.imu[2] = msg.imu_head.linear_acceleration.z
			self.sensors.accel_head = self.imu

			self.imu[0] = msg.imu_body.linear_acceleration.x
			self.imu[1] = msg.imu_body.linear_acceleration.y
			self.imu[2] = msg.imu_body.linear_acceleration.z
			self.sensors.accel_body = self.imu

			self.sensors.wheel_speed = msg.wheel_speed_opto.data
			self.sensors.kinematic_joints = msg.kinematic_joints.position
			self.sensors.flags = msg.flags.data

			# update pose controller (whether or not we are using it
			# as a controller, we are using it to estimate dynamic pose)
			self.pose_controller.update_sensors(np.array(self.sensors.wheel_speed))
			self.n_pose += 1
			if self.n_pose == self.n_pose_lim:
				self.n_pose = 0
				t = time.time()
				dt = t - self.t_pose
				self.t_pose = t
				# set n_pose_lim to 50 to print out the time interval
				# at which 50 updates are being received and processed
				# to check if the controller is keeping up with the
				# data arriving from the robot
				print(dt)

			# Set flag for callback
			self.sens_pop[0] = True

		elif type == "head_touch":
			# Format touch sensor data as array of 14 bools
			self.sensors.touch_head = [bool(int(x)) for x in '{:014b}'.format(msg.data)]

			# Set flag for callback
			self.sens_pop[1] = True

		elif type == "body_touch":
			# Format touch sensor data as array of 14 bools
			self.sensors.touch_body = [bool(int(x)) for x in '{:014b}'.format(msg.data)]

			# Set flag for callback
			self.sens_pop[2] = True

		# Call regisered callback
		if self.callbacks.sensors is not None and all(self.sens_pop):
			self.callbacks.sensors(copy.deepcopy(self.sensors))

			# Reset flags for callback
			self.sens_pop = [False, False, False]

	def callback_package(self, msg):

		if self.n_debug >= 0:
			self.n_debug += 1
			if self.n_debug == 50:
				t = time.time()
				self.n_debug = 0
				print("callback_package x 50 in", t - self.t_debug, "secs")
				self.t_debug = t

		# We use this callback to advance the clock
		# Topic is transmitted at 50Hz
		if self.release_threads:
			self.time_no_data = 0.0
			self.time_now += 0.02

		# Store data
		self.populate_sensor_data("package", msg)

	def callback_touch_body(self, msg):
		# Store data
		self.populate_sensor_data("body_touch", msg)

	def callback_touch_head(self, msg):
		# Store data
		self.populate_sensor_data("head_touch", msg)

	def populate_cameras(self, index, frame):

		self.cameras_active = True

		# if CAM_L
		if index == miro.constants.CAM_L:
			self.cameras[index] = frame
			self.cameras_dispatch[index] = frame

			# callback
			if self.callbacks.camera_left != None:
				self.callbacks.camera_left(frame)

		# if CAM_R
		elif index == miro.constants.CAM_R:
			self.cameras[index] = frame
			self.cameras_dispatch[index] = frame

			# callback
			if self.callbacks.camera_right != None:
				self.callbacks.camera_right(frame)

	def callback_cam_left(self, ros_image):
		try:
			im = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
		except Exception as e:
			self.warn("conversion of left camera image failed", persist=True)
		else:
			self.populate_cameras(miro.constants.CAM_L, CameraFrame(0, ros_image.header.stamp, im))

	def callback_cam_right(self, ros_image):
		try:
			im = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
		except Exception as e:
			self.warn("conversion of right image failed", persist=True)
		else:
			self.populate_cameras(miro.constants.CAM_R, CameraFrame(1, ros_image.header.stamp, im))

	def callback_mics(self, msg):

		# reshape into 4 x 500 array
		data = np.asarray(msg.data)
		self.mics.data = np.transpose(data.reshape((4, 500)))

		# Call regisered callback
		if self.callbacks.microphones != None:
			self.callbacks.microphones(copy.copy(self.mics))

	#### SETS ####################################################

	def set_vel(self, lin_vel=None, ang_vel=None, timeout=0):

		# Get current velocity values
		cmd_vel = self.msg_cmd_vel.get()

		# Linear velocity commanded
		if lin_vel != None:
			# Set linear velocity between upper and lower bonds
			cmd_vel.twist.linear.x = self.clip_with_warn(lin_vel, -miro.constants.WHEEL_MAX_SPEED_M_PER_S, miro.constants.WHEEL_MAX_SPEED_M_PER_S,
				"set_vel", "linear velocity")

		# Angular velocity commanded
		if ang_vel != None:
			# Set angular velocity between upper and lower bonds
			cmd_vel.twist.angular.z = self.clip_with_warn(ang_vel, -miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S, miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S,
				"set_vel", "angular velocity")

		# Set values to commanded values
		self.msg_cmd_vel.set(cmd_vel, timeout)

	def set_kin(self, joint, ang, timeout=0):
		# Check joint_index is valid
		assert joint in KIN_JOINTS, "out-of-range joint index passed to set_kin()"

		# Get current commanded kinematic joint angles
		kin_joints = self.msg_kin_joints.get()

		# Keep commanded angle between upper and lower bounds
		if joint == miro.constants.JOINT_LIFT:
			kin_joints.position[joint] = self.clip_with_warn(ang, miro.constants.LIFT_RAD_MIN, miro.constants.LIFT_RAD_MAX,
				"set_kin", "lift angle (radians)")

		elif joint == miro.constants.JOINT_PITCH:
			kin_joints.position[joint] = self.clip_with_warn(ang, miro.constants.PITCH_RAD_MIN, miro.constants.PITCH_RAD_MAX,
				"set_kin", "pitch angle (radians)")

		elif joint == miro.constants.JOINT_YAW:
			kin_joints.position[joint] = self.clip_with_warn(ang, miro.constants.YAW_RAD_MIN, miro.constants.YAW_RAD_MAX,
				"set_kin", "yaw angle (radians)")

		# Set value to commanded value
		self.msg_kin_joints.set(kin_joints, timeout)


	def set_cos(self, joint, pos, timeout=0):
		# Check joint_index is valid
		assert joint in COS_JOINTS, "out-of-range joint index passed to set_cos()"

		# Get current commanded cosmetic joint angles
		cos_joints = self.msg_cos_joints.get()

		# Keep commanded position between upper and lower bounds
		cos_joints.data[joint] = self.clip_with_warn(pos, 0.0, 1.0, "set_cos", "joint position")

		# Set value to commanded value
		self.msg_cos_joints.set(cos_joints, timeout)

	def set_illum(self, leds, rgb, brightness=255, timeout=0):

		# Get current illum values
		illum = self.msg_illum.get()

		# Represent leds as binary string in LSB first
		led_bits = '{:06b}'.format(leds)[::-1]

		# Keep commanded values between upper and lower bounds
		br = self.clip_with_warn(brightness, 0, 255, "set_illum", "brightness")
		r = self.clip_with_warn(rgb[0], 0, 255, "set_illum", "red value")
		g = self.clip_with_warn(rgb[1], 0, 255, "set_illum", "green value")
		b = self.clip_with_warn(rgb[2], 0, 255, "set_illum", "blue value")

		# Create single int from rgb + bright
		value = (int(br) << 24) | (r << 16) | (g << 8) | b

		# Iterate through bits and set values for commanded bits
		for i, bit in enumerate(led_bits):
			if bit == '1':
				illum.data[i] = value

		# Command new values
		self.msg_illum.set(illum, timeout)

	#### POST ####################################################
	def post_flags(self, cmd_flags, op=None):

		# Check operation is valid
		assert op in self.flag_cmds, "Unrecognised operation passed to post_flags()"

		# Get current flags
		flags = self.msg_flags.get()

		if op == None:
			# Set flags to commanded value
			flags.data = cmd_flags

		elif op == "set":
			# Set commanded flags and ignore others
			flags.data |= cmd_flags

		elif op == "clear":
			# Clear commanded flags and ignore others
			flags.data &= ~cmd_flags

		elif op == "toggle":
			# Clear commanded flags and ignore others
			flags.data ^= cmd_flags

		# publish value once
		self.msg_flags.set(flags, 0)
		self.pub_flags.publish(self.msg_flags.get())

	def post_tone(self, freq, dur, vol):
		# Get currently commanded value
		tone = self.msg_tone.get()

		# Keep commanded values between upper and lower bounds
		tone.data[miro.constants.TONE_FREQ] = self.clip_with_warn(freq, 200, 2000, "post_tone", "frequency")
		tone.data[miro.constants.TONE_VOL] = self.clip_with_warn(vol, 0, 255, "post_tone", "volume")
		tone.data[miro.constants.TONE_DUR] = dur

		# Set value to commanded value
		self.msg_tone.set(tone, timeout=1)

		# publish value once
		self.pub_tone.publish(self.msg_tone.get())


	#### GETS ####################################################
	def get_sensors(self):
		return copy.deepcopy(self.sensors)

	def get_cliff(self):
		return self.sensors.cliff

	def get_sonar(self):
		return self.sensors.sonar

	def get_light(self):
		return self.sensors.light

	def get_touch_head(self):
		return self.sensors.touch_head

	def get_touch_body(self):
		return self.sensors.touch_body

	def get_accel_head(self):
		return self.sensors.accel_head

	def get_accel_body(self):
		return self.sensors.accel_body

	def get_cameras(self):
		return copy.copy(self.cameras)

	def get_microphones(self):
		return copy.copy(self.mics)

	def get_time(self):
		# We use the physical clock which is advanced in ROS callback
		return self.time_now

	def get_flags(self):
		return self.sensors.flags
