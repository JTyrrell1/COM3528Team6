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

import math
import numpy as np
import copy

import miro2 as miro

# this controller operates by trying to keep a notional "fovea"
# in the location it is expected to be in if the robot moved
# exactly as commanded. using a fovea, which is separated from
# the axis of rotation, avoids the singularity around small
# errors near the rotational axis, at the cost of somewhat
# modified behaviour for large errors. some balance between
# these concerns is provided through the parameter "foveal_dist".
#
# the controller can be used in different modes. most simply,
# after initialisation, pass wheel_speed reports from the opto
# encoders in using update_sensors(), desired movements through
# command_velocity(), and pass the return value from that down
# to control/cmd_vel.
#
# alternatively, if you wish to use a separate motion planning
# model, you can call command_position() directly and pass its
# return value to control/cmd_vel.

class PoseController:

	def __init__(self, params = None):

		# debug
		self.trace_file = None
		self.record_trace_file = False

		# parameters
		#
		# foveal_dist should be short for tight rotational control
		# and longer for smoother movement (at the cost of more
		# 'poor parking' style of movement, that is less accurate
		# alignment during movements).
		#
		# the most conservative controller has sensor_lag=0, which
		# means we do not try to do any open loop control ahead of
		# the available sensor data. the real system (sim or robot)
		# doesn't attain commanded velocity until around 10 samples
		# but is off and away before that, so a linear model of
		# that is something like sensor_lag=9, gain_at_sensor_lag=0.1.
		# the default is to use about half of that.
		#
		# non-zero relaxation would be useful for long-term velocity
		# control, to avoid integral wind-up, but we leave it off by
		# default.
		#
		# gate_open_samples allows a little bit of clean up after
		# the input velocity drops to zero. 10 is enough for smooth
		# operation, a bit more improves final position reached at
		# the cost of more jerky coming to a stop.
		#
		# the gains (gain_r and gain_theta) control how fast the
		# controller pushes to catch up with the target pose, and
		# can be tuned for sharpness against smoothness.
		self.foveal_dist = 0.2
		self.sensor_lag = 4
		self.gain_at_sensor_lag = 0.2
		self.tau_relax = 0.0
		self.gate_open_samples = 20
		self.gain_r = 5.0
		self.gain_theta = 8.0

		# state
		self.gate_open = 0

		# handle passed params
		self.update_params(params)

		# controller estimate of pose in WORLD
		self.pose_est = miro.lib.Pose()

		# target pose in WORLD
		self.pose_tgt = miro.lib.Pose()
		self.pose_tgt_bak = miro.lib.Pose()

	def update_params(self, params):

		if not params is None:
			for key, value in params.items():
				if key == "foveal_dist":
					self.foveal_dist = value
				elif key == "sensor_lag":
					self.sensor_lag = value
				elif key == "gain_at_sensor_lag":
					self.gain_at_sensor_lag = value
				elif key == "tau_relax":
					self.tau_relax = value
				elif key == "gate_open_samples":
					self.gate_open_samples = value
				elif key == "gain_r":
					self.gain_r = value
				elif key == "gain_theta":
					self.gain_theta = value
				else:
					raise ValueError("unrecognised parameter key '" + key + "'")

		# get relax constant
		if self.tau_relax <= 0:
			self.gamma_relax = 0.0
		else:
			self.gamma_relax = 0.02 / self.tau_relax
			if self.gamma_relax > 1.0:
				self.gamma_relax = 1.0

		# build array of recent output command velocities
		# to support complementary filter at output
		self.dpose_hist = [miro.lib.DeltaPose()] * self.sensor_lag

		# compute complementary filter gains
		if self.sensor_lag > 0:
			self.gain_per_lag = (1.0 - self.gain_at_sensor_lag) / self.sensor_lag

	def update_sensors(self, wheel_speed, T = miro.constants.PLATFORM_TICK_SEC):

		# convert from speed to delta
		wheel_speed *= T

		# convert to dr/dtheta
		body_vel = miro.lib.wheel_speed2pose_vel(wheel_speed)

		# update estimate of pose
		self.pose_est.update(body_vel)

		# record debug information for dev/pose_controller/plot_trace
		if self.record_trace_file:
			if self.trace_file is None:
				self.trace_file = open("/run/user/1001/pose_controller_trace", "w")
			self.trace_file.write(str(wheel_speed[0]) + ", " + str(wheel_speed[1]) + ", " + str(body_vel) + " " + str(self.pose_est) + " " + str(self.pose_tgt) + "\n")

	def command_position(self, pose_tgt, T = miro.constants.PLATFORM_TICK_SEC, master_gain = 1.0):

		# call this function to provide the target model and
		# return the controlled velocity (with specified gain).

		# open gate if pose target is different from last sample
		if not self.pose_tgt.equals(self.pose_tgt_bak):
			self.gate_open = self.gate_open_samples
		elif self.gate_open > 0:
			# close gate gate_open_samples after last change
			self.gate_open -= 1

		# store pose target so we can detect when it is changing
		self.pose_tgt_bak.set(self.pose_tgt)

		# compute target fovea location
		xt = pose_tgt.x + np.cos(pose_tgt.theta) * self.foveal_dist
		yt = pose_tgt.y + np.sin(pose_tgt.theta) * self.foveal_dist

		# estimate physical pose using that estimated from sensors
		# but lagged, and recent command velocities sent out
		pose_est = copy.copy(self.pose_est)
		gain = self.gain_at_sensor_lag
		for i in range(0, len(self.dpose_hist)):
			pose_est.update(self.dpose_hist[i] * gain)
			gain += self.gain_per_lag

		# map that into egocentric space
		(xte, yte) = pose_est.egocentric(xt, yt)

		# find error from target fovea location
		xe = xte - self.foveal_dist
		ye = yte - 0.0

		# do control
		dr = xe * self.gain_r * master_gain
		dtheta = ye / self.foveal_dist * self.gain_theta * master_gain

		# apply platform constraints
		dr = np.clip(dr, -miro.constants.WHEEL_MAX_SPEED_M_PER_S, miro.constants.WHEEL_MAX_SPEED_M_PER_S)
		dtheta = np.clip(dtheta, -miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S, miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S)

		# apply gate
		if self.gate_open == 0:

			# gate output control off when input control is off
			# so we don't get jittering about zero with noise;
			# any remaining error will be worked on when the
			# next input control signals come in
			dr = 0.0
			dtheta = 0.0

		# compute controlled velocity
		dpose = miro.lib.DeltaPose(dr, dtheta)

		# store history of these for use in complementary filter
		if self.sensor_lag > 0:
			self.dpose_hist = self.dpose_hist[1:] + [dpose * T]

		# return
		return dpose

	def command_velocity(self, cmd_vel, T = miro.constants.PLATFORM_TICK_SEC, master_gain = 1.0):

		# call this function to move the target model for T
		# seconds with the specified desired velocity, and
		# return the controlled velocity (with specified gain).

		# if gate is closed
		if self.gate_open == 0:

			# relax target pose towards estimated pose so as not
			# to build up too much of an error over time
			self.pose_tgt.x += self.gamma_relax * (self.pose_est.x - self.pose_tgt.x)
			self.pose_tgt.y += self.gamma_relax * (self.pose_est.y - self.pose_tgt.y)
			dtheta = self.pose_est.theta - self.pose_tgt.theta
			if dtheta > np.pi:
				dtheta -= np.pi * 2.0
			if dtheta < -np.pi:
				dtheta += np.pi * 2.0
			self.pose_tgt.theta += self.gamma_relax * (dtheta)

		# drive target pose with commanded velocity
		dpose = miro.lib.DeltaPose(cmd_vel.dr * T, cmd_vel.dtheta * T)
		self.pose_tgt.update(dpose)

		# delegate to position controller
		return self.command_position(self.pose_tgt, T, master_gain)
