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

import numpy as np
import miro2 as miro
import rospy
import os
import imp



class ROS(object):

	def __init__(self):

		self.log_level = rospy.DEBUG
		self.robot_name = "miro"

		# use MIRO_ROBOT_NAME
		MIRO_ROBOT_NAME = os.getenv('MIRO_ROBOT_NAME')
		if (not MIRO_ROBOT_NAME is None) and (len(MIRO_ROBOT_NAME) > 0):
			self.robot_name = MIRO_ROBOT_NAME

	def get_log_level_string(self):

		if self.log_level == rospy.DEBUG:
			return "verb"
		if self.log_level == rospy.INFO:
			return "info"
		if self.log_level == rospy.WARN:
			return "warn"
		if self.log_level == rospy.ERROR:
			return "silent"
		if self.log_level == rospy.FATAL:
			return "silent"

	def get_cmd_opt_string(self):

		opt = " "
		opt += " log=" + self.get_log_level_string()
		opt += " robot=" + self.robot_name
		return opt



class Platform(object):

	def __init__(self):

		# time for movement to settle after commands stop changing
		self.t_mov_settle = 0.5

		# time for head IMU to stop reporting jerk after eye blink command
		self.t_blink_settle = 0.5

		# time for IMU to stop reporting jerk after any cosmetic command change in that section
		# see dev/system_monitor/record_cos_and_jerk
		self.t_head_cos_settle = 0.8
		self.t_body_cos_settle = 0.8

		# time for our own vocalisations to fall out of the microphone buffers
		self.t_vocalising_settle = 0.1

	def finalize(self, pars):

		self.n_mov_settle = int(self.t_mov_settle * pars.timing.tick_hz)
		self.n_blink_settle = int(self.t_blink_settle * pars.timing.tick_hz)
		self.n_head_cos_settle = int(self.t_head_cos_settle * pars.timing.tick_hz)
		self.n_body_cos_settle = int(self.t_body_cos_settle * pars.timing.tick_hz)
		self.n_vocalising_settle = int(self.t_vocalising_settle * pars.timing.tick_hz)



class Express(object):

	def __init__(self):

		# lights
		self.ORANGE_red = 128
		self.ORANGE_grn = 64
		self.ORANGE_blu = 0
		self.led_phase_range = np.pi * 0.5
		self.led_phase_separation = np.pi * 0.5

		# eyelids
		self.eyelids_droop_on_touch = 0.5
		self.blink_mean_interval = 500
		self.blink_refractory_period = 100
		self.blink_period = 8
		self.double_blink_period = 16
		self.double_blink_prob = 0.2

		# tail
		self.tail_wag_max_amp = 0.5



class Body(object):

	def __init__(self):

		self.body_pose_filt_gain_dr = 0.8
		self.body_pose_filt_gain_dtheta = 0.8
		self.wheels_sleep_suppress_gain = 1.0
		self.lift_sleep_droop_gain = 1.0
		self.body_pose_filt_L = 10
		self.sonar_min_range = 0.10
		self.sonar_max_range = 0.25
		self.max_accel_mmpsps = 4000.0
		self.max_decel_mmpsps = 4000.0



class Selection(object):

	def __init__(self):

		self.selection_hysteresis = 0.05
		self.selection_noise_mag = 0.01



class Lower(object):

	def __init__(self):

		self.user_touch_tau_attack = 0.25
		self.user_touch_tau_release = 0.5
		self.user_touch_min = 0.01
		self.user_touch_gain = 0.2

		# probability / period filter on user interactivity
		self.interact_prob = 1.0 # probability of interaction in each interact_period
		self.interact_period = 10.0 # seconds

		# touch sensor masks mask out sensor elements that are unreliable
		self.touch_head_mask = 0xFFFF
		self.touch_body_mask = 0xFFF0

		# lists of touch elements involved in each stroke line
		# format is [<first bit>, <number of bits>, <stroke polarity>]
		#
		# NB: central line is disabled because MIRO strokes itself there!
		#	[0, 4, -1.0],
		self.stroke_lines = [
			[4, 5, 1.0],
			[9, 5, 1.0]
		]

		# ambient sound level filter
		self.sound_level_tau = 10.0

	def finalize(self, pars):

		fS = pars.timing.tick_hz
		self.user_touch_gamma_attack = miro.lib.tau2gamma( self.user_touch_tau_attack, fS )
		self.user_touch_gamma_release = miro.lib.tau2gamma( self.user_touch_tau_release, fS )
		self.sound_level_gamma = miro.lib.tau2gamma( self.sound_level_tau, fS )



class Decode(object):

	def __init__(self):

		# image height at which we process within demo
		self.image_height = 100 # 90, 100, 120, 150

	def finalize(self, pars):

		self.image_width = self.image_height * 16 / 9



class DetectAudio(object):

	def __init__(self):

		self.inter_ear_distance = 0.104 # metres
		self.raw_magnitude_thresh = 0.01 # normalized; audio event processing skipped unless over thresh
		self.assumed_sound_source_height = 1.0 # metres
		self.assumed_sound_source_range = 1.5 # metres

	def finalize(self, pars):

		speed_of_sound = 343.0 # m/s
		self.inter_ear_lag = self.inter_ear_distance / speed_of_sound * miro.constants.MIC_SAMPLE_RATE



class DetectFace(object):

	def __init__(self):

		self.resources_dir = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades"

	def finalize(self, pars):

		pass



class Action(object):

	def __init__(self):

		self.priority_idle = 0.1
		self.priority_uninterruptable = 1.0
		self.priority_high = 0.9
		self.priority_attend = 0.8
		self.priority_medium = 0.5

		# probability that an action is executed (rather than shammed)
		self.action_prob = 1.0

		self.fixation_region_width = 0.5
		self.range_estimate_min = 0.1
		self.range_estimate_max = 2.0
		self.size_large = 0.1

		self.halt_stall_input_filt = 0.333
		self.halt_stall_output_filt = 0.15
		self.halt_stall_acc_filt = 0.1
		self.halt_stall_thresh = 0.25
		self.halt_stall_eff_gain = 0.5
		self.halt_stall_acc_gain = 15.0
		self.halt_num_steps = 20

		self.orient_speed_sec_per_rad = 2.0
		self.orient_gaze_target_radius = 1.0
		self.orient_min_steps = 25
		self.orient_max_steps = 100
		self.orient_appetitive_commitment = 0.2
		self.orient_in_HEAD = False
		self.orient_follow_arc = True

		self.move_fixation_thresh = 0.8
		self.move_size_gain = 0.2
		self.move_fixation_gain = 0.6
		self.move_valence_gain = 0.5
		self.move_arousal_gain = 0.5

		self.approach_speed_mps = 0.2
		self.approach_min_steps = 50
		self.approach_max_steps = 300
		self.approach_appetitive_commitment = 0.5

		self.flee_speed_mps = 0.3
		self.flee_min_steps = 50
		self.flee_max_steps = 300
		self.flee_appetitive_commitment = -0.2

		self.avert_base_prio = 0.0
		self.avert_algorithm = 'body'
		self.avert_mean_speed = 0.2
		self.avert_min_steps = 75
		self.avert_max_steps = 200
		self.avert_retreat_distance = 0.4
		self.avert_turn_distance = 0.1
		self.avert_variability = 0.1
		self.avert_yaw_gain = 0.1
		#self.avert_retreat_boost = 0.0
		#self.avert_retreat_boost_tau = 0.25

		self.retreat_distance_m = 0.6
		self.retreat_speed_mps = 0.2
		self.retreat_rand_gain = 0.3

		self.cliff_thresh = 6
		self.cliff_margin = 1

		# known sizes of particular objects are used to estimate range
		self.face_size_m = 0.25
		self.ball_size_m = 0.055
		self.april_size_m = 0.060

		# these are the definitions of the source types, each of which
		# has a distinct appetitive value
		self.priority_source_count = 3
		self.priority_source_index_face = 0
		self.priority_source_index_ball = 1
		self.priority_source_index_april = 2
		self.priority_source_appetitive_value = np.array([0.25, 0.25, 1.0])

	def finalize(self, pars):

		# derived
		fS = pars.timing.tick_hz
		self.fixation_width_recip = 1.0 / self.fixation_region_width
		self.size_large_recip = 1.0 / self.size_large
		self.approach_speed_spm = 1.0 / self.approach_speed_mps
		self.flee_speed_spm = 1.0 / self.flee_speed_mps
		#self.avert_retreat_boost_lambda = miro.lib.tau2lambda(self.avert_retreat_boost_tau, fS)



class Spatial(object):

	def __init__(self):

		self.degrees_hindsight = 30
		self.association_angle = 0.0872 # 5 degrees is 0.0872 rad
		self.pri_decay_lambda = 0.25
		self.pri_peak_height_thresh = 0.75
		self.audio_event_azim_size_rad = 0.1
		self.audio_event_elev_size_rad = 0.3
		self.audio_event_gain = 5.0
		self.audio_event_gain_making_noise = 1.0
		self.mov_gain = 1.0
		self.mov_gain_adapt_tgt = 0.04
		self.mov_gain_adapt_tau = 5.0
		self.mov_gain_adapt_max = 3.0
		self.mov_gain_adapt_min = 0.5
		self.ball_gain = 0.5
		self.face_gain = 0.65
		self.april_gain_at_1m = 0.35

	def finalize(self, pars):

		fS = pars.timing.tick_hz
		self.audio_event_azim_size_recip = 1.0 / self.audio_event_azim_size_rad
		self.audio_event_elev_size_recip = 1.0 / self.audio_event_elev_size_rad
		self.mov_gain_adapt_gamma = miro.lib.tau2gamma( self.mov_gain_adapt_tau, fS )



class Affect(object):

	def __init__(self):

		self.valence_stroke_gain = 0.05
		self.valence_pet_gain = 0.025
		self.valence_audio_level_min = 0.2
		self.valence_audio_level_max = 1.0
		self.tau_audio_level_accum = 0.1
		self.valence_audio_gain = -0.05
		self.valence_jerk_thresh = 1.0
		self.valence_jerk_thresh_2 = 4.0
		self.valence_jerk_slope = 0.5
		self.valence_jerk_gain = -0.4
		self.valence_orient_fixation_gain = 0.1
		self.valence_sleep_blocked_thresh = 0.75
		self.valence_sleep_blocked_gain = -0.001
		self.valence_action_target_gain = 0.5
		self.arousal_action_target_gain = 1.0
		self.arousal_stroke_thresh = 0.75
		self.arousal_stroke_gain = 0.05
		self.arousal_pet_gain = 0.025
		self.arousal_audio_level_min = 0.2
		self.arousal_audio_level_max = 1.0
		self.arousal_audio_gain = 0.025
		self.arousal_light_mean_offset = 0.2
		self.arousal_light_mean_gain = 0.1
		self.arousal_nominal = 0.5
		self.arousal_gain_rtc = 0.125
		self.arousal_target_emotion_weighting = 2.0
		self.tau_respond_drive = 1.0
		self.tau_neutral_valence_asleep = 120.0
		self.tau_neutral_valence_awake = 600.0
		self.tau_mood = 4.0
		self.tau_emotion = 4.0
		self.sleep_dyn_p = 1.0 * 0.0002
		self.sleep_dyn_q = -1.0 * 0.0002
		self.sleep_dyn_a = -1.0 * 0.002
		self.sleep_dyn_b = 2.0 * 0.002
		self.sleep_dyn_c = -8.0 * 0.002
		self.fast_sleep_dyn_gain = 25.0

	def finalize(self, pars):

		self.arousal_target_emotion_weighting_recip = 1.0 / ( 1.0 +
								self.arousal_target_emotion_weighting )
		self.valence_audio_level_rng_recip = 1.0 / (
							self.valence_audio_level_max -
							self.valence_audio_level_min )
		self.arousal_audio_level_rng_recip = 1.0 / (
							self.arousal_audio_level_max -
							self.arousal_audio_level_min )

		fS = pars.timing.tick_hz
		self.lambda_audio_level_accum = miro.lib.tau2lambda( self.tau_audio_level_accum, fS )
		self.gamma_neutral_valence_asleep = miro.lib.tau2gamma(self.tau_neutral_valence_asleep, fS)
		self.gamma_neutral_valence_awake = miro.lib.tau2gamma(self.tau_neutral_valence_awake, fS)
		self.gamma_mood = miro.lib.tau2gamma(self.tau_mood, fS)
		self.gamma_emotion = miro.lib.tau2gamma(self.tau_emotion, fS)
		self.gamma_respond_drive = miro.lib.tau2gamma(self.tau_respond_drive, fS)



class Flags( object ):

	def __init__( self ):

		# most defaults are 1 (enabled)
		self.AFFECT_ENABLE				= 1
		self.AFFECT_ADJUST_RTC			= 1
		self.AFFECT_VALENCE_DYNAMICS	= 1
		self.AFFECT_AROUSAL_DYNAMICS	= 1
		self.AFFECT_ENABLE_SLEEP		= 1
		self.AFFECT_ENABLE_UNHAPPY		= 1
		self.AFFECT_FROM_CLOCK			= 1
		self.AFFECT_FROM_TOUCH			= 1
		self.AFFECT_FROM_LIGHT			= 1
		self.AFFECT_FROM_SOUND			= 1
		self.AFFECT_FROM_WAKEFULNESS	= 1
		self.AFFECT_FROM_ACCEL			= 1
		self.AFFECT_FROM_SLEEP_BLOCKED	= 1
		self.AFFECT_FROM_ACTION_TARGET	= 1

		self.EXPRESS_ENABLE				= 1
		self.EXPRESS_THROUGH_LIGHT		= 1
		self.EXPRESS_THROUGH_TAIL		= 1
		self.EXPRESS_THROUGH_EARS		= 1
		self.EXPRESS_THROUGH_EYELIDS	= 1
		self.EXPRESS_THROUGH_VOICE		= 1
		self.EXPRESS_THROUGH_NECK		= 1
		self.EXPRESS_THROUGH_WHEELS		= 1

		self.ACTION_ENABLE				= 1
		self.ACTION_ENABLE_INPUT		= 1
		self.ACTION_MODULATE_BY_SONAR	= 1
		self.ACTION_MODULATE_BY_CLIFF	= 1
		self.ACTION_ENABLE_SONAR_STOP	= 1
		self.ACTION_ENABLE_MOVE_AWAY	= 1
		self.ACTION_HALT_ON_STALL		= 1
		self.ACTION_ENABLE_SPECIAL		= 1

		self.SALIENCE_FROM_MOTION		= 1
		self.SALIENCE_FROM_SOUND		= 1
		self.SALIENCE_FROM_FACE			= 1
		self.SALIENCE_FROM_BALL			= 1
		self.SALIENCE_FROM_APRIL		= 1

		self.BODY_ENABLE_CLIFF_REFLEX	= 1
		self.BODY_ENABLE_TRANSLATION	= 1
		self.BODY_ENABLE_ROTATION		= 1
		self.BODY_ENABLE_NECK_MOVEMENT	= 1

		# developer flags accessible from MIROapp (for now)
		self.DEBUG_SONAR				= 0
		self.DEBUG_DETECTION			= 0

class Dev( object ):

	def __init__( self ):

		# default is all disabled for production systems
		self.focus_action			= None
		self.RANDOMIZE_VALENCE		= 0
		self.FAST_SLEEP_DYNAMICS	= 0
		self.DEBUG_HALT				= 0
		self.DETECT_FACE			= 0
		self.DETECT_BALL			= 0
		self.MULL_ONLY				= 0
		self.ORIENT_ONLY			= 0
		self.NO_FLEE				= 0
		self.RUN_TEST_ACTION		= 0
		self.RECONFIG_CAMERA_QUICK	= 0
		self.DEBUG_ACTION_PARAMS	= 0
		self.DEBUG_ORIENT_START		= 0
		self.DEBUG_ORIENTS			= 0
		self.SEND_DEBUG_TOPICS		= 0
		self.DEBUG_WRITE_TRACES		= 0
		self.IGNORE_CLIFF_SENSORS	= 0
		self.START_CAMS_HORIZ		= 0
		self.SHOW_LOC_EYE			= 0
		self.DEBUG_AUTO_STOP		= 0
		self.SIMULATE_CLIFF			= 0

	def allow(self):

		# print
		print ("allowing developer options...")

		# select one action (as well as mull) to allow
		# use None to disable
		self.focus_action = "special"

		# dev flags
		self.DEBUG_ACTION_PARAMS	= 1
		self.SEND_DEBUG_TOPICS		= 1

class CorePars (object):

	def __init__(self):

		# import platform pars
		platform_pars = miro.lib.platform_pars.PlatformPars()
		self.timing = platform_pars.timing
		self.geom = platform_pars.geom
		self.camera = platform_pars.camera

		# augment with core pars
		self.ros = ROS()
		self.platform = Platform()
		self.lower = Lower()
		self.decode = Decode()
		self.express = Express()
		self.body = Body()
		self.selection = Selection()
		self.action = Action()
		self.spatial = Spatial()
		self.affect = Affect()
		self.detect_audio = DetectAudio()
		self.detect_face = DetectFace()
		self.flags = Flags()
		self.dev = Dev()

		# set default demo_flags
		self.demo_flags = "vst"
		self.allow_dev = False
		
		# extra functionality to prevent changing flag
		self.fix_default_flags = True
		self.default_demo_flags = self.demo_flags

		# finalize components
		self.finalize_components()

	def demo_flag_state_string(self, state):

		return "(FLAG SET)" if state else "-"

	def demo_flag_norm_on(self, flag, desc):

		# process a flag and return a "normally on" state unless the flag is set
		state = flag in self.demo_flags
		print (flag, "|", desc.ljust(32), "|", self.demo_flag_state_string(state))
		return 0 if state else 1

	def demo_flag_norm_off(self, flag, desc):

		# process a flag and return a "normally off" state unless the flag is set
		return 1 - self.demo_flag_norm_on(flag, desc)

	def action_demo_flags(self):

		# report
		flags = self.demo_flags
		print ("----------------------------------------------------------------")
		print ("demo_flags: \"" + flags + "\"")
		print ("----------------------------------------------------------------")

		# handle flags
		# DOCLINK demo_flags
		# follow DOCLINK (in lib/miro2, lib/miro2_docs) to see all places where flags must be sync'd
		#
		# note "A" is actioned in on_system_ready.sh
		#
		self.flags.EXPRESS_THROUGH_VOICE = self.demo_flag_norm_on("v", "disable vocalisation")
		self.flags.BODY_ENABLE_TRANSLATION = self.demo_flag_norm_on("t", "disable translation")
		self.flags.BODY_ENABLE_ROTATION = self.demo_flag_norm_on("r", "disable rotation")
		self.flags.BODY_ENABLE_NECK_MOVEMENT = self.demo_flag_norm_on("n", "disable neck movement")
		#
		self.flags.AFFECT_ENABLE_SLEEP = self.demo_flag_norm_on("s", "disable sleep")
		self.flags.BODY_ENABLE_CLIFF_REFLEX = self.demo_flag_norm_on("c", "disable cliff reflex")
		self.flags.ACTION_MODULATE_BY_CLIFF = self.demo_flag_norm_on("u", "disable shun cliffs")
		self.flags.AFFECT_FROM_SOUND = self.demo_flag_norm_on("d", "disable affect from sound")
		#
		self.flags.SALIENCE_FROM_MOTION = self.demo_flag_norm_on("M", "disable attend motion")
		self.flags.SALIENCE_FROM_SOUND = self.demo_flag_norm_on("S", "disable attend sound")
		self.flags.SALIENCE_FROM_FACE = self.demo_flag_norm_on("F", "disable attend face")
		self.flags.SALIENCE_FROM_BALL = self.demo_flag_norm_on("B", "disable attend ball")
		self.flags.SALIENCE_FROM_APRIL = self.demo_flag_norm_on("G", "disable attend control cube")
		#
		self.flags.EXPRESS_THROUGH_TAIL = self.demo_flag_norm_on("T", "disable express through tail")
		self.flags.EXPRESS_THROUGH_LIGHT = self.demo_flag_norm_on("L", "disable express through light")
		self.flags.EXPRESS_THROUGH_EARS = self.demo_flag_norm_on("E", "disable express through ears")
		self.flags.EXPRESS_THROUGH_EYELIDS = self.demo_flag_norm_on("Y", "disable express through eyelids")
		#
		self.flags.AFFECT_ENABLE_UNHAPPY = self.demo_flag_norm_on("h", "disable negative valence")
		self.flags.ACTION_ENABLE_MOVE_AWAY = self.demo_flag_norm_on("a", "disable move away")
		self.flags.ACTION_HALT_ON_STALL = self.demo_flag_norm_on("H", "disable halt on stall")
		self.flags.ACTION_MODULATE_BY_SONAR = self.demo_flag_norm_on("m", "disable sonar modulation")
		self.flags.ACTION_ENABLE_SONAR_STOP = self.flags.ACTION_MODULATE_BY_SONAR
		self.flags.ACTION_ENABLE_SPECIAL = self.demo_flag_norm_on("x", "disable special responses")
		#
		self.flags.DEBUG_SONAR = self.demo_flag_norm_off("P", "[debug] debug sonar modulation")
		self.flags.DEBUG_DETECTION = self.demo_flag_norm_off("Q", "[debug] debug detection")
		#
		# dev flags are not accessible from MIROapp, but must be "allowed" by locally setting "D";
		# this ensures that if we accidentally leave any DEV flags on in a release, they are muted
		# on production systems
		self.allow_dev = self.demo_flag_norm_off("D", "[dev] allow developer options")

		# report
		print ("----------------------------------------------------------------")

	def finalize_components(self):

		# you must call this function after you change any parameters
		self.platform.finalize(self)
		self.lower.finalize(self)
		self.decode.finalize(self)
		self.action.finalize(self)
		self.affect.finalize(self)
		self.detect_audio.finalize(self)
		self.detect_face.finalize(self)
		self.spatial.finalize(self)

	def finalize(self):

		# changes to parameters are stored in demo_parameters.py
		# which allows the user to change anything in this object
		import_path = os.getenv("MIRO_DIR_CONFIG")
		module_name = "demo_parameters"

		# try to find the user config file
		try:
			q = imp.find_module(module_name, [import_path,])
			try:
				module = imp.load_module(module_name, q[0], q[1], q[2])
				q[0].close()
				module.update_parameters(self)

				# for developer convenience, we also allow demo_flags
				# in this file, but we don't advertise this to the user
				try:
					self.demo_flags = module.demo_flags
				except:
					pass
			except:
				# not parsed
				print("failed to import", import_path + "/demo_parameters.py")
				if q[0]:
					q[0].close()
		except:
			# not found
			print("user parameters not found at", import_path + "/demo_parameters.py")
			pass

		# some parameters need to be accessible through MIROapp
		# and may also affect the bridge; these are stored in
		# platform_parameters (this includes "demo_flags" which
		# at time of writing does not affect the bridge, but may
		# do in future if we add more flags).
		params_filename = import_path + "/platform_parameters"
		if not os.path.isfile(params_filename):
			miro.lib.warning("platform parameters not found at " + params_filename)

		else:
			with open(params_filename) as f:
				file = f.readlines()

			for line in file:

				try:

					line = line.strip()

					if len(line) == 0:
						continue

					if line[0] == "#":
						continue

					eq = line.find("=")
					if eq == -1:
						continue

					key = line[0:eq]
					val = line[eq+1:]

					if key == "cliff_thresh":
						if val:
							self.action.cliff_thresh = int(val)
					if key == "cliff_margin":
						if val:
							self.action.cliff_margin = int(val)
					if key == "behav_prob":
						if val:
							self.action.action_prob = float(val) / 100.0
							self.lower.interact_prob = float(val) / 100.0
					if key == "demo_flags":
						self.demo_flags = val

				except:

					print ("exception handling line:", line)
		
		if self.fix_default_flags:
			self.demo_flags = self.default_demo_flags

		# action demo_flags
		self.action_demo_flags()

		# if allow_dev
		if self.allow_dev:
			self.dev.allow()

		# finalize components (again, in case any changes happened above)
		self.finalize_components()

		# final reports
		if self.lower.interact_prob == 1.0:
			print ("not rolling interact dice (interact_prob = 100%)")
		if self.action.action_prob == 1.0:
			print ("not rolling sham dice (action_prob = 100%)")

		# report
		print ("parameters now finalized")
		print ("----------------------------------------------------------------")
