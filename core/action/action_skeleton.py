import numpy as np
import tf
import rospy
import copy
import time
import miro2 as miro

from . import action_types

class ActionSkeleton(action_types.ActionTemplate):

	def finalize(self):
        """
            Used to initialise class variables, same as __init__
        """
		
		self.name = "skeleton"      # need to define a name for the action
		self.prio = 0.0             # helps track priority when computing

	def compute_priority(self):
		"""
			This function is necessary to allow the stimuli to influence whether and action
            is chosen or not, self.prio is adjusted 
		"""

		# inhibit action complete based on flag or not
		if not self.pars.flags.ACTION_SKELETON:
			return 0.0

		# continue normal calculation if there is no flag
		curr_time = rospy.get_time()

        # this template activates the 
		if curr_time > 60:
			self.start_time = rospy.time()
			self.prio = 1.0
		else:
			self.prio = 1.0 * 0.8 # decay if not
		
		return self.prio

	def start(self):
		"""
            This function is used when the action is first selected, it determines the number of steps 
            (service calls) that the action will perform before the action is completed. This function
            can also be used to change the state based on what is required for the action
		"""

        # ~40-50 steps per second
		step = 100

        self.action_start_time = rospy.get_time()
        
		# start pattern
		self.clock.start(step)

	def service(self):
		"""
			This function maintains the execution of the action while the clock is active,
            every "step" calls the service function.
		"""

        # This example will cause MiRo to spin for ~2 seconds (100 steps) every 60 seconds
        self.apply_push_body(np.array([0.0, 5.0, 0.0]), flags=miro.constants.PUSH_FLAG_VELOCITY)

    # Optional Functions

    def stop(self):
        """
            This is called when an action is complete or a different action surpasses the priority
            of the current action, this can be useful for resetting class variables etc
        """
        
        self.action_start_time = 0

        # Setting the priority to 0 after the action switches can be good for preventing the 
        # action from going on and off repeatedly
        self.prio = 0 
