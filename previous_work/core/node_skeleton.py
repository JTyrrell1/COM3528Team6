import numpy as np
import tf
import rospy
import copy
import time
import miro2 as miro

from . import action_types

class NodeSkeleton(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "skeleton")

	def tick(self,msg):
		"""
			The tick function is unique for each node
		
