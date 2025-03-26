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

import miro2 as miro

class DeltaPose:

	def __init__(self, dr = 0.0, dtheta = 0.0):

		self.dr = dr
		self.dtheta = dtheta

	def __str__(self):

		return "[" + str(self.dr) + ", " + str(self.dtheta) + "]"

	def __mul__(self, factor):

		return DeltaPose(self.dr * factor, self.dtheta * factor)

	def zero(self):

		self.dr = 0.0
		self.dtheta = 0.0

class Pose:

	def __init__(self, x = 0.0, y = 0.0, theta = 0.0):

		self.x = x
		self.y = y
		self.theta = theta

	def __str__(self):

		return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + "]"

	def equals(self, rhs):

		return self.x == rhs.x and self.y == rhs.y and self.theta == rhs.theta

	def set(self, rhs):

		self.x = rhs.x
		self.y = rhs.y
		self.theta = rhs.theta

	def egocentric(self, x, y):

		# map provided point into the Pose
		x -= self.x
		y -= self.y
		cos = np.cos(-self.theta)
		sin = np.sin(-self.theta)
		return (cos * x - sin * y, sin * x + cos * y)

	def update(self, dpose):

		# update theta
		self.theta += dpose.dtheta

		# update position
		self.x += np.cos(self.theta) * dpose.dr
		self.y += np.sin(self.theta) * dpose.dr
