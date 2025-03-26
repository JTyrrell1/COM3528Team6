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

BUFFER_STUFF_SAMPLES = 4000
MAX_STREAM_MSG_SIZE = (4096 - 48)

BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# how long to record before playing back in seconds?
RECORD_TIME = 5

MIC_SAMPLE_RATE = 20000

SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

import time
import sys
import os
import numpy as np
import wave, struct
import node

import miro2 as miro

from node_detect_audio_engine import *



class NodeRecord(node.Node):

	def __init__(self, sys):

		self.micbuf = np.zeros((0, 4), 'uint16')
		self.outbuf = None
		
		node.Node.__init__(self, sys, "record_audio")


	def tick_mics(self):

		msg = self.input.mics

	def record(self):
		msg = self.input.mics

		# getting None type error for msg.data
		while not msg is None:
			msg = self.input.mics
			msg.data = np.reshape(msg.data,(500,4))

			# append mic data to store
			self.micbuf = np.concatenate((self.micbuf, msg.data))

			# report
			sys.stdout.write(".")
			sys.stdout.flush()

			# finished recording?
			if self.micbuf.shape[0] >= SAMPLE_COUNT:

				# end recording
				self.outbuf = self.micbuf
				self.micbuf = None

			# write output file
			outfilename = 'recording.wav'
			file = wave.open(outfilename, 'wb')
			file.setsampwidth(2)
			file.setframerate(MIC_SAMPLE_RATE)

			# saves audio recording 
			file.setnchannels(2)
			x = np.reshape(self.outbuf[:, [0, 1]], (-1))
			for s in x:
				file.writeframes(struct.pack('<h', s))
			
			file.close()
			print ("wrote output file at", outfilename)



