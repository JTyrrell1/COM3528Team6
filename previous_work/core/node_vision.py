from collections import deque 
import pandas as pd 
import rospy
import time
from sensor_msgs.msg import CompressedImage
import sys
import os
import numpy as np
import torch
from deepface import DeepFace 
import cv2
from cv_bridge import CvBridge, CvBridgeError
from collections import deque, Counter
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from defisheye import Defisheye
from ultralytics import YOLO
import apriltag

import node

class NodeVision(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "vision")

		options = apriltag.DetectorOptions( \
				families='tag16h5',
				border=1,
				nthreads=4,
				quad_decimate=1.0,
				quad_blur=0.0,
				refine_edges=True,
				refine_decode=False,
				refine_pose=False,
				debug=False,
				quad_contours=True)

		self.detector = apriltag.Detector(options)

		# emotion recognition 
		self.emotion = []
		self.face_in_view = False
		self.curr_emotion = ""
		self.face_timer = 0
		self.vision_models_active = True
		self.look_for_apriltag = False
		self.count = 0

		# object detection
		self.object_detect_active = False
		self.objects = []

		self.model =  YOLO("yolov10m.pt") # can change the model here, based on the ultralytics
		#self.items_to_track = ['mouse', 'banana', 'cell phone', 'apple', 'orange'] # need to change this once decided which objects are suitable for detection 
		self.items_to_track = ['car','book','cell phone','mouse','apple','banana','orange','cup','fork','knife','spoon','bottle','sports ball']
		self.classes_ =  [2,73, 67, 64, 47, 46, 49, 41, 42, 43, 44, 39, 32] # classes to be detected by miro 
		#self.items_to_track = ['book','cell phone','mouse','apple','banana','orange','cup','fork','knife','spoon','bottle','sports ball']
		self.detected_items = {item: deque([0]*5, maxlen=10) for item in self.items_to_track}
		#self.classes_ = [73, 67, 64, 47, 46, 49, 41, 42, 43, 44, 39, 32]

		# gesture recognition
		self._video_mode = False
		model_path = "./models/gesture_recognizer.task"  
		base_options = python.BaseOptions(model_asset_path=model_path)
		self.options = vision.GestureRecognizerOptions(base_options=base_options, 
												 running_mode=mp.tasks.vision.RunningMode.VIDEO if self._video_mode else mp.tasks.vision.RunningMode.IMAGE)
		self.recognizer = vision.GestureRecognizer.create_from_options(self.options)
		history_size = 1
		self.gesture_history = deque(maxlen=history_size)
		self.counter = 0 
		self.general_timer = 0

		# april tag
		self.window = [False] * 5

		# camera parameters
		self.input_camera = [None, None]
		self.mtx = np.array([
	        [1.04358065e+03, 0, 3.29969935e+02],
	        [0, 1.03845278e+03, 1.68243114e+02],
	        [0, 0, 1]])
		self.dist = np.array([[-3.63299415e+00, 1.52661324e+01, -7.23780207e-03, -7.48630198e-04, -3.20700124e+01]])
		self.focal_length = 330

		# ROS -> OpenCV converter
		self.image_converter = CvBridge()

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# subscribe
		self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
					CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
		self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
					CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

		# report
		print ("recording from 2 cameras, press CTRL+C to halt...")

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)

	def callback_cam(self, ros_image, index):

		# silently (ish) handle corrupted JPEG frames
		try:

			# convert compressed ROS image to raw CV image
			image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
			unfisheye_img = cv2.undistort(image,self.mtx,self.dist, None)

			# store image for display
			if unfisheye_img.all != None:
				self.input_camera[index] = unfisheye_img

		except CvBridgeError as e:

			# swallow error, silently
			#print(e)
			pass
		
	def tick(self):
		#cv2.imshow("caml", self.input_camera[0])
		#cv2.imshow("camr", self.input_camera[1])
		#cv2.waitKey(1)
		# Checks if face has been in view for the last 200 repetitions (~ 4 secs)
		if self.nodes.spatial.face_in_view:
			self.face_in_view = True
			self.nodes.spatial.face_in_view = False
			self.face_timer = 30
		elif self.face_timer > 0:
			self.face_timer = self.face_timer - 1
		else:
			self.face_in_view = False
			
		# state
		channels_to_process = [0, 1]
		outfile = [None, None, None]
		"""
		if self.look_for_apriltag:
			self.vision_models_active = False
			result_l = []
			result_r = []
			if not self.input_camera[0] is None:
				grey = cv2.cvtColor(self.input_camera[0], cv2.COLOR_BGR2GRAY)
				result_l = self.detector.detect(grey)
			
			if not self.input_camera[1] is None:
				grey = cv2.cvtColor(self.input_camera[1], cv2.COLOR_BGR2GRAY)
				result_r = self.detector.detect(grey)

			if result_l != [] or result_r != []:
				print("VISSIBLE")
				self.count = self.count + 1
			else:
				print("NOT")
				self.count = 0 
			if self.count > 2:
				print("DONE")

				self.look_for_apriltag = False
				self.count = 0
		"""
		image = self.input_camera[0]

		if self.look_for_apriltag and not image is None:
			outcome = self.check_april(image, 0)

			if outcome:
				self.look_for_apriltag = False
				print("DONE")
				self.window = [False] * 5

		# for each channel to process
		gestures = [None,None]
		for index in channels_to_process:

			# get image
			image = self.input_camera[index]
			self.input_camera[index] = image

			if not image is None:
				#if index == 0:
					#cv2.imshow("dsa",image)
					#cv2.waitKey(1)
				# handle
				self.input_camera[index] = None
				
				if self.vision_models_active == True:
					
					# Processes emotion if a face is present
					#if self.face_in_view and self.face_timer % 10 == 0 :
					#	print(self.face_timer)
					#	self.recognise_emotions(image)
					#	self.process_deepface()
					
					# Processes the hand gesture if there is one
					gestures[index] = self.get_gestures(image)		
					#print(gestures[index])

					if gestures[index] == "Closed_Fist" or gestures[index] == "Thumb_Down" or gestures[index] == "Thumb_Up":  
						gestures[index] = None
				# processes objects in the scene 
				if self.object_detect_active:
					rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
					self.objects = self.get_objects(rgb_img)
					print(f"Objs detected: {self.objects}")

		if gestures == [None,None]:
			self.gesture = ""
		else:
			if not gestures[0] == None:
				self.gesture = gestures[0]
			else:
				self.gesture = gestures[1]

		# for each camera
		for index in range(len(outfile)):

			# if open, release
			if not outfile[index] is None:
				outfile[index].release()

	# emotion recognition ________________________________________________________________

	def recognise_emotions(self,  input_image):
		emotion_rows_stored = 10
		race_rows_stored = 10
		gender_rows_stored = 10 

		objs = DeepFace.analyze(
			img_path = input_image, 
			actions = ['emotion'],
			enforce_detection = False,
		)

		#emotion related 
		self.emotion.append(objs[0]['emotion'])
		if len(self.emotion)>emotion_rows_stored:
			self.emotion.pop(0)
		return input_image

	def process_deepface(self):
		df_emotion = pd.DataFrame(self.emotion)
		emotion_winner = df_emotion.sum().idxmax()
		self.curr_emotion = emotion_winner

		print(f"I think you are {self.curr_emotion}")

	# gesture recognition ________________________________________________________________
	
	def get_gestures(self, input_image):
		image = mp.Image(image_format=mp.ImageFormat.SRGB, data=input_image)
		recognition_result = self.recognizer.recognize(image)
		for _, gesture in enumerate(recognition_result.gestures):
			
				#print(f"Appending: {gesture[0].category_name} {not gesture[0].category_name=='None'}")
			self.gesture_history.append(gesture[0].category_name)
			if (len(self.gesture_history)>0):
				most_common_class, count = Counter(self.gesture_history).most_common(1)[0]
				last_valid_detection = most_common_class
			else:
				last_valid_detection = None 
			if last_valid_detection == "None":
				last_valid_detection = None
			# Get the top gesture from the recognition result
			return last_valid_detection
		
	# object detection __________________________________________________________________

	def get_objects(self, input_image):
		results = self.model(input_image, verbose = False, classes = self.classes_)
		for r in results:
			detected_classes =r.boxes.cls.cpu().numpy().astype(int)
			#all_classes = r.names # get all the classes names 
			#results[0].show() # for checking, draws bboxes on top of the input image 

		mapped_classes = np.unique([self.model.names[int(cls)] for cls in detected_classes])

		for item in self.items_to_track:
			self.detected_items[item].append(1 if item in mapped_classes else 0)
		confirmed_detections = [item for item, dq in self.detected_items.items() if sum(dq) >= 4]
		return confirmed_detections

################################################################

	def check_april(self, image, index):
		#print(image.shape)
		if index == 0:
			image = image[0:360,200:640]
		else:
			image = image[0:360,0:440]

		#cv2.imshow("dsaw",image)
		#cv2.waitKey(1)
		grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		result = self.detector.detect(grey)

		result = True if result != [] else False
		self.update_window(result)

		counter = Counter(self.window)

		mode_value = max(counter, key=counter.get)

		return mode_value


	def update_window(self,new_val):
		if len(self.window) == 5:
			self.window.pop(0)
		
		self.window.append(new_val)
		print(self.window)

def error(msg):
	print(msg)
	sys.exit(0)


#{0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 
#10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 
# 19: 'cow', 20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 
# 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 
# 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon',
#  45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 
# 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 
# 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator',
#  73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'}

#[73, 67, 64, 47, 46, 49, 41, 42, 43, 44, 39, 32]
