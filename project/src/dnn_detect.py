#!/usr/bin/env python

import cv2
import os

DATA_FOLDER = os.path.join(os.getenv('HOME'),'bags')
CONFIDENCE_THRESHOLD = 0.2

class Dnn:
	def __init__(self, model_folder, mp4_filename, threshold=0.4):
		self.nms_threshold = threshold

		self.vc = cv2.VideoCapture(os.path.join(DATA_FOLDER, mp4_filename))

		self.net = cv2.dnn.readNet(os.path.join(model_folder, "yolov4-mish-416.weights"), os.path.join(model_folder, "yolov4-mish-416.cfg"))
		self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
		self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

		self.model = cv2.dnn_DetectionModel(self.net)
		self.model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

		self.stop = False
		self.dtime = 0
		self.last_time = None

	def detect(self, frame, header):
		classes, _, boxes = self.model.detect(frame, CONFIDENCE_THRESHOLD, self.nms_threshold)

		new_boxes = []
		for clnum, cl in enumerate(classes):
			if cl[0] == 0 and boxes[clnum][2] >= 40:
				new_boxes.append(boxes[clnum])

		if self.last_time:
			self.dtime = header.stamp.secs - self.last_time
			self.last_time = header.stamp.secs

		else:
			self.last_time = header.stamp.secs
		
		if len(new_boxes) > 0:
			self.stop = True
			
		elif self.dtime < 1:
			self.stop = self.stop

		else:
			self.stop = False
			self.dtime = 0

		for box in new_boxes:
			cv2.rectangle(frame, box, (0, 255, 255), 2)
		
		cv2.putText(frame, str(self.stop), (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		cv2.imshow("detections", frame)
	