#!/usr/bin/env python

import cv2
import time
import os
import argparse

DATA_FOLDER = os.path.join(os.getenv('HOME'),'bags')
CONFIDENCE_THRESHOLD = 0.2
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]

class Dnn:
	def __init__(self, model_folder, mp4_filename, threshold=0.4):
		self.nms_threshold = threshold

		self.vc = cv2.VideoCapture(os.path.join(DATA_FOLDER, mp4_filename))

		self.net = cv2.dnn.readNet(os.path.join(model_folder, "yolov4-mish-416.weights"), os.path.join(model_folder, "yolov4-mish-416.cfg"))
		self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
		self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

		self.model = cv2.dnn_DetectionModel(self.net)
		self.model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

		self.class_names = []
		with open(os.path.join(model_folder, "coco.names"), "r") as f:
			self.class_names = [cname.strip() for cname in f.readlines()]

	def detect(self, frame):
		start = time.time()
		classes, scores, boxes = self.model.detect(frame, CONFIDENCE_THRESHOLD, self.nms_threshold)
		end = time.time()

		start_drawing = time.time()
		self.draw(frame, zip(classes, scores, boxes))
		end_drawing = time.time()

		fps_label = "FPS: %.2f (excluding drawing time of %.2fms)" % (1 / (end - start), (end_drawing - start_drawing) * 1000)
		cv2.putText(frame, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
		cv2.imshow("detections", frame)

	def draw(self, frame, zip):
		for (classid, score, box) in zip:
			color = COLORS[int(classid) % len(COLORS)]
			label = "%s : %f" % (self.class_names[classid[0]], score)
			cv2.rectangle(frame, box, color, 2)
			cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Image detection from dnn detects')
	parser.add_argument('folder', default=False, type=str, help='Folder data is stored in')
	parser.add_argument('video', default=False, type=str, help='Name of video for testing')

	args, unknown = parser.parse_known_args()
	if unknown: 
		print('Unknown args:',unknown)
		exit()

	dnn = Dnn(args.folder, args.video)

	while cv2.waitKey(1) != ord('q'):
		(grabbed, frame) = dnn.vc.read()
		if not grabbed:
			print("Not grabbed")
			exit()

		dnn.detect(frame)
