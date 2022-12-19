# import necessary packages
import cv2
import dlib
import time
from csv import writer
from datetime import datetime

class ObjCenter:
	def __init__(self):
		# load OpenCV's Haar cascade face detector
		# print("[INFO] loading HAAR Cascade filter")
		# self.cascade_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
		# load dlib's HOG + Linear SVM face detector
		# print("[INFO] loading HOG + Linear SVM face detector...")
		self.hog_detector = dlib.get_frontal_face_detector()
		# load dlib's CNN face detector
		# print("[INFO] loading CNN face detector...")
		self.cnn_detector = dlib.cnn_face_detection_model_v1("mmod_human_face_detector.dat")


	def convert_and_trim_bb(self, image, rect):
		# extract the starting and ending (x, y)-coordinates of the
		# bounding box
		# print("##################################################")
		startX = rect.left()
		startY = rect.top()
		endX = rect.right()
		endY = rect.bottom()
		# ensure the bounding box coordinates fall within the spatial
		# dimensions of the image
		startX = max(0, startX)
		startY = max(0, startY)
		endX = min(endX, image.shape[1])
		endY = min(endY, image.shape[0])
		# compute the width and height of the bounding box
		w = endX - startX
		h = endY - startY
		# return our bounding box coordinates
		return (startX, startY, w, h)

	def detect_face_rects_using_HOG(self, image):
		rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		# perform face detection using dlib's face detector
		start = time.time()
		results = self.hog_detector(rgb, 0)
		end = time.time()
		# convert the resulting dlib rectangle objects to bounding boxes,
		# then ensure the bounding boxes are all within the bounds of the
		# input image
		boxes = [self.convert_and_trim_bb(image, r) for r in results]
		return boxes

	def detect_face_rects_using_CNN(self, image):
		# load the input image from disk, resize it, and convert it from
		# BGR to RGB channel ordering (which is what dlib expects)
		# image = imutils.resize(image, width=600)
		rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		# perform face detection using dlib's face detector
		start = time.time()
		results = self.cnn_detector(rgb, 0)
		print(results)
		end = time.time()
		boxes = [self.convert_and_trim_bb(image, r.rect) for r in results]
		print("This is my box" )
		print(boxes)
		return boxes

	def detect_face_rects_using_cascade(self, image):
		# convert the frame to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# detect all faces in the input frame
		start = time.time()
	
		rects = self.cascade_detector.detectMultiScale(gray, scaleFactor=1.05,
			minNeighbors=9, minSize=(30, 30),
			flags=cv2.CASCADE_SCALE_IMAGE)
		end = time.time()
		return rects


	def update(self, frame, frameCenter):
		rects = self.detect_face_rects_using_HOG(frame)
		
		# check to see if a face was found
		if len(rects) > 0:
			largest_rect = rects[0]
			# loop over the bounding boxes

			for rect in rects:
				(_, _, w_largest, h_largest) = largest_rect
				(_,_, w, h) = rect
				if (w_largest*h_largest) < (w*h):
					largest_rect = rect
			# extract the bounding box coordinates of the face and
			# use the coordinates to determine the center of the
			# face
			(x, y, w, h) = largest_rect
			faceX = int(x + (w / 2.0))
			faceY = int(y + (h / 2.0))
			print(datetime.now(), frameCenter[0], frameCenter[1], faceX, faceY)
			# return the center (x, y)-coordinates of the face
			return ((faceX, faceY), rects[0])
		# otherwise no faces were found, so return the center of the
		# frame
		return (frameCenter, None)