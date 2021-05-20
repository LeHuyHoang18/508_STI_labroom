from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import imutils
import time
import cv2
import os
counter=0
def detect_and_predict_fire(frame,fireNet):







	fire=[]
	locs = []
	preds = fireNet.predict()






fireNet = load_model("fire_detector.model")


print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()

# loop over the frames from the video stream
while True:
	frame = vs.read()
	frame = imutils.resize(frame, width=400)


	(locs, preds) = detect_and_predict_fire(frame,fireNet)


	for (box, pred) in zip(locs, preds):

		(startX, startY, endX, endY) = box
		(firee, nonfire) = pred


		label = "fire" if firee >=  nonfire else "No fire"
		color = (0, 255, 0) if label == "fire" else (0, 0, 255)


		label = "{}: {:.2f}%".format(label, max(mask,withoutMask) * 100)


		cv2.putText(frame, label, (startX, startY - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
		cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)


	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		break


cv2.destroyAllWindows()
vs.stop()
