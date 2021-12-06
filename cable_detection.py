from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
from imutils.video import VideoStream
import numpy as np
import imutils
import cv2
import time
print("Load MODELS/.........")
DNet=load_model("damage_detection.model")
cls=["damage","original"]
vs = VideoStream(src=0).start()
width=(224,224)
# loop over the frames from the video stream
time.sleep(1.0)

while True:
    frame = vs.read()
    if frame is None:
        break
    frame = cv2.resize(frame, width)
    faces = img_to_array(frame)

    faces = faces / 255
    faces = np.expand_dims(faces, [0])
    preds = DNet.predict(faces)
    j = np.argmax(preds)
   # (damage, original) = preds
    label=cls[j]
    text = label if label == "Non-damaged" else "WARNING! damaged!"
    color = (0, 255, 0) if label == "damage" else (0, 0, 255)
    #label = "{}: {:.2f}%".format(label, max(damage, original) * 100)
    cv2.putText(frame,label,(30,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

    cv2.imshow("Frame",frame)
    key=cv2.waitKey(1) & 0xFF
    if key==ord("q"):
        break
cv2.destroyAllWindows()
#vs.stop()


