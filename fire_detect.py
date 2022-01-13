from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
from imutils.video import VideoStream
import numpy as np
import imutils
import cv2
import time
from skimage.segmentation import slic
from skimage.segmentation import mark_boundaries
from skimage.util import img_as_float
from skimage import io
def extract_bounded_nonzero(input):

    # take the first channel only (for speed)

    gray = input[:, :, 0];

    rows = np.any(gray, axis=1)
    cols = np.any(gray, axis=0)
    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]
    return input[cmin:cmax,rmin:rmax]
print("Load MODELS/.........")
DNet=load_model("fire_net_detector.model")
cls=["fire","default"]
vs = VideoStream(src=0).start()
width=(224,224)
fps = video.get(cv2.CAP_PROP_FPS)
frame_time = round(1000/fps)
# loop over the frames from the video stream
time.sleep(1.0)

while True:
    frame = vs.read()
    if frame is None:
        break
    frame = cv2.resize(frame, width)
    faces = img_to_array(frame)
    image = img_as_float(io.frame)
    faces = faces / 255
    faces = np.expand_dims(faces, [0])
    preds = DNet.predict(faces)
    j = np.argmax(preds)
    slic = cv2.ximgproc.createSuperpixelSLIC(frame, region_size=22)
    slic.iterate(10)
    #segments = slic(image, n_segments=250, sigma=5)
    segments = slic.getLabels()
    for (i, segVal) in enumerate(np.unique(segments)):

        # Construct a mask for the segment
        mask = np.zeros(small_frame.shape[:2], dtype = "uint8")
        mask[segments == segVal] = 255

        # get contours (first checking if OPENCV >= 4.x)

        if (int(cv2.__version__.split(".")[0]) >= 4):
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # create the superpixel by applying the mask

        # N.B. this creates an image of the full frame with this superpixel being the only non-zero
        # (i.e. not black) region. CNN training/testing classification is performed using these
        # full frame size images, rather than isolated small superpixel images.
        # Using the approach, we re-use the same InceptionV1-OnFire architecture as described in
        # the paper [Dunnings / Breckon, 2018] with no changes trained on full frame images each
        # containing an isolated superpixel with the rest of the image being zero/black.

        superpixel = cv2.bitwise_and(small_frame, small_frame, mask = mask)

        # N.B. ... but for the later work using the InceptionV3-OnFire and InceptionV4-OnFire architecture
        # as described in the paper [Samarth / Breckon, 2019] we instead centre and pad the resulting
        # image with zeros

        if ((args.model_to_use == 3) or (args.model_to_use == 4)):


            superpixel = cv2.cvtColor(superpixel, cv2.COLOR_BGR2RGB)



            superpixel = pad_image(extract_bounded_nonzero(superpixel), 224, 224)


        output = DNet.predict([superpixel])
        if round(output[0][0]) == 1: # equiv. to 0.5 threshold in [Dunnings / Breckon, 2018],  [Samarth/Bhowmik/Breckon, 2019] test code
            # draw the contour
            # if prediction for FIRE was TRUE (round to 1), draw GREEN contour for superpixel
            cv2.drawContours(small_frame, contours, -1, (0,255,0), 1)

        else:
            # if prediction for FIRE was FALSE, draw RED contour for superpixel
            cv2.drawContours(small_frame, contours, -1, (0,0,255), 1)

    # stop the timer and convert to ms. (to see how long processing and display takes)

    stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000

    # image display and key handling

    cv2.imshow(windowName, small_frame)

    # wait fps time or less depending on processing time taken (e.g. 1000ms / 25 fps = 40 ms)

    key = cv2.waitKey(max(2, frame_time - int(math.ceil(stop_t)))) & 0xFF
    if (key == ord('x')):
        keepProcessing = False
    elif (key == ord('f')):
        cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)