# import the necessary packages
import numpy as np
import cv2
import time
from skimage.feature import hog
from sklearn.externals import joblib
from nms import nms
import argparse
 
def appendRects(i, j, conf, c, rects):
    x = int((j)*pow(scaleFactor, c))
    y = int((i)*pow(scaleFactor, c))
    w = int((64)*pow(scaleFactor, c))
    h = int((128)*pow(scaleFactor, c))
    rects.append((x, y, conf, w, h))

parser = argparse.ArgumentParser(description='To read image name')

parser.add_argument('-i', "--image", help="Path to the test image")
parser.add_argument('-d','--downscale', help="Downscale ratio", default=1.2, type=float)
parser.add_argument('-v', '--visualize', help="Visualize the sliding window", action="store_true")
parser.add_argument('-w', '--winstride', help="Pixels to move in one step, in any direction", default=8, type=int)
parser.add_argument('-n', '--nms_threshold', help="Threshold Values between 0 to 1 for NMS thresholding. Default is 0.2", default=0.001, type=float)
args = vars(parser.parse_args())



clf = joblib.load("pedestrian.pkl")

scaleFactor = args["downscale"]
inverse = 1.0/scaleFactor
winStride = (args["winstride"], args["winstride"])
# winSize = (128, 64)
winSize = (128, 64)

rects = []


cv2.startWindowThread()

# open webcam video stream
cap = cv2.VideoCapture(0)




while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # frame = cv2.imread(args["image"])

    # resizing for faster detection
    frame = cv2.resize(frame, (266, 190))
    #print(frame.shape)
    # using a greyscale picture, also for faster detection
    img = frame.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    h, w = gray.shape
    count = 0

    while (h >= 128 and w >= 64):

    # print (gray.shape)

        h, w = gray.shape
        horiz = w - 64
        vert = h - 128
        # print (horiz, vert)
        i = 0
        j = 0
        while i < vert:
            j = 0
            while j < horiz:

                portion = gray[i:i+winSize[0], j:j+winSize[1]]
                features = hog(portion, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), block_norm="L2")

                result = clf.predict([features])

                if args["visualize"]:
                    visual = gray.copy()
                    cv2.rectangle(visual, (j, i), (j+winSize[1], i+winSize[0]), (0, 0, 255), 2)
                    cv2.imshow("visual", visual)
                    cv2.waitKey(1)

                if int(result[0]) == 1:
                    #print (result, i, j)
                    confidence = clf.decision_function([features])
                    appendRects(i, j, confidence, count, rects)


                j = j + winStride[0]

            i = i + winStride[1]

        gray = cv2.resize(gray, (int(w*inverse), int(h*inverse)), interpolation=cv2.INTER_AREA)
        count = count + 1
        # print (count)

    # print (rects)

    nms_rects = nms(rects, args["nms_threshold"])

    # for (a, b, conf, c, d) in rects:
    #     cv2.rectangle(frame, (a, b), (a+c, b+d), (0, 255, 0), 2)

    # cv2.imshow("Before NMS", frame)
    # cv2.waitKey(0)



    for (a, b, conf, c, d) in nms_rects:
        cv2.rectangle(img, (a, b), (a+c, b+d), (0, 255, 0), 2)

    cv2.imshow("After NMS", img)


    # detect people in the image
    # returns the bounding boxes for the detected objects
    # boxes, weights = hog.detectMultiScale(frame, winStride=(2,2), padding = (8,8), scale = 1.05 )

    # boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    # for (xA, yA, xB, yB) in boxes:
    #     # display the detected boxes in the colour picture
    #     cv2.rectangle(frame, (xA, yA), (xB, yB),
    #                       (0, 255, 0), 2)
    
    # Write the output video 
    # out.write(frame.astype('uint8'))
    # Display the resulting frame
    # cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
# and release the output
# out.release()
# finally, close the window
cv2.destroyAllWindows()
cv2.waitKey(1)
