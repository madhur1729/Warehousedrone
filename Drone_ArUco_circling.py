import cv2
import numpy as np
from djitellopy import tello
from time import sleep
import math

me = tello.Tello()
me.connect()
print(me.get_battery())

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
arucoParams = cv2.aruco.DetectorParameters_create()
me.takeoff()
me.streamon()
while True:
    img = me.get_frame_read().frame
    image: None = cv2.resize(img, (360, 240))

    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = [int(topRight[0]), int(topRight[1])]
            bottomRight = [int(bottomRight[0]), int(bottomRight[1])]
            bottomLeft = [int(bottomLeft[0]), int(bottomLeft[1])]
            topLeft = [int(topLeft[0]), int(topLeft[1])]
            cor = np.array([topRight,bottomRight,bottomLeft,topLeft])
            area = cv2.contourArea(cor)
            print(area)
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            # cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            vel_mag = math.sqrt((cX-180)*2+(cY-120)*2)
            z = (120-cY)
            x = (cX - 180)
            if(area < 2300):
                y = 30
            elif(area > 3000):
                y = -30
        if abs(x) >= 5:
            x = int(30 * x/vel_mag)
            z = int(30 * z/vel_mag)
            me.send_rc_control(x, y, z, 0)
            sleep(0.5)
            print(x,y,z)

        ##print("[INFO] ArUco marker ID: {}".format(markerID))
        # show the output image
    else:
        me.send_rc_control(0, 0, 0, 0)
        sleep(0.1)
    cv2.imshow("Image", image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        me.land()
        break
me.streamoff()
cv2.destroyAllWindows()