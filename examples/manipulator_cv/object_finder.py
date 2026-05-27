#
# object_finder.py
#

import cv2
import numpy as np

class ObjectFinder:

    def __init__(self, thr):
        low_theshold, high_threshold = thr
        self.lower_limit = np.array(low_theshold)
        self.higher_limit = np.array(high_threshold)

    def find(self, img):
        binary_image = cv2.inRange(img, self.lower_limit, self.higher_limit)
        contours, hierarchy = cv2.findContours(binary_image,
                                                cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            cv2.drawContours(img, contours, -1,  (255, 255, 255))
            cnt = contours[0]
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                # get the centroid of the contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.circle(img, (cx, cy), 3, (0,0,0), -1)
        else:
            cx = -1
            cy = -1

        return cx, cy, binary_image
