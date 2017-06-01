#!/usr/bin/env python
from collections import deque
import numpy as np
import argparse
import imutils
import roslib
roslib.load_manifest('blob_tracking')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray as circlecenter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class ball_tracker:

  def __init__(self):
    self.bridge = CvBridge()
    self.center_pub = rospy.Publisher("ball_center", circlecenter, queue_size=10)
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

  def callback(self,data):
    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)

    #greenLower = (150, 100, 140)
    #greenUpper = (190, 255, 255)
    pts = deque(maxlen=64)

    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
	# find the largest contour in the mask, then use
	# it to compute the minimum enclosing circle and
	# centroid
	c = max(cnts, key=cv2.contourArea)
	((x, y), radius) = cv2.minEnclosingCircle(c)
	M = cv2.moments(c)
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        my_center = circlecenter()
        my_center.data = [x, y]
        self.center_pub.publish(my_center)

	# only proceed if the radius meets a minimum size
	if radius > 10:
		# draw the circle and centroid on the frame,
		# then update the list of tracked points
		cv2.circle(frame, (int(x), int(y)), int(radius),
			(0, 255, 255), 2)
		cv2.circle(frame, center, 5, (0, 0, 255), -1)
                print(hsv[x, y, 0])
   	 	print(hsv[x, y, 1])
    		print(hsv[x, y, 2])
                print('**************************************')
 
    # update the points queue
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in xrange(1, len(pts)):
	# if either of the tracked points are None, ignore
	# them
	if pts[i - 1] is None or pts[i] is None:
		continue
 
	# otherwise, compute the thickness of the line and
	# draw the connecting lines
	thickness = int(np.sqrt(64.0 / float(i + 1)) * 2.5)
	cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
 
    # show the frame to our screen
    #cv2.imshow("Frame", frame)
    
    cv2.imwrite('hue.png', mask[0])
    cv2.imwrite('sat.png', mask[1])
    cv2.imwrite('val.png', mask[2])
    cv2.imshow("Mask", mask)
    key = cv2.waitKey(1) 
 
def main(args):
  ic = ball_tracker()
  rospy.init_node('ball_tracker', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv) 













