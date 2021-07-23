#!/usr/bin/env python2
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge
import imutils

bridge = CvBridge()
def callback(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # cv2.waitKey(3)
    img = cv2.medianBlur(img,5)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
     
    # Threshold of blue in HSV space
    lower = np.array([200, 200, 200])
    upper = np.array([255, 255, 255])
    cv2.imshow("img",img)
    cv2.waitKey(1)
    # preparing the mask to overlay
    # mask = cv2.inRange(hsv, lower, upper)
    # cv2.imshow("orig",mask)
    # cv2.waitKey(1)
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    # gray = cv2.bitwise_and(img, img, mask = mask)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, 30, 200)
    
    cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    for c in cnts:
        if(cv2.contourArea(c) > 150):
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img,(int(x),int(y)),(int(x+w),int(y+h)),(0,0,255),2)
            # print(cv2.contourArea(c))
            org = (int(x),int(y))
            cv2.putText(img,str(int(cv2.contourArea(c))),org,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255))
            # cv2.imshow("img",img)
            # cv2.waitKey(1)

    # cnt = cnts[0]
    # cv2.imshow("Show",img)

    # # cv2.imshow("gray", gray)
    # # cv2.waitKey(1)
    # circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
    # # inverse ratio = 1
    # # min dist between circles is set to 50
    # # print(circles)
    
    # if circles is not None:
    #     circles = np.round(circles[0, :]).astype("int")
	# for (x, y, r) in circles:
	# 	cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
	# 	cv2.imshow("frame",img)
	# 	cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('object_detection')
    sub = rospy.Subscriber('/virat/camera1/image_raw', Image, callback)
    rospy.spin()
