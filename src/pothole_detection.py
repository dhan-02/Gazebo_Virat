#!/usr/bin/env python3
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import imutils
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped 

from Gazebo_Virat.msg import Centres
from image_geometry import PinholeCameraModel as pcm

class Pothole_Detection:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.virat_camera = pcm() 
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('/virat/camera1/image_raw', Image, self.callback_image)
        rospy.Subscriber('/virat/camera1/camera_info', CameraInfo, self.callback_info)
        self.pub = rospy.Publisher('/virat/camera_detected_points',Centres,queue_size=100)
        self.transform = TransformStamped()
    def callback_image(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv2.waitKey(3)
        img = cv2.medianBlur(img,5)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask_w = cv2.inRange(gray, 240, 255)
        cnts = cv2.findContours(mask_w, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        centres = Centres()
        try:
            transform = self.tf_buffer.lookup_transform("odom",
                                       "camera_link",
                                       rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        for c in cnts:                                
            if(cv2.contourArea(c) > 500):
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(img,(int(x+w/2),int(y+h/2)),(int(x+5+w/2),int(y+5+h/2)),(0,0,255),-1)
                centres.x.append(int(x+w/2))
                centres.y.append(int(y+h/2))
                # org = (int(x),int(y))
                # cv2.putText(img,str(int(cv2.contourArea(c))),org,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255))
        cv2.imshow("img",img)
        cv2.waitKey(1)
        self.pub.publish(centres)
        
    def callback_info(self,info_msg):
        self.virat_camera.fromCameraInfo(info_msg)
    
   
if __name__ == '__main__':

    rospy.init_node('object_detection')    
    detector = Pothole_Detection()
    rospy.spin()
